import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import time
import threading

from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Float32

import gwpspider_interfaces.srv as gwp_services
from gwpspider_interfaces.msg import DynamixelMotorsData, GrippersStates
from gwpspider_interfaces import gwp_interfaces_data as gid
from utils import custom_interface_helper as cih
from calculations import mathtools as mt
from configuration import robot_config as rc
from configuration import spider

class Safety(Node):
    def __init__(self):
        Node.__init__(self, 'safety_node')

        self.grippers_states_locker = threading.Lock()
        self.grippers_attached_states = np.array([False] * spider.NUMBER_OF_LEGS)

        self.battery_voltage_locker = threading.Lock()
        self.battery_voltage = None

        self.dxl_data_locker = threading.Lock()
        self.currents_sum = None
        self.hw_errors = None
        self.currents_buffer = np.zeros((self.CURRENTS_WINDOW_SIZE, self.CURRENTS_ARRAY_ROWS, self.CURRENTS_ARRAY_COLUMNS))
        self.counter = 0

        self.toggle_safety_locker = threading.Lock()
        self.do_monitor_safety = True

        self.reentrant_callback_group = ReentrantCallbackGroup()
        self.__init_interfaces()

        # self.safety_loop()
    
    @property
    def TIME_INTEGRATION_WINDOW(self):
        return 1.0
    
    @property
    def MAX_CURRENT(self):
        return 3.0
    
    @property
    def CURRENTS_WINDOW_SIZE(self):
        return int(self.TIME_INTEGRATION_WINDOW * rc.CONTROLLER_FREQUENCY)
    
    @property
    def CURRENTS_ARRAY_COLUMNS(self):
        return 3
    
    @property
    def CURRENTS_ARRAY_ROWS(self):
        return 5
    
    @property
    def CURRENTS_SUM_THRESHOLD(self):
        return self.MAX_CURRENT * self.CURRENTS_WINDOW_SIZE
    
    @property
    def MIN_ALLOWED_VOLTAGE(self):
        return 15.8
    
    def dynamixel_motors_data_callback(self, msg):
        hw_errors = cih.unpack_2d_array_message(msg.motor_errors)
        currents = cih.unpack_2d_array_message(msg.currents)

        currents_sum, self.currents_buffer, self.counter = mt.integrate_array(self.currents_buffer, currents, self.counter)

        with self.battery_voltage_locker:
            battery_voltage = self.battery_voltage
        with self.toggle_safety_locker:
            do_monitor_safety = self.do_monitor_safety

        is_hw_errors = np.any(hw_errors) if hw_errors is not None else False
        is_current_overload_error = np.any(abs(currents_sum) > self.CURRENTS_SUM_THRESHOLD) if currents_sum is not None else False
        is_battery_voltage_error = ((battery_voltage < self.MIN_ALLOWED_VOLTAGE) and self.grippers_attached_states.all()) if battery_voltage is not None else False

        # self.get_logger().info(f"BATTERY VOLTAGE: {battery_voltage}")

        if do_monitor_safety:
            if is_hw_errors or is_current_overload_error or is_battery_voltage_error:
                with self.toggle_safety_locker:
                    self.do_monitor_safety = False
                # This also forces working procedure in app module to return False.
                stop_legs_request = Trigger.Request()
                stop_legs_response = cih.async_service_call_from_service(self.stop_legs_movement_client, stop_legs_request)
                stop_pump_request = Trigger.Request()
                stop_pump_response = cih.async_service_call_from_service(self.stop_water_pump_client, stop_pump_request)  

                time.sleep(1.5)
            
            if is_battery_voltage_error:
                self.get_logger().info("BATTERY VOLTAGE ERROR")
                start_transition_request = gwp_services.SendStringCommand.Request(command = rc.TRANSITION_STATE)
                start_transition_response = cih.async_service_call_from_service(self.states_manager_client, start_transition_request)

        # self.get_logger().info(f"DXL DATA: {errors, currents}")
            
    def grippers_states_callback(self, msg):
        with self.grippers_states_locker:
            self.grippers_attached_states = np.array([
                msg.first_gripper.is_attached,
                msg.second_gripper.is_attached,
                msg.third_gripper.is_attached,
                msg.fourth_gripper.is_attached,
                msg.fifth_gripper.is_attached
            ])
        # self.get_logger().info(f"GRIPPERS: {self.grippers_attached_states}")
    
    def battery_voltage_callback(self, msg):
        with self.battery_voltage_locker:
            self.battery_voltage = msg.data
        # self.get_logger().info(f"VOLTAGE: {self.battery_voltage}")
    
    def toggle_safety_callback(self, request, response):
        with self.toggle_safety_locker:
            self.do_monitor_safety = request.data
        
        response.success = self.do_monitor_safety == request.data
        return response

    def __init_interfaces(self):
        self.toggle_safety_service = self.create_service(SetBool, gid.TOGGLE_SAFETY_SERVICE, callback = self.toggle_safety_callback, callback_group = self.reentrant_callback_group)

        self.toggle_controller_client = self.create_client(gwp_services.ToggleController, gid.TOGGLE_CONTROLLER_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.toggle_controller_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Toggle controller service not available...")

        self.breaks_controller_client = self.create_client(gwp_services.BreaksControl, gid.BREAKS_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.breaks_controller_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Breaks controller service not available...")

        self.apply_force_client = self.create_client(gwp_services.ApplyForcesOnLegs, gid.APPLY_FORCES_ON_LEGS_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.apply_force_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Apply forces on legs service not available...")

        self.toggle_controller_mode_client = self.create_client(gwp_services.ToggleAdditionalControllerMode, gid.TOGGLE_ADDITIONAL_CONTROLLER_MODE_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.toggle_controller_mode_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Toggle additional controller mode service not available...")  
        
        self.toggle_motors_torque_client = self.create_client(gwp_services.ToggleMotorsTorque, gid.TOGGLE_MOTORS_TORQUE_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.toggle_motors_torque_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Toggle motors torques service not available...") 

        self.stop_legs_movement_client = self.create_client(Trigger, gid.STOP_LEGS_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.stop_legs_movement_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Stop legs movement service not available...")
        
        self.water_pump_client = self.create_client(gwp_services.ControlWaterPump, gid.WATER_PUMP_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.water_pump_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Water pump service not available...") 
        
        self.stop_water_pump_client = self.create_client(Trigger, gid.STOP_WATER_PUMP_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.stop_water_pump_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Stop water pump service not available...") 

        self.states_manager_client = self.create_client(gwp_services.SendStringCommand, gid.STATES_MANAGER_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.states_manager_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("States manager service not available...") 

        self.grippers_states_subscriber = self.create_subscription(GrippersStates, gid.GRIPPER_STATES_TOPIC, self.grippers_states_callback, 1, callback_group = self.reentrant_callback_group) 
        self.dynamixel_motors_data_subscriber = self.create_subscription(DynamixelMotorsData, gid.DYNAMIXEL_MOTORS_DATA_TOPIC, self.dynamixel_motors_data_callback, 1, callback_group = self.reentrant_callback_group)
        self.battery_voltage_subscriber = self.create_subscription(Float32, gid.BATTERY_VOLTAGE_TOPIC, self.battery_voltage_callback, 1, callback_group = self.reentrant_callback_group)

        self.get_logger().info("Safety is running.")
        
def main():
    rclpy.init()
    safety = Safety()
    executor = MultiThreadedExecutor()
    rclpy.spin(safety, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()