import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import time
import threading

from std_srvs.srv import Trigger
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
        self.grippers_attached_states = [False] * spider.NUMBER_OF_LEGS

        self.battery_voltage_locker = threading.Lock()
        self.battery_voltage = None

        self.reentrant_callback_group = ReentrantCallbackGroup()
        self.__init_interfaces()

        self.currents_buffer = np.zeros((self.CURRENTS_WINDOW_SIZE, self.CURRENTS_ARRAY_ROWS, self.CURRENTS_ARRAY_COLUMNS))
        self.counter = 0

        self.do_monitor_motors_states = True
    
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
        return 14.2
    
    def dynamixel_motors_data_callback(self, msg):
        errors = cih.unpack_2d_array_message(msg.motor_errors)
        currents = cih.unpack_2d_array_message(msg.currents)

        currents_sum, self.currents_buffer, self.counter = mt.integrate_array(self.currents_buffer, currents, self.counter)

        with self.battery_voltage_locker:
            battery_voltage = self.battery_voltage
        
        if battery_voltage < self.MIN_ALLOWED_VOLTAG and self.grippers_attached_states.all():
            stop_legs_request = Trigger.Request()
            stop_legs_response = cih.async_service_call_from_service(self.stop_legs_movement_client, stop_legs_request)


        if self.do_monitor_motors_states:
            is_hw_error = np.any(errors)
            is_current_overload_error = np.any(abs(currents_sum) > self.CURRENTS_SUM_THRESHOLD)

            if is_current_overload_error or is_hw_error:
                self.do_monitor_motors_states = False

                # Stop any movement.
                if is_current_overload_error:
                    self.get_logger().info("SAFETY STOP - CURRENT")
                if is_hw_error:
                    self.get_logger().info(f"SAFETY STOP - HW ERROR, error codes: {errors}")
                stop_legs_request = Trigger.Request()
                stop_legs_response = cih.async_service_call_from_service(self.stop_legs_movement_client, stop_legs_request)

                # Stop all water pumps.
                stop_pump_request = Trigger.Request()
                stop_pump_response = cih.async_service_call_from_service(self.stop_water_pump_client, stop_pump_request)
                # with self.grippers_states_locker: 
                #     attached_legs = [i for i, j in enumerate(self.grippers_attached_states) if j]
                
                # # Activate the breaks.
                # self.get_logger().info("ACTIVATE BREAKS")
                # activate_breaks_request = gwp_services.BreaksControl.Request(command = rc.ACTIVATE_BREAKS_COMMAND, break_id = rc.ALL_BREAKS_INDEX)
                # activate_breaks_response = cih.async_service_call_from_service(self.breaks_controller_client, activate_breaks_request)

                # if not len(attached_legs) == spider.NUMBER_OF_LEGS:
                #     not_attached_leg_array = np.delete(spider.LEGS_IDS, attached_legs)
                #     for leg in not_attached_leg_array:
                #         deactivate_breaks_request = gwp_services.BreaksControl.Request(command = rc.RELEASE_BREAKS_COMMAND, break_id = int(leg))
                #         deactivate_breaks_response = cih.async_service_call_from_service(self.breaks_controller_client, deactivate_breaks_request)

                # # Start force mode (fd = 0).
                # self.get_logger().info("START FORCE MODE")
                # forces = np.zeros((len(attached_legs), 3))
                # forces[:, 1] = np.ones(len(attached_legs)) * 0.5
                # apply_force_request = cih.prepare_apply_forces_on_legs_request((attached_legs, forces))
                # apply_force_response = cih.async_service_call_from_service(self.apply_force_client, apply_force_request)

                # # TODO: Implement that waiting in some different way.
                # time.sleep(15)

                # # Turn off force mode.
                # self.get_logger().info("STOP FORCE MODE")
                # toggle_additional_controller_mode_request = cih.prepare_toggle_controller_mode_request((rc.FORCE_MODE, rc.STOP_COMMAND))
                # toggle_additiona_controller_mode_response = cih.async_service_call_from_service(self.toggle_controller_mode_client, toggle_additional_controller_mode_request)

    def grippers_states_callback(self, msg):
        with self.grippers_states_locker:
            self.grippers_attached_states = [
                msg.first_gripper.is_attached,
                msg.second_gripper.is_attached,
                msg.third_gripper.is_attached,
                msg.fourth_gripper.is_attached,
                msg.fifth_gripper.is_attached
            ]
    
    def battery_voltage_callback(self, msg):
        with self.battery_voltage_locker:
            self.battery_voltage = msg.data

    def __init_interfaces(self):
        self.toggle_controller_client = self.create_client(gwp_services.ToggleController, gid.TOGGLE_CONTROLLER_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.toggle_controller_client.wait_for_service(timeout_sec = 1.0):
            print("Toggle controller service not available...")

        self.breaks_controller_client = self.create_client(gwp_services.BreaksControl, gid.BREAKS_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.breaks_controller_client.wait_for_service(timeout_sec = 1.0):
            print("Breaks controller service not available...")

        self.apply_force_client = self.create_client(gwp_services.ApplyForcesOnLegs, gid.APPLY_FORCES_ON_LEGS_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.apply_force_client.wait_for_service(timeout_sec = 1.0):
            print("Apply forces on legs service not available...")

        self.toggle_controller_mode_client = self.create_client(gwp_services.ToggleAdditionalControllerMode, gid.TOGGLE_ADDITIONAL_CONTROLLER_MODE_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.toggle_controller_mode_client.wait_for_service(timeout_sec = 1.0):
            print("Toggle additional controller mode service not available...")  
        
        self.toggle_motors_torque_client = self.create_client(gwp_services.ToggleMotorsTorque, gid.TOGGLE_MOTORS_TORQUE_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.toggle_motors_torque_client.wait_for_service(timeout_sec = 1.0):
            print("Toggle motors torques service not available...") 

        self.stop_legs_movement_client = self.create_client(Trigger, gid.STOP_LEGS_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.stop_legs_movement_client.wait_for_service(timeout_sec = 1.0):
            print("Stop legs movement service not available...")
        
        self.water_pump_client = self.create_client(gwp_services.ControlWaterPump, gid.WATER_PUMP_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.water_pump_client.wait_for_service(timeout_sec = 1.0):
            print("Water pump service not available...") 
        
        self.stop_water_pump_client = self.create_client(Trigger, gid.STOP_WATER_PUMP_SERVICE, callback_group = self.reentrant_callback_group)


        self.grippers_states_subscriber = self.create_subscription(GrippersStates, gid.GRIPPER_STATES_TOPIC, self.grippers_states_callback, 1, callback_group = self.reentrant_callback_group) 
        self.dynamixel_motors_data_subscriber = self.create_subscription(DynamixelMotorsData, gid.DYNAMIXEL_MOTORS_DATA_TOPIC, self.dynamixel_motors_data_callback, 1)
        self.battery_voltage_subscriber = self.create_subscription(Float32, gid.BATTERY_VOLTAGE_TOPIC, self.battery_voltage_callback, 1, callback_group = self.reentrant_callback_group)
        


def main():
    rclpy.init()
    safety = Safety()
    executor = MultiThreadedExecutor()
    rclpy.spin(safety, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()