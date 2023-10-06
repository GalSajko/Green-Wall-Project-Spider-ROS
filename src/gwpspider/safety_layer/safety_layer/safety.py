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

        self.monitor_battery_voltage_locker = threading.Lock()
        self.monitor_battery_voltage = True

        self.dxl_data_locker = threading.Lock()
        self.currents_sum = None
        self.hw_errors = None
        self.currents_buffer = np.zeros((self.CURRENTS_WINDOW_SIZE, self.CURRENTS_ARRAY_ROWS, self.CURRENTS_ARRAY_COLUMNS))
        self.counter = 0

        self.monitor_hw_errors_locker = threading.Lock()
        self.monitor_hw_errors = True

        self.reentrant_callback_group = ReentrantCallbackGroup()
        self.__init_interfaces()
    
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
        return 220 #self.MAX_CURRENT * self.CURRENTS_WINDOW_SIZE
    
    @property
    def MIN_ALLOWED_VOLTAGE(self):
        return 14.5
    
    def dynamixel_motors_data_callback(self, msg):
        hw_errors = cih.unpack_2d_array_message(msg.motor_errors)
        currents = cih.unpack_2d_array_message(msg.currents)

        with self.battery_voltage_locker:
            battery_voltage = self.battery_voltage
        with self.monitor_battery_voltage_locker:
            do_monitor_voltage = self.monitor_battery_voltage
        with self.monitor_hw_errors_locker:
            monitor_hw_errors = self.monitor_hw_errors

        is_hw_errors, is_current_overload_error, is_battery_voltage_error = self.__get_errors(currents, hw_errors, battery_voltage)
        if (is_hw_errors or is_current_overload_error) and monitor_hw_errors:   
            with self.monitor_hw_errors_locker:
                self.monitor_hw_errors = False
            if is_hw_errors:
                self.get_logger().info("HW ERROR TRIGGERED.")
            elif is_current_overload_error:
                self.get_logger().info("CURRENT OVERLOAD ERROR TRIGGERED.")
            immediate_stop_request = Trigger.Request()
            immediate_stop_response = cih.async_service_call_from_service(self.immediate_stop_trigger_client, immediate_stop_request)

        if is_battery_voltage_error and do_monitor_voltage:
            with self.monitor_battery_voltage_locker:
                self.monitor_battery_voltage = False
            self.get_logger().info("BATTERY VOLTAGE ERROR TRIGGERED.")
            trigger_battery_voltage_error_request = SetBool.Request(data = True)
            trigger_battery_voltage_error_response = cih.async_service_call_from_service(self.battery_voltage_error_trigger_client, trigger_battery_voltage_error_request)
            
    def grippers_states_callback(self, msg):
        with self.grippers_states_locker:
            self.grippers_attached_states = np.array([
                msg.first_gripper.is_attached,
                msg.second_gripper.is_attached,
                msg.third_gripper.is_attached,
                msg.fourth_gripper.is_attached,
                msg.fifth_gripper.is_attached
            ])
    
    def battery_voltage_callback(self, msg):
        with self.battery_voltage_locker:
            self.battery_voltage = msg.data
    
    def toggle_battery_voltage_monitoring_callback(self, request, response):
        with self.monitor_battery_voltage_locker:
            self.monitor_battery_voltage = request.data
            response.success = self.monitor_battery_voltage == request.data
        
        return response

    def toggle_hw_errors_monitoring_callback(self, request, response):
        with self.monitor_hw_errors_locker:
            self.monitor_hw_errors = request.data
            response.success = self.monitor_hw_errors == request.data
        
        return response
    
    def __get_errors(self, currents, hw_errors, battery_voltage):
        currents_sum, self.currents_buffer, self.counter = mt.integrate_array(self.currents_buffer, currents, self.counter)
        is_hw_errors = np.any(hw_errors) if hw_errors is not None else False
        is_current_overload_error = np.any(abs(currents_sum) > self.CURRENTS_SUM_THRESHOLD) if currents_sum is not None else False
        with self.grippers_states_locker:
            grippers_attached_states = self.grippers_attached_states
        is_battery_voltage_error = ((battery_voltage < self.MIN_ALLOWED_VOLTAGE) and grippers_attached_states.all()) if battery_voltage is not None else False  

        return is_hw_errors, is_current_overload_error, is_battery_voltage_error   

    def __init_interfaces(self):
        self.monitor_battery_voltage_trigger_service = self.create_service(SetBool, gid.TOGGLE_BATTERY_VOLTAGE_MONITORING_SERVICE, callback = self.toggle_battery_voltage_monitoring_callback, callback_group = self.reentrant_callback_group)
        self.monitor_hw_errors_trigger_service = self.create_service(SetBool, gid.TOGGLE_HW_ERRORS_MONITORING_SERVICE, callback = self.toggle_hw_errors_monitoring_callback, callback_group = self.reentrant_callback_group)
        
        self.immediate_stop_trigger_client = self.create_client(Trigger, gid.IMMEDIATE_STOP_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.immediate_stop_trigger_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Imeediate stop trigger service not available...")

        self.battery_voltage_error_trigger_client = self.create_client(SetBool, gid.BATTERY_VOLTAGE_TRIGGER_SERVICE, callback_group = self.reentrant_callback_group)   
        while not self.battery_voltage_error_trigger_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Battery voltage error trigger service not available...")

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