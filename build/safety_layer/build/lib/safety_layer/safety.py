import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import time

import gwpspider_interfaces.srv as gwp_services
from gwpspider_interfaces.msg import DynamixelMotorsData
from gwpspider_interfaces import gwp_interfaces_data as gid
from utils import custom_interface_helper as cih
from calculations import mathtools as mt
from configuration import robot_config as rc

class Safety(Node):
    def __init__(self):
        Node.__init__(self, 'safety_node')

        self.reentrant_callback_group = ReentrantCallbackGroup()
        self.__init_interfaces()

        self.currents_buffer = np.zeros((self.CURRENTS_WINDOW_SIZE, self.CURRENTS_ARRAY_ROWS, self.CURRENTS_ARRAY_COLUMNS))
        self.counter = 0

        self.current_sum = 0

        self.do_monitor_motors_states = True
    
    @property
    def CURRENTS_WINDOW_SIZE(self):
        return 100
    
    @property
    def CURRENTS_ARRAY_COLUMNS(self):
        return 3
    
    @property
    def CURRENTS_ARRAY_ROWS(self):
        return 5
    
    @property
    def CURRENTS_THRESHOLD(self):
        return 1000
    
    def dynamixel_motors_data_callback(self, msg):
        errors = cih.unpack_2d_array_message(msg.motor_errors)
        currents = cih.unpack_2d_array_message(msg.currents)

        self.current_sum += 1
        # currents_sum, self.currents_buffer, self.counter = mt.integrate_array(self.currents_buffer, currents, self.counter)
        
        if self.do_monitor_motors_states:
            if np.any(errors) or self.current_sum > 5000: #np.any(currents_sum > self.CURRENTS_THRESHOLD):
                self.do_monitor_motors_states = False

                # Stop the controller
                self.get_logger().info("STOP CONTROLLER")
                toggle_controller_request = gwp_services.ToggleController.Request(command = rc.STOP_COMMAND)
                toggle_controller_response = cih.async_service_call_from_service(self.toggle_controller_client, toggle_controller_request)

                # TODO: Check (and use later) attached legs.
                attached_legs = [0, 1, 2, 3, 4]
                
                # Activate the breaks
                self.get_logger().info("ACTIVATE BREAKS")
                activate_breaks_request = gwp_services.BreaksControl.Request(command = rc.ACTIVATE_BREAKS_COMMAND, break_id = rc.ALL_BREAKS_INDEX)
                activate_breaks_response = cih.async_service_call_from_service(self.breaks_controller_client, activate_breaks_request)

                # Start controller
                self.get_logger().info("START CONTROLLER")
                toggle_controller_request = gwp_services.ToggleController.Request(command = rc.START_COMMAND)
                toggle_controller_response = cih.async_service_call_from_service(self.toggle_controller_client, toggle_controller_request)

                # Start force mode (fd = 0)
                self.get_logger().info("START FORCE MODE")
                apply_force_request = cih.prepare_apply_forces_on_legs_request((attached_legs, np.zeros((len(attached_legs), 3))))
                apply_force_response = cih.async_service_call_from_service(self.apply_force_client, apply_force_request)

                # TODO: Implement that waiting in some different way.
                time.sleep(60)

                # Turn off force mode.
                self.get_logger().info("STOP FORCE MODE")
                toggle_additional_controller_mode_request = cih.prepare_toggle_controller_mode_request((rc.FORCE_MODE, rc.STOP_COMMAND))
                toggle_additiona_controller_mode_response = cih.async_service_call_from_service(self.toggle_controller_mode_client, toggle_additional_controller_mode_request)

                # Turn off torques in motors of attached legs.
                self.get_logger().info("TURN OFF TORQUES")
                toggle_motors_torques_request = cih.prepare_toggle_motors_torque_request((attached_legs, rc.DISABLE_LEGS_COMMAND))
                toggle_motors_torques_response = cih.async_service_call_from_service(self.toggle_motors_torque_client, toggle_motors_torques_request)
                          




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

        self.dynamixel_motors_data_subscriber = self.create_subscription(DynamixelMotorsData, gid.DYNAMIXEL_MOTORS_DATA_TOPIC, self.dynamixel_motors_data_callback, 1, callback_group = self.reentrant_callback_group)


def main():
    rclpy.init()
    safety = Safety()
    executor = MultiThreadedExecutor()
    rclpy.spin(safety, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()