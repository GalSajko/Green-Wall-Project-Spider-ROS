import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String

import time
import threading

from configuration import robot_config, spider
from arduino_communication.arduino_comm import ArduinoComm

from gwpspider_interfaces.srv import MoveGripper
from gwpspider_interfaces.msg import GripperState, GrippersStates
from gwpspider_interfaces import gwp_interfaces_data as gid

class GrippersController(Node):
    def __init__(self):
        Node.__init__(self, 'grippers_controller')
        
        self.arduino_comm = ArduinoComm(self.DEVICE_NAME, self.RECEIVED_MESSAGE_LENGTH)

        self.grippers_states = None
        self.grippers_states_locker = threading.Lock()
        self.serial_comm_locker = threading.Lock()

        self.callback_group = ReentrantCallbackGroup()
        self.move_service = self.create_service(MoveGripper, gid.MOVE_GRIPPER_SERVICE, self.move_gripper_callback, callback_group = self.callback_group)
        self.states_publisher = self.create_publisher(GrippersStates, gid.GRIPPER_STATES_TOPIC, 1, callback_group = self.callback_group)
        timer_period = 0.0
        self.timer = self.create_timer(timer_period, self.publish_states_callback, callback_group = self.callback_group)
    
    @property
    def RECEIVED_MESSAGE_LENGTH(self):
        return 11
    
    @property
    def DEVICE_NAME(self):
        return 'ttyUSB_GRIPPERS'
    
    @property
    def MAX_ALLOWED_GRIPPER_MOVEMENT_TIME(self):
        return 3.5
    
    def move_gripper_callback(self, request, response):
        leg_id = request.instructions.leg_id
        command = request.instructions.command

        if leg_id not in spider.LEGS_IDS:
            self.get_logger().info(f"Leg with ID {leg_id} was not recognized.")
            response.success = False
            return response
        
        if command not in (robot_config.OPEN_GRIPPER_COMMAND, robot_config.CLOSE_GRIPPER_COMMAND):
            self.get_logger().info(f"Command {command} was not recognized.")
            response.success = False
            return response
        
        msg = command + str(leg_id) + '\n'
        with self.serial_comm_locker:
            self.arduino_comm.write(msg)
    
        if command == robot_config.OPEN_GRIPPER_COMMAND:
            excpected_state = robot_config.IS_GRIPPER_OPEN_RESPONSE
            message = 'open'
        else:
            excpected_state = robot_config.IS_GRIPPER_CLOSE_RESPONSE
            message = 'close'

        with self.grippers_states_locker:
            states = self.grippers_states
        
        # start_time = time.time()
        # while states[leg_id] != excpected_state:
        #     with self.grippers_states_locker:
        #         states = self.grippers_states
            # elapsed_time = time.time() - start_time
            # if elapsed_time > self.MAX_ALLOWED_GRIPPER_MOVEMENT_TIME:
            #     self.get_logger().info(f"Gripper did not {message} correctly")
            #     response.success = False
            #     return response
        time.sleep(2.0)

        response.success = True
        return response
    
    def publish_states_callback(self):
        msg = GrippersStates()
        with self.serial_comm_locker:
            states = self.arduino_comm.read()
        with self.grippers_states_locker:
            self.grippers_states = states
            
        for leg_id in spider.LEGS_IDS:
            sub_msg = GripperState()

            fingers_state = states[leg_id]
            switch_state = states[leg_id + 5]

            sub_msg.fingers_state = fingers_state
            sub_msg.switch_state = switch_state
            sub_msg.is_attached = fingers_state == robot_config.IS_GRIPPER_CLOSE_RESPONSE and switch_state == robot_config.IS_GRIPPER_CLOSE_RESPONSE
            
            if leg_id == 0:
                msg.first_gripper = sub_msg
            elif leg_id == 1:
                msg.second_gripper = sub_msg
            elif leg_id == 2:
                msg.third_gripper = sub_msg
            elif leg_id == 3:
                msg.fourth_gripper = sub_msg
            elif leg_id == 4:
                msg.fifth_gripper = sub_msg

        self.states_publisher.publish(msg)

def main():
    rclpy.init()
    grippers_controller = GrippersController()
    executor = MultiThreadedExecutor()
    rclpy.spin(grippers_controller, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()