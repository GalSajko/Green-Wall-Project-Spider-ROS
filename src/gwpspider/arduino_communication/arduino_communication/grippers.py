import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String

import time
import threading

from configuration import ros_config, robot_config, spider
from arduino_communication.arduino_comm import ArduinoComm

from gwpspider_interfaces.srv import MoveGripper

class GrippersController(Node):
    def __init__(self):
        Node.__init__(self, 'grippers_controller')
        
        self.arduino_comm = ArduinoComm(self.DEVICE_NAME, self.RECEIVED_MESSAGE_LENGTH)

        self.grippers_states = None
        self.grippers_states_locker = threading.Lock()

        self.callback_group = ReentrantCallbackGroup()
        self.move_service = self.create_service(MoveGripper, ros_config.MOVE_GRIPPER_SERVICE, self.move_gripper_callback, callback_group = self.callback_group)
        self.states_publisher = self.create_publisher(String, ros_config.GRIPPER_STATES_TOPIC, 10, callback_group = self.callback_group)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.publish_states_callback, callback_group = self.callback_group)
    
    @property
    def RECEIVED_MESSAGE_LENGTH(self):
        return 11
    
    @property
    def DEVICE_NAME(self):
        return 'ttyUSB_GRIPPERS'
    
    @property
    def MAX_ALLOWED_GRIPPER_MOVEMENT_TIME(self):
        return 3.0
    
    def move_gripper_callback(self, request, response):
        leg_id = request.instructions.leg_id
        command = request.instructions.command

        if leg_id not in spider.LEGS_IDS:
            print(f"Leg with ID {leg_id} was not recognized.")
            response.success = False
            return response
        
        if command not in (robot_config.OPEN_GRIPPER_COMMAND, robot_config.CLOSE_GRIPPER_COMMAND):
            print(f"Command {command} was not recognized.")
            response.success = False
            return response
        
        msg = command + str(leg_id) + '\n'
        self.arduino_comm.write(msg)
    
        if command == robot_config.OPEN_GRIPPER_COMMAND:
            excpected_state = robot_config.IS_GRIPPER_OPEN_RESPONSE
            message = 'open'
        else:
            excpected_state = robot_config.IS_GRIPPER_CLOSE_RESPONSE
            message = 'close'

        with self.grippers_states_locker:
            states = self.grippers_states
        
        start_time = time.time()
        while states[leg_id] != excpected_state:
            with self.grippers_states_locker:
                states = self.grippers_states
                time.sleep(0.01)
            elapsed_time = time.time() - start_time
            if elapsed_time > self.MAX_ALLOWED_GRIPPER_MOVEMENT_TIME:
                print(f"Gripper did not {message} correctly")
                response.success = False
                return response

        response.success = True
        return response
    
    def publish_states_callback(self):
        msg = String()
        with self.grippers_states_locker:
            self.grippers_states = self.arduino_comm.read()
            msg.data = self.grippers_states
        self.states_publisher.publish(msg)

def main():
    rclpy.init()
    grippers_controller = GrippersController()
    executor = MultiThreadedExecutor()
    rclpy.spin(grippers_controller, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()