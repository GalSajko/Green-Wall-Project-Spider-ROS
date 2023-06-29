import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np

from configuration import ros_config
from utils import json_file_manager
from utils import custom_interface_helper

from gwpspider_interfaces.srv import GetWalkingInstructions, GetModifiedWalkingInstructions, MoveLeg, MoveSpider, DistributeForces

class App(Node):
    def __init__(self):
        Node.__init__(self, 'application')

        self.json_file_manager = json_file_manager.JsonFileManager()

        self.callback_group = ReentrantCallbackGroup()

        # self.get_walking_instructions_client = self.create_client(GetWalkingInstructions, ros_config.GET_WALKING_INSTRUCTIONS_SERVICE)
        self.get_modified_walking_instructions_client = self.create_client(GetModifiedWalkingInstructions, ros_config.GET_MODIFIED_WALKING_INSTRUCTION_SERVICE, callback_group = self.callback_group)
        while not self.get_modified_walking_instructions_client.wait_for_service(timeout_sec = 1.0):
            print("Service not available...")
        # self.move_leg_client = self.create_client(MoveLeg, ros_config.MOVE_LEG_SERVICE)
        # self.move_spider_client = self.create_client(MoveSpider, ros_config.MOVE_SPIDER_SERVICE)

        # self.distribute_forces_client = self.create_client(DistributeForces, ros_config.DISTRIBUTE_FORCES_SERVICE)
        self.working()

    def working(self):
        goal_pose = np.array([3.0, 1.5, 0.3, 0.0])
        spider_pose, _, start_legs_positions = self.json_file_manager.read_spider_state()
        walking_instructions_request = custom_interface_helper.prepare_modified_walking_instructions_request((start_legs_positions, goal_pose))
        
        walking_instructions_response = custom_interface_helper.async_service_call(self.get_modified_walking_instructions_client, walking_instructions_request, self)

        poses = custom_interface_helper.unpack_2d_array_message(walking_instructions_response.walking_instructions.poses)
        pins = custom_interface_helper.unpack_3d_array_message(walking_instructions_response.walking_instructions.pins_instructions)

        print(poses, pins)

        # return poses, pins

def main():
    rclpy.init()
    app = App()
    executor = MultiThreadedExecutor()
    rclpy.spin(app, executor)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()


