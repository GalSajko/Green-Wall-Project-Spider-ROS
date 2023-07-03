import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import time

from configuration import ros_config, spider, robot_config
from utils import json_file_manager
from utils import custom_interface_helper
from calculations import kinematics as kin
from calculations import transformations as tf

from gwpspider_interfaces.srv import GetWalkingInstructions, GetModifiedWalkingInstructions, MoveLeg, MoveSpider, DistributeForces, GetSpiderPose

class App(Node):
    def __init__(self):
        Node.__init__(self, 'application')

        self.json_file_manager = json_file_manager.JsonFileManager()

        self.callback_group = ReentrantCallbackGroup()

        # self.get_walking_instructions_client = self.create_client(GetWalkingInstructions, ros_config.GET_WALKING_INSTRUCTIONS_SERVICE)
        self.get_modified_walking_instructions_client = self.create_client(GetModifiedWalkingInstructions, ros_config.GET_MODIFIED_WALKING_INSTRUCTION_SERVICE, callback_group = self.callback_group)
        while not self.get_modified_walking_instructions_client.wait_for_service(timeout_sec = 1.0):
            print("Path planning service not available...")

        self.move_leg_client = self.create_client(MoveLeg, ros_config.MOVE_LEG_SERVICE, callback_group = self.callback_group)
        while not self.move_leg_client.wait_for_service(timeout_sec = 1.0):
            print("Leg moving service not available...")

        self.move_spider_client = self.create_client(MoveSpider, ros_config.MOVE_SPIDER_SERVICE, callback_group = self.callback_group)
        while not self.move_spider_client.wait_for_service(timeout_sec = 1.0):
            print("Spider moving service not available...")   

        self.distribute_forces_client = self.create_client(DistributeForces, ros_config.DISTRIBUTE_FORCES_SERVICE, callback_group = self.callback_group)
        while not self.distribute_forces_client.wait_for_service(timeout_sec = 1.0):
            print("Distribute forces service not available...")

        self.get_spider_pose_client = self.create_client(GetSpiderPose, ros_config.GET_SPIDER_POSE_SERVICE, callback_group = self.callback_group)
        while not self.get_spider_pose_client.wait_for_service(timeout_sec = 1.0):
            print("Distribute forces service not available...")

        print("All services available.")

        time.sleep(1)

        self.working()

    def working(self):
        goal_pose = np.array([3.0, 1.5, 0.3, 0.0])
        spider_pose, _, start_legs_positions = self.json_file_manager.read_spider_state()

        walking_instructions_request = custom_interface_helper.prepare_modified_walking_instructions_request((start_legs_positions, goal_pose))
        walking_instructions_response = custom_interface_helper.async_service_call(self.get_modified_walking_instructions_client, walking_instructions_request, self)

        poses = custom_interface_helper.unpack_2d_array_message(walking_instructions_response.walking_instructions.poses)
        pins_instructions = custom_interface_helper.unpack_3d_array_message(walking_instructions_response.walking_instructions.pins_instructions)

        for step, pose in enumerate(poses):
            current_pins_positions = pins_instructions[step, :, 1:]
            current_legs_moving_order = pins_instructions[step, :, 0].astype(int)
            if step == 0:
                move_spider_request = custom_interface_helper.prepare_move_spider_request((current_legs_moving_order, current_pins_positions, pose, 5.0))
                move_spider_response = custom_interface_helper.async_service_call(self.move_spider_client, move_spider_request, self)
                
                self.json_file_manager.update_whole_dict(pose, current_pins_positions, current_legs_moving_order)

                distribute_forces_request = custom_interface_helper.prepare_distribute_forces_request(spider.LEGS_IDS)
                distribute_forces_resonse = custom_interface_helper.async_service_call(self.distribute_forces_client, distribute_forces_request, self)

                continue
            
            previous_pins_positions = np.array(pins_instructions[step - 1, :, 1:])
            move_spider_request = custom_interface_helper.prepare_move_spider_request((current_legs_moving_order, previous_pins_positions, pose, 2.5))
            move_spider_response = custom_interface_helper.async_service_call(self.move_spider_client, move_spider_request, self)

            self.json_file_manager.update_whole_dict(pose, previous_pins_positions, current_legs_moving_order)

            previous_to_current_pins_offsets = current_pins_positions - previous_pins_positions
            for idx, leg_id in enumerate(current_legs_moving_order):
                if previous_to_current_pins_offsets[idx].any():
                    distribute_forces_request = custom_interface_helper.prepare_distribute_forces_request(np.delete(spider.LEGS_IDS, leg_id))
                    distribute_forces_resonse = custom_interface_helper.async_service_call(self.distribute_forces_client, distribute_forces_request, self)

                    _, _, legs_global_positions = self.json_file_manager.read_spider_state()
                    get_spider_pose_request = custom_interface_helper.prepare_get_spider_pose_request((spider.LEGS_IDS, legs_global_positions))
                    get_spider_pose_response = custom_interface_helper.async_service_call(self.get_spider_pose_client, get_spider_pose_request, self)
                    spider_pose = get_spider_pose_response.spider_pose.data
                    rpy = spider_pose[3:]
                    pin_to_pin_vector_in_local, _ = tf.get_pin_to_pin_vector_in_local(leg_id, rpy, previous_pins_positions[idx], current_pins_positions[idx])
                    print(pin_to_pin_vector_in_local)

                    move_leg_request = custom_interface_helper.prepare_move_leg_request((
                        leg_id,
                        pin_to_pin_vector_in_local,
                        robot_config.BEZIER_TRAJECTORY,
                        robot_config.LEG_ORIGIN,
                        2.5,
                        True,
                        pose,
                        True
                    ))
                    self.json_file_manager.update_pins(leg_id, current_pins_positions[idx])
                    move_leg_response = custom_interface_helper.async_service_call(self.move_leg_client, move_leg_request, self)

def main():
    rclpy.init()
    app = App()
    executor = MultiThreadedExecutor()
    rclpy.spin(app, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()