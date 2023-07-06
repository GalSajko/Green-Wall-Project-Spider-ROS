import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import time
import threading

from configuration import ros_config, spider, robot_config
from utils import json_file_manager
from utils import custom_interface_helper
from calculations import kinematics as kin
from calculations import transformations as tf

from gwpspider_interfaces.srv import GetWalkingInstructions, GetModifiedWalkingInstructions, MoveLeg, MoveSpider, MoveGripper, DistributeForces, GetSpiderPose, ToggleAdditionalControllerMode, MoveLegVelocityMode
from gwpspider_interfaces.msg import GrippersStates

class App(Node):
    def __init__(self):
        Node.__init__(self, 'application')
        self.is_init = True

        self.json_file_manager = json_file_manager.JsonFileManager()
        self.grippers_attached_states = [False] * spider.NUMBER_OF_LEGS
        self.grippers_states_locker = threading.Lock()

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
            print("Spider pose service not available...")

        self.toggle_controller_mode_client = self.create_client(ToggleAdditionalControllerMode, ros_config.TOGGLE_ADDITIONAL_CONTROLLER_MODE_SERVICE, callback_group = self.callback_group)
        while not self.toggle_controller_mode_client.wait_for_service(timeout_sec = 1.0):
            print("Toggle additional controller mode service not available...")

        self.move_leg_velocity_mode_client = self.create_client(MoveLegVelocityMode, ros_config.MOVE_LEG_VELOCITY_MODE_SERVICE, callback_group = self.callback_group)
        while not self.move_leg_velocity_mode_client.wait_for_service(timeout_sec = 1.0):
            print("Velocity mode leg moving service not available...")
        
        self.move_gripper_client = self.create_client(MoveGripper, ros_config.MOVE_GRIPPER_SERVICE, callback_group = self.callback_group)
        while not self.move_gripper_client.wait_for_service(timeout_sec = 1.0):
            print("Gripper moving service not available...")        

        self.grippers_states_subscriber = self.create_subscription(GrippersStates, ros_config.GRIPPER_STATES_TOPIC, self.grippers_states_callback, 1, callback_group = self.callback_group)

        print("All services available.")

        time.sleep(1)

        self.working()

    def working(self):
        poses, pins_instructions = self.__get_movement_instructions()

        for step, pose in enumerate(poses):
            current_pins_positions = pins_instructions[step, :, 1:]
            current_legs_moving_order = pins_instructions[step, :, 0].astype(int)

            if step == 0:
                self.__move_spider(current_legs_moving_order, current_pins_positions, pose, 5.0) 
                self.json_file_manager.update_whole_dict(pose, current_pins_positions, current_legs_moving_order)
                self.__distribute_forces(spider.LEGS_IDS)
                continue
            
            previous_pins_positions = np.array(pins_instructions[step - 1, :, 1:])
            self.__move_spider(current_legs_moving_order, previous_pins_positions, pose, 2.5)

            self.json_file_manager.update_whole_dict(pose, previous_pins_positions, current_legs_moving_order)

            previous_to_current_pins_offsets = current_pins_positions - previous_pins_positions
            for idx, leg_id in enumerate(current_legs_moving_order):
                if previous_to_current_pins_offsets[idx].any():
                    self.__move_leg_to_next_pin(leg_id, previous_to_current_pins_offsets[idx], current_pins_positions[idx])

    def grippers_states_callback(self, msg):
        with self.grippers_states_locker:
            self.grippers_attached_states = [
                msg.first_gripper.is_attached,
                msg.second_gripper.is_attached,
                msg.third_gripper.is_attached,
                msg.fourth_gripper.is_attached,
                msg.fifth_gripper.is_attached
            ]
        
    def __get_movement_instructions(self):
        # TODO: Add service call for getting plant and watering instructions, modify walking instructions call.
        if self.is_init:
            # Call service for plant position and watering info.

            goal_pose = np.array([3.6, 1.5, 0.3, 0.0])
            spider_pose, _, start_legs_positions = self.json_file_manager.read_spider_state()

            walking_instructions_request = custom_interface_helper.prepare_modified_walking_instructions_request((start_legs_positions, goal_pose))
            walking_instructions_response = custom_interface_helper.async_service_call(self.get_modified_walking_instructions_client, walking_instructions_request, self)

            poses = custom_interface_helper.unpack_2d_array_message(walking_instructions_response.walking_instructions.poses)
            pins_instructions = custom_interface_helper.unpack_3d_array_message(walking_instructions_response.walking_instructions.pins_instructions)

        return poses, pins_instructions
    
    def __move_spider(self, legs_ids, legs_positions, pose, duration):
        move_spider_request = custom_interface_helper.prepare_move_spider_request((legs_ids, legs_positions, pose, duration))
        move_spider_response = custom_interface_helper.async_service_call(self.move_spider_client, move_spider_request, self)

        return move_spider_response.success

    def __distribute_forces(self, legs_ids):
        distribute_forces_request = custom_interface_helper.prepare_distribute_forces_request(legs_ids)
        distribute_forces_response = custom_interface_helper.async_service_call(self.distribute_forces_client, distribute_forces_request, self)

        return distribute_forces_response.success
    
    def __get_spider_pose(self, legs_ids):
        _, _, legs_global_positions = self.json_file_manager.read_spider_state()
        get_spider_pose_request = custom_interface_helper.prepare_get_spider_pose_request((legs_ids, legs_global_positions[legs_ids]))
        get_spider_pose_response = custom_interface_helper.async_service_call(self.get_spider_pose_client, get_spider_pose_request, self)
        spider_pose = get_spider_pose_response.spider_pose.data

        return spider_pose
    
    def __move_leg_to_next_pin(self, leg_id, pin_to_pin_vector_in_global, goal_pin_position):
        self.__distribute_forces(np.delete(spider.LEGS_IDS, leg_id))

        spider_pose = self.__get_spider_pose(spider.LEGS_IDS)
        rpy = spider_pose[3:]
        pin_to_pin_vector_in_local, leg_base_orientation_in_local = tf.get_global_vector_in_local(leg_id, rpy, pin_to_pin_vector_in_global)

        move_leg_request = custom_interface_helper.prepare_move_leg_request((
            leg_id,
            pin_to_pin_vector_in_local,
            robot_config.BEZIER_TRAJECTORY,
            robot_config.LEG_ORIGIN,
            2.5,
            True,
            [],
            True,
            True
        ))
        self.json_file_manager.update_pins(leg_id, goal_pin_position)
        move_leg_response = custom_interface_helper.async_service_call(self.move_leg_client, move_leg_request, self)

        with self.grippers_states_locker:
            is_attached = self.grippers_attached_states[leg_id]
            self.get_logger().info(f"IS ATTACHED: {is_attached}")
        if not is_attached:
            self.__automatic_correction(leg_id, leg_base_orientation_in_local, goal_pin_position)
    
    def __automatic_correction(self, leg_id, leg_base_orientation_in_local, goal_pin_position):
        global_z_direction_in_local = np.dot(leg_base_orientation_in_local, np.array([0.0, 0.0, 1.0], dtype = np.float32))
        detach_z_offset = 0.08
        offset_value = 0.1
        detach_position = np.copy(goal_pin_position)
        detach_position[2] += detach_z_offset
        # offsets = np.array([
        #     [0.0, 0.0, 0.0],
        #     [0.0, offset_value, 0.0],
        #     [offset_value, offset_value, 0.0],
        #     [-offset_value, offset_value, 0.0],
        #     [offset_value, 0.0, 0.0],
        #     [-offset_value, 0.0, 0.0],
        #     [0.0, -offset_value, 0.0],
        #     [offset_value, -offset_value, 0.0],
        #     [-offset_value, -offset_value, 0.0],
        # ])
        offsets = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 0.25, 0.0],
            [0.0, -0.25, 0.0],
            [-0.25, 0.0, 0.0],
            [0.25, 0.0, 0.0]
        ])
        spider_pose = self.__get_spider_pose(np.delete(spider.LEGS_IDS, leg_id))
        for offset in offsets:
            move_leg_request = custom_interface_helper.prepare_move_leg_request((
                leg_id,
                detach_position,
                robot_config.MINJERK_TRAJECTORY,
                robot_config.GLOBAL_ORIGIN,
                2.0,
                False,
                spider_pose,
                True,
                False
            ))
            move_leg_response = custom_interface_helper.async_service_call(self.move_leg_client, move_leg_request, self)

            local_offset = tf.get_global_vector_in_local(leg_id, spider_pose[3:], offset)[0]
            velocity_direction = np.array([global_z_direction_in_local[0] + local_offset[0], global_z_direction_in_local[1] + local_offset[1], -global_z_direction_in_local[2]], dtype = np.float32)
            move_leg_velocity_mode_request = custom_interface_helper.prepare_move_leg_velocity_mode_request((leg_id, velocity_direction))
            move_leg_velocity_mode_response = custom_interface_helper.async_service_call(self.move_leg_velocity_mode_client, move_leg_velocity_mode_request, self)

            move_gripper_request = custom_interface_helper.prepare_move_gripper_request((leg_id, robot_config.CLOSE_GRIPPER_COMMAND))
            move_gripper_response = custom_interface_helper.async_service_call(self.move_gripper_client, move_gripper_request, self)
            
            with self.grippers_states_locker:
                is_attached = self.grippers_attached_states[leg_id]
            self.get_logger().info(f"IS ATTACHED: {is_attached}")
            if is_attached:
                return

def main():
    rclpy.init()
    app = App()
    executor = MultiThreadedExecutor()
    rclpy.spin(app, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()