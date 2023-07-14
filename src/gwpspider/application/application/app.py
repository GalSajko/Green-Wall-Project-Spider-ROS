import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import time
import threading

from configuration import spider, robot_config
from utils import json_file_manager
from utils import custom_interface_helper
from calculations import kinematics as kin
from calculations import transformations as tf

import gwpspider_interfaces.srv as gwp_services
from gwpspider_interfaces.msg import GrippersStates, LegsStates
from gwpspider_interfaces import gwp_interfaces_data as gid

from std_msgs.msg import Float32MultiArray

class App(Node):
    def __init__(self):
        Node.__init__(self, 'application')
        self.is_init = True
        self.was_watering_or_refilling_successful = False

        self.use_prediction_model = True

        self.spider_pose = None

        self.json_file_manager = json_file_manager.JsonFileManager()
        self.grippers_attached_states = [False] * spider.NUMBER_OF_LEGS
        self.grippers_states_locker = threading.Lock()

        self.legs_local_positions = None
        self.legs_positions_locker = threading.Lock()

        self.callback_group = ReentrantCallbackGroup()
        self.__init_interfaces()

        self.working()
    
    @property
    def LEG_MOVEMENT_DURATION_AMP(self):
        return 12.5

    def working(self):
        poses, pins_instructions, plant_or_refill_position, watering_or_refill_leg_id, volume = self.__get_movement_instructions()

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
        
        self.__watering(watering_or_refill_leg_id, plant_or_refill_position, poses[-1], volume)

    def grippers_states_callback(self, msg):
        with self.grippers_states_locker:
            self.grippers_attached_states = [
                msg.first_gripper.is_attached,
                msg.second_gripper.is_attached,
                msg.third_gripper.is_attached,
                msg.fourth_gripper.is_attached,
                msg.fifth_gripper.is_attached
            ]
    
    def read_legs_states_callback(self, msg):
        positions = custom_interface_helper.unpack_2d_array_message(msg.legs_local_positions)
        with self.legs_positions_locker:
            self.legs_local_positions = positions
        
    def __get_movement_instructions(self):
        spider_pose, _, start_legs_positions = self.json_file_manager.read_spider_state()

        # spider_goal_request = gwp_services.SpiderGoal.Request()
        # spider_goal_request.watered = self.was_watering_or_refilling_successful
        # spider_goal_response = custom_interface_helper.async_service_call(self.get_spider_goal_client, spider_goal_request, self)

        watering_position = np.array([2.1, 2.125, 0.0])
        go_refill = False
        volume = 25.0

        watering_or_refill_leg_id, watering_or_refill_pose = tf.get_watering_leg_and_pose(spider_pose, watering_position, go_refill)

        if self.is_init:
            walking_instructions_request = custom_interface_helper.prepare_modified_walking_instructions_request((start_legs_positions, watering_or_refill_pose))
            walking_instructions_response = custom_interface_helper.async_service_call(self.get_modified_walking_instructions_client, walking_instructions_request, self)
        else:
            walking_instructions_request = custom_interface_helper.prepare_walking_instructions_request((spider_pose, watering_or_refill_pose))
            walking_instructions_response = custom_interface_helper.async_service_call(self.get_walking_instructions_client, walking_instructions_request, self)

        poses = custom_interface_helper.unpack_2d_array_message(walking_instructions_response.walking_instructions.poses)
        pins_instructions = custom_interface_helper.unpack_3d_array_message(walking_instructions_response.walking_instructions.pins_instructions)

        return poses, pins_instructions, watering_position, watering_or_refill_leg_id, volume
    
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

        duration = self.LEG_MOVEMENT_DURATION_AMP * np.linalg.norm(pin_to_pin_vector_in_global)

        move_leg_request = custom_interface_helper.prepare_move_leg_request((
            leg_id,
            pin_to_pin_vector_in_local,
            robot_config.BEZIER_TRAJECTORY,
            robot_config.LEG_ORIGIN,
            duration,
            True,
            [],
            True,
            True,
            self.use_prediction_model
        ))
        self.json_file_manager.update_pins(leg_id, goal_pin_position)
        move_leg_response = custom_interface_helper.async_service_call(self.move_leg_client, move_leg_request, self)

        with self.grippers_states_locker:
            is_attached = self.grippers_attached_states[leg_id]
        if not is_attached:
            global_z_direction_in_local = np.dot(leg_base_orientation_in_local, np.array([0.0, 0.0, 1.0], dtype = np.float32))
            self.__automatic_correction(leg_id, global_z_direction_in_local, goal_pin_position)
    
    def __automatic_correction(self, leg_id, correction_direction, goal_pin_position):
        detach_z_offset = 0.08
        y_offset_value = 0.25
        x_offset_value = 0.12
        detach_position = np.copy(goal_pin_position)
        detach_position[2] += detach_z_offset

        offsets = np.array([
            [0.0, 0.0, 0.0],
            [0.0, y_offset_value, 0.0],
            [x_offset_value, y_offset_value, 0.0],
            [-x_offset_value, y_offset_value, 0.0],
            [x_offset_value, 0.0, 0.0],
            [-x_offset_value, 0.0, 0.0],
            [0.0, -y_offset_value / 2, 0.0],
            [x_offset_value, -y_offset_value / 2, 0.0],
            [-x_offset_value, -y_offset_value / 2, 0.0],
        ])

        spider_pose = self.__get_spider_pose(np.delete(spider.LEGS_IDS, leg_id))
        for idx, offset in enumerate(offsets):           
            is_offset = idx == 0
            position = correction_direction * detach_z_offset if is_offset else local_detach_position

            move_leg_request = custom_interface_helper.prepare_move_leg_request((
                leg_id,
                position,
                robot_config.MINJERK_TRAJECTORY,
                robot_config.LEG_ORIGIN,
                2.0,
                is_offset,
                [],
                True,
                False,
                False
            ))
            move_leg_response = custom_interface_helper.async_service_call(self.move_leg_client, move_leg_request, self)

            if idx == 0:
                with self.legs_positions_locker:
                        local_detach_position = self.legs_local_positions[leg_id]

            local_offset = tf.get_global_vector_in_local(leg_id, spider_pose[3:], offset)[0]
            velocity_direction = np.array([correction_direction[0] + local_offset[0], correction_direction[1] + local_offset[1], -correction_direction[2]], dtype = np.float32)
            move_leg_velocity_mode_request = custom_interface_helper.prepare_move_leg_velocity_mode_request((leg_id, velocity_direction))
            move_leg_velocity_mode_response = custom_interface_helper.async_service_call(self.move_leg_velocity_mode_client, move_leg_velocity_mode_request, self)

            move_gripper_request = custom_interface_helper.prepare_move_gripper_request((leg_id, robot_config.CLOSE_GRIPPER_COMMAND))
            move_gripper_response = custom_interface_helper.async_service_call(self.move_gripper_client, move_gripper_request, self)
            
            with self.grippers_states_locker:
                is_attached = self.grippers_attached_states[leg_id]
            if is_attached:
                return
            
    def __watering(self, leg_id, position, spider_pose, volume):
        # TODO: Activate breaks here.
        self.__distribute_forces(np.delete(spider.LEGS_IDS, leg_id))
        with self.legs_positions_locker:
            leg_local_position_before_watering = self.legs_local_positions[leg_id]

        watering_position = np.copy(position)
        y_watering_offset = 0.125
        watering_position[1] += y_watering_offset
        move_leg_request = custom_interface_helper.prepare_move_leg_request((
            leg_id,
            watering_position,
            robot_config.BEZIER_TRAJECTORY,
            robot_config.GLOBAL_ORIGIN,
            2.5,
            False,
            spider_pose,
            True,
            False,
            False
        ))
        move_leg_response = custom_interface_helper.async_service_call(self.move_leg_client, move_leg_request, self)

        if leg_id not in (1, 2):
            pump_id = 0
        else:
            pump_id = leg_id

        water_pump_request = custom_interface_helper.prepare_water_pump_request((pump_id, volume))
        water_pump_response = custom_interface_helper.async_service_call(self.water_pump_client, water_pump_request, self)

        move_leg_request = custom_interface_helper.prepare_move_leg_request((
            leg_id,
            leg_local_position_before_watering,
            robot_config.BEZIER_TRAJECTORY,
            robot_config.LEG_ORIGIN,
            2.5,
            False,
            [],
            False,
            True,
            False
        ))
        move_leg_response = custom_interface_helper.async_service_call(self.move_leg_client, move_leg_request, self)

        with self.grippers_states_locker:
            is_attached = self.grippers_attached_states[leg_id]
        if not is_attached:
            rpy = self.__get_spider_pose(np.delete(spider.LEGS_IDS, leg_id))[3:]
            correction_direction = tf.get_global_vector_in_local(leg_id, rpy, np.array([0.0, 0.0, 1.0], dtype = np.float32))[0]
            self.__automatic_correction(leg_id, correction_direction, leg_local_position_before_watering)

    def __init_interfaces(self):
        self.get_walking_instructions_client = self.create_client(gwp_services.GetWalkingInstructions, gid.GET_WALKING_INSTRUCTIONS_SERVICE)
        while not self.get_walking_instructions_client.wait_for_service(timeout_sec = 1.0):
            print("Path planning service not available...")

        self.get_modified_walking_instructions_client = self.create_client(gwp_services.GetModifiedWalkingInstructions, gid.GET_MODIFIED_WALKING_INSTRUCTION_SERVICE, callback_group = self.callback_group)
        while not self.get_modified_walking_instructions_client.wait_for_service(timeout_sec = 1.0):
            print("Modified path planning service not available...")

        self.move_leg_client = self.create_client(gwp_services.MoveLeg, gid.MOVE_LEG_SERVICE, callback_group = self.callback_group)
        while not self.move_leg_client.wait_for_service(timeout_sec = 1.0):
            print("Leg moving service not available...")

        self.move_spider_client = self.create_client(gwp_services.MoveSpider, gid.MOVE_SPIDER_SERVICE, callback_group = self.callback_group)
        while not self.move_spider_client.wait_for_service(timeout_sec = 1.0):
            print("Spider moving service not available...")

        self.distribute_forces_client = self.create_client(gwp_services.DistributeForces, gid.DISTRIBUTE_FORCES_SERVICE, callback_group = self.callback_group)
        while not self.distribute_forces_client.wait_for_service(timeout_sec = 1.0):
            print("Distribute forces service not available...")

        self.get_spider_pose_client = self.create_client(gwp_services.GetSpiderPose, gid.GET_SPIDER_POSE_SERVICE, callback_group = self.callback_group)
        while not self.get_spider_pose_client.wait_for_service(timeout_sec = 1.0):
            print("Spider pose service not available...")

        self.toggle_controller_mode_client = self.create_client(gwp_services.ToggleAdditionalControllerMode, gid.TOGGLE_ADDITIONAL_CONTROLLER_MODE_SERVICE, callback_group = self.callback_group)
        while not self.toggle_controller_mode_client.wait_for_service(timeout_sec = 1.0):
            print("Toggle additional controller mode service not available...")

        self.move_leg_velocity_mode_client = self.create_client(gwp_services.MoveLegVelocityMode, gid.MOVE_LEG_VELOCITY_MODE_SERVICE, callback_group = self.callback_group)
        while not self.move_leg_velocity_mode_client.wait_for_service(timeout_sec = 1.0):
            print("Velocity mode leg moving service not available...")
        
        self.move_gripper_client = self.create_client(gwp_services.MoveGripper, gid.MOVE_GRIPPER_SERVICE, callback_group = self.callback_group)
        while not self.move_gripper_client.wait_for_service(timeout_sec = 1.0):
            print("Gripper moving service not available...")  

        self.water_pump_client = self.create_client(gwp_services.ControlWaterPump, gid.WATER_PUMP_SERVICE, callback_group = self.callback_group)
        while not self.water_pump_client.wait_for_service(timeout_sec = 1.0):
            print("Water pump service not available...")        

        self.get_spider_goal_client = self.create_client(gwp_services.SpiderGoal, gid.SEND_GOAL_SERVICE, callback_group = self.callback_group)
        while not self.move_gripper_client.wait_for_service(timeout_sec = 1.0):
            print("Spider goal service not available...")  

        self.grippers_states_subscriber = self.create_subscription(GrippersStates, gid.GRIPPER_STATES_TOPIC, self.grippers_states_callback, 1, callback_group = self.callback_group)
        self.legs_states_subscriber = self.create_subscription(LegsStates, gid.LEGS_STATES_TOPIC, self.read_legs_states_callback, 1, callback_group = self.callback_group)

        print("All services available.")

        time.sleep(1)

def main():
    rclpy.init()
    app = App()
    executor = MultiThreadedExecutor()
    rclpy.spin(app, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()