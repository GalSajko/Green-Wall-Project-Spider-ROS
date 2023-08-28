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
from calculations import transformations as tf
from calculations import kinematics as kin

import gwpspider_interfaces.srv as gwp_services
from gwpspider_interfaces.msg import GrippersStates, LegsStates
from gwpspider_interfaces import gwp_interfaces_data as gid

from std_msgs.msg import Float32MultiArray, Float32, Int8MultiArray
from std_srvs.srv import Empty, SetBool

class App(Node):
    def __init__(self):
        Node.__init__(self, 'application') 

        self.is_init = True
        self.use_prediction_model = False

        self.spider_pose = None

        self.json_file_manager = json_file_manager.JsonFileManager()
        self.grippers_attached_states = [False] * spider.NUMBER_OF_LEGS
        self.grippers_states_locker = threading.Lock()

        self.legs_local_positions = None
        self.joints_positions = None
        self.legs_states_locker = threading.Lock()

        self.battery_voltage = 0
        self.battery_voltage_locker = threading.Lock()

        self.callback_group = ReentrantCallbackGroup()

        self.__init_interfaces()
        
        self.get_logger().info("Application is running.")
        # self.working()
    
    @property
    def LEG_MOVEMENT_DURATION_AMP(self):
        return 12.5

    @property
    def WORKING_VOLTAGE(self):
        return 16.0
    
    def states_manager_callback(self, request, _):
        if request.message == robot_config.WORKING_STATE:
            self.get_logger().info("WORKING STARTED")
            self.working()
        elif request.message == robot_config.TRANSITION_STATE:
            self.get_logger().info("TRANSITIONING INTO CHARGING POSITION")
            self.transition_to_charging_position()

    def working(self):
        while True:
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
                if not self.__move_spider(current_legs_moving_order, previous_pins_positions, pose, 2.5):
                    self.get_logger().info("Working stopped due to error in spider movement.")
                    return False

                self.json_file_manager.update_whole_dict(pose, previous_pins_positions, current_legs_moving_order)

                previous_to_current_pins_offsets = current_pins_positions - previous_pins_positions
                for idx, leg_id in enumerate(current_legs_moving_order):
                    if previous_to_current_pins_offsets[idx].any():
                        if not self.__move_leg_to_next_pin(leg_id, previous_to_current_pins_offsets[idx], current_pins_positions[idx]):
                            self.get_logger().info("Working stopped due to error in leg movement.")
                            return False
            
            if not self.__watering(watering_or_refill_leg_id, plant_or_refill_position, poses[-1], volume):
                self.get_logger().info("Working stopped due to error while watering.")
                return False
            
    def transition_to_charging_position(self):
        _, current_pins, current_positions = self.json_file_manager.read_spider_state()
        self.get_logger().info(f"CURRENT PINS: {current_pins}")
        offsets_request = gwp_services.GetOffsetsToChargingPosition.Request(current_pins = Int8MultiArray(data = current_pins))
        offsets_response = custom_interface_helper.async_service_call_from_service(self.get_offsets_to_charging_position_client, offsets_request)

        offsets = custom_interface_helper.unpack_2d_array_message(offsets_response.offsets)
        offsets = np.c_[offsets, np.zeros(len(offsets))]
        offsets = np.array(offsets, dtype = np.float32)
        self.get_logger().info(f"OFFSETS: {offsets}")

        goal_positions = np.round(current_positions + offsets, 2)

        for leg in spider.LEGS_IDS:
            if (offsets[leg] == np.zeros(3, dtype = np.float32)).all():
                continue
            self.__move_leg_to_next_pin(leg, offsets[leg], goal_positions[leg])
        
        activate_breaks_request = gwp_services.BreaksControl.Request(command = robot_config.ACTIVATE_BREAKS_COMMAND, break_id = robot_config.ALL_BREAKS_INDEX)
        activate_breaks_response = custom_interface_helper.async_service_call_from_service(self.breaks_controller_client, activate_breaks_request)
        
        forces = np.zeros((spider.NUMBER_OF_LEGS, 3))
        forces[:, 1] = np.ones(spider.NUMBER_OF_LEGS) * 0.5
        apply_forces_request = custom_interface_helper.prepare_apply_forces_on_legs_request((spider.LEGS_IDS, forces))
        apply_forces_response = custom_interface_helper.async_service_call_from_service(self.apply_force_client, apply_forces_request)
        
        time.sleep(3)

        disable_motors_request = custom_interface_helper.prepare_toggle_motors_torque_request(([11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43, 51, 52, 53], robot_config.DISABLE_LEGS_COMMAND))
        disable_motors_response = custom_interface_helper.async_service_call_from_service(self.toggle_motors_torque_client, disable_motors_request)

        with self.battery_voltage_locker:
            battery_voltage = self.battery_voltage
        
        while not battery_voltage >= self.WORKING_VOLTAGE:
            with self.battery_voltage_locker:
                battery_voltage = self.battery_voltage
                time.sleep(5)

        toggle_additional_controller_mode_request = custom_interface_helper.prepare_toggle_controller_mode_request((robot_config.FORCE_MODE, robot_config.STOP_COMMAND))
        toggle_additional_controller_mode_response = custom_interface_helper.async_service_call_from_service(self.toggle_controller_mode_client, toggle_additional_controller_mode_request)

        toggle_safety_request = SetBool.Request(data = True)
        toggle_safety_response = custom_interface_helper.async_service_call_from_service(self.toggle_safety_client, toggle_safety_request)

        self.get_logger().info("CHARGING FINISHED")
        time.sleep(5)
        
        enable_motors_request = custom_interface_helper.prepare_toggle_motors_torque_request(([11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43, 51, 52, 53], robot_config.ENABLE_LEGS_COMMAND))
        enable_motors_response = custom_interface_helper.async_service_call_from_service(self.toggle_motors_torque_client, enable_motors_request)

        release_breaks_request = gwp_services.BreaksControl.Request(command = robot_config.RELEASE_BREAKS_COMMAND, break_id = robot_config.ALL_BREAKS_INDEX)
        release_breaks_response = custom_interface_helper.async_service_call_from_service(self.breaks_controller_client, release_breaks_request)
        
        self.working()
        

 
    def battery_voltage_callback(self, msg):
        with self.battery_voltage_locker:
            self.battery_voltage = msg.data
            
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
        joints_positions = custom_interface_helper.unpack_2d_array_message(msg.joints_positions)
        with self.legs_states_locker:
            self.legs_local_positions = positions
            self.joints_positions = joints_positions
    
    def spider_pose_publisher_callback(self):
        spider_pose = self.__get_spider_pose()

        msg = Float32MultiArray(data = spider_pose)
        self.spider_pose_publisher.publish(msg)
     
    def __get_movement_instructions(self):
        spider_pose, _, start_legs_positions = self.json_file_manager.read_spider_state()

        # spider_goal_request = gwp_services.SpiderGoal.Request()
        # spider_goal_response = custom_interface_helper.async_service_call(self.get_spider_goal_client, spider_goal_request, self)
        # watering_position = spider_goal_response.watering_position
        # go_refill = spider_goal_response.go_refill
        # volume = spider_goal_response.volume

        random_goals = np.array([
            [0.1, 0.74, 0.0],
            [0.1, 0.99, 0.0],
            [0.3, 0.74, 0.0],
            [0.3, 0.99, 0.0],
            [1.1, 0.74, 0.0],
            [1.1, 0.99, 0.0],
            [0.9, 0.74, 0.0],
            [0.9, 0.99, 0.0]
        ])
        watering_position = random_goals[np.random.randint(0, len(random_goals) - 1)]
        go_refill = False
        volume = 30

        # self.get_logger().info(f"GOAL INFO: {spider_goal_response}")
        self.get_logger().info(f"GOAL INFO: {watering_position}")

        watering_or_refill_leg_id, watering_or_refill_pose = tf.get_watering_leg_and_pose(spider_pose, watering_position, go_refill)

        if self.is_init:
            walking_instructions_request = custom_interface_helper.prepare_modified_walking_instructions_request((start_legs_positions, watering_or_refill_pose))
            walking_instructions_response = custom_interface_helper.async_service_call_from_service(self.get_modified_walking_instructions_client, walking_instructions_request)
            self.is_init = False
        else:
            walking_instructions_request = custom_interface_helper.prepare_walking_instructions_request((spider_pose, watering_or_refill_pose))
            walking_instructions_response = custom_interface_helper.async_service_call_from_service(self.get_walking_instructions_client, walking_instructions_request)

        poses = custom_interface_helper.unpack_2d_array_message(walking_instructions_response.walking_instructions.poses)
        pins_instructions = custom_interface_helper.unpack_3d_array_message(walking_instructions_response.walking_instructions.pins_instructions)

        if go_refill:
            watering_or_refill_position = watering_or_refill_pose[:3] + spider.REFILLING_LEG_OFFSET
            return poses, pins_instructions, watering_or_refill_position, watering_or_refill_leg_id, float(volume)

        return poses, pins_instructions, watering_position, watering_or_refill_leg_id, float(volume)
    
    def __move_spider(self, legs_ids, legs_positions, pose, duration):
        move_spider_request = custom_interface_helper.prepare_move_spider_request((legs_ids, legs_positions, pose, duration))
        move_spider_response = custom_interface_helper.async_service_call_from_service(self.move_spider_client, move_spider_request)

        return move_spider_response.success

    def __distribute_forces(self, legs_ids):
        distribute_forces_request = custom_interface_helper.prepare_distribute_forces_request(legs_ids)
        distribute_forces_response = custom_interface_helper.async_service_call_from_service(self.distribute_forces_client, distribute_forces_request)

        return distribute_forces_response.success

    def __get_spider_pose(self):
        with self.grippers_states_locker:
            used_legs = [i for i, j in enumerate(self.grippers_attached_states) if j]

        with self.legs_states_locker:
            q_a = self.joints_positions
        
        _, _, legs_global_positions = self.json_file_manager.read_spider_state()
        spider_pose = kin.get_spider_pose(used_legs, legs_global_positions[used_legs], q_a)

        return spider_pose
    
    def __move_leg_to_next_pin(self, leg_id, pin_to_pin_vector_in_global, goal_pin_position):
        self.__distribute_forces(np.delete(spider.LEGS_IDS, leg_id))

        rpy = self.__get_spider_pose()[3:]
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
        move_leg_response = custom_interface_helper.async_service_call_from_service(self.move_leg_client, move_leg_request)
        if not move_leg_response.success:
            return False

        with self.grippers_states_locker:
            is_attached = self.grippers_attached_states[leg_id]
        if not is_attached:
            global_z_direction_in_local = np.dot(leg_base_orientation_in_local, np.array([0.0, 0.0, 1.0], dtype = np.float32))
            correction_response = self.__automatic_correction(leg_id, global_z_direction_in_local, goal_pin_position)
            if not correction_response:
                return False
        
        return True
    
    def __automatic_correction(self, leg_id, correction_direction, goal_pin_position, origin = robot_config.GLOBAL_ORIGIN):
        detach_z_offset = 0.05
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

        spider_pose = self.__get_spider_pose()
        for offset in offsets:
            move_leg_request = custom_interface_helper.prepare_move_leg_request((
                leg_id,
                detach_position,
                robot_config.MINJERK_TRAJECTORY,
                origin,
                2.0,
                False,
                spider_pose,
                True,
                False,
                False
            ))
            move_leg_response = custom_interface_helper.async_service_call_from_service(self.move_leg_client, move_leg_request)
            if not move_leg_response.success:
                return False

            local_offset = tf.get_global_vector_in_local(leg_id, spider_pose[3:], offset)[0]
            velocity_direction = np.array([correction_direction[0] + local_offset[0], correction_direction[1] + local_offset[1], -correction_direction[2]], dtype = np.float32)
            move_leg_velocity_mode_request = custom_interface_helper.prepare_move_leg_velocity_mode_request(([leg_id], velocity_direction, robot_config.FORCE_THRESHOLD_TYPE))
            move_leg_velocity_mode_response = custom_interface_helper.async_service_call_from_service(self.move_leg_velocity_mode_client, move_leg_velocity_mode_request)
            if not move_leg_velocity_mode_response.success:
                return False

            move_gripper_request = custom_interface_helper.prepare_move_gripper_request((leg_id, robot_config.CLOSE_GRIPPER_COMMAND))
            _ = custom_interface_helper.async_service_call_from_service(self.move_gripper_client, move_gripper_request)
            
            with self.grippers_states_locker:
                if self.grippers_attached_states[leg_id]:
                    return True
            
        return False
            
    def __watering(self, leg_id, position, spider_pose, volume):
        self.__distribute_forces(np.delete(spider.LEGS_IDS, leg_id))
        with self.legs_states_locker:
            leg_local_position_before_watering = self.legs_local_positions[leg_id]

        move_leg_request = custom_interface_helper.prepare_move_leg_request((
            leg_id,
            position,
            robot_config.BEZIER_TRAJECTORY,
            robot_config.GLOBAL_ORIGIN,
            2.5,
            False,
            spider_pose,
            True,
            False,
            False
        ))
        move_leg_response = custom_interface_helper.async_service_call_from_service(self.move_leg_client, move_leg_request)
        if not move_leg_response.success:
            return False

        if leg_id not in (1, 2):
            pump_id = 0
        else:
            pump_id = leg_id

        water_pump_request = custom_interface_helper.prepare_water_pump_request((pump_id, volume))
        water_pump_response = custom_interface_helper.async_service_call_from_service(self.water_pump_client, water_pump_request)
        if not water_pump_response.success:
            return False

        watering_success_request = Empty.Request()
        watering_success_response = custom_interface_helper.async_service_call_from_service(self.set_watering_success_flag_client, watering_success_request)

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
        move_leg_response = custom_interface_helper.async_service_call_from_service(self.move_leg_client, move_leg_request)
        if not move_leg_response.success:
            return False

        with self.grippers_states_locker:
            is_attached = self.grippers_attached_states[leg_id]
        if not is_attached:
            rpy = self.__get_spider_pose()[3:]
            correction_direction = tf.get_global_vector_in_local(leg_id, rpy, np.array([0.0, 0.0, 1.0], dtype = np.float32))[0]
            correction_response = self.__automatic_correction(leg_id, correction_direction, leg_local_position_before_watering, robot_config.LEG_ORIGIN)
            if not correction_response:
                return False
        
        return True

    def __init_interfaces(self):
        self.toggle_motors_torque_client = self.create_client(gwp_services.ToggleMotorsTorque, gid.TOGGLE_MOTORS_TORQUE_SERVICE, callback_group = self.callback_group)
        while not self.toggle_motors_torque_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Motors torque service not available...")  

        self.get_walking_instructions_client = self.create_client(gwp_services.GetWalkingInstructions, gid.GET_WALKING_INSTRUCTIONS_SERVICE)
        while not self.get_walking_instructions_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Path planning service not available...")

        self.get_modified_walking_instructions_client = self.create_client(gwp_services.GetModifiedWalkingInstructions, gid.GET_MODIFIED_WALKING_INSTRUCTION_SERVICE, callback_group = self.callback_group)
        while not self.get_modified_walking_instructions_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Modified path planning service not available...")

        self.move_leg_client = self.create_client(gwp_services.MoveLeg, gid.MOVE_LEG_SERVICE, callback_group = self.callback_group)
        while not self.move_leg_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Leg moving service not available...")

        self.move_spider_client = self.create_client(gwp_services.MoveSpider, gid.MOVE_SPIDER_SERVICE, callback_group = self.callback_group)
        while not self.move_spider_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Spider moving service not available...")

        self.distribute_forces_client = self.create_client(gwp_services.DistributeForces, gid.DISTRIBUTE_FORCES_SERVICE, callback_group = self.callback_group)
        while not self.distribute_forces_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Distribute forces service not available...")

        self.apply_force_client = self.create_client(gwp_services.ApplyForcesOnLegs, gid.APPLY_FORCES_ON_LEGS_SERVICE, callback_group = self.callback_group)
        while not self.apply_force_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Apply forces on legs service not available...")

        self.toggle_controller_mode_client = self.create_client(gwp_services.ToggleAdditionalControllerMode, gid.TOGGLE_ADDITIONAL_CONTROLLER_MODE_SERVICE, callback_group = self.callback_group)
        while not self.toggle_controller_mode_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Toggle additional controller mode service not available...")

        self.move_leg_velocity_mode_client = self.create_client(gwp_services.MoveLegVelocityMode, gid.MOVE_LEG_VELOCITY_MODE_SERVICE, callback_group = self.callback_group)
        while not self.move_leg_velocity_mode_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Velocity mode leg moving service not available...")
        
        self.move_gripper_client = self.create_client(gwp_services.MoveGripper, gid.MOVE_GRIPPER_SERVICE, callback_group = self.callback_group)
        while not self.move_gripper_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Gripper moving service not available...")  

        self.water_pump_client = self.create_client(gwp_services.ControlWaterPump, gid.WATER_PUMP_SERVICE, callback_group = self.callback_group)
        while not self.water_pump_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Water pump service not available...")       

        self.breaks_controller_client = self.create_client(gwp_services.BreaksControl, gid.BREAKS_SERVICE, callback_group = self.callback_group)
        while not self.breaks_controller_client.wait_for_service(timeout_sec = 1.0):
            print("Breaks controller service not available...") 

        self.get_spider_goal_client = self.create_client(gwp_services.SpiderGoal, gid.SEND_GOAL_SERVICE, callback_group = self.callback_group)
        while not self.get_spider_goal_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Spider goal service not available...")  

        self.set_watering_success_flag_client = self.create_client(Empty, gid.SET_WATERING_SUCCESS_SERVICE, callback_group = self.callback_group)
        while not self.set_watering_success_flag_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Watering success flag service not available...")
        
        self.get_offsets_to_charging_position_client = self.create_client(gwp_services.GetOffsetsToChargingPosition, gid.GET_OFFSETS_TO_CHARGING_POSITION_SERVICE, callback_group = self.callback_group)
        while not self.get_offsets_to_charging_position_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Offsets to charging position service not available...")       
        
        self.toggle_safety_client = self.create_client(SetBool, gid.TOGGLE_SAFETY_SERVICE, callback_group = self.callback_group)
        while not self.toggle_safety_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Toggle safety service not available...") 
        
        self.states_manager_service = self.create_service(gwp_services.Messages, gid.STATES_MANAGER_SERVICE, callback = self.states_manager_callback)

        self.grippers_states_subscriber = self.create_subscription(GrippersStates, gid.GRIPPER_STATES_TOPIC, self.grippers_states_callback, 1, callback_group = self.callback_group)
        self.legs_states_subscriber = self.create_subscription(LegsStates, gid.LEGS_STATES_TOPIC, self.read_legs_states_callback, 1, callback_group = self.callback_group)
        self.battery_voltage_subscriber = self.create_subscription(Float32, gid.BATTERY_VOLTAGE_TOPIC, self.battery_voltage_callback, 1, callback_group = self.callback_group)

        self.spider_pose_publisher = self.create_publisher(Float32MultiArray, gid.SPIDER_POSE_TOPIC, 1, callback_group = self.callback_group)
        self.spider_pose_publisher_timer = self.create_timer(5, self.spider_pose_publisher_callback, callback_group = self.callback_group)

        self.get_logger().info("All services available.")

        time.sleep(1)

def main():
    rclpy.init()
    app = App()
    executor = MultiThreadedExecutor()
    rclpy.spin(app, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()