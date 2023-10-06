"""This module contains code for controlling a robot on highest level.
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import time
import threading

from gwpconfig import commconstants as cc

from configuration import spider, robot_config
from utils import json_file_manager
from utils import custom_interface_helper
from calculations import transformations as tf
from calculations import kinematics as kin
from calculations import mathtools as mt

import gwpspider_interfaces.srv as gwp_services
from gwpspider_interfaces.msg import GrippersStates, LegsStates
from gwpspider_interfaces import gwp_interfaces_data as gid

from std_msgs.msg import Float32MultiArray, Float32, Int16MultiArray
from std_srvs.srv import Empty, SetBool, Trigger

class App(Node):
    def __init__(self):
        Node.__init__(self, 'application')
        self.state = None

        self.is_working = False
        self.is_working_locker = threading.Lock()

        self.disable_motors_if_error = False

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
        self.is_battery_voltage_error = False
        self.battery_voltage_locker = threading.Lock()
        self.battery_voltage_error_locker = threading.Lock()

        self.battery_voltage_buffer = np.zeros(100, dtype = np.float32)
        self.battery_voltage_counter = 0

        self.callback_group = ReentrantCallbackGroup()

        self.__init_interfaces()
        
        self.get_logger().info("Application is running.")
    
    @property
    def LEG_MOVEMENT_DURATION_AMP(self):
        return 12.5

    @property
    def WORKING_VOLTAGE(self):
        return 16.0
    
    def states_manager_callback(self, request: gwp_services.SendStringCommand.Request, response: gwp_services.SendStringCommand.Response) -> gwp_services.SendStringCommand.Response:
        """States manager service callback for switching between two possible robot's states: either 'working' or 'transitioning into charging position'. 
        Service type used for calling this service is gwp_services.SendStringCommand (custom service type).

        Args:
            request (gwpspider_interfaces.srv.SendStringCommand.Request): String (constant) with desired command, either robot_config.WORKING_STATE or robot_config.TRANSITION_STATE. Other strings are invalid.
            response (gwpspider_interfaces.srv.SendStringCommand.Response): gwpspider_interfaces.srv.SendStringCommand service response, defined as boolean.

        Returns:
            gwpspider_interfaces.srv.SendStringCommand.Response: True if given command was successfully executed, False otherwise.
        """
        if request.command == robot_config.WORKING_STATE:
            self.get_logger().info("WORKING STARTED")
            success = self.working()

        elif request.command == robot_config.TRANSITION_STATE:
            self.get_logger().info("TRANSITIONING INTO CHARGING POSITION")
            success = self.transition_to_charging_position()
        
        response.success = success
        return response

    def working(self) -> bool:
        """Main method for working procedure.

        Returns:
            bool: Returns False if working is stopped due to any reason. True is never returned because in ideal case the working is never stopped.
        """
        self.__init_working_related_services()

        with self.is_working_locker:
            self.is_working = True

        self.is_init = True

        while True:
            poses, pins_instructions, plant_or_refill_position, watering_or_refill_leg_id, volume = self.__get_movement_instructions()
            for step, pose in enumerate(poses):
                current_pins_positions = pins_instructions[step, :, 1:]
                current_legs_moving_order = pins_instructions[step, :, 0].astype(int)

                if step == 0:
                    self.disable_motors_if_error = True
                    if not self.__move_spider(current_legs_moving_order, current_pins_positions, pose, 5.0):
                        self.get_logger().info("Working stopped due to error in spider movement.")
                        with self.is_working_locker:
                            self.is_working = False
                        return False
                    
                    with self.battery_voltage_error_locker:
                        if self.is_battery_voltage_error:
                            self.get_logger().info("Working stopped due to low battery voltage.")
                            with self.is_working_locker:
                                self.is_working = False
                            return False
                        
                    self.json_file_manager.update_whole_dict(pose, current_pins_positions, current_legs_moving_order)
                    self.__distribute_forces(spider.LEGS_IDS)
                    self.disable_motors_if_error = False
                    continue
                
                previous_pins_positions = np.array(pins_instructions[step - 1, :, 1:])
                if not self.__move_spider(current_legs_moving_order, previous_pins_positions, pose, 2.5):
                    self.get_logger().info("Working stopped due to error in spider movement.")
                    with self.is_working_locker:
                        self.is_working = False
                    return False
                
                with self.battery_voltage_error_locker:
                    if self.is_battery_voltage_error:
                        self.get_logger().info("Working stopped due to low battery voltage.")
                        with self.is_working_locker:
                            self.is_working = False
                        return False

                self.json_file_manager.update_whole_dict(pose, previous_pins_positions, current_legs_moving_order)

                previous_to_current_pins_offsets = current_pins_positions - previous_pins_positions
                for idx, leg_id in enumerate(current_legs_moving_order):
                    if previous_to_current_pins_offsets[idx].any():
                        if not self.__move_leg_to_next_pin(leg_id, previous_to_current_pins_offsets[idx], current_pins_positions[idx]):
                            self.get_logger().info("Working stopped due to error in leg movement.")
                            with self.is_working_locker:
                                self.is_working = False
                            return False
                        
                        with self.battery_voltage_error_locker:
                            if self.is_battery_voltage_error:
                                self.get_logger().info("Working stopped due to low battery voltage.")
                                with self.is_working_locker:
                                    self.is_working = False
                                return False
            
            if not self.__watering(watering_or_refill_leg_id, plant_or_refill_position, volume):
                self.get_logger().info("Working stopped due to error while watering.")
                with self.is_working_locker:
                    self.is_working = False
                return False
            
            with self.battery_voltage_error_locker:
                if self.is_battery_voltage_error:
                    self.get_logger().info("Working stopped due to low battery voltage.")
                    with self.is_working_locker:
                        self.is_working = False
                    return False
            
    def transition_to_charging_position(self) -> bool:
        """Logic for transitioning into charging position. Before transition begins, working (spider's movement) needs to be stopped. Transition is made of the following steps:

        - legs (one-by-one) are moved to correct pins for charging,
        - upper legs' breaks are activated,
        - controller is switched into force-mode and spider moves its body into resting pose,
        - motors are disabled,
        - robot waits until battery voltage exceeds predefined threshold (defined as App.WORKING_VOLTAGE),
        - motors are re-enabled,
        - breaks are released.

        After that, spider is ready to continue with its work. It waits for human operator to manually call states manager service with requst given as robot_config.WORKING_STATE.

        Returns:
            bool: True if transition is successfull (and battery is re-charged). False otherwise.
        """
        transitioning_message_request = gwp_services.Messages.Request(message = cc.TRANSITIONING_TO_RESTING_PHASE_MESSAGE  )
        _ = custom_interface_helper.async_service_call_from_service(self.messaging_client, transitioning_message_request)
        with self.is_working_locker:
            if self.is_working:
                return False
        
        start_legs_request = SetBool.Request(data = False)
        _ = custom_interface_helper.async_service_call_from_service(self.toggle_legs_movement_client, start_legs_request)

        if not self.__move_legs_and_body_to_charging_position():
            self.get_logger().info("Transition stopped due to hw error.")

        activate_breaks_request = gwp_services.BreaksControl.Request(command = robot_config.ACTIVATE_BREAKS_COMMAND, break_id = robot_config.ALL_BREAKS_INDEX)
        _ = custom_interface_helper.async_service_call_from_service(self.breaks_controller_client, activate_breaks_request)

        self.__go_to_resting_pose()

        disable_motors_request = custom_interface_helper.prepare_toggle_motors_torque_request((np.array([np.array([11, 12, 13]) + i * 10 for i in range(spider.NUMBER_OF_LEGS)]).flatten(), robot_config.DISABLE_LEGS_COMMAND))
        _ = custom_interface_helper.async_service_call_from_service(self.toggle_motors_torque_client, disable_motors_request)

        with self.battery_voltage_locker:
            battery_voltage = self.battery_voltage
        
        while battery_voltage < self.WORKING_VOLTAGE:
            with self.battery_voltage_locker:
                battery_voltage = self.battery_voltage
            time.sleep(5)
        time.sleep(5)

        toggle_additional_controller_mode_request = custom_interface_helper.prepare_toggle_controller_mode_request((robot_config.FORCE_MODE, robot_config.STOP_COMMAND))
        _ = custom_interface_helper.async_service_call_from_service(self.toggle_controller_mode_client, toggle_additional_controller_mode_request)

        self.get_logger().info("CHARGING FINISHED")
        with self.battery_voltage_error_locker:
            self.is_battery_voltage_error = False
        time.sleep(5)

        start_battery_voltage_monitoring_request = SetBool.Request(data = True)
        _ = custom_interface_helper.async_service_call_from_service(self.monitor_battery_voltage_client, start_battery_voltage_monitoring_request)

        enable_motors_request = custom_interface_helper.prepare_toggle_motors_torque_request((np.array([np.array([11, 12, 13]) + i * 10 for i in range(spider.NUMBER_OF_LEGS)]).flatten(), robot_config.ENABLE_LEGS_COMMAND))
        _ = custom_interface_helper.async_service_call_from_service(self.toggle_motors_torque_client, enable_motors_request)

        time.sleep(2)

        release_breaks_request = gwp_services.BreaksControl.Request(command = robot_config.RELEASE_BREAKS_COMMAND, break_id = robot_config.ALL_BREAKS_INDEX)
        _ = custom_interface_helper.async_service_call_from_service(self.breaks_controller_client, release_breaks_request)

        return True

    def immediate_stop_trigger_callback(self, _, response: Trigger.Response) -> Trigger.Response:
        """Service callback used when robot's movement (and working) needs to be stopped immediately. This stops movement of all legs, even if one (or more) of them are currently detached and causes the robot to go in idle state. 
        Service type used for calling this service is Trigger.

        Args:
            _ (None): Not used.
            response (Trigger.Response): Trigger service response, defined as boolean.

        Returns:
            Trigger.Response: True if working has been stopped successfully, False otherwise.
        """
        self.get_logger().info("STOPPING THE MOVEMENT DUE TO HW ERROR.")
        _ = self.__start_idle_state()

        with self.is_working_locker:
            is_working = self.is_working
        while is_working:
            with self.is_working_locker:
                is_working = self.is_working
            time.sleep(0.1)
        response.success = is_working is False

        return response
    
    def set_battery_voltage_error_flag_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Service callback that sets battery voltage error flag to True. Setting this flag to True stops the working procedure. This service is called from the safety layer, when battery voltage drops bellow certain threshold,
        defined in safety layer module. This call also triggers transition into charging position. Service type used for calling this service is SetBool. 

        Args:
            request (SetBool.Request): Desired boolean value to set battery voltage error flag on.
            response (SetBool.Response): SetBool service response, defined as boolean.

        Returns:
            SetBool.Response: True if transition into charging position was successfull, False otherwise.
        """
        self.get_logger().info("TRANSITION TO CHARGING POSITION TRIGGERED.")

        with self.battery_voltage_error_locker:
            self.is_battery_voltage_error = request.data
        
        with self.is_working_locker:
            is_working = self.is_working
        while is_working:
            with self.is_working_locker:
                is_working = self.is_working
            time.sleep(0.1)
        
        transition_success = self.transition_to_charging_position()

        response.success = transition_success
        return response

    def battery_voltage_callback(self, msg: Float32):
        """Subscriber of topic with name, defined as gwp_interfaces_data.BATTERY_VOLTAGE_TOPIC. Used message type is Float32. Topic is published by safety node and it messages current battery voltage. 
        Here, battery voltage is also smoothened used the running average algorithm, to avoid faulty readings or current spikes/drops caused by spider's movement.

        Args:
            msg (Float32): Message of type Float32, containing data with current battery voltage.
        """
        with self.battery_voltage_locker:
            battery_voltage = msg.data
            self.battery_voltage, self.battery_voltage_buffer, self.battery_voltage_counter = mt.running_average(self.battery_voltage_buffer, self.battery_voltage_counter, battery_voltage)
            
    def grippers_states_callback(self, msg: GrippersStates):
        """Subscriber of topic with name, defined as gwp_interfaces_data.GRIPPER_STATES_TOPIC. Used message type is GrippersStates (custom message). Topic is published by grippers_controller node and 
        it messages current states of grippers (gripper's state depends on states of fingers and microswitch). 

        Args:
            msg (GrippersStates): Message of type GrippersStates (custom message), containing informations about current grippers' states.
        """
        with self.grippers_states_locker:
            self.grippers_attached_states = [
                msg.first_gripper.is_attached,
                msg.second_gripper.is_attached,
                msg.third_gripper.is_attached,
                msg.fourth_gripper.is_attached,
                msg.fifth_gripper.is_attached
            ]
    
    def read_legs_states_callback(self, msg: LegsStates):
        """Subscriber of topic with name, defined as gwp_interfaces_data.LESG_STATES_TOPIC. Used message type is LegsStates (custom message). Topic is published by legs_states_publisher node and it messages current states
        of all legs (legs' local positions, angles in joints, torques in joints and forces on the tips of the legs). This subscbriber reads only legs' local positions and joints' positions, because other 
        informations about legs' states are not relevant for this module.

        Args:
            msg (LegsStates): Message of type LegsStates (custom message), containing informations about current legs' states.
        """
        positions = custom_interface_helper.unpack_2d_array_message(msg.legs_local_positions)
        joints_positions = custom_interface_helper.unpack_2d_array_message(msg.joints_positions)
        with self.legs_states_locker:
            self.legs_local_positions = positions
            self.joints_positions = joints_positions
    
    def spider_pose_publisher_callback(self):
        """Publisher of topic with name, defined as gwp_interfaces_data.SPIDER_POSE_TOPIC. Used message type if Float32MultiArray. Publisher is publishing current spider's pose, given in global origin. 
        Spider's pose is given as position (x, y, z) and orientation, given as roll, pitch and yaw angles.
        """
        spider_pose = self.__get_spider_pose()

        msg = Float32MultiArray(data = spider_pose)
        self.spider_pose_publisher.publish(msg)

    def __start_idle_state(self) -> bool:
        """Method which stops spider's movement. Additionally it also stops all water pumps and contracts tube holder on leg 2. After that it sends the spider into resting pose. If this method is called during
        spider is standing up, motors are disabled (instead of just stopped).

        Returns:
            bool: True if legs and water pumps were successfully stopped. Note: this method does not (yet) handle any errors that can occur during execution, so False value is never returned.
        """
        resting_message_request = gwp_services.Messages.Request(message = cc.RESTING_PHASE_STARTED_MESSAGE )
        _ = custom_interface_helper.async_service_call_from_service(self.messaging_client, resting_message_request)

        tube_holder_request = SetBool.Request(data = False)
        _ = custom_interface_helper.async_service_call_from_service(self.tube_holder_client, tube_holder_request)

        if self.disable_motors_if_error:
            self.get_logger().info("DISABLING MOTOROS DUE TO HW ERROR WHILE STANDING UP.")
            disable_motors_request = custom_interface_helper.prepare_toggle_motors_torque_request((np.array([np.array([11, 12, 13]) + i * 10 for i in range(spider.NUMBER_OF_LEGS)]).flatten(), robot_config.DISABLE_LEGS_COMMAND))
            _ = custom_interface_helper.async_service_call_from_service(self.toggle_motors_torque_client, disable_motors_request)

        self.get_logger().info("STOP LEGS")
        stop_legs_request = SetBool.Request(data = True)
        stop_legs_response = custom_interface_helper.async_service_call_from_service(self.toggle_legs_movement_client, stop_legs_request)
        self.get_logger().info("STOP PUMPS")
        stop_pump_request = Trigger.Request()
        stop_pump_response = custom_interface_helper.async_service_call_from_service(self.stop_water_pump_client, stop_pump_request)

        if not self.disable_motors_if_error:
            self.__go_to_resting_pose()
        
            toggle_additional_controller_mode_request = custom_interface_helper.prepare_toggle_controller_mode_request((robot_config.FORCE_MODE, robot_config.STOP_COMMAND))
            _ = custom_interface_helper.async_service_call_from_service(self.toggle_controller_mode_client, toggle_additional_controller_mode_request)

        return stop_legs_response.success and stop_pump_response.success
    
    def __get_movement_instructions(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, int, float]:
        """This method requests (with the service call) and unpacks received movement instructions. It is called at the beginning of each plant-to-plant movement (meaning, after plant was successfully watered, spider requests 
        instructions for the next watering). Received instructions are used to calculate all spider's poses on the path, all positions of pins that will be used, plant position, id of leg that will be used for watering 
        (or refilling spider's water tank) and the volume of water that will be used for watering the plant or (re)filled into the spider's water tank.

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray, int, float]: Tuple of arrays of spider's poses, pins' positions, watering (or refilling) position, watering (or refilling) leg id and volume of water (float).
        """
        spider_pose, _, start_legs_positions = self.json_file_manager.read_spider_state()

        spider_goal_request = gwp_services.SpiderGoal.Request()
        spider_goal_response = custom_interface_helper.async_service_call_from_service(self.get_spider_goal_client, spider_goal_request)
        watering_position = spider_goal_response.watering_position
        go_refill = spider_goal_response.go_refill
        volume = spider_goal_response.volume

        self.get_logger().info(f"GOAL INFO: {spider_goal_response}")

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
    
    def __move_spider(self, legs_ids: np.ndarray, legs_positions: np.ndarray, pose: np.ndarray, duration: float) -> bool:
        """Wrapper method for calling the service, intended to move the spider's body as a parallel robot movement.

        Args:
            legs_ids (np.ndarray): Ids of legs, that will be used in body movement.
            legs_positions (np.ndarray): Positions of used legs, given in global origin.
            pose (np.ndarray): Goal pose, given in global origin.
            duration (float): Desired duration of movement.

        Returns:
            bool: True if movement was successfull, False otherwise.
        """
        move_spider_request = custom_interface_helper.prepare_move_spider_request((legs_ids, legs_positions, pose, duration))
        move_spider_response = custom_interface_helper.async_service_call_from_service(self.move_spider_client, move_spider_request)

        return move_spider_response.success

    def __distribute_forces(self, legs_ids: np.ndarray) -> bool:
        """Wrapper method for calling the service, intended to distribute (or/and offload) forces on legs.

        Args:
            legs_ids (np.ndarray): Ids of legs, between which the forces will be distributed. If this is given as spider.LEGS_IDS (all legs included), then the force will be optimally distributed among 
            all five legs. If one of the legs' ids is excluded, then the force will be distributed among other four legs and force from that leg will be offloaded.

        Returns:
            bool: True if distribution was successfull, False otherwise.
        """
        distribute_forces_request = custom_interface_helper.prepare_distribute_forces_request(legs_ids)
        distribute_forces_response = custom_interface_helper.async_service_call_from_service(self.distribute_forces_client, distribute_forces_request)

        return distribute_forces_response.success

    def __get_spider_pose(self) -> np.ndarray:
        """This method first checks which legs are attached to the pins and then uses those legs to calculate current spider's pose.

        Returns:
            np.ndarray: Current spider's pose.
        """
        with self.grippers_states_locker:
            used_legs = [i for i, j in enumerate(self.grippers_attached_states) if j]

        with self.legs_states_locker:
            q_a = self.joints_positions
        
        _, _, legs_global_positions = self.json_file_manager.read_spider_state()
        spider_pose = kin.get_spider_pose(used_legs, legs_global_positions[used_legs], q_a)

        return spider_pose
    
    def __move_leg_to_next_pin(self, leg_id: int, pin_to_pin_vector_in_global: np.ndarray, goal_pin_position: np.ndarray, adjust_spider_pose: bool = False) -> bool:
        """Wrapper method for moving selected leg from on pin to another. Desired goal position is given as a position offset between current and goal position and not as a absolute position. 
        If leg misses the goal pin in first try, automatic correction procedure is executed.

        Args:
            leg_id (int): Leg id.
            pin_to_pin_vector_in_global (np.ndarray): Position offset, given as vector from current to the goal pin, given in global origin.
            goal_pin_position (np.ndarray): Position of goal pin, given in global origin.
            adjust_spider_pose (bool, optional): If True, the spider will adjust its pose before leg-movement. This is used when goal pin cannot be reached otherwise. Defaults to False.

        Returns:
            bool: True if leg-movement was successfull, False otherwise.
        """
        with self.grippers_states_locker:
            all_legs_attached = np.array(self.grippers_attached_states).all()
        if not all_legs_attached:
            self.get_logger().info("Not all legs are attached to the wall. Moving into resting pose.")
            self.__start_idle_state()
            return False
        
        self.__distribute_forces(np.delete(spider.LEGS_IDS, leg_id))

        if adjust_spider_pose:
            self.__adjust_spider_pose_before_leg_movement(leg_id, pin_to_pin_vector_in_global, goal_pin_position)
        
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
    
    def __move_legs_and_body_to_charging_position(self):
        """Method for moving legs on correct pins before charging starts. After legs, spider also moves its body into charging pose, by lowering it towards the wall. 
        By executing this procedure, legs are moved on such pins that three legs are connected on one and other two on opposite polarity. That ensures that battery will charge.
        """
        _, current_pins, current_positions = self.json_file_manager.read_spider_state()
        offsets, goal_positions = self.__get_charging_position_offsets(current_pins, current_positions)
        for leg in spider.LEGS_IDS:
            if (offsets[leg] == np.zeros(3, dtype = np.float32)).all():
                continue
            self.__move_leg_to_next_pin(leg, offsets[leg], goal_positions[leg], adjust_spider_pose = True)

        desired_pose = self.__get_spider_pose()
        desired_pose[2] = 0.15
        self.__move_spider(spider.LEGS_IDS, self.json_file_manager.read_spider_state()[2], desired_pose, 2.0)
    
    def __adjust_spider_pose_before_leg_movement(self, leg_id: int, pin_to_pin_vector_in_global: np.ndarray, goal_pin_position: np.ndarray):
        """This method moves spider's body (if neccessary) before leg-movement. By that it ensures that legs' goal position (pin) is in reach.

        Args:
            leg_id (int): Leg id.
            pin_to_pin_vector_in_global (np.ndarray): Position offset, given as vector from current to the goal pin, given in global origin.
            goal_pin_position (np.ndarray): Position of goal pin, given in global origin.
        """
        desired_spider_distance_from_pin = 0.6
        with self.legs_states_locker:
            current_local_position = self.legs_local_positions[leg_id]
        rpy = self.__get_spider_pose()[3:]
        pin_to_pin_vector_in_local, _ = tf.get_global_vector_in_local(leg_id, rpy, pin_to_pin_vector_in_global)
        goal_local_position = current_local_position + pin_to_pin_vector_in_local
        if np.linalg.norm(goal_local_position) > spider.LEG_LENGTH_MAX_LIMIT:
            current_spider_pose = self.__get_spider_pose()[:3]
            xy_direction_to_goal_pin = np.array(goal_pin_position - current_spider_pose)[:2]
            xy_direction_to_goal_pin = xy_direction_to_goal_pin / np.linalg.norm(xy_direction_to_goal_pin)
            adjusted_spider_pose = np.append(goal_pin_position[:2] - xy_direction_to_goal_pin * desired_spider_distance_from_pin, [spider.SPIDER_WALKING_HEIGHT, 0.0])
            _, _, legs_positions = self.json_file_manager.read_spider_state()
            self.__move_spider(spider.LEGS_IDS, legs_positions, adjusted_spider_pose, 3.0)

    def __get_charging_position_offsets(self, current_pins: np.ndarray, current_positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """This method calculates all needed position offsets for legs' movement, to move legs on charging positions.

        Args:
            current_pins (np.ndarray): Array of ids of currently used pins.
            current_positions (np.ndarray): Array of current legs' positions, given in global origin.

        Returns:
            tuple[np.ndarray, np.ndarray]: Tuple of positions given as offsets from current to goal pins and absolute goal positions, given in global origin.
        """
        offsets_request = gwp_services.GetOffsetsToChargingPosition.Request(current_pins = Int16MultiArray(data = current_pins))
        offsets_response = custom_interface_helper.async_service_call_from_service(self.get_offsets_to_charging_position_client, offsets_request)

        offsets = custom_interface_helper.unpack_2d_array_message(offsets_response.offsets)
        offsets = np.c_[offsets, np.zeros(len(offsets))]
        offsets = np.array(offsets, dtype = np.float32)

        goal_positions = np.round(current_positions + offsets, 2)

        return offsets, goal_positions
    
    def __go_to_resting_pose(self):
        """This method causes the spider to go into the resting pose. By using a force mode, spider lowers its body in the direction of gravity, in a controlled way. 
        If one of the leg is not attached, it moves away from the wall to avoid damaging the plants during lowering of the body.
        """
        with self.grippers_states_locker:
            attached_legs = np.where(self.grippers_attached_states)[0]
        if len(attached_legs) == spider.NUMBER_OF_LEGS - 1:
            detached_leg = np.delete(spider.LEGS_IDS, attached_legs)

            stop_legs_request = SetBool.Request(data = False)
            _ = custom_interface_helper.async_service_call_from_service(self.toggle_legs_movement_client, stop_legs_request)

            move_leg_request = custom_interface_helper.prepare_move_leg_request((
                detached_leg,
                # TODO: define this array as constant.
                np.array([0.3, 0.0, 0.15]),
                robot_config.MINJERK_TRAJECTORY,
                robot_config.LEG_ORIGIN,
                2.5,
                False,
                [],
                True,
                False,
                False
            ))
            _ = custom_interface_helper.async_service_call_from_service(self.move_leg_client, move_leg_request)
            
        forces = np.zeros((len(attached_legs), 3))
        # TODO: Define this multiplication value (-0.5) as constant.
        forces[:, 1] = np.ones(len(attached_legs)) * -0.5
        apply_forces_request = custom_interface_helper.prepare_apply_forces_on_legs_request((attached_legs, forces))
        _ = custom_interface_helper.async_service_call_from_service(self.apply_force_client, apply_forces_request)
        time.sleep(6)
    
    def __automatic_correction(self, leg_id: int, correction_direction: np.ndarray, goal_pin_position: np.ndarray, origin: str = robot_config.GLOBAL_ORIGIN) -> bool:
        """Method for automatic correction of leg's movement. It's called in case, that leg misses its goal (pin). Automatic correction is executed by trying and grabbing in close (predefined) vicinity of the goal pin.
        During those tries, leg is put into velocity mode, meaning that it is moving in a given direction with given velocity, until the certain force threshold is reached. After that the movement stops and gripper tries to
        grab a pin. If it is successfull, automatic correction procedure stops. Otherwise leg retracts and tries in next direction.

        Args:
            leg_id (int): Leg id.
            correction_direction (np.ndarray): Velocity direction. Should be the same as global z-axis.
            goal_pin_position (np.ndarray): Position of goal pin, given in global origin.
            origin (str, optional): Origin in which the leg's movement is executed. Defaults to robot_config.GLOBAL_ORIGIN.

        Returns:
            bool: True, if correction was successfull, False otherwise.
        """
        # TODO: Define this numbers as constants.
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

        spider_pose = self.__get_spider_pose()
        for _ in range(2):
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
            
    def __watering(self, leg_id: int, position: np.ndarray, volume: float) -> bool:
        """This method defines a watering procedure for watering the plant or refilling the spider's water tank. It moves the spider's leg, dedicated to watering or refilling on
        given position, activates (and de-activates) water pump, and moves leg back on initial position. Before leg's movement, forces are optimally distributed among other four legs.
        After watering of refilling is successfully finished, it informes the server about it. Server uses this information to start calculating next spider's goal.

        Args:
            leg_id (int): Leg id used for watering or refilling.
            position (np.ndarray): Position of watering or refilling, given in global origin.
            volume (float): Volume of water that will be used for watering or will be refilled in water tank.

        Returns:
            bool: True if watering or refilling was successfull, False otherwise.
        """
        _, _, legs_positions = self.json_file_manager.read_spider_state()
        current_leg_position = legs_positions[leg_id]
        offset_vector = position - current_leg_position
        rpy = self.__get_spider_pose()[3:]
        pin_to_plant_vector_in_local, _ = tf.get_global_vector_in_local(leg_id, rpy, offset_vector)

        self.__distribute_forces(np.delete(spider.LEGS_IDS, leg_id))
        with self.legs_states_locker:
            leg_local_position_before_watering = self.legs_local_positions[leg_id]

        move_leg_request = custom_interface_helper.prepare_move_leg_request((
            leg_id,
            pin_to_plant_vector_in_local,
            robot_config.BEZIER_TRAJECTORY,
            robot_config.LEG_ORIGIN,
            2.5,
            True,
            [],
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
        self.get_logger().info(f"{water_pump_response}")
        if not water_pump_response.success:
            return False

        watering_success_request = Empty.Request()
        _ = custom_interface_helper.async_service_call_from_service(self.set_watering_success_flag_client, watering_success_request)

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
    
    def __init_working_related_services(self):
        """Initialize all interfaces which are needed for working procedure to begin.
        """
        start_message_request = gwp_services.Messages.Request(message = cc.WORKING_PHASE_STARTED_MESSAGE)
        _ = custom_interface_helper.async_service_call_from_service(self.messaging_client, start_message_request)

        deactivate_breaks_request = gwp_services.BreaksControl.Request(command = robot_config.RELEASE_BREAKS_COMMAND, break_id = robot_config.ALL_BREAKS_INDEX)
        _ = custom_interface_helper.async_service_call_from_service(self.breaks_controller_client, deactivate_breaks_request)
        time.sleep(1.5)

        tube_holder_request = SetBool.Request(data = False)
        _ = custom_interface_helper.async_service_call_from_service(self.tube_holder_client, tube_holder_request)

        start_legs_request = SetBool.Request(data = False)
        _ = custom_interface_helper.async_service_call_from_service(self.toggle_legs_movement_client, start_legs_request)

        start_battery_voltage_monitoring_request = SetBool.Request(data = True)
        _ = custom_interface_helper.async_service_call_from_service(
            self.monitor_battery_voltage_client,
            start_battery_voltage_monitoring_request
        )

        start_hw_errors_monitoring_request = SetBool.Request(data = True)
        _ = custom_interface_helper.async_service_call_from_service(self.monitor_hw_errors_client, start_hw_errors_monitoring_request)

    def __init_interfaces(self):
        """Initialize all needed interfaces.
        """
        self.toggle_motors_torque_client = self.create_client(gwp_services.ToggleMotorsTorque, gid.TOGGLE_MOTORS_TORQUE_SERVICE, callback_group = self.callback_group)
        while not self.toggle_motors_torque_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Motors torque service not available...")

        self.get_walking_instructions_client = self.create_client(gwp_services.GetWalkingInstructions, gid.GET_WALKING_INSTRUCTIONS_SERVICE, callback_group = self.callback_group)
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
        
        self.toggle_legs_movement_client = self.create_client(SetBool, gid.STOP_LEGS_SERVICE, callback_group = self.callback_group)
        while not self.toggle_legs_movement_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Stop legs movement service not available...")

        self.move_leg_velocity_mode_client = self.create_client(gwp_services.MoveLegVelocityMode, gid.MOVE_LEG_VELOCITY_MODE_SERVICE, callback_group = self.callback_group)
        while not self.move_leg_velocity_mode_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Velocity mode leg moving service not available...")
        
        self.move_gripper_client = self.create_client(gwp_services.MoveGripper, gid.MOVE_GRIPPER_SERVICE, callback_group = self.callback_group)
        while not self.move_gripper_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Gripper moving service not available...")

        self.water_pump_client = self.create_client(gwp_services.ControlWaterPump, gid.WATER_PUMP_SERVICE, callback_group = self.callback_group)
        while not self.water_pump_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Water pump service not available...")
        
        self.tube_holder_client = self.create_client(SetBool, gid.TUBE_HOLDER_SERVICE, callback_group = self.callback_group)
        while not self.tube_holder_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Tube holder service not available...")

        self.stop_water_pump_client = self.create_client(Trigger, gid.STOP_WATER_PUMP_SERVICE, callback_group = self.callback_group)
        while not self.stop_water_pump_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Stop water pump service not available...")

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
        
        self.monitor_battery_voltage_client = self.create_client(SetBool, gid.TOGGLE_BATTERY_VOLTAGE_MONITORING_SERVICE, callback_group = self.callback_group)
        while not self.monitor_battery_voltage_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Toggle safety service not available...")

        self.monitor_hw_errors_client = self.create_client(SetBool, gid.TOGGLE_HW_ERRORS_MONITORING_SERVICE, callback_group = self.callback_group)
        while not self.monitor_hw_errors_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Toggle safety service not available...")

        self.messaging_client = self.create_client(gwp_services.Messages, gid.MESSAGE_SERVICE, callback_group = self.callback_group)
        while not self.messaging_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Messaging service not available...")
        
        self.states_manager_service = self.create_service(gwp_services.SendStringCommand, gid.STATES_MANAGER_SERVICE, callback = self.states_manager_callback, callback_group = self.callback_group)
        self.immediate_stop_trigger_service = self.create_service(Trigger, gid.IMMEDIATE_STOP_SERVICE, callback = self.immediate_stop_trigger_callback, callback_group = self.callback_group)
        self.battery_voltage_service = self.create_service(SetBool, gid.BATTERY_VOLTAGE_TRIGGER_SERVICE, callback = self.set_battery_voltage_error_flag_callback, callback_group = self.callback_group)

        self.grippers_states_subscriber = self.create_subscription(GrippersStates, gid.GRIPPER_STATES_TOPIC, self.grippers_states_callback, 1, callback_group = self.callback_group)
        self.legs_states_subscriber = self.create_subscription(LegsStates, gid.LEGS_STATES_TOPIC, self.read_legs_states_callback, 1, callback_group = self.callback_group)
        self.battery_voltage_subscriber = self.create_subscription(Float32, gid.BATTERY_VOLTAGE_TOPIC, self.battery_voltage_callback, 1, callback_group = self.callback_group)

        self.spider_pose_publisher = self.create_publisher(Float32MultiArray, gid.SPIDER_POSE_TOPIC, 1, callback_group = self.callback_group)
        self.spider_pose_publisher_timer = self.create_timer(5, self.spider_pose_publisher_callback, callback_group = self.callback_group)

        self.get_logger().info("All services available.")

        time.sleep(1)

def main():
    """Main entry point.
    """
    rclpy.init()
    app = App()
    executor = MultiThreadedExecutor()
    rclpy.spin(app, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()