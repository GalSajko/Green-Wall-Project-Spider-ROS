import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import threading
import queue
import time

from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger, SetBool

from configuration import robot_config, spider
from utils import custom_interface_helper, json_file_manager
from calculations import kinematics as kin
from calculations import dynamics as dyn
from calculations import transformations as tf

import gwpspider_interfaces.srv as gwp_services
from gwpspider_interfaces.msg import LegsStates, DynamixelMotorsData
from gwpspider_interfaces import gwp_interfaces_data as gid

class JointVelocityController(Node):
    """Velocity controller class.

    Args:
        Node (Node): ROS node.
    """
    def __init__(self):
        Node.__init__(self, 'joint_velocity_controller')

        self.use_prediction_model = True
        self.json_file_manager = json_file_manager.JsonFileManager()

        self.legs_last_positons_locker = threading.Lock()
        self.last_legs_positions = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.last_legs_position_errors = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)

        self.toggle_controller_locker = threading.Lock()
        self.do_init = True
        self.do_run = True

        self.toggle_movement_locker = threading.Lock()
        self.do_stop_movement = False

        self.force_mode_locker = threading.Lock()
        self.is_force_mode = False
        self.f_d = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.force_mode_legs_ids = None

        self.velocity_mode_locker = threading.Lock()
        self.is_velocity_mode = False
        self.velocity_mode_direction = None
        self.velocity_mode_legs_ids = None
        self.velocity_threshold_type = None

        self.command_queues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]
        self.sentinel = object()

        self.k_p = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * robot_config.K_P
        self.k_d = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * robot_config.K_D

        self.legs_states_locker = threading.Lock()
        self.legs_local_positions = None
        self.joints_positions = None
        self.legs_forces = None
        self.joints_torques = None

        self.dxl_data_locker = threading.Lock()
        self.currents = []
        
        self.callback_group = ReentrantCallbackGroup()

        self.__init_interfaces()

        # self.__set_bus_watchdog(30)

        self.get_logger().info("Controller is running.")
    
    @property
    def PERIOD(self) -> float:
        """Control loop period.

        Returns:
            float: Control loop period.
        """
        return 1.0 / robot_config.CONTROLLER_FREQUENCY
    
    @property
    def MAX_ALLOWED_FORCE(self) -> float:
        """Maximum allowed force, to be applied on legs' tips, during force-mode control.

        Returns:
            float: Maximal allowed force.
        """
        return 5.0

    @property
    def MAX_ALLOWED_CURRENT(self) -> float:
        """Maximum allowed current in motors, considered only, when controller is in velocity mode.

        Returns:
            float: Maximum allowed current in motors.
        """
        return 2.0
    
    @property
    def VELOCITY_AMP_FACTOR(self) -> float:
        """Amplification factor of legs' tips velocities, considered only when controller is in velocity mode.

        Returns:
            float: Amplification factor of legs' tips velocities.
        """
        return 0.1
    
    @property
    def MAX_VELOCITY_MODE_TIME(self) -> float:
        """Maximum time, that controller is allowed to run in velocity mode.

        Returns:
            float: Maximum time, that controller is allowed to run in velocity mode.
        """
        return 2.0
    
    @property
    def THIRD_JOINT_LOWER_LIMIT(self) -> float:
        """Lower limit of third joint, to avoid over-contraction (because of mechanical limitations).

        Returns:
            float: Lower limit of third joint.
        """
        return -2.30
    
    def controller_callback(self):
        """Callback for publishing on topic, defined with name gwp_interfaces_data.COMMANDED_JOINTS_VELOCITIES_TOPIC. This callback implements the entire control loop, for controlling 
        robot's joints' velocities. Output of controller are commanded joints' velocities, which are published on mentioned topic.
        """
        with self.legs_states_locker:
            x_a = self.legs_local_positions
            q_a = self.joints_positions
            f_a = self.legs_forces
        
        with self.toggle_controller_locker:
            do_run = self.do_run

        with self.force_mode_locker:
            is_force_mode = self.is_force_mode
            f_d = self.f_d
            force_mode_legs_ids = self.force_mode_legs_ids
        
        with self.velocity_mode_locker:
            is_velocity_mode = self.is_velocity_mode
            velocity_mode_direction = self.velocity_mode_direction
            velocity_mode_legs_ids = self.velocity_mode_legs_ids
            threshold_type = self.velocity_threshold_type
        
        with self.dxl_data_locker:
            currents = self.currents

        if do_run and x_a is not None:
            if self.do_init:
                with self.legs_last_positons_locker:
                    self.last_legs_positions = x_a
                self.do_init = False

            x_d, dx_d, ddx_d = self.__get_pos_vel_acc_from_queues()

            if is_force_mode:
                legs_position_offsets_in_spider, legs_velocities_in_spider = self.__force_position_p_controller(f_d, f_a)
                legs_position_offsets_in_local, legs_velocities_in_local = kin.get_xd_xdd_from_offsets(force_mode_legs_ids, legs_position_offsets_in_spider, legs_velocities_in_spider)

                new_x_d = x_d[force_mode_legs_ids] + legs_position_offsets_in_local
                new_x_d_norms = np.linalg.norm(new_x_d, axis = 1)
                allowed_legs_ids = np.where(new_x_d_norms < spider.LEG_LENGTH_MAX_LIMIT)
                on_limit_legs_ids = np.where(new_x_d_norms >= spider.LEG_LENGTH_MAX_LIMIT)

                x_d[force_mode_legs_ids[allowed_legs_ids]] = new_x_d[allowed_legs_ids]
                dx_d[force_mode_legs_ids[allowed_legs_ids]] = legs_velocities_in_local[allowed_legs_ids]
                x_d[force_mode_legs_ids[on_limit_legs_ids]] = x_a[force_mode_legs_ids[on_limit_legs_ids]]

                with self.legs_last_positons_locker:
                    self.last_legs_positions[force_mode_legs_ids] = x_a[force_mode_legs_ids]
            
            if is_velocity_mode:
                dx_d[velocity_mode_legs_ids] = self.VELOCITY_AMP_FACTOR * velocity_mode_direction

                if threshold_type == robot_config.FORCE_THRESHOLD_TYPE:
                    overloaded_legs = np.intersect1d(np.where(abs(f_a[:, 2]) > self.MAX_ALLOWED_FORCE), velocity_mode_legs_ids)

                elif threshold_type == robot_config.CURRENT_THRESHOLD_TYPE:
                    overloaded_legs = list(set(np.where(currents > self.MAX_ALLOWED_CURRENT)[0]))
  
                if len(overloaded_legs) != 0 and not np.any(q_a[overloaded_legs][:, 2] < self.THIRD_JOINT_LOWER_LIMIT):
                    dx_d[overloaded_legs] = np.zeros((len(overloaded_legs), 3))

                with self.legs_last_positons_locker:
                    self.last_legs_positions[velocity_mode_legs_ids] = x_a[velocity_mode_legs_ids] + dx_d[velocity_mode_legs_ids] * self.PERIOD

            dx_c, self.last_legs_position_errors = self.__position_velocity_pd_controller(x_a, x_d, dx_d, ddx_d)
            dq_c = kin.get_joints_velocities(q_a, dx_c)

            if is_force_mode:
                dq_c[dq_c > 1.0] = 1.0
                dq_c[dq_c < -1.0] = -1.0
        
        else:
            dq_c = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG))

        msg = custom_interface_helper.create_multiple_2d_array_messages([dq_c])
        self.controller_publisher.publish(msg)

    def move_leg_callback(self, request: gwp_services.MoveLeg.Request, response: gwp_services.MoveLeg.Response) -> gwp_services.MoveLeg.Response:
        """Service callback for executing the leg movement. Service type used for calling this service is MoveLeg (custom service type).

        Args:
            request (gwp_services.MoveLeg.Request): MoveLeg service request, which has to contain all the needed arguments for executing the movement. For all needed
            arguments look at service type declaration.
            response (gwp_services.MoveLeg.Response): MoveLeg service response, defined as boolean.

        Returns:
            gwp_services.MoveLeg.Response: True, if movement was executed successfully, False otherwise.
        """
        if not self.do_run:
            self.get_logger().info("Controller is not running.")
            response.success = False
            return response
        
        leg_id = request.leg_id
        duration = request.duration
        goal_position = request.goal_position.data
        trajectory_type = request.trajectory_type
        origin = request.origin
        is_offset = request.is_offset
        spider_pose = request.spider_pose.data
        open_gripper = request.open_gripper
        close_gripper = request.close_gripper
        use_prediction_model = request.use_prediction_model

        is_pin_to_pin_movement = open_gripper and close_gripper

        if leg_id not in spider.LEGS_IDS:
            response.success = False
            return response
        
        if origin not in (robot_config.LEG_ORIGIN, robot_config.GLOBAL_ORIGIN):
            self.get_logger().info(f"Origin '{origin}' not recognized.")
            response.success = False
            return response
        
        if origin == robot_config.GLOBAL_ORIGIN and len(spider_pose) == 0:
            self.get_logger().info("If goal position is given in global origin, spider pose should also be given.")
            response.success = False
            return response
        
        self.command_queues[leg_id] = queue.Queue()

        if is_pin_to_pin_movement:
            self.__apply_forces_on_leg_tips(leg_id, np.array([0.0, 0.0, -self.MAX_ALLOWED_FORCE]))
        if open_gripper:
            if not self.__move_gripper(leg_id, robot_config.OPEN_GRIPPER_COMMAND):
                response.success = False
                return response
            
        with self.legs_states_locker:
            legs_current_positions = self.legs_local_positions

        leg_goal_position_in_local = tf.convert_in_local_goal_positions(leg_id, legs_current_positions[leg_id], goal_position, origin, is_offset, spider_pose)

        if use_prediction_model:
            offset = self.__get_correction_offset(leg_id, legs_current_positions, leg_goal_position_in_local, spider_pose)
            leg_goal_position_in_local = leg_goal_position_in_local + offset

        position_trajectory, velocity_trajectory, acceleration_trajectory = self.__get_trajectory(
            legs_current_positions[leg_id],
            leg_goal_position_in_local,
            trajectory_type,
            duration,
        )

        with self.force_mode_locker:
            self.is_force_mode = False
        time.sleep(0.1)

        for idx, position in enumerate(position_trajectory):
            self.command_queues[leg_id].put([position[:3], velocity_trajectory[idx][:3], acceleration_trajectory[idx][:3]])
        self.command_queues[leg_id].put(self.sentinel)

        if not self.__wait_with_safety(duration + 0.5):
            response.success = False
            return response

        if close_gripper:
            force_to_apply = np.array([0.0, 0.0, -self.MAX_ALLOWED_FORCE])
            self.__apply_forces_on_leg_tips(leg_id, force_to_apply)

            if not self.__move_gripper(leg_id, robot_config.CLOSE_GRIPPER_COMMAND):
                with self.force_mode_locker:
                    self.is_force_mode = False
                response.success = False
                return response
            
        with self.force_mode_locker:
            self.is_force_mode = False
        time.sleep(0.1)
        
        response.success = True
        return response

    def move_spider_callback(self, request: gwp_services.MoveSpider.Request, response: gwp_services.MoveSpider.Response) -> gwp_services.MoveSpider.Response:
        """Service callback for executing the movement of spider's body. Service type used for calling this service is MoveSpider.

        Args:
            request (gwp_services.MoveSpider.Resquest): MoveSpider service request. For all needed arguments look at service type declaration.
            response (gwp_services.MoveSpider.Response): MoveSpider service response, defined as boolean.

        Returns:
            gwp_services.MoveSpider.Response: True, if movement was executed successfully, False otherwise.
        """
        if not self.do_run:
            self.get_logger().info("Controller is not running.")
            response.success = False
            return response
        
        legs_ids = request.legs_ids.data
        used_pins_positions = custom_interface_helper.unpack_2d_array_message(request.used_pins_positions)
        goal_spider_pose = request.goal_spider_pose.data
        duration = request.duration
        
        if len(legs_ids) != len(used_pins_positions):
            self.get_logger().info("Number of moving legs and given goal positions should be the same.")
            response.success = False
            return response

        with self.legs_states_locker:
            legs_current_positions = self.legs_local_positions

        x_d = np.zeros((len(legs_ids), int(duration / self.PERIOD), 3), dtype = np.float32)
        dx_d = np.zeros((len(legs_ids), int(duration / self.PERIOD), 3), dtype = np.float32)
        ddx_d = np.zeros((len(legs_ids), int(duration / self.PERIOD), 3) , dtype = np.float32)

        for leg_id in legs_ids:
            if leg_id not in spider.LEGS_IDS:
                response.success = False
                return response
            self.command_queues[leg_id] = queue.Queue()
        
        legs_goal_positions_in_local = np.zeros((len(legs_ids), 3))
        for idx, leg_id in enumerate(legs_ids):
            leg_goal_position_in_local = tf.convert_in_local_goal_positions(
                leg_id,
                legs_current_positions[leg_id],
                used_pins_positions[idx],
                robot_config.GLOBAL_ORIGIN,
                False,
                goal_spider_pose
            )
            legs_goal_positions_in_local[leg_id] = leg_goal_position_in_local

            position_trajectory, velocity_trajectory, acceleration_trajectory = self.__get_trajectory(
                legs_current_positions[leg_id],
                leg_goal_position_in_local,
                robot_config.MINJERK_TRAJECTORY,
                duration
            )

            x_d[idx] = position_trajectory[:, :3]
            dx_d[idx] = velocity_trajectory[:, :3]
            ddx_d[idx] = acceleration_trajectory[:, :3]

        for i in range(len(x_d[0])):
            for idx, leg_id in enumerate(legs_ids):
                self.command_queues[leg_id].put([x_d[idx][i], dx_d[idx][i], ddx_d[idx][i]])

        for leg_id in legs_ids:
            self.command_queues[leg_id].put(self.sentinel)

        if not self.__wait_with_safety(duration + 0.5):
            response.success = False
            return response
        
        response.success = True
        return response

    def read_dxl_data_callback(self, msg: DynamixelMotorsData):
        """Subscriber of topic with name, defined as gwp_interfaces_data.DYNAMIXEL_MOTORS_DATA_TOPIC. In this module, only data about currents in the motors is used.

        Args:
            msg (DynamixelMotorsData): Message type used for communicating in this topic (custom message type).
        """
        with self.dxl_data_locker:
            self.currents = custom_interface_helper.unpack_2d_array_message(msg.currents)

    def read_legs_states_callback(self, msg: LegsStates):
        """Subscriber of topic with name, defined as gwp_interfaces_data.LEGS_STATES_TOPIC.

        Args:
            msg (LegsStates): Message type, used for communicating in this topic (custom message type).
        """
        with self.legs_states_locker:
            self.joints_positions = np.reshape(msg.joints_positions.data, (msg.joints_positions.layout.dim[0].size, msg.joints_positions.layout.dim[1].size))
            self.legs_local_positions = np.reshape(msg.legs_local_positions.data, (msg.legs_local_positions.layout.dim[0].size, msg.legs_local_positions.layout.dim[1].size))
            self.legs_forces = np.reshape(msg.forces.data, (msg.forces.layout.dim[0].size, msg.forces.layout.dim[1].size))
            self.joints_torques = np.reshape(msg.torques.data, (msg.torques.layout.dim[0].size, msg.torques.layout.dim[1].size))
        
    def toggle_controller_callback(self, request: gwp_services.ToggleController.Request, response: gwp_services.ToggleController.Response) -> gwp_services.ToggleController.Response:
        """Service callback for toggling controller's state on and off. Service type, used for calling this service is ToggleController (custom service type).

        Args:
            request (gwp_services.ToggleController.Request): ToggleController service request, defined as string. Should contain valid command: robot_config.START_COMMAND, for 
            turning on the controller or robot_config.STOP_COMMAND for turning it off.
            response (gwp_services.ToggleController.Response): ToggleController service response, defined as boolean.

        Returns:
            gwp_services.ToggleController.Response: True, if toggling was successfull, False otherwise.
        """
        if request.command not in (robot_config.START_COMMAND, robot_config.STOP_COMMAND):
            response.success = False
            return response

        with self.toggle_controller_locker:
            self.do_run = request.command == robot_config.START_COMMAND
        
        if request.command == robot_config.STOP_COMMAND:
            self.command_queues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]
        
        response.success = True
        return response
    
    def toggle_additional_controller_mode_callback(
            self,
            request: gwp_services.ToggleAdditionalControllerMode.Request,
            response: gwp_services.ToggleAdditionalControllerMode.Response) -> gwp_services.ToggleAdditionalControllerMode.Response:
        """Service callback for toggling additional controller modes on or off. Additional controller modes are force and velocity mode. Service type, used for 
        calling this service is ToggleAdditionalControllerMode (custom service type).

        Args:
            request (gwp_services.ToggleAdditionalControllerMode.Request): ToggleAdditionalControllerMode request, defined as two strings. First is used to specify 
            controller mode (either robot_config.VELOCITY_MODE or robot_config.FORCE_MODE) and other for giving a command (either robot_config.START_COMMAND or robot_config.STOP_COMMAND). 
            response (gwp_services.ToggleAdditionalControllerMode.Response): ToggleAdditionalControllerMode response, defined as boolean.

        Returns:
            gwp_services.ToggleAdditionalControllerMode.Response: True if toggling was successfull, False otherwise.
        """
        if request.mode not in (robot_config.VELOCITY_MODE, robot_config.FORCE_MODE):
            self.get_logger().info(f"Controller mode '{request.mode}' not recognized.")
            response.success = False
            return response
        
        bool_command = request.command == robot_config.START_COMMAND

        if request.mode == robot_config.VELOCITY_MODE:
            with self.velocity_mode_locker:
                self.is_velocity_mode = bool_command
        
        else:
            with self.force_mode_locker:
                self.is_force_mode = bool_command
        
        response.success = True
        return response
    
    def distribute_forces_callback(
            self,
            request: gwp_services.DistributeForces.Request,
            response: gwp_services.DistributeForces.Response) -> gwp_services.DistributeForces.Response:
        """Service callback for optimally distributing forces among spider's legs or to offload selected leg by distributing forces among other four legs. 
        Service type used for calling this service is DistributeForces (custom service type).

        Args:
            request (gwp_services.DistributeForces.Request): DistributeForces service request, defined as Int8MultiArray. Should contain ids of legs, among which the forces will be distributed.
            response (gwp_services.DistributeForces.Response): DistributeForces service response, defined as boolean.

        Returns:
            gwp_services.DistributeForces.Response: True if distribution was successfull, False otherwise.
        """
        with self.force_mode_locker:
            self.force_mode_legs_ids = np.array(request.legs_ids.data, dtype = np.int8)
            self.is_force_mode = True

        offload_leg_id = np.setdiff1d(spider.LEGS_IDS, request.legs_ids.data)
        if len(offload_leg_id) > 1:
            response.success = False
            return response
        
        start_time = time.perf_counter()
        elapsed_time = 0
        while elapsed_time < robot_config.FORCE_DISTRIBUTION_DURATION:
            with self.legs_states_locker:
                actual_torques = self.joints_torques
                actual_angles = self.joints_positions
            
            distributed_forces = dyn.calculate_distributed_forces(actual_torques, actual_angles, request.legs_ids.data, offload_leg_id)
            
            if len(offload_leg_id):
                distributed_forces = np.insert(distributed_forces, offload_leg_id[0], np.zeros(3, dtype = np.float32), axis = 0)

            with self.force_mode_locker:
                self.f_d = distributed_forces.astype(np.float32)

            elapsed_time = time.perf_counter() - start_time
            time.sleep(self.PERIOD)
        
        with self.force_mode_locker:
            self.is_force_mode = False

        response.success = True
        return response
    
    def apply_forces_on_legs_callback(
            self,
            request: gwp_services.ApplyForcesOnLegs.Request,
            response: gwp_services.ApplyForcesOnLegs.Response) -> gwp_services.ApplyForcesOnLegs.Response:
        """Service callback used for applying desired forces on desired legs' tips. Service type used for calling this service is ApplyForcesOnLegs (custom service type).

        Args:
            request (gwp_services.ApplyForcesOnLegs.Request): ApplyForcesOnLegs service request, defined as Int8MultiArray which should contain ids of desired legs and 
            Float32MultiArray, which sould contain desired corresponding forces.
            response (gwp_services.ApplyForcesOnLegs.Response): ApplyForcesOnLegs service response, defined as boolean.

        Returns:
            gwp_services.ApplyForcesOnLegs.Response: True if forces were successfully applied, False otherwise.
        """
        legs_ids = np.array(request.legs_ids.data)
        for leg_id in legs_ids:
            if leg_id not in spider.LEGS_IDS:
                self.get_logger().info(f"Leg with ID {leg_id} was not recognized.")
                response.success = False
                return response
    
        desired_forces = custom_interface_helper.unpack_2d_array_message(request.desired_forces)
        self.__apply_forces_on_leg_tips(legs_ids, desired_forces)
        
        response.success = True
        return response
    
    def move_legs_velocity_mode_callback(
            self,
            request: gwp_services.MoveLegVelocityMode.Request,
            response: gwp_services.MoveLegVelocityMode.Response) -> gwp_services.MoveLegVelocityMode.Response:
        """Service callback for moving selected legs in velocity mode. Service type used for calling this service is MoveLegVelocityMode (custom service type).

        Args:
            request (gwp_services.MoveLegVelocityMode.Request): MoveLegVelocityMode service request, defined as Int8MultiArray for specifying desired legs ids, Float32MultiArray for
            specifying desired corresponding velocity directions and string for specifying desired threshold type 
            (when value exceeds the threshold, velocity drops to zero; either robot_config.FORCE_THRESHOLD_TYPE or robot_config.CURRENT_THRESHOLD_TYPE).
            response (gwp_services.MoveLegVelocityMode.Response): MoveLegVelocityMode response, defined as boolean.

        Returns:
            gwp_services.MoveLegVelocityMode.Response: True if movement was successfull, False otherwise.
        """
        legs_ids = request.legs_ids.data
        for leg_id in legs_ids:
            if leg_id not in spider.LEGS_IDS:
                self.get_logger().info(f"Leg with ID {leg_id} was not recognized.")
                response.success = False
                return response
        
        if request.threshold_type not in (robot_config.FORCE_THRESHOLD_TYPE, robot_config.CURRENT_THRESHOLD_TYPE):
            self.get_logger().info(f"Threshold type '{request.threshold_type}' was not recognized.")
            response.success = False
            return response
        
        with self.velocity_mode_locker:
            self.velocity_mode_legs_ids = legs_ids
            self.is_velocity_mode = True
            self.velocity_mode_direction = np.array(request.velocity_mode_direction.data)
            self.velocity_threshold_type = request.threshold_type

        success = self.__wait_with_safety(self.MAX_VELOCITY_MODE_TIME)

        with self.velocity_mode_locker:
            self.is_velocity_mode = False
            
        response.success = success
        return response
    
    def update_last_legs_positions_callback(self, _, response: Trigger.Response) -> Trigger.Response:
        """Service callback for updating last legs' positions. Service type used for calling this servie is Trigger.

        Args:
            _ (None): Not used.
            response (Trigger.Response): Trigger service response, defined as boolean.

        Returns:
            Trigger.Response: True, if updating was successfull, False otherwise.
        """
        with self.legs_states_locker:
            x_a = self.legs_local_positions
        
        if x_a is not None:
            self.command_queues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]
            with self.legs_last_positons_locker:
                self.last_legs_positions = x_a

            response.success = True
            return response
        
        response.success = False
        return response
    
    def toggle_legs_movement_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Service callback for imediate stop of all legs' movements. Service type used for calling this service is SetBool.

        Args:
            request (SetBool.Request): SetBool service request, defined as boolean. If True, legs' movements will be stopped.
            response (SetBool.Response): SetBool service response, defined as boolean.

        Returns:
            SetBool.Response: True if movement was successfully toggled, False otherwise.
        """
        with self.toggle_movement_locker:
            self.do_stop_movement = request.data
            response.success = self.do_stop_movement == request.data

        self.command_queues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]

        return response
    
    def __get_spider_pose(self, legs_ids: np.ndarray, legs_global_positions: np.ndarray) -> np.ndarray:
        """Wrapper method for calculating spider's current pose, given in global origin.

        Args:
            legs_ids (np.ndarray): Ids of legs, used in calculation.
            legs_global_positions (np.ndarray): Corresponding legs' global positions.

        Returns:
            np.ndarray: Spider's pose, given as position (x, y, z) and orientaion (roll, pitch, yaw) in global origin.
        """
        with self.legs_states_locker:
            q_a = self.joints_positions
        
        spider_pose = kin.get_spider_pose(legs_ids, legs_global_positions, q_a)

        return spider_pose

    def __position_velocity_pd_controller(self, x_a: np.ndarray, x_d: np.ndarray, dx_d: np.ndarray, ddx_d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """PD controller. Feed-forward velocity is used only in force mode, otherwise its values are zeros.

        Args:
            x_a (np.ndarray): 5x3 array of desired legs' positions.
            x_d (np.ndarray): 5x3 array of actual legs' positions.
            dx_d (np.ndarray): 5x3 array of feed-forward velocities.
            ddx_d (np.ndarray): 5x3 array of feed-forward accelerations.

        Returns:
            tuple[np.ndarray, np.ndarray]: Commanded velocities, current position errors.
        """
        legs_position_errors = np.array(x_d - x_a)
        dx_e = (legs_position_errors - self.last_legs_position_errors) / self.PERIOD
        dx_c = np.array(self.k_p * legs_position_errors + self.k_d * dx_e + dx_d + robot_config.K_ACC * ddx_d, dtype = np.float32)

        return dx_c, legs_position_errors

    def __force_position_p_controller(self, f_d: np.ndarray, f_a: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Force-position P controller. Calculate position offsets and desired legs velocities in spider's origin from desired forces.

        Args:
            f_d (np.ndarray): 5x3 array of desired forces in spider's origin.
            f_a (np.ndarray): 5x3 array of measured current forces in spider's origin.

        Returns:
            tuple[np.ndarray, np.ndarray]: Position offsets and velocities.
        """
        force_errors = f_d - f_a
        legs_positions_in_spider = force_errors * robot_config.K_P_FORCE
        offsets = legs_positions_in_spider * self.PERIOD

        return offsets, legs_positions_in_spider
    
    def __get_pos_vel_acc_from_queues(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Read current desired position, velocity and acceleration from queues for each leg. If leg-queue is empty, keep leg on latest position.

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray]: Desired positions, velocities and accelerations.
        """
        x_d = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        dx_d = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        ddx_d = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)

        for leg in spider.LEGS_IDS:
            try:
                queue_data = self.command_queues[leg].get(False)
                if queue_data is not self.sentinel:
                    with self.legs_last_positons_locker:
                        self.last_legs_positions[leg] = queue_data[0]
                if queue_data is self.sentinel:
                    with self.legs_last_positons_locker:
                        x_d[leg] = np.copy(self.last_legs_positions[leg])
                    dx_d[leg] = np.zeros(3, dtype = np.float32)
                    ddx_d[leg] = np.zeros(3, dtype = np.float32)
                else:
                    x_d[leg] = queue_data[0]
                    dx_d[leg] = queue_data[1]
                    ddx_d[leg] = queue_data[2]
            except queue.Empty:
                with self.legs_last_positons_locker:
                    x_d[leg] = np.copy(self.last_legs_positions[leg])
                dx_d[leg] = np.zeros(3, dtype = np.float32)
                ddx_d[leg] = np.zeros(3, dtype = np.float32)

        return x_d, dx_d, ddx_d

    def __get_trajectory(
            self,
            current_position_in_local: np.ndarray,
            goal_position_in_local: np.ndarray,
            trajectory_type: str,
            duration: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Wrapper method for calculating movement trajectories (position, velocity and acceleration).

        Args:
            current_position_in_local (np.ndarray): Leg's current position, given in local origin.
            goal_position_in_local (np.ndarray): Leg's goal position, given in local origin.
            trajectory_type (str): Type of trajectory (either robot_config.MINJERK_TRAJECTORY or robot_config.BEZIER_TRAJECTORY)
            duration (float): Desired duration of movement.

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray]: Tuple of arrays, containing position, velocity and acceleration trajectories.
        """
        get_trajectory_request = custom_interface_helper.prepare_trajectory_request((current_position_in_local, goal_position_in_local, duration, trajectory_type))
        trajectory_response = custom_interface_helper.async_service_call_from_service(self.leg_trajectory_client, get_trajectory_request)

        position_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.position_trajectory)
        velocity_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.velocity_trajectory)
        acceleration_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.acceleration_trajectory)

        return position_trajectory, velocity_trajectory, acceleration_trajectory
    
    def __move_gripper(self, leg_id: int, command: str) -> bool:
        """Wrapper method for moving (opening or closing) selected gripper.

        Args:
            leg_id (int): Leg's id.
            command (str): Desired command (either robot_config.OPEN_GRIPPER_COMMAND or robot_config.CLOSE_GRIPPER_COMMAND)

        Returns:
            bool: True if gripper's movement was successfull, False otherwise.
        """
        get_move_gripper_request = custom_interface_helper.prepare_move_gripper_request((leg_id, command))
        move_gripper_response = custom_interface_helper.async_service_call_from_service(self.move_gripper_client, get_move_gripper_request)

        if command == robot_config.OPEN_GRIPPER_COMMAND:
            message = 'open'
        else:
            message = 'close'

        if not move_gripper_response.success:
            self.get_logger().info(f"Gripper did not {message} correctly.")
        
        return move_gripper_response.success
    
    def __get_correction_offset(self, leg_id, current_positions, goal_position, spider_pose):
        """Not in use.
        """
        one_hot_legs = np.zeros(spider.NUMBER_OF_LEGS, dtype = np.int8)
        one_hot_legs[leg_id] = 1
        if not spider_pose:
            legs_global_positions = self.json_file_manager.read_spider_state()[2]
            spider_pose = self.__get_spider_pose(spider.LEGS_IDS, legs_global_positions)

        offset_request = custom_interface_helper.prepare_get_correction_offset_request((current_positions, spider_pose[3:], goal_position, one_hot_legs))
        offset_response = custom_interface_helper.async_service_call_from_service(self.get_correction_offset_client, offset_request)

        return np.array(offset_response.correction_offset.data, dtype = np.float32)
    
    def __apply_forces_on_leg_tips(self, legs_ids: np.ndarray, desired_forces: np.ndarray):
        """Wrapper method for applying desired forces on desired legs' tips.

        Args:
            legs_ids (np.ndarray): Ids of legs.
            desired_forces (np.ndarray): Desired corresponding forces.
        """
        try:
            _ = iter(legs_ids)
            legs_ids = np.array(legs_ids, dtype = np.int8)
        except TypeError:
            legs_ids = np.array([legs_ids], dtype = np.int8)
        with self.force_mode_locker:
            self.force_mode_legs_ids = legs_ids
            self.is_force_mode = True
            self.f_d[legs_ids] = np.array(desired_forces, dtype = np.float32)
    
    def __wait_with_safety(self, duration: float) -> bool:
        """Wait for desired duration and monitor safety trigger.

        Args:
            duration (float): Desired waiting duration.

        Returns:
            bool: True if waiting was not interrupted with safety trigger, False otherwise.
        """
        start_time = time.time()
        elapsed_time = 0
        while elapsed_time < duration:
            with self.toggle_movement_locker:
                if self.do_stop_movement:
                    self.command_queues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]
                    return False
            elapsed_time = time.time() - start_time
            time.sleep(0.01)
        return True

    def __set_bus_watchdog(self, value: int):
        """Not in use.
        """
        request = gwp_services.SetBusWatchdog.Request()
        request.value = value
        response = custom_interface_helper.async_service_call(self.set_bus_watchdog_service, request, self)
    
    def __init_interfaces(self):
        """Initialize all needed interfaces.
        """
        self.legs_states_subscriber = self.create_subscription(LegsStates, gid.LEGS_STATES_TOPIC, self.read_legs_states_callback, 1, callback_group = self.callback_group)

        self.test_dxl_data = self.create_subscription(DynamixelMotorsData, gid.DYNAMIXEL_MOTORS_DATA_TOPIC, self.read_dxl_data_callback, 1, callback_group = self.callback_group)

        self.toggle_controller_service = self.create_service(gwp_services.ToggleController, gid.TOGGLE_CONTROLLER_SERVICE, self.toggle_controller_callback)
        self.move_leg_service = self.create_service(gwp_services.MoveLeg, gid.MOVE_LEG_SERVICE, self.move_leg_callback, callback_group = self.callback_group)
        self.move_spider_service = self.create_service(gwp_services.MoveSpider, gid.MOVE_SPIDER_SERVICE, self.move_spider_callback, callback_group = self.callback_group)
        self.move_leg_velocity_mode_service = self.create_service(gwp_services.MoveLegVelocityMode, gid.MOVE_LEG_VELOCITY_MODE_SERVICE, self.move_legs_velocity_mode_callback, callback_group = self.callback_group)
        self.distribute_forces_service = self.create_service(gwp_services.DistributeForces, gid.DISTRIBUTE_FORCES_SERVICE, self.distribute_forces_callback, callback_group = self.callback_group)
        self.apply_force_on_leg_service = self.create_service(gwp_services.ApplyForcesOnLegs, gid.APPLY_FORCES_ON_LEGS_SERVICE, self.apply_forces_on_legs_callback, callback_group = self.callback_group)
        self.update_last_legs_positions_service = self.create_service(Trigger, gid.UPDATE_LAST_LEGS_POSITIONS_SERVICE, self.update_last_legs_positions_callback, callback_group = self.callback_group)
        self.toggle_additional_controller_mode_service = self.create_service(
            gwp_services.ToggleAdditionalControllerMode,
            gid.TOGGLE_ADDITIONAL_CONTROLLER_MODE_SERVICE,
            self.toggle_additional_controller_mode_callback,
            callback_group = self.callback_group
        )
        self.stop_legs_service = self.create_service(SetBool, gid.STOP_LEGS_SERVICE, self.toggle_legs_movement_callback, callback_group = self.callback_group)

        self.leg_trajectory_client = self.create_client(gwp_services.GetLegTrajectory, gid.GET_LEG_TRAJECTORY_SERVICE, callback_group = self.callback_group)
        while not self.leg_trajectory_client.wait_for_service(timeout_sec = 1.0):
            print("Trajectory service not available...")  
        self.move_gripper_client = self.create_client(gwp_services.MoveGripper, gid.MOVE_GRIPPER_SERVICE, callback_group = self.callback_group)
        while not self.move_gripper_client.wait_for_service(timeout_sec = 1.0):
            print("Gripper service not available...")          
        self.get_correction_offset_client = self.create_client(gwp_services.GetCorrectionOffset, gid.GET_CORRECTION_OFFSET_SERVICE, callback_group = self.callback_group)
        while not self.get_correction_offset_client.wait_for_service(timeout_sec = 1.0):
            print("Offset prediction model service not available...")  
        self.set_bus_watchdog_service = self.create_client(gwp_services.SetBusWatchdog, gid.SET_BUS_WATCHDOG_SERVICE, callback_group = self.callback_group)
        while not self.set_bus_watchdog_service.wait_for_service(timeout_sec = 1.0):
            print("Bus watchdog service not available...")  

        self.controller_publisher = self.create_publisher(Float32MultiArray, gid.COMMANDED_JOINTS_VELOCITIES_TOPIC, 10, callback_group = self.callback_group)
        self.timer = self.create_timer(self.PERIOD, self.controller_callback, callback_group = self.callback_group)

def main():
    """Main entry point.
    """
    rclpy.init()
    joint_velocity_controller = JointVelocityController()
    executor = MultiThreadedExecutor()
    rclpy.spin(joint_velocity_controller, executor)
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
