import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import threading
import queue
import time

from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

from configuration import robot_config, spider
from utils import custom_interface_helper, json_file_manager
from calculations import kinematics as kin
from calculations import dynamics as dyn
from calculations import transformations as tf

from gwpspider_interfaces.msg import LegsStates
from gwpspider_interfaces.srv import MoveLeg, GetLegTrajectory, MoveSpider, ToggleController, DistributeForces, MoveGripper, ApplyForceLeg, GetSpiderPose, MoveLegVelocityMode, ToggleAdditionalControllerMode, GetCorrectionOffset, SetBusWatchdog
from gwpspider_interfaces import gwp_interfaces_data as gid

class JointVelocityController(Node):
    def __init__(self):
        Node.__init__(self, 'joint_velocity_controller')

        self.use_prediction_model = True
        self.json_file_manager = json_file_manager.JsonFileManager()

        self.legs_last_positons_locker = threading.Lock()
        self.last_legs_positions = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.last_legs_position_errors = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)

        self.do_init = True
        self.toggle_controller_locker = threading.Lock()
        self.do_run = True

        self.force_mode_locker = threading.Lock()
        self.is_force_mode = False
        self.f_d = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.force_mode_legs_ids = None

        self.velocity_mode_locker = threading.Lock()
        self.is_velocity_mode = False
        self.velocity_mode_direction = None
        self.velocity_mode_leg_id = None

        self.command_queues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]
        self.sentinel = object()

        self.k_p = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * robot_config.K_P
        self.k_d = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * robot_config.K_D

        self.legs_states_locker = threading.Lock()
        self.legs_local_positions = None
        self.joints_positions = None
        self.legs_forces = None
        self.joints_torques = None
        
        self.callback_group = ReentrantCallbackGroup()

        self.__init_interfaces()

        # self.__set_bus_watchdog(30)

        self.get_logger().info("Controller is running.")
    
    @property
    def PERIOD(self):
        return 1.0 / robot_config.CONTROLLER_FREQUENCY
    
    @property
    def LEG_MOVEMENT_PRECISION(self):
        return 0.001
    
    @property
    def SPIDER_MOVEMENT_PRECISION(self):
        return 0.005
    
    @property
    def MAX_ALLOWED_FORCE(self):
        return 5.5
    
    @property
    def VELOCITY_AMP_FACTOR(self):
        return 0.1
    
    @property
    def MAX_VELOCITY_MODE_TIME(self):
        return 2.0

    
    def controller_callback(self):
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
            velocity_mode_legs_ids = self.velocity_mode_leg_id

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
                dx_d[velocity_mode_legs_ids] = self.VELOCITY_AMP_FACTOR * velocity_mode_direction * int(np.linalg.norm(f_a[velocity_mode_legs_ids]) < self.MAX_ALLOWED_FORCE)
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

    def move_leg_callback(self, request, response):
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
            self.__apply_force_on_leg_tip(leg_id, np.array([0.0, 0.0, -self.MAX_ALLOWED_FORCE]))
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

        for idx, position in enumerate(position_trajectory):
            self.command_queues[leg_id].put([position[:3], velocity_trajectory[idx][:3], acceleration_trajectory[idx][:3]])
        self.command_queues[leg_id].put(self.sentinel)

        with self.legs_states_locker:
            position_error = np.linalg.norm(leg_goal_position_in_local - self.legs_local_positions[leg_id])
        while np.linalg.norm(position_error) > self.LEG_MOVEMENT_PRECISION:
            with self.legs_states_locker:
                position_error = np.linalg.norm(leg_goal_position_in_local - self.legs_local_positions[leg_id])
            time.sleep(0.001)

        if close_gripper:
            force_to_apply = np.array([0.0, 0.0, -self.MAX_ALLOWED_FORCE])
            self.__apply_force_on_leg_tip(leg_id, force_to_apply)
            # time.sleep(0.5)

            if not self.__move_gripper(leg_id, robot_config.CLOSE_GRIPPER_COMMAND):
                response.success = False
                return response
            
        with self.force_mode_locker:
            self.is_force_mode = False
        
        response.success = True
        return response

    def move_spider_callback(self, request, response):
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

        time.sleep(duration + 0.5)
        
        response.success = True
        self.get_logger().info("Spider has moved.")
        return response
 
    def read_legs_states_callback(self, msg):
        with self.legs_states_locker:
            self.joints_positions = np.reshape(msg.joints_positions.data, (msg.joints_positions.layout.dim[0].size, msg.joints_positions.layout.dim[1].size))
            self.legs_local_positions = np.reshape(msg.legs_local_positions.data, (msg.legs_local_positions.layout.dim[0].size, msg.legs_local_positions.layout.dim[1].size))
            self.legs_forces = np.reshape(msg.forces.data, (msg.forces.layout.dim[0].size, msg.forces.layout.dim[1].size))
            self.joints_torques = np.reshape(msg.torques.data, (msg.torques.layout.dim[0].size, msg.torques.layout.dim[1].size))
        
    def toggle_controller_callback(self, request, response):
        if request.command not in (robot_config.START_COMMAND, robot_config.STOP_COMMAND):
            response.success = False
            return response

        with self.toggle_controller_locker:
            self.do_run = request.command == robot_config.START_COMMAND
        
        if request.command == robot_config.STOP_COMMAND:
            self.command_queues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]
        
        response.success = True
        return response
    
    def toggle_additional_controller_mode_callback(self, request, response):
        if request.mode not in (robot_config.VELOCITY_MODE, robot_config.FORCE_MODE):
            self.get_logger().info(f"Controller mode {request.mode} not recognized.")
            response.success = False
            return response
        
        bool_command = request.command == robot_config.START_COMMAND

        if request.mode == robot_config.VELOCITY_MODE:
            with self.velocity_mode_locker:
                self.is_velocity_mode = bool_command
        
        else:
            with self.force_mode_locker:
                self.is_force_mode = bool_command
    
    def distribute_forces_callback(self, request, response):
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
    
    def apply_force_on_leg_callback(self, request, response):
        if request.leg_id not in spider.LEGS_IDS:
            self.get_logger().info(f"Leg with ID {request.leg_id} was not recognized.")
            response.success = False
            return response
    
        self.__apply_force_on_leg_tip(request.leg_id, request.desired_force.data)
        
        response.success = True
        return response
    
    def move_leg_velocity_mode_callback(self, request, response):
        if request.leg_id not in spider.LEGS_IDS:
            self.get_logger().info(f"Leg with ID {request.leg_id} was not recognized.")
            response.success = False
            return response
        
        with self.velocity_mode_locker:
            self.velocity_mode_leg_id = request.leg_id
            self.is_velocity_mode = True
            self.velocity_mode_direction = np.array(request.velocity_mode_direction.data)
        
        start_time = time.time()
        while True:
            with self.legs_states_locker:
                f_a = self.legs_forces
            elapsed_time = time.time() - start_time
            if np.linalg.norm(f_a[request.leg_id]) > self.MAX_ALLOWED_FORCE or elapsed_time > self.MAX_VELOCITY_MODE_TIME:
                break
            time.sleep(0.001)

        with self.velocity_mode_locker:
            self.is_velocity_mode = False
            
        response.success = True
        return response
    
    def update_last_legs_positions_callback(self, _, response):
        with self.legs_states_locker:
            x_a = self.legs_local_positions
        
        if x_a is not None:
            with self.legs_last_positons_locker:
                self.last_legs_positions = x_a

            response.success = True
            return response
        
        response.success = False
        return response
    
    def __get_spider_pose(self, legs_ids, legs_global_positions):
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

    def __get_trajectory(self, current_position_in_local, goal_position_in_local, trajectory_type, duration):
        get_trajectory_request = custom_interface_helper.prepare_trajectory_request((current_position_in_local, goal_position_in_local, duration, trajectory_type))
        trajectory_response = custom_interface_helper.async_service_call_from_service(self.leg_trajectory_client, get_trajectory_request)

        position_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.position_trajectory)
        velocity_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.velocity_trajectory)
        acceleration_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.acceleration_trajectory)

        return position_trajectory, velocity_trajectory, acceleration_trajectory
    
    def __move_gripper(self, leg_id, command):
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
        one_hot_legs = np.zeros(spider.NUMBER_OF_LEGS, dtype = np.int8)
        one_hot_legs[leg_id] = 1
        if not spider_pose:
            legs_global_positions = self.json_file_manager.read_spider_state()[2]
            spider_pose = self.__get_spider_pose(spider.LEGS_IDS, legs_global_positions)

        offset_request = custom_interface_helper.prepare_get_correction_offset_request((current_positions, spider_pose[3:], goal_position, one_hot_legs))
        offset_response = custom_interface_helper.async_service_call_from_service(self.get_correction_offset_client, offset_request)

        return np.array(offset_response.correction_offset.data, dtype = np.float32)
    
    def __apply_force_on_leg_tip(self, leg_id, desired_force):
        with self.force_mode_locker:
            self.force_mode_legs_ids = np.array([leg_id], dtype = np.int8)
            self.is_force_mode = True
            self.f_d[leg_id] = np.array(desired_force, dtype = np.float32)

    def __set_bus_watchdog(self, value):
        request = SetBusWatchdog.Request()
        request.value = value
        response = custom_interface_helper.async_service_call(self.set_bus_watchdog_service, request, self)
    
    def __init_interfaces(self):
        self.legs_states_subscriber = self.create_subscription(LegsStates, gid.LEGS_STATES_TOPIC, self.read_legs_states_callback, 1, callback_group = self.callback_group)

        self.toggle_controller_service = self.create_service(ToggleController, gid.TOGGLE_CONTROLLER_SERVICE, self.toggle_controller_callback)
        
        self.move_leg_service = self.create_service(MoveLeg, gid.MOVE_LEG_SERVICE, self.move_leg_callback, callback_group = self.callback_group)
        self.move_spider_service = self.create_service(MoveSpider, gid.MOVE_SPIDER_SERVICE, self.move_spider_callback, callback_group = self.callback_group)
        self.leg_trajectory_client = self.create_client(GetLegTrajectory, gid.GET_LEG_TRAJECTORY_SERVICE, callback_group = self.callback_group)
        self.move_gripper_client = self.create_client(MoveGripper, gid.MOVE_GRIPPER_SERVICE, callback_group = self.callback_group)
        self.move_leg_velocity_mode_service = self.create_service(MoveLegVelocityMode, gid.MOVE_LEG_VELOCITY_MODE_SERVICE, self.move_leg_velocity_mode_callback, callback_group = self.callback_group)
        self.get_correction_offset_client = self.create_client(GetCorrectionOffset, gid.GET_CORRECTION_OFFSET_SERVICE, callback_group = self.callback_group)

        self.controller_publisher = self.create_publisher(Float32MultiArray, gid.COMMANDED_JOINTS_VELOCITIES_TOPIC, 10, callback_group = self.callback_group)
        self.timer = self.create_timer(self.PERIOD, self.controller_callback, callback_group = self.callback_group)

        self.distribute_forces_service = self.create_service(DistributeForces, gid.DISTRIBUTE_FORCES_SERVICE, self.distribute_forces_callback, callback_group = self.callback_group)
        self.apply_force_on_leg_service = self.create_service(ApplyForceLeg, gid.APPLY_FORCE_ON_LEG_SERVICE, self.apply_force_on_leg_callback, callback_group = self.callback_group)
        self.update_last_legs_positions_service = self.create_service(Trigger, gid.UPDATE_LAST_LEGS_POSITIONS_SERVICE, self.update_last_legs_positions_callback, callback_group = self.callback_group)
        self.toggle_additional_controller_mode_service = self.create_service(
            ToggleAdditionalControllerMode,
            gid.TOGGLE_ADDITIONAL_CONTROLLER_MODE_SERVICE,
            self.toggle_additional_controller_mode_callback,
            callback_group = self.callback_group
        )
        self.set_bus_watchdog_service = self.create_client(SetBusWatchdog, gid.SET_BUS_WATCHDOG_SERVICE, callback_group = self.callback_group)

def main():
    rclpy.init()
    joint_velocity_controller = JointVelocityController()
    executor = MultiThreadedExecutor()
    rclpy.spin(joint_velocity_controller, executor)
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
