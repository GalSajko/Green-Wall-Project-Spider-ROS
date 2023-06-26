import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import threading
import queue
import time

from std_msgs.msg import Float32MultiArray

from configuration import config, spider
from utils import custom_interface_helper
from calculations import kinematics as kin
from calculations import dynamics as dyn
from calculations import transformations as tf

from gwpspider_interfaces.msg import LegsStates
from gwpspider_interfaces.srv import MoveLeg, GetLegTrajectory, MoveSpider, ToggleController, DistributeForces, MoveGripper

class JointVelocityController(Node):
    def __init__(self):
        Node.__init__(self, 'joint_velocity_controller')

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

        self.command_queues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]
        self.sentinel = object()

        self.k_p = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * config.K_P
        self.k_d = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * config.K_D

        self.legs_states_locker = threading.Lock()
        self.legs_local_positions = None
        self.joints_positions = None
        self.legs_forces = None
        self.joints_torques = None

        self.legs_states_subscriber = self.create_subscription(LegsStates, 'legs_states', self.read_legs_states_callback, 1)

        self.toggle_controller_service = self.create_service(ToggleController, 'toggle_controller_service', self.toggle_controller_callback)

        self.moving_callbacks_group = ReentrantCallbackGroup()
        self.move_leg_service = self.create_service(MoveLeg, 'move_leg_service', self.move_leg_callback, callback_group = self.moving_callbacks_group)
        self.move_spider_service = self.create_service(MoveSpider, 'move_spider_service', self.move_spider_callback, callback_group = self.moving_callbacks_group)
        self.leg_trajectory_client = self.create_client(GetLegTrajectory, 'get_leg_trajectory', callback_group = self.moving_callbacks_group)
        self.move_gripper_client = self.create_client(MoveGripper, 'move_gripper_service', callback_group = self.moving_callbacks_group)

        self.controller_callbacks_group = ReentrantCallbackGroup()
        self.controller_publisher = self.create_publisher(Float32MultiArray, 'commanded_joints_velocities', 10, callback_group = self.controller_callbacks_group)
        self.timer = self.create_timer(self.PERIOD, self.controller_callback, callback_group = self.controller_callbacks_group)
        self.distribute_forces_service = self.create_service(DistributeForces, 'distribute_forces_service', self.distribute_forces_callback, callback_group = self.controller_callbacks_group)
    
    @property
    def PERIOD(self):
        return 1.0 / config.CONTROLLER_FREQUENCY
    
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

        if do_run:
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


            dx_c, self.last_legs_position_errors = self.__position_velocity_pd_controlloer(x_a, x_d, dx_d, ddx_d)
            dq_c = kin.get_joints_velocities(q_a, dx_c)
        
        else:
            dq_c = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG))

        msg = custom_interface_helper.create_multiple_2d_array_messages([dq_c])
        self.controller_publisher.publish(msg)

    def move_leg_callback(self, request, response):
        leg_id = request.leg
        duration = request.duration
        goal_position = request.goal_position.data
        trajectory_type = request.trajectory_type
        origin = request.origin
        is_offset = request.is_offset
        spider_pose = request.spider_pose.data
        use_gripper = request.use_gripper

        if leg_id not in spider.LEGS_IDS:
            response.success = False
            return response
        
        if origin not in (config.LEG_ORIGIN, config.GLOBAL_ORIGIN):
            print(f"Origin '{origin}' not recognized.")
            response.success = False
            return response
        
        if origin == config.GLOBAL_ORIGIN and len(spider_pose) == 0:
            print("If goal position is given in global origin, spider pose should also be given.")
            response.success = False
            return response

        with self.legs_states_locker:
            leg_current_position = self.legs_local_positions[leg_id]

        event = threading.Event()
        def done_callback(_):
            nonlocal event
            event.set()
        
        leg_goal_position_in_local = tf.convert_in_local_goal_positions(leg_id, leg_current_position, goal_position, origin, is_offset, spider_pose)

        get_trajectory_request = custom_interface_helper.prepare_trajectory_request((leg_current_position, leg_goal_position_in_local, duration, trajectory_type))

        future = self.leg_trajectory_client.call_async(get_trajectory_request)
        future.add_done_callback(done_callback)
        event.wait()
        trajectory_response = future.result()
        event.clear()

        position_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.position_trajectory)
        velocity_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.velocity_trajectory)
        acceleration_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.acceleration_trajectory)

        self.command_queues[leg_id] = queue.Queue()

        if use_gripper:
            get_move_gripper_request = custom_interface_helper.prepare_move_gripper_request((leg_id, config.OPEN_GRIPPER_COMMAND))
            future = self.move_gripper_client.call_async(get_move_gripper_request)
            future.add_done_callback(done_callback)
            event.wait()
            move_gripper_response = future.result()
            event.clear()

            if not move_gripper_response.success:
                print("Gripper did not open correctly.")
                response.success = False
                return response

        for idx, position in enumerate(position_trajectory):
            self.command_queues[leg_id].put([position[:3], velocity_trajectory[idx][:3], acceleration_trajectory[idx][:3]])
        self.command_queues[leg_id].put(self.sentinel)

        position_error = np.linalg.norm(leg_goal_position_in_local - self.legs_local_positions[leg_id])
        print(f"Position error before movement: {position_error}")
        while np.linalg.norm(position_error) > 0.001:
            position_error = np.linalg.norm(leg_goal_position_in_local - self.legs_local_positions[leg_id])
            print(f"Position error during movement: {position_error}")
            time.sleep(0.001)
        
        if use_gripper:
            get_move_gripper_request = custom_interface_helper.prepare_move_gripper_request((leg_id, config.CLOSE_GRIPPER_COMMAND))
            future = self.move_gripper_client.call_async(get_move_gripper_request)
            future.add_done_callback(done_callback)
            event.wait()
            move_gripper_response = future.result()
            event.clear()
            
            if not move_gripper_response.success:
                print("Gripper did not close correctly.")
                response.success = False
                return response
        
            response.success = True
            return response

    def move_spider_callback(self, request, response):
        legs_ids = request.legs.data
        duration = request.duration
        trajectory_type = request.trajectory_type
        origin = request.origin
        legs_goal_positions = custom_interface_helper.unpack_2d_array_message(request.goal_positions)
        spider_pose = request.spider_pose.data

        if origin not in (config.LEG_ORIGIN, config.GLOBAL_ORIGIN):
            print(f"Origin '{origin}' not recognized.")
            response.success = False
            return response
        
        if origin == config.GLOBAL_ORIGIN and len(spider_pose) == 0:
            print("If goal position is given in global origin, spider pose should also be given.")
            response.success = False
            return response
        
        if len(legs_ids) != len(legs_goal_positions):
            print("Number of moving legs and given goal positions should be the same.")
            response.success = False
            return response

        with self.legs_states_locker:
            legs_current_positions = self.legs_local_positions

        x_d = np.zeros((len(legs_ids), int(duration / self.PERIOD), 3), dtype = np.float32)
        dx_d = np.zeros((len(legs_ids), int(duration / self.PERIOD), 3), dtype = np.float32)
        ddx_d = np.zeros((len(legs_ids), int(duration / self.PERIOD), 3) , dtype = np.float32)

        for leg in legs_ids:
            if leg not in spider.LEGS_IDS:
                response.success = False
                return response  
            self.command_queues[leg] = queue.Queue()
        
        for idx, leg in enumerate(legs_ids):
            event = threading.Event()
            def done_callback(_):
                nonlocal event
                event.set()

            leg_goal_position_in_local = tf.convert_in_local_goal_positions(leg, legs_current_positions[leg], legs_goal_positions[idx], origin, False, spider_pose)
            get_trajectory_request = custom_interface_helper.prepare_trajectory_request((legs_current_positions[leg], leg_goal_position_in_local, duration, trajectory_type))

            future = self.leg_trajectory_client.call_async(get_trajectory_request)
            future.add_done_callback(done_callback)
            event.wait()
            trajectory_response = future.result()

            position_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.position_trajectory)
            velocity_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.velocity_trajectory)
            acceleration_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.acceleration_trajectory)

            x_d[idx] = position_trajectory[:, :3]
            dx_d[idx] = velocity_trajectory[:, :3]
            ddx_d[idx] = acceleration_trajectory[:, :3]

        for i in range(len(x_d[0])):
            for idx, leg in enumerate(legs_ids):
                self.command_queues[leg].put([x_d[idx][i], dx_d[idx][i], ddx_d[idx][i]])

        for leg in legs_ids:
            self.command_queues[leg].put(self.sentinel)
        
        response.success = True
        return response
 
    def read_legs_states_callback(self, msg):
        with self.legs_states_locker:
            self.joints_positions = np.reshape(msg.joints_positions.data, (msg.joints_positions.layout.dim[0].size, msg.joints_positions.layout.dim[1].size))
            self.legs_local_positions = np.reshape(msg.legs_local_positions.data, (msg.legs_local_positions.layout.dim[0].size, msg.legs_local_positions.layout.dim[1].size))
            self.legs_forces = np.reshape(msg.forces.data, (msg.forces.layout.dim[0].size, msg.forces.layout.dim[1].size))
            self.joints_torques = np.reshape(msg.torques.data, (msg.torques.layout.dim[0].size, msg.torques.layout.dim[1].size))
        
    def toggle_controller_callback(self, request, response):
        if request.command not in (config.START_CONTROLLER_COMMAND, config.STOP_CONTROLLER_COMMAND):
            response.success = False
            return response

        with self.toggle_controller_locker:
            self.do_run = request.command == config.START_CONTROLLER_COMMAND
        
        if request.command == config.STOP_CONTROLLER_COMMAND:
            self.command_queues = [queue.Queue() for _ in range(spider.NUMBER_OF_LEGS)]
        
        response.success = True
        return response
    
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
        while elapsed_time < config.FORCE_DISTRIBUTION_DURATION:
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

    def __position_velocity_pd_controlloer(self, x_a: np.ndarray, x_d: np.ndarray, dx_d: np.ndarray, ddx_d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
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
        dx_c = np.array(self.k_p * legs_position_errors + self.k_d * dx_e + dx_d + config.K_ACC * ddx_d, dtype = np.float32)

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
        legs_positions_in_spider = force_errors * config.K_P_FORCE
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

def main():
    rclpy.init()
    joint_velocity_controller = JointVelocityController()
    executor = MultiThreadedExecutor()
    rclpy.spin(joint_velocity_controller, executor)
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
