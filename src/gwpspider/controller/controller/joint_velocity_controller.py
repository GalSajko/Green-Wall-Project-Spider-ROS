import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import threading
import queue

from std_msgs.msg import Float32MultiArray

from configuration import config, spider
from utils import custom_interface_helper
from calculations import kinematics as kin

from gwpspider_interfaces.msg import LegsStates
from gwpspider_interfaces.srv import MoveLeg, GetLegTrajectory, MoveSpider

class JointVelocityController(Node):
    def __init__(self):
        Node.__init__(self, 'joint_velocity_controller')

        self.last_legs_positions = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.last_legs_position_errors = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.do_init = True
        self.legs_last_positons_locker = threading.Lock()

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

        self.callback_group = ReentrantCallbackGroup()
        self.move_leg_service = self.create_service(MoveLeg, 'move_leg_service', self.move_leg_callback, callback_group = self.callback_group)
        self.move_spider_service = self.create_service(MoveSpider, 'move_spider_service', self.move_spider_callback, callback_group = self.callback_group)
        self.leg_trajectory_client = self.create_client(GetLegTrajectory, 'get_leg_trajectory', callback_group = self.callback_group)

        self.controller_publisher = self.create_publisher(Float32MultiArray, 'commanded_joints_velocities', 10)
        self.timer = self.create_timer(self.PERIOD, self.controller_callback)
    
    @property
    def PERIOD(self):
        return 1.0 / config.CONTROLLER_FREQUENCY
    
    def controller_callback(self):
        x_d, dx_d, ddx_d = self.__get_pos_vel_acc_from_queues()
        with self.legs_states_locker:
            x_a = self.legs_local_positions
            q_a = self.joints_positions

        if self.do_init:
            with self.legs_last_positons_locker:
                self.last_legs_positions = x_a
            self.do_init = False

        dx_c, self.last_legs_position_errors = self.__ee_position_velocity_pd_controlloer(x_a, x_d, dx_d, ddx_d)
        dq_c = kin.get_joints_velocities(q_a, dx_c)

        msg = custom_interface_helper.create_multiple_2d_array_messages([dq_c])
        self.controller_publisher.publish(msg)

    def move_leg_callback(self, request, response):
        leg_id = request.leg
        duration = request.duration
        goal_position = request.goal_position.data
        trajectory_type = request.trajectory_type

        if leg_id not in spider.LEGS_IDS:
            response.success = False
            return response

        with self.legs_states_locker:
            leg_current_position = self.legs_local_positions[leg_id]

        event = threading.Event()
        def done_callback(_):
            nonlocal event
            event.set()
        
        get_trajectory_request = custom_interface_helper.prepare_trajectory_request((leg_current_position, goal_position, duration, trajectory_type))

        future = self.leg_trajectory_client.call_async(get_trajectory_request)
        future.add_done_callback(done_callback)
        event.wait()
        trajectory_response = future.result()

        position_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.position_trajectory)
        velocity_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.velocity_trajectory)
        acceleration_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.acceleration_trajectory)

        self.command_queues[leg_id] = queue.Queue()

        for idx, position in enumerate(position_trajectory):
            self.command_queues[leg_id].put([position[:3], velocity_trajectory[idx][:3], acceleration_trajectory[idx][:3]])
        self.command_queues[leg_id].put(self.sentinel)

        response.success = True
        return response

    def move_spider_callback(self, request, response):
        legs_ids = request.legs.data
        duration = request.duration
        trajectory_type = request.trajectory_type
        legs_goal_positions = custom_interface_helper.unpack_2d_array_message(request.goal_positions)
        spider_pose = request.spider_pose.data

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

            get_trajectory_request = custom_interface_helper.prepare_trajectory_request((legs_current_positions[idx], legs_goal_positions[idx], duration, trajectory_type))

            future = self.leg_trajectory_client.call_async(get_trajectory_request)
            future.add_done_callback(done_callback)
            event.wait()
            trajectory_response = future.result()

            position_trajectory = custom_interface_helper.unpack_2d_array_message(trajectory_response.trajectories.position_trajectory)
            print(position_trajectory)
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

    def __ee_position_velocity_pd_controlloer(self, x_a: np.ndarray, x_d: np.ndarray, dx_d: np.ndarray, ddx_d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
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
