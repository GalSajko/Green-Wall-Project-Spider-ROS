import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import threading

from calculations import kinematics as kin
from calculations import dynamics as dyn
from calculations import mathtools
from configuration import robot_config, ros_config, spider
from utils import custom_interface_helper

from gwpspider_interfaces.msg import DynamixelMotorsData, LegsStates, BnoData


class LegsStatesPublisher(Node):
    def __init__(self):
        Node.__init__(self, 'legs_states_publisher')

        self.callback_group = ReentrantCallbackGroup()

        self.motors_data_subscriber = self.create_subscription(DynamixelMotorsData, ros_config.DYNAMIXEL_MOTORS_DATA_TOPIC, self.calculate_legs_states_callback, 1, callback_group = self.callback_group)
        self.bno_data_subscriber = self.create_subscription(BnoData, ros_config.BNO_DATA_TOPIC, self.read_gravity_vector_callback, 1, callback_group = self.callback_group)

        self.graviy_vector_locker = threading.Lock()
        self.legs_states_msg_locker = threading.Lock()

        self.gravity_vector = None
        self.legs_states_msg = None

        self.force_buffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.tau_buffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.tau_counter = 0
        self.force_counter = 0

        self.legs_states_publisher = self.create_publisher(LegsStates, ros_config.LEGS_STATES_TOPIC, 1, callback_group = self.callback_group)
        self.timer = self.create_timer(0.005, self.publish_legs_states_callback, callback_group = self.callback_group)

    def publish_legs_states_callback(self):
        with self.legs_states_msg_locker:
            legs_states_msg = self.legs_states_msg

        if legs_states_msg is not None:
            self.legs_states_publisher.publish(self.legs_states_msg)

    def calculate_legs_states_callback(self, msg):
        q_a = np.reshape(msg.positions.data, (msg.positions.layout.dim[0].size, msg.positions.layout.dim[1].size))
        i_a = np.reshape(msg.currents.data, (msg.currents.layout.dim[0].size, msg.currents.layout.dim[1].size))

        with self.graviy_vector_locker:
            gravity_vector = self.gravity_vector

        if gravity_vector is not None:
            x_a = kin.all_legs_positions(q_a, robot_config.LEG_ORIGIN)
            tau, f = dyn.get_torques_and_forces_on_legs_tips(q_a, i_a, gravity_vector)   
            tau_mean, self.tau_buffer, self.tau_counter = mathtools.running_average(self.tau_buffer, self.tau_counter, tau)
            force_mean, self.force_buffer, self.force_counter = mathtools.running_average(self.force_buffer, self.force_counter, f)

            msg_list = custom_interface_helper.create_multiple_2d_array_messages((q_a, x_a, force_mean, tau_mean))

            with self.legs_states_msg_locker:
                self.legs_states_msg = LegsStates(joints_positions = msg_list[0], legs_local_positions = msg_list[1], forces = msg_list[2], torques = msg_list[3])

    def read_gravity_vector_callback(self, msg):
        with self.graviy_vector_locker:
            self.gravity_vector = np.array(msg.gravity_vector.data)

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    legs_states_publisher = LegsStatesPublisher()
    rclpy.spin(legs_states_publisher, executor)
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()