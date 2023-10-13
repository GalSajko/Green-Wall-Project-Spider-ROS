import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import threading

from calculations import kinematics as kin
from calculations import dynamics as dyn
from calculations import mathtools
from configuration import robot_config, spider
from utils import custom_interface_helper

from gwpspider_interfaces.msg import DynamixelMotorsData, LegsStates, BnoData
from gwpspider_interfaces import gwp_interfaces_data as gid

class LegsStatesPublisher(Node):
    """Class for calculating current legs' states from current motors' states and publishing calculated legs' states on gwp_interfaces_data.LEGS_STATES_TOPIC topic.

    Args:
        Node (Node): ROS node.
    """
    def __init__(self):
        Node.__init__(self, 'legs_states_publisher')

        self.graviy_vector_locker = threading.Lock()

        self.gravity_vector = None

        self.force_buffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.tau_buffer = np.zeros((10, spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.tau_counter = 0
        self.force_counter = 0

        self.callback_group = ReentrantCallbackGroup()
        self.motors_data_subscriber = self.create_subscription(DynamixelMotorsData, gid.DYNAMIXEL_MOTORS_DATA_TOPIC, self.calculate_legs_states_callback, 1, callback_group = self.callback_group)
        self.bno_data_subscriber = self.create_subscription(BnoData, gid.BNO_DATA_TOPIC, self.read_gravity_vector_callback, 1, callback_group = self.callback_group)
        self.legs_states_publisher = self.create_publisher(LegsStates, gid.LEGS_STATES_TOPIC, 1, callback_group = self.callback_group)

        self.get_logger().info("Legs states publisher is running.")

    def calculate_legs_states_callback(self, msg: DynamixelMotorsData):
        """Subscriber of topic with name, defined as gwp_intrfaces_data.DYNAMIXEL_MOTORS_DATA_TOPIC. In the same period, subscriber reads the data from the motors, use it 
        for calculate legs' states and publish legs' states on topic with name, defined as gwp_interfaces_data.LEGS_STATES_TOPIC.

        Args:
            msg (DynamixelMotorsData): Message type used for communication on topic with name, defined as gwp_intrfaces_data.DYNAMIXEL_MOTORS_DATA_TOPIC (custom message type).
        """
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

            legs_states_msg = LegsStates(joints_positions = msg_list[0], legs_local_positions = msg_list[1], forces = msg_list[2], torques = msg_list[3])
            
            self.legs_states_publisher.publish(legs_states_msg)

    def read_gravity_vector_callback(self, msg: BnoData):
        """Subscriber callback for reading gravity vector direction from topic with name, defined as gwp_interfaces_data.BNO_DATA_TOPIC.

        Args:
            msg (BnoData): Message type used for communication on topic with name, defined as gwp_interfaces_data.BNO_DATA_TOPIC (custom message type).
        """
        with self.graviy_vector_locker:
            self.gravity_vector = np.array(msg.gravity_vector.data)

def main():
    """Main entry point.
    """
    rclpy.init()
    executor = MultiThreadedExecutor()
    legs_states_publisher = LegsStatesPublisher()
    rclpy.spin(legs_states_publisher, executor)
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()