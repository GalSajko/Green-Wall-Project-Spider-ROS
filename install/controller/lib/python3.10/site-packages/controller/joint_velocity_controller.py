import rclpy
from rclpy.node import Node

import numpy as np

from configuration import config, spider

class JointVelocityController(Node):
    def __init__(self):
        Node.__init__(self, 'joint_velocity_controller')

        self.last_legs_positions = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)
        self.last_legs_position_errors = np.zeros((spider.NUMBER_OF_LEGS, 3), dtype = np.float32)

        self.k_p = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * config.K_P
        self.k_d = np.ones((spider.NUMBER_OF_LEGS, 3), dtype = np.float32) * config.K_D
    
    @property
    def PERIOD(self):
        return 1.0 / config.CONTROLLER_FREQUENCY

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