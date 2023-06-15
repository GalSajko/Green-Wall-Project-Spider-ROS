import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
import numpy as np
import os

from configuration import config, spider
from gwpspider_interfaces.srv import ToggleMotorsTorque


class MotorDriver(Node):
    def __init__(self):
        Node.__init__(self, 'dynamixel_driver')

        self.__set_usb_low_latency()

        self.motors_ids = np.array([
            [11, 12, 13],
            [21, 22, 23],
            [31, 32, 33],
            [41, 42, 43],
            [51, 52, 53]
        ], dtype = np.int8)

        self.port_handler = PortHandler(self.USB_DEVICE_NAME)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        self.__init_port()

        self.toggle_torque_service = self.create_service(ToggleMotorsTorque, 'toggle_motors_torque', self.toggle_motors_torque_callback)
    
    #region properties
    @property
    def MOTOR_SERIES(self):
        return 'X_SERIES'

    @property
    def TORQUE_ENABLE_ADDR(self):
        return 64
    
    @property
    def GOAL_VELOCITY_ADDR(self):
        return 104
    
    @property
    def PRESENT_POSITION_ADDR(self):
        return 132
    
    @property
    def PRESENT_CURRENT_ADDR(self):
        return 126
    
    @property
    def BUS_WATCHDOG_ADDR(self):
        return 98
    
    @property
    def HARDWARE_ERROR_ADDR(self):
        return 70
    
    @property
    def PRESENT_TEMPERATURE_ADDR(self):
        return 146
    
    @property
    def BAUDRATE(self):
        return 4000000
    
    @property
    def PROTOCOL_VERSION(self):
        return 2.0
    
    @property
    def USB_DEVICE_NAME(self):
        return '/dev/ttyUSB_DXL'
    
    @property
    def PRESENT_POSITION_DATA_LENGTH(self):
        return 4
    
    @property
    def PRESENT_CURRENT_DATA_LENGTH(self):
        return 2
    
    @property
    def GOAL_VELOCITY_DATA_LENGTH(self):
        return 4
    
    @property
    def HARDWARE_ERROR_DATA_LENGTH(self):
        return 1
    
    @property
    def PRESENT_TEMPERATURE_DATA_LENGTH(self):
        return 1
    
    @property
    def MAX_WORKING_TEMPERATURE(self):
        return 55
    #endregion

    def toggle_motors_torque_callback(self, request, response):
        legs_ids = request.legs.data
        for leg_id in legs_ids:
            if leg_id not in spider.LEGS_IDS:
                print(f"Leg with id {leg_id} does not exist.")
                response.success = False
                return response
            
        motors_array = self.motors_ids[request.legs.data].flatten()
        action = request.command == config.ENABLE_LEGS_COMMAND
        message = 'enabled' if action else 'disabled'

        for motor_id in motors_array:
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.TORQUE_ENABLE_ADDR, action)
            success = self.__comm_result_and_error_reader(result, error)
            if success:
                print(f"Motor {motor_id} has been successfully {message}.")
            else:
                response.success = False
                return response
        
        response.success = True
        return response

    def __set_usb_low_latency(self):
        """Set USB device latency on 1ms.
        """
        cmd = f'sudo setserial {self.USB_DEVICE_NAME} low_latency'
        os.system(cmd)


    def __comm_result_and_error_reader(self, result: str, error: str) -> bool:
        """Read communication result and error.

        Args:
            result (str): Result.
            error (str): Error.

        Returns:
            bool: True if communication was successfull, False otherwise.
        """
        if result != COMM_SUCCESS:
            print(f"{self.packet_handler.getTxRxResult(result)}")
            return False
        if error != 0:
            print(f"{self.packet_handler.getTxRxResult(error)}")
            return False

        return True
    
    def __init_port(self):
        """Initialize USB port and set baudrate. Note that baudrate should match with baudrate that is already set on motors.
        """
        if self.port_handler.openPort():
            print("Port successfully opened.")
        else:
            print("Failed to open the port.")

        if self.port_handler.setBaudRate(self.BAUDRATE):
            print("Baudrate successfully changed.")
        else:
            print("Failed to change the baudrate.")

def main():
    rclpy.init()
    motor_driver = MotorDriver()
    rclpy.spin(motor_driver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()