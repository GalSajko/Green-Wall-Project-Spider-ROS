import rclpy
from rclpy.node import Node

import dynamixel_sdk as dxl
import numpy as np
import os
import time

from std_msgs.msg import Float32MultiArray

from configuration import config, spider
from utils import mappers, custom_interface_helper

from gwpspider_interfaces.srv import ToggleMotorsTorque, SetBusWatchdog, RebootMotors
from gwpspider_interfaces.msg import DynamixelMotorsData

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

        self.port_handler = dxl.PortHandler(self.USB_DEVICE_NAME)
        self.packet_handler = dxl.PacketHandler(self.PROTOCOL_VERSION)

        self.group_sync_read_position = dxl.GroupSyncRead(self.port_handler, self.packet_handler, self.PRESENT_POSITION_ADDR, self.PRESENT_POSITION_DATA_LENGTH)
        self.group_sync_read_current = dxl.GroupSyncRead(self.port_handler, self.packet_handler, self.PRESENT_CURRENT_ADDR, self.PRESENT_CURRENT_DATA_LENGTH)
        self.group_sync_read_hardware_error = dxl.GroupSyncRead(self.port_handler, self.packet_handler, self.HARDWARE_ERROR_ADDR, self.HARDWARE_ERROR_DATA_LENGTH)
        self.group_sync_read_temperature = dxl.GroupSyncRead(self.port_handler, self.packet_handler, self.PRESENT_TEMPERATURE_ADDR, self.PRESENT_TEMPERATURE_DATA_LENGTH)

        self.group_sync_write_velocity = dxl.GroupSyncWrite(self.port_handler, self.packet_handler, self.GOAL_VELOCITY_ADDR, self.GOAL_VELOCITY_DATA_LENGTH)
        
        self.__init_group_read_write()
        self.__init_port()
        self.__toggle_motors_torque(spider.LEGS_IDS, config.ENABLE_LEGS_COMMAND)

        self.toggle_torque_service = self.create_service(ToggleMotorsTorque, 'toggle_motors_torque', self.toggle_motors_torque_callback)
        self.set_bus_watchdog_service = self.create_service(SetBusWatchdog, 'set_bus_watchdog', self.set_bus_watchdog_callback)
        self.reboot_motors_service = self.create_service(RebootMotors, 'reboot_motors', self.reboot_motors_callback)

        self.motors_data_publisher = self.create_publisher(DynamixelMotorsData, 'dynamixel_motors_data', 1)
        timer_period = 0.007
        self.timer = self.create_timer(timer_period, self.sync_read_motors_data_callback)

        self.joints_velocity_subscriber = self.create_subscription(Float32MultiArray, 'commanded_joints_velocities', self.sync_write_motors_velocities_callback, 10)
    
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

    #region callbacks
    def sync_write_motors_velocities_callback(self, msg):
        """Write commanded velocities to motors.
        """
        velocities = np.reshape(msg.data, (msg.layout.dim[0].size, msg.layout.dim[1].size))

        for leg in spider.LEGS_IDS:
            motors_in_leg = self.motors_ids[leg]
            encoder_velocities = mappers.map_model_velocities_to_velocity_encoder_values(velocities[leg]).astype(int)
            for i, motor in enumerate(motors_in_leg):
                commanded_joints_velocities_in_bytes = [
                    dxl.DXL_LOBYTE(dxl.DXL_LOWORD(encoder_velocities[i])),
                    dxl.DXL_HIBYTE(dxl.DXL_LOWORD(encoder_velocities[i])),
                    dxl.DXL_LOBYTE(dxl.DXL_HIWORD(encoder_velocities[i])),
                    dxl.DXL_HIBYTE(dxl.DXL_HIWORD(encoder_velocities[i])),
                ]
                success = self.group_sync_write_velocity.changeParam(motor, commanded_joints_velocities_in_bytes)
                if not success:
                    print(f"Failed changing Group Sync Writer parameter in motor {motor}")
        success = self.group_sync_write_velocity.txPacket()
        if success != dxl.COMM_SUCCESS:
            print("Failed to write velocities to motors.")
        
    def sync_read_motors_data_callback(self):
        """Read positions, currents, hardware errors and temperature registers from all connected motors.
        """
        _ = self.group_sync_read_current.fastSyncRead()
        _ = self.group_sync_read_position.fastSyncRead()
        _ = self.group_sync_read_hardware_error.fastSyncRead()
        _ = self.group_sync_read_temperature.fastSyncRead()

        currents = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        positions = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        hardware_errors = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
        temperatures = np.zeros((spider.NUMBER_OF_LEGS, spider.NUMBER_OF_MOTORS_IN_LEG), dtype = np.float32)
            
        for leg in spider.LEGS_IDS:
            for idx, motor_in_leg in enumerate(self.motors_ids[leg]):
                positions[leg][idx] = self.group_sync_read_position.getData(motor_in_leg, self.PRESENT_POSITION_ADDR, self.PRESENT_POSITION_DATA_LENGTH)
                currents[leg][idx] = self.group_sync_read_current.getData(motor_in_leg, self.PRESENT_CURRENT_ADDR, self.PRESENT_CURRENT_DATA_LENGTH)
                hardware_errors[leg][idx] = self.group_sync_read_hardware_error.getData(motor_in_leg, self.HARDWARE_ERROR_ADDR, self.HARDWARE_ERROR_DATA_LENGTH)
                temperatures[leg][idx] = self.group_sync_read_temperature.getData(motor_in_leg, self.PRESENT_TEMPERATURE_ADDR, self.PRESENT_TEMPERATURE_DATA_LENGTH)

        mapped_positions = mappers.map_position_encoder_values_to_model_angles_radians(positions)
        mapped_currents = mappers.map_current_encoder_values_to_motors_currents_ampers(currents)

        msg_list = custom_interface_helper.create_multiple_2d_array_messages((mapped_positions, mapped_currents, hardware_errors, temperatures))
        msg = DynamixelMotorsData(positions = msg_list[0], currents = msg_list[1], motor_errors = msg_list[2], temperatures = msg_list[3])

        self.motors_data_publisher.publish(msg)
    
    def toggle_motors_torque_callback(self, request, response):
        """Toggle torque in motors.
        """
        legs_ids = request.legs.data
        command = request.command

        response.success = self.__toggle_motors_torque(legs_ids, command)
        return response

    def set_bus_watchdog_callback(self, request, response):
        """Set bus watchdog in motors.
        """
        motors_array = self.motors_ids.flatten()
        for motor_id in motors_array:
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.BUS_WATCHDOG_ADDR, int(request.value))
            success = self.__comm_result_and_error_reader(result, error)
            if success:
                print(f"Watchdog on motor {motor_id} has been successfully set to {request.value}")
            else:
                response.success = False
                return response
        
        response.success = True
        return response
    
    def reboot_motors_callback(self, request, response):
        """Reboot motors.
        """
        motors_to_reboot = np.array(request.motors.data, dtype = np.int8)
        motors_to_reboot = motors_to_reboot.flatten()
        for motor in motors_to_reboot:
            if motor not in self.motors_ids:
                print(f"Motor with id {motor} does not exits - it cannot be rebooted.")
                response.success = False
                return response
            
            result, error = self.packet_handler.reboot(self.port_handler, int(motor))
            success = self.__comm_result_and_error_reader(result, error)
            if not success:
                print(f"Error while rebooting motor with id {motor}.")
                response.success = False
                return response
            
            if str(motor)[1] == '3':
                result, error = self.packet_handler.reboot(self.port_handler, int(motor + 1))
                success = self.__comm_result_and_error_reader(result, error)
                if not success:
                    print(f"Error while rebooting motor with id {motor + 1}.")
                    response.success = False
                    return response
        
        print(f"Motors with ids {motors_to_reboot} have been successfully rebooted.")
        response.success = True
        return response
    #endregion
    
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
        if result != dxl.COMM_SUCCESS:
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
    
    def __init_group_read_write(self) -> bool:
        """Add parameters for all motors into storage for group-sync reading and writing.

        Returns:
            bool: True if adding parameters was successful, False otherwise.
        """
        self.group_sync_read_current.clearParam()
        self.group_sync_read_position.clearParam()
        self.group_sync_read_hardware_error.clearParam()
        self.group_sync_read_temperature.clearParam()

        result_add_params = self.__add_group_sync_read_params()
        if not result_add_params:
            return False

        for leg in spider.LEGS_IDS:
            init_velocities = np.zeros(spider.NUMBER_OF_MOTORS_IN_LEG)
            encoder_velocities = mappers.map_model_velocities_to_velocity_encoder_values(init_velocities).astype(int)
            for i, motor in enumerate(self.motors_ids[leg]):
                init_velocity_in_bytes = [
                    dxl.DXL_LOBYTE(dxl.DXL_LOWORD(encoder_velocities[i])),
                    dxl.DXL_HIBYTE(dxl.DXL_LOWORD(encoder_velocities[i])),
                    dxl.DXL_LOBYTE(dxl.DXL_HIWORD(encoder_velocities[i])),
                    dxl.DXL_HIBYTE(dxl.DXL_HIWORD(encoder_velocities[i])),
                ]
                result_write = self.group_sync_write_velocity.addParam(motor, init_velocity_in_bytes)
                if not result_write:
                    print(f"Failed adding parameter {motor} to Group Sync Writer.")
                    return False

        return True
    
    def __add_group_sync_read_params(self) -> bool:
        """Add parameters (motors ids) to group sync reader parameters storages - for position and current reading.

        Returns:
            bool: True if adding was successfull, False otherwise.
        """
        for motor in self.motors_ids.flatten():
            result_position = self.group_sync_read_position.addParam(motor)
            result_current = self.group_sync_read_current.addParam(motor)
            result_hardware_error = self.group_sync_read_hardware_error.addParam(motor)
            result_temperature = self.group_sync_read_temperature.addParam(motor)
            if not (result_position and result_current and result_hardware_error and result_temperature):
                print(f"Failed adding parameter {motor} to Group Sync Reader.")
                return False
        return True
    
    def __toggle_motors_torque(self, legs_ids, command):
        for leg_id in legs_ids:
            if leg_id not in spider.LEGS_IDS:
                print(f"Leg with id {leg_id} does not exist.")
                return False
            
        motors_array = self.motors_ids[legs_ids].flatten()
        action = command == config.ENABLE_LEGS_COMMAND
        message = 'enabled' if action else 'disabled'

        for motor_id in motors_array:
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.TORQUE_ENABLE_ADDR, action)
            success = self.__comm_result_and_error_reader(result, error)
            if success:
                print(f"Motor {motor_id} has been successfully {message}.")
            else:
                return False
        
        return True

def main():
    rclpy.init()
    motor_driver = MotorDriver()
    rclpy.spin(motor_driver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()