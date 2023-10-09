import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import time
import threading

from std_msgs.msg import Float32MultiArray, Float32
from std_srvs.srv import Trigger, SetBool

from arduino_communication.arduino_comm import ArduinoComm

from gwpspider_interfaces.srv import ControlWaterPump, InitBno, BreaksControl
from gwpspider_interfaces.msg import BnoData
from gwpspider_interfaces import gwp_interfaces_data as gid
from configuration import robot_config as rc

class WaterPumpBnoController(Node):
    """Class for controlling water pumps and reading data from IMO sensor, attached to Arduino.

    Args:
        Node (Node): ROS node.
    """
    def __init__(self):
        Node.__init__(self, 'water_pumps_bno_controller')

        self.reentrant_callback_group = ReentrantCallbackGroup()
        self.serial_comm_locker = threading.Lock()

        self.do_stop_water_pump = False
        self.stop_signal_locker = threading.Lock()

        self.__initialize_interfaces()
    
    @property
    def PUMPS_FLOWS(self) -> list:
        """Calculated flows of water pumps.

        Returns:
            list: Calculated flows of water pumps.
        """
        return [1.97, 2.05, 4.28]

    @property
    def RECEIVED_MESSAGE_LENGTH(self) -> int:
        """Length of messages, that are sent from Arduino.

        Returns:
            int: Length of messages, that are sent from Arduino.
        """
        return 23
    
    @property
    def DEVICE_NAME(self) -> str:
        """Name of Arduino device.

        Returns:
            str: Name of Arduino device.
        """
        return 'ttyUSB_BNOWP'
    
    @property
    def PUMP_OFF_COMMAND(self) -> str:
        """Command for turning off the water pump.

        Returns:
            str: Command for turning off the water pump.
        """
        return '0'
    
    @property
    def PUMP_ON_COMMAND(self) -> str:
        """Command for turning on the water pump.

        Returns:
            str: Command for turning on the water pump.
        """
        return '1'
    
    @property
    def BREAKS_OFF_COMMAND(self) -> str:
        """Command for releasing the breaks.

        Returns:
            str: Command for releasing the breaks.
        """
        return '4'

    @property
    def BREAKS_ON_COMMAND(self) -> str:
        """Command for activating the breaks.

        Returns:
            str: Command for activating the breaks.
        """
        return '5'
    
    def water_pump_control_callback(self, request: ControlWaterPump.Request, response: ControlWaterPump.Response) -> ControlWaterPump.Response:
        """Service callback for controlling water pumps, by sending correct command to Arduino. Command consists of pump's id and volume of water
        to be pumped. Service type used for calling this service is ControlWaterPump (custom service type).

        Args:
            request (ControlWaterPump.Request): Int64 for pump id and float64 for volume.
            response (ControlWaterPump.Response): ControlWaterPump service response, defined as boolean.

        Returns:
            ControlWaterPump.Response: True, if selected water pump successfully pumped the given amount of water, False otherwise.
        """
        pump_id = request.pump
        volume = request.volume

        if pump_id == 2:
            self.__send_message_to_arduino(rc.EXPAND_TUBE_COMMAND)

        flow = self.PUMPS_FLOWS[pump_id]
        watering_time = volume / flow

        with self.stop_signal_locker:
            self.do_stop_water_pump = False

        self.__send_message_to_arduino(self.PUMP_ON_COMMAND + str(pump_id))
        self.get_logger().info("START WATER PUMP SIGNAL SENT")

        total_time = watering_time
        if pump_id == 2:
            total_time += 4.0
        check_for_contraction = True

        start_time = time.time()
        elapsed_time = 0.0
        while elapsed_time < total_time:

            if check_for_contraction:
                if pump_id == 2 and elapsed_time > watering_time:
                    self.__send_message_to_arduino(rc.CONTRACT_TUBE_COMMAND)
                    check_for_contraction = False

            with self.stop_signal_locker:
                if self.do_stop_water_pump:
                    self.__send_message_to_arduino(self.PUMP_OFF_COMMAND + str(pump_id))
                    response.success = False
                    return response
                
            elapsed_time = time.time() - start_time
            time.sleep(0.01)

        self.__send_message_to_arduino(self.PUMP_OFF_COMMAND + str(pump_id))
        self.get_logger().info("STOP WATER PUMP SIGNAL SENT.")

        response.success = True
        return response
    
    def stop_pump_callback(self, _, response: Trigger.Response) -> Trigger.Response:
        """Service callback for imediate stop of currently working water pump. Service type used for calling this service is Trigger.

        Args:
            _ (None): Not used.
            response (Trigger.Response): Trigger.Response, defined as boolean.
        Returns:
            Trigger.Response: Always returns True.
        """
        with self.stop_signal_locker:
            self.do_stop_water_pump = True
        
        response.success = True
        return response
    
    def tube_holder_control_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Service callback for controlling a tube holder on third leg (used for refilling). Tube holder will expand if given
        request.data is True and contract otherwise. Service type used for calling this service is SetBool.

        Args:
            request (SetBool.Request): SetBool service request, defined as boolean.
            response (SetBool.Response): SetBool service response, defined as boolean.

        Returns:
            SetBool.Response: Always returns True.
        """
        if request.data:
            command = rc.EXPAND_TUBE_COMMAND
        else:
            command = rc.CONTRACT_TUBE_COMMAND

        self.__send_message_to_arduino(command)
        
        response.success = True
        return response
    
    def breaks_control_callback(self, request: BreaksControl.Request, response: BreaksControl.Response) -> BreaksControl.Response:
        """Service callback for controlling a breaks in third joints. Breaks are controlled by providing string command and id of break.
        Service type used for calling this service is BreaksControl (custom service type).

        Args:
            request (BreaksControl.Request): BreaksControl request defined as string (must be either water_pumps_bno.BREAKS_OFF_COMMAND or water_pumps_bno.BREAKS_ON_COMMAND)
            and id of break (0 - 4), given as integer. If id is given as integer 5, then all five breaks command will be applied to all five breaks.
            response (BreaksControl.Response): BreaksControl response, defined as boolean.

        Returns:
            BreaksControl.Response: True, if break(s) executed the given command successfully, False otherwise.
        """
        break_id = request.break_id
        command = request.command

        if command not in (self.BREAKS_OFF_COMMAND, self.BREAKS_ON_COMMAND):
            self.get_logger().info(f"Command {command} not recognized.")
            response.success = False
            return response
        
        if not (0 <= break_id <= 5):
            self.get_logger().info(f"Break with ID {break_id} is not recognized.")
            response.success = False
            return response

        self.__send_message_to_arduino(command + str(break_id))

        response.success = True
        return response
    
    def init_bno_callback(self, request, response):
        """Not in use.
        """
        self.__send_message_to_arduino(request.command)

        response.success = True
        return response
    
    def get_bno_and_battery_data_callback(self):
        """Callback for publishing data on topics with names, defined as gwp_interfaces_data.BNO_DATA_TOPIC and gwp_interfaces_data.BATTERY_VOLTAGE_TOPIC.
        BNO_DATA_TOPIC is used for communicating current direction of gravity vector and BATTERY_VOLTAGE_DATA for communicating current voltage on robot's battery.
        """
        with self.serial_comm_locker:
            received_msg = self.arduino_comm.read()

        try:
            x_gravity = float(received_msg[0 : 6])
            y_gravity = float(received_msg[6 : 12])
            z_gravity = float(received_msg[12 : 18])        

            battery_voltage = float(received_msg[18 : 23])

            gravity_vector = np.array([x_gravity, y_gravity, z_gravity], dtype = np.float32)

            gravity_vector_msg = Float32MultiArray(data = gravity_vector)
            rpy_msg = Float32MultiArray(data = [])

            bno_msg = BnoData(rpy = rpy_msg, gravity_vector = gravity_vector_msg)
            voltage_msg = Float32(data = battery_voltage)
            
            self.bno_reading_publisher.publish(bno_msg)
            self.battery_voltage_publisher.publish(voltage_msg)
        except:
            self.get_logger().info("Error while reading gravity vector data.")

    def __send_message_to_arduino(self, msg: str):
        """Helper function for sending string data to Arduino.

        Args:
            msg (str): Data to be sent.
        """
        with self.serial_comm_locker:
            self.arduino_comm.write(msg)

    def __initialize_interfaces(self):
        """Initialize all needed interfaces.
        """
        self.arduino_comm = ArduinoComm(self.DEVICE_NAME, self.RECEIVED_MESSAGE_LENGTH)
        self.water_pump_service = self.create_service(ControlWaterPump, gid.WATER_PUMP_SERVICE, self.water_pump_control_callback, callback_group = self.reentrant_callback_group)
        self.stop_water_pump_service = self.create_service(Trigger, gid.STOP_WATER_PUMP_SERVICE, self.stop_pump_callback, callback_group = self.reentrant_callback_group)
        self.init_bno_service = self.create_service(InitBno, gid.INIT_BNO_SERVICE, self.init_bno_callback, callback_group = self.reentrant_callback_group)
        self.breaks_control_service = self.create_service(BreaksControl, gid.BREAKS_SERVICE, self.breaks_control_callback, callback_group = self.reentrant_callback_group)
        self.tube_holder_control_service = self.create_service(SetBool, gid.TUBE_HOLDER_SERVICE, self.tube_holder_control_callback, callback_group = self.reentrant_callback_group)

        timer_period = 0
        self.bno_reading_publisher = self.create_publisher(BnoData, gid.BNO_DATA_TOPIC, 1, callback_group = self.reentrant_callback_group)
        self.timer = self.create_timer(timer_period, self.get_bno_and_battery_data_callback, callback_group = self.reentrant_callback_group)
        self.battery_voltage_publisher = self.create_publisher(Float32, gid.BATTERY_VOLTAGE_TOPIC, 1, callback_group = self.reentrant_callback_group)

def main():
    """Main entry point.
    """
    rclpy.init()
    water_pumps_bno_controller = WaterPumpBnoController()
    executor = MultiThreadedExecutor()
    rclpy.spin(water_pumps_bno_controller, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()