import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import threading

from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Float32

from gwpspider_interfaces.msg import DynamixelMotorsData, GrippersStates
from gwpspider_interfaces import gwp_interfaces_data as gid
from utils import custom_interface_helper as cih
from calculations import mathtools as mt
from configuration import robot_config as rc
from configuration import spider

class Safety(Node):
    """Class that's containing safety logic.

    Args:
        Node (Node): ROS node.
    """
    def __init__(self):
        Node.__init__(self, 'safety_node')

        self.grippers_states_locker = threading.Lock()
        self.grippers_attached_states = np.array([False] * spider.NUMBER_OF_LEGS)

        self.battery_voltage_locker = threading.Lock()
        self.battery_voltage = None

        self.monitor_battery_voltage_locker = threading.Lock()
        self.monitor_battery_voltage = True

        self.dxl_data_locker = threading.Lock()
        self.currents_sum = None
        self.hw_errors = None
        self.currents_buffer = np.zeros((self.CURRENTS_WINDOW_SIZE, self.CURRENTS_ARRAY_ROWS, self.CURRENTS_ARRAY_COLUMNS))
        self.counter = 0

        self.monitor_hw_errors_locker = threading.Lock()
        self.monitor_hw_errors = True

        self.reentrant_callback_group = ReentrantCallbackGroup()
        self.__init_interfaces()
    
    @property
    def TIME_INTEGRATION_WINDOW(self) -> float:
        """Time window for array integration.

        Returns:
            float: Time window.
        """
        return 1.0
    
    @property
    def MAX_CURRENT(self) -> float:
        """Maximum allowed current.

        Returns:
            float: Maximum allowed current.
        """
        return 3.0
    
    @property
    def CURRENTS_WINDOW_SIZE(self) -> int:
        """Currents array size.

        Returns:
            int: Currents array size.
        """
        return int(self.TIME_INTEGRATION_WINDOW * rc.CONTROLLER_FREQUENCY)
    
    @property
    def CURRENTS_ARRAY_COLUMNS(self) -> int:
        """Number of columns in currents array.

        Returns:
            int: Number of columns in currents array.
        """
        return 3
    
    @property
    def CURRENTS_ARRAY_ROWS(self) -> int:
        """Number of rows in currents array.

        Returns:
            int: Number of rows in currents array.
        """
        return 5
    
    @property
    def CURRENTS_SUM_THRESHOLD(self) -> float:
        """Currents array integration threshold.

        Returns:
            float: Integration threshold.
        """
        return 220 #self.MAX_CURRENT * self.CURRENTS_WINDOW_SIZE
    
    @property
    def MIN_ALLOWED_VOLTAGE(self) -> float:
        """Minumum allowed battery voltage.

        Returns:
            float: Minumum allowed battery voltage.
        """
        return 14.5
    
    def dynamixel_motors_data_callback(self, msg: DynamixelMotorsData):
        """Subscriber callback for reading the data from topic with name, defined as gwp_interfaces_data.DYNAMIXEL_MOTORS_DATA_TOPIC. This method reads the data from the topic and current battery voltage. 
        This data is used for determining whether or not to set the error flag, which stops spider's movement. Error is caused either by hardware error in the motors, overloaded (by current) motors, or by voltage drop on battery. In case of 
        motors' error (hardware of current overload), spider's movement is stopped imediatelly. In case of too-low battery voltage error, spider reconfigures itself into a charging position and than goes into resting (charging) mode.

        Args:
            msg (DynamixelMotorsData): Message type used for communication in topic with name, defined as gwp_interfaces_data.DYNAMIXEL_MOTORS_DATA_TOPIC (custom message type).
        """
        hw_errors = cih.unpack_2d_array_message(msg.motor_errors)
        currents = cih.unpack_2d_array_message(msg.currents)

        with self.battery_voltage_locker:
            battery_voltage = self.battery_voltage
        with self.monitor_battery_voltage_locker:
            do_monitor_voltage = self.monitor_battery_voltage
        with self.monitor_hw_errors_locker:
            monitor_hw_errors = self.monitor_hw_errors

        is_hw_errors, is_current_overload_error, is_battery_voltage_error = self.__get_errors(currents, hw_errors, battery_voltage)
        if (is_hw_errors or is_current_overload_error) and monitor_hw_errors:   
            with self.monitor_hw_errors_locker:
                self.monitor_hw_errors = False
            if is_hw_errors:
                self.get_logger().info("HW ERROR TRIGGERED.")
            elif is_current_overload_error:
                self.get_logger().info("CURRENT OVERLOAD ERROR TRIGGERED.")
            immediate_stop_request = Trigger.Request()
            _ = cih.async_service_call_from_service(self.immediate_stop_trigger_client, immediate_stop_request)

        if is_battery_voltage_error and do_monitor_voltage:
            with self.monitor_battery_voltage_locker:
                self.monitor_battery_voltage = False
            self.get_logger().info("BATTERY VOLTAGE ERROR TRIGGERED.")
            trigger_battery_voltage_error_request = SetBool.Request(data = True)
            _ = cih.async_service_call_from_service(self.battery_voltage_error_trigger_client, trigger_battery_voltage_error_request)
            
    def grippers_states_callback(self, msg: GrippersStates):
        """Subscriber of topic with name, defined as gwp_interfaces_data.GRIPPER_STATES_TOPIC. It reads grippers' states (either attached or not-attached, represented by True or False values).

        Args:
            msg (GrippersStates): Message type used for communication in this topic (custom message type).
        """
        with self.grippers_states_locker:
            self.grippers_attached_states = np.array([
                msg.first_gripper.is_attached,
                msg.second_gripper.is_attached,
                msg.third_gripper.is_attached,
                msg.fourth_gripper.is_attached,
                msg.fifth_gripper.is_attached
            ])
    
    def battery_voltage_callback(self, msg: Float32):
        """Subscriber of topic with name, defined as gwp_interfaces_data.BATTERY_VOLTAGE_TOPIC.

        Args:
            msg (Float32): Float32 message type, representing current battery voltage.
        """
        with self.battery_voltage_locker:
            self.battery_voltage = msg.data
    
    def toggle_battery_voltage_monitoring_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Service callback used for toggling a flag which tells the programm, whether or not to monitor (or takes into consideration, while determining error state) a potential battery voltage. Service 
        type used for calling this service is SetBool.

        Args:
            request (SetBool.Request): SetBool service request, defined as boolean value. If True, battery voltage is monitored.
            response (SetBool.Response): SetBool service response, defined as boolean.

        Returns:
            SetBool.Response: True, if monitoring flag was set successfully, False otherwise.
        """
        with self.monitor_battery_voltage_locker:
            self.monitor_battery_voltage = request.data
            response.success = self.monitor_battery_voltage == request.data
        
        return response

    def toggle_hw_errors_monitoring_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """Service callback used for togling a flag, which tells the programm, whether or not to monitor a motors' potential hardware error. Service type used for calling
        this service is SetBool.

        Args:
            request (SetBool.Request): SetBool service request, defined as boolean value. If True, hardware error is monitored.
            response (SetBool.Response): SetBool service response, defined as boolean.

        Returns:
            SetBool.Response: True, if monitoring flag was set successfully, False otherwise.
        """
        with self.monitor_hw_errors_locker:
            self.monitor_hw_errors = request.data
            response.success = self.monitor_hw_errors == request.data
        
        return response
    
    def __get_errors(self, currents: np.ndarray, hw_errors: np.ndarray, battery_voltage: float) -> tuple[bool, bool, bool]:
        """Helper method which determines, whether or not a specific type of error is present.

        Args:
            currents (np.ndarray): Array of currents in motors.
            hw_errors (np.ndarray): Array of hardware errors in motors. Elements are either 0 (no error in specific motor) or 1 (specific motor is in hardware error state).
            battery_voltage (float): Current battery voltage.

        Returns:
            tuple[bool, bool, bool]: Bool value for each type of error: hardware error, current-overload error and battery voltage error. True means, that specific type of error was detected.
        """
        currents_sum, self.currents_buffer, self.counter = mt.integrate_array(self.currents_buffer, currents, self.counter)
        is_hw_errors = np.any(hw_errors) if hw_errors is not None else False
        is_current_overload_error = np.any(abs(currents_sum) > self.CURRENTS_SUM_THRESHOLD) if currents_sum is not None else False
        with self.grippers_states_locker:
            grippers_attached_states = self.grippers_attached_states
        is_battery_voltage_error = ((battery_voltage < self.MIN_ALLOWED_VOLTAGE) and grippers_attached_states.all()) if battery_voltage is not None else False  

        return is_hw_errors, is_current_overload_error, is_battery_voltage_error 

    def __init_interfaces(self):
        """Initialize all needed interfaces.
        """
        self.monitor_battery_voltage_trigger_service = self.create_service(SetBool, gid.TOGGLE_BATTERY_VOLTAGE_MONITORING_SERVICE, callback = self.toggle_battery_voltage_monitoring_callback, callback_group = self.reentrant_callback_group)
        self.monitor_hw_errors_trigger_service = self.create_service(SetBool, gid.TOGGLE_HW_ERRORS_MONITORING_SERVICE, callback = self.toggle_hw_errors_monitoring_callback, callback_group = self.reentrant_callback_group)
        
        self.immediate_stop_trigger_client = self.create_client(Trigger, gid.IMMEDIATE_STOP_SERVICE, callback_group = self.reentrant_callback_group)
        while not self.immediate_stop_trigger_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Imeediate stop trigger service not available...")

        self.battery_voltage_error_trigger_client = self.create_client(SetBool, gid.BATTERY_VOLTAGE_TRIGGER_SERVICE, callback_group = self.reentrant_callback_group)   
        while not self.battery_voltage_error_trigger_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Battery voltage error trigger service not available...")

        self.grippers_states_subscriber = self.create_subscription(GrippersStates, gid.GRIPPER_STATES_TOPIC, self.grippers_states_callback, 1, callback_group = self.reentrant_callback_group) 
        self.dynamixel_motors_data_subscriber = self.create_subscription(DynamixelMotorsData, gid.DYNAMIXEL_MOTORS_DATA_TOPIC, self.dynamixel_motors_data_callback, 1, callback_group = self.reentrant_callback_group)
        self.battery_voltage_subscriber = self.create_subscription(Float32, gid.BATTERY_VOLTAGE_TOPIC, self.battery_voltage_callback, 1, callback_group = self.reentrant_callback_group)

        self.get_logger().info("Safety is running.")
        
def main():
    """Main entry point.
    """
    rclpy.init()
    safety = Safety()
    executor = MultiThreadedExecutor()
    rclpy.spin(safety, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()