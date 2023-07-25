import rclpy
from rclpy.node import Node
import numpy as np
import time

from std_msgs.msg import Float32MultiArray, Float32

from arduino_communication.arduino_comm import ArduinoComm

from gwpspider_interfaces.srv import ControlWaterPump, InitBno
from gwpspider_interfaces.msg import BnoData
from gwpspider_interfaces import gwp_interfaces_data as gid

class WaterPumpBnoController(Node):
    def __init__(self):
        Node.__init__(self, 'water_pumps_bno_controller')

        self.PUMPS_FLOWS = [1.97, 2.05, 4.28]

        self.arduino_comm = ArduinoComm(self.DEVICE_NAME, self.RECEIVED_MESSAGE_LENGTH)
        self.water_pump_service = self.create_service(ControlWaterPump, gid.WATER_PUMP_SERVICE, self.water_pump_control_callback)
        self.init_bno_service = self.create_service(InitBno, gid.INIT_BNO_SERVICE, self.init_bno_callback)

        timer_period = 0
        self.bno_reading_publisher = self.create_publisher(BnoData, gid.BNO_DATA_TOPIC, 1)
        self.timer = self.create_timer(timer_period, self.get_gravity_vector_callback)
        self.battery_voltage_publisher = self.create_publisher(Float32, gid.BATTERY_VOLTAGE_TOPIC, 1)
    
    @property
    def RECEIVED_MESSAGE_LENGTH(self):
        return 35
    
    @property
    def DEVICE_NAME(self):
        return 'ttyUSB_BNOWP'
    
    @property
    def PUMP_OFF_COMMAND(self):
        return '0'
    
    @property
    def PUMP_ON_COMMAND(self):
        return '1'
    
    @property
    def BREAKS_OFF_COMMAND(self):
        return '4'

    @property
    def BREAKS_ON_COMMAND(self):
        return '5'
    
    def water_pump_control_callback(self, request, response):
        pump_id = request.pump
        volume = request.volume

        flow = self.PUMPS_FLOWS[pump_id]
        watering_time = volume / flow

        msg = self.PUMP_ON_COMMAND + str(pump_id) + '\n'
        self.arduino_comm.write(msg)

        start_time = time.time()
        elapsed_time = 0.0
        while elapsed_time < watering_time:
            elapsed_time = time.time() - start_time
            time.sleep(0.01)

        msg = self.PUMP_OFF_COMMAND + str(pump_id) + '\n'
        self.arduino_comm.write(msg)

        response.success = True
        return response
    
    def breaks_control_callback(self, request, response):
        break_id = request.break_id
        command = request.command

        msg = command + str(break_id) + '\n'
        self.arduino_comm.write(msg)

        response.success = True
        return response
    
    def init_bno_callback(self, request, response):
        msg = request.command + '\n'
        self.arduino_comm.write(msg)

        response.success = True
        return response
    
    def get_gravity_vector_callback(self):
        received_msg = self.arduino_comm.read()

        roll = float(received_msg[0 : 5])
        pitch = float(received_msg[5 : 10])
        yaw = float(received_msg[10 : 15])        

        x_gravity = float(received_msg[15 : 20])
        y_gravity = float(received_msg[20 : 25])
        z_gravity = float(received_msg[25 : 30])

        # battery_voltage = float(received_msg[30 : 34])
        battery_voltage = float('17.4')

        gravity_vector = np.array([x_gravity, y_gravity, z_gravity], dtype = np.float32)
        rpy = np.array([roll, pitch, yaw], dtype = np.float32)

        gravity_vector_msg = Float32MultiArray(data = gravity_vector)
        rpy_msg = Float32MultiArray(data = rpy)

        bno_msg = BnoData(rpy = rpy_msg, gravity_vector = gravity_vector_msg)
        voltage_msg = Float32(data = battery_voltage)
        
        self.bno_reading_publisher.publish(bno_msg)
        self.battery_voltage_publisher.publish(voltage_msg)


def main():
    rclpy.init()
    water_pumps_bno_controller = WaterPumpBnoController()
    rclpy.spin(water_pumps_bno_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()