import rclpy
from rclpy.node import Node
import numpy as np

from arduino_communication.arduino_comm import ArduinoComm
from gwpspider_interfaces.srv import ControlWaterPump, InitBno
from gwpspider_interfaces.msg import BnoData

from std_msgs.msg import Float32MultiArray

class WaterPumpBnoController(Node):
    def __init__(self):
        Node.__init__(self, 'water_pumps_bno_controller')

        self.arduino_comm = ArduinoComm(self.DEVICE_NAME, self.RECEIVED_MESSAGE_LENGTH)
        self.water_pump_service = self.create_service(ControlWaterPump, 'water_pump_service', self.water_pump_control_callback)
        self.init_bno_service = self.create_service(InitBno, 'init_bno_service', self.init_bno_callback)

        timer_period = 0
        self.bno_reading_publisher = self.create_publisher(BnoData, 'bno_readings', 10)
        self.timer = self.create_timer(timer_period, self.get_gravity_vector_callback)
    
    @property
    def RECEIVED_MESSAGE_LENGTH(self):
        return 31
    
    @property
    def DEVICE_NAME(self):
        return 'ttyUSB_BNOWP'
    
    def water_pump_control_callback(self, request, response):
        pump_id = request.instructions.pump
        command = request.instructions.command

        msg = command + str(pump_id) + '\n'
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

        gravity_vector = np.array([x_gravity, y_gravity, z_gravity], dtype = np.float32)
        rpy = np.array([roll, pitch, yaw], dtype = np.float32)

        gravity_vector_msg = Float32MultiArray(data = gravity_vector)
        rpy_msg = Float32MultiArray(data = rpy)

        msg = BnoData(rpy = rpy_msg, gravity_vector = gravity_vector_msg)
        
        self.bno_reading_publisher.publish(msg)


def main():
    rclpy.init()
    water_pumps_bno_controller = WaterPumpBnoController()
    rclpy.spin(water_pumps_bno_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()