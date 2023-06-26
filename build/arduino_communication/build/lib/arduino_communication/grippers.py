import rclpy
from rclpy.node import Node
from arduino_communication.arduino_comm import ArduinoComm
from std_msgs.msg import String

import time

from gwpspider_interfaces.srv import MoveGripper

class GrippersController(Node):
    def __init__(self):
        Node.__init__(self, 'grippers_controller')
        
        self.arduino_comm = ArduinoComm(self.DEVICE_NAME, self.RECEIVED_MESSAGE_LENGTH)

        self.move_service = self.create_service(MoveGripper, 'move_gripper_service', self.move_gripper_callback)
        self.states_publisher = self.create_publisher(String, 'grippers_states', 10)
        timer_period = 0
        self.timer = self.create_timer(timer_period, self.publish_states_callback)
    
    @property
    def RECEIVED_MESSAGE_LENGTH(self):
        return 11
    
    @property
    def DEVICE_NAME(self):
        return 'ttyUSB_GRIPPERS'
    
    def move_gripper_callback(self, request, response):
        leg_id = request.instructions.leg
        command = request.instructions.command

        msg = command + str(leg_id) + '\n'
        self.arduino_comm.write(msg)

        time.sleep(1.5)

        response.success = True
        return response
    
    def publish_states_callback(self):
        states = self.arduino_comm.read()
        msg = String()
        msg.data = states
        self.states_publisher.publish(msg)

def main():
    rclpy.init()
    grippers_controller = GrippersController()
    rclpy.spin(grippers_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()