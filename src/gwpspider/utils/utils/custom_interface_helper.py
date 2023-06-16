from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from gwpspider_interfaces.msg import DynamixelMotorsData

def create_dynamixel_motors_message(data):
    all_msgs = []
    for motors_data in data:
        msg = Float32MultiArray(data = motors_data.flatten())
        msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        msg.layout.dim[0].label = 'rows'
        msg.layout.dim[0].size = 5
        msg.layout.dim[1].label = 'columns'
        msg.layout.dim[1].size = 3
        all_msgs.append(msg)

    dxl_msg = DynamixelMotorsData(positions = all_msgs[0], currents = all_msgs[1], motor_errors = all_msgs[2], temperatures = all_msgs[3])    

    return dxl_msg    