import numpy as np

from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from gwpspider_interfaces.srv import GetLegTrajectory, MoveGripper
from gwpspider_interfaces.msg import GripperCommand
 
def create_multiple_2d_array_messages(data):
    all_msgs = []
    for single_data in data:
        msg = Float32MultiArray(data = single_data.flatten())
        msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        msg.layout.dim[0].label = 'rows'
        msg.layout.dim[0].size = single_data.shape[0]
        msg.layout.dim[1].label = 'columns'
        msg.layout.dim[1].size = single_data.shape[1]
        if len(data) > 1:
            all_msgs.append(msg)
        else:
            all_msgs = msg
    
    return all_msgs

def create_multiple_3d_array_messages(data):
    all_msgs = []
    for single_data in data:
        msg = Float32MultiArray(data = single_data.flatten())
        msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension(), MultiArrayDimension()]
        msg.layout.dim[0].label = 'channels'
        msg.layout.dim[0].size = single_data.shape[0]
        msg.layout.dim[1].label = 'rows'
        msg.layout.dim[1].size = single_data.shape[1]
        msg.layout.dim[2].label = 'columns'
        msg.layout.dim[2].size = single_data.shape[2]
        if len(data) > 1:
            all_msgs.append(msg)
        else:
            all_msgs = msg
    
    return all_msgs

def unpack_2d_array_message(msg):
    data = msg.data
    shape = (msg.layout.dim[0].size, msg.layout.dim[1].size)
    reshaped_data = np.reshape(data, shape)

    return reshaped_data

def unpack_3d_array_message(msg):
    data = msg.data
    shape = (msg.layout.dim[0].size, msg.layout.dim[1].size, msg.layout.dim[2].size)
    reshaped_data = np.reshape(data, shape)

    return reshaped_data

def prepare_trajectory_request(request_data):
    current_position, goal_position, duration, trajectory_type = request_data

    request = GetLegTrajectory.Request()
    request.current_position = Float32MultiArray(data = current_position)
    request.goal_position = Float32MultiArray(data = goal_position)
    request.duration = duration
    request.trajectory_type = trajectory_type

    return request

def prepare_move_gripper_request(request_data):
    leg, command = request_data

    request = MoveGripper.Request()
    request.instructions.leg = leg
    request.instructions.command = command

    return request

