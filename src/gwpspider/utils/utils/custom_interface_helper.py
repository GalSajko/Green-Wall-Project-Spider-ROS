import rclpy

import numpy as np
import threading

from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int8MultiArray

from configuration import robot_config
import gwpspider_interfaces.srv as gwp_services

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

    request = gwp_services.GetLegTrajectory.Request()
    request.current_position = Float32MultiArray(data = current_position)
    request.goal_position = Float32MultiArray(data = goal_position)
    request.duration = duration
    request.trajectory_type = trajectory_type

    return request

def prepare_move_gripper_request(request_data):
    leg_id, command = request_data

    request = gwp_services.MoveGripper.Request()
    request.instructions.leg_id = int(leg_id)
    request.instructions.command = command

    return request

def prepare_walking_instructions_request(request_data):
    start_pose, goal_pose = request_data

    request = gwp_services.GetWalkingInstructions.Request()
    request.start_pose = Float32MultiArray(data = start_pose)
    request.goal_pose = Float32MultiArray(data = goal_pose)

    return request

def prepare_modified_walking_instructions_request(request_data):
    start_legs_positions, goal_pose = request_data

    request = gwp_services.GetModifiedWalkingInstructions.Request()
    request.start_legs_positions = create_multiple_2d_array_messages([start_legs_positions])
    request.goal_pose = Float32MultiArray(data = goal_pose)

    return request

def prepare_move_spider_request(request_data):
    legs_ids, used_pins_positions, goal_spider_pose, duration = request_data

    request = gwp_services.MoveSpider.Request()
    request.legs_ids = Int8MultiArray(data = legs_ids)
    request.used_pins_positions = create_multiple_2d_array_messages([used_pins_positions])
    request.goal_spider_pose = Float32MultiArray(data = goal_spider_pose)
    request.duration = duration

    return request

def prepare_distribute_forces_request(request_data):
    request = gwp_services.DistributeForces.Request()
    request.legs_ids = Int8MultiArray(data = request_data)

    return request

def prepare_move_leg_request(request_data):
    leg_id, goal_position, trajectory_type, origin, duration, is_offset, spider_pose, open_gripper, close_gripper, use_prediction_model = request_data

    request = gwp_services.MoveLeg.Request()
    request.leg_id = int(leg_id)
    request.goal_position = Float32MultiArray(data = goal_position)
    request.trajectory_type = trajectory_type
    request.origin = origin
    request.duration = duration
    request.is_offset = is_offset
    request.spider_pose = Float32MultiArray(data = spider_pose)
    request.open_gripper = open_gripper
    request.close_gripper = close_gripper
    request.use_prediction_model = use_prediction_model

    return request

def prepare_get_spider_pose_request(request_data):
    legs_ids, legs_global_positions = request_data

    request = gwp_services.GetSpiderPose.Request()
    request.legs_ids = Int8MultiArray(data = legs_ids)
    request.legs_global_positions = create_multiple_2d_array_messages([legs_global_positions])

    return request

def prepare_move_leg_velocity_mode_request(request_data):
    leg_id, velocity_mode_direction = request_data

    request = gwp_services.MoveLegVelocityMode.Request()
    request.leg_id = int(leg_id)
    request.velocity_mode_direction = Float32MultiArray(data = velocity_mode_direction)

    return request

def prepare_toggle_controller_mode_request(request_data):
    mode, command = request_data

    request = gwp_services.ToggleAdditionalControllerMode.Request()
    request.mode = mode
    request.command = command

    return request

def prepare_get_correction_offset_request(request_data):
    legs_current_positions, rpy, leg_goal_position, one_hot_legs = request_data

    request = gwp_services.GetCorrectionOffset.Request()
    request.legs_positions = Float32MultiArray(data = legs_current_positions.flatten())
    request.rpy = Float32MultiArray(data = rpy)
    request.commanded_position = Float32MultiArray(data = leg_goal_position)
    request.one_hot_legs = Int8MultiArray(data = one_hot_legs)

    return request 

def async_service_call_from_service(client, request):
    event = threading.Event()
    def done_callback(_):
        nonlocal event
        event.set()

    future = client.call_async(request)
    future.add_done_callback(done_callback)
    event.wait()
    response = future.result()

    return response

def async_service_call(client, request, node):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    return future.result()