import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.service import SrvTypeRequest

import numpy as np
import threading
from typing import Union

from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int8MultiArray

import gwpspider_interfaces.srv as gwp_services

def create_multiple_2d_array_messages(data: np.ndarray) -> list:
    """From given data (given as array of 2-d arrays), creates multiple 2-d Float32MultiArray messages.

    Args:
        data (np.ndarray): Data to be written in 2-d Float32MultiArray messages.

    Returns:
        list: List of constructed 2-d Float32Multiarray messages.
    """
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

def create_multiple_3d_array_messages(data: np.ndarray) -> list:
    """From given data (given as array of 3-d arrays), creates multiple 3-d Float32MultiArray messages.

    Args:
        data (np.ndarray): Data to be written in 3-d Float32MultiArray messages.

    Returns:
        list: List of constructed 3-d Float32Multiarray messages.
    """
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

def unpack_2d_array_message(msg: Union[Float32MultiArray, Int8MultiArray]) -> np.ndarray:
    """Unpacks any type of ROS's MultiArray messages and packs them into numpy.ndarray.

    Args:
        msg (Union[Float32MultiArray, Int8MultiArray]): Type of message with data, that can be unpacked into 2-d array.
    Returns:
        np.ndarray: 2-d array with unpacked data.
    """
    data = msg.data
    shape = (msg.layout.dim[0].size, msg.layout.dim[1].size)
    reshaped_data = np.reshape(data, shape)

    return reshaped_data

def unpack_3d_array_message(msg: Union[Float32MultiArray, Int8MultiArray]) -> np.ndarray:
    """Unpacks any type of ROS's MultiArray messages and packs them into numpy.ndarray.

    Args:
        msg (Union[Float32MultiArray, Int8MultiArray]): Type of message with data, that can be unpacked into 3-d array.
    Returns:
        np.ndarray: 3-d array with unpacked data.
    """   
    data = msg.data
    shape = (msg.layout.dim[0].size, msg.layout.dim[1].size, msg.layout.dim[2].size)
    reshaped_data = np.reshape(data, shape)

    return reshaped_data

def prepare_trajectory_request(request_data: tuple[np.ndarray, np.ndarray, float, str]) -> gwp_services.GetLegTrajectory.Request:
    """Prepares a request for calling the GetLegTrajectory service type.

    Args:
        request_data (tuple[np.ndarray, np.ndarray, float, str]): Data that is required by the service, given as tuple.

    Returns:
        gwp_services.GetLegTrajectory.Request: GetLegTrajectory service type request.
    """
    current_position, goal_position, duration, trajectory_type = request_data

    request = gwp_services.GetLegTrajectory.Request()
    request.current_position = Float32MultiArray(data = current_position)
    request.goal_position = Float32MultiArray(data = goal_position)
    request.duration = duration
    request.trajectory_type = trajectory_type

    return request

def prepare_move_gripper_request(request_data: tuple[Union[int, float], str]) -> gwp_services.MoveGripper.Request:
    """Prepares a request for calling the MoveGripper service type.

    Args:
        request_data (tuple[Union[int, float], str]): Data that is required by the service, given as tuple.

    Returns:
        gwp_services.MoveGripper.Request: MoveGripper service type request.
    """
    leg_id, command = request_data

    request = gwp_services.MoveGripper.Request()
    request.instructions.leg_id = int(leg_id)
    request.instructions.command = command

    return request

def prepare_walking_instructions_request(request_data: tuple[np.ndarray, np.ndarray]) -> gwp_services.GetWalkingInstructions.Request:
    """Prepares a request for calling the GetWalkingInstructions service type.

    Args:
        request_data (tuple[np.ndarray, np.ndarray]): Data that is required by the service, given as tuple.

    Returns:
        gwp_services.GetWalkingInstructions.Request: GetWalkingInstructions service type request.
    """
    start_pose, goal_pose = request_data

    request = gwp_services.GetWalkingInstructions.Request()
    request.start_pose = Float32MultiArray(data = start_pose)
    request.goal_pose = Float32MultiArray(data = goal_pose)

    return request

def prepare_modified_walking_instructions_request(request_data: tuple[np.ndarray, np.ndarray]) -> gwp_services.GetModifiedWalkingInstructions.Request:
    """Prepares a request for calling the GetModifiedWalkingInstructions service type.

    Args:
        request_data (tuple[np.ndarray, np.ndarray]): Data that is required by the service, given as tuple.

    Returns:
        gwp_services.GetModifiedWalkingInstructions.Request: GetModifiedWalkingInstructions service type request.
    """
    start_legs_positions, goal_pose = request_data

    request = gwp_services.GetModifiedWalkingInstructions.Request()
    request.start_legs_positions = create_multiple_2d_array_messages([start_legs_positions])
    request.goal_pose = Float32MultiArray(data = goal_pose)

    return request

def prepare_move_spider_request(request_data: tuple[Union[list, np.ndarray], np.ndarray, np.ndarray, float]) -> gwp_services.MoveSpider.Request:
    """Prepares a request for calling the MoveSpider service type.

    Args:
        request_data (tuple[Union[list, np.ndarray], np.ndarray, np.ndarray, float]): Data that is required by the service, given as tuple.

    Returns:
        gwp_services.MoveSpider.Request: MoveSpider service type request.
    """
    legs_ids, used_pins_positions, goal_spider_pose, duration = request_data

    request = gwp_services.MoveSpider.Request()
    request.legs_ids = Int8MultiArray(data = legs_ids)
    request.used_pins_positions = create_multiple_2d_array_messages([used_pins_positions])
    request.goal_spider_pose = Float32MultiArray(data = goal_spider_pose)
    request.duration = duration

    return request

def prepare_distribute_forces_request(request_data: np.ndarray) -> gwp_services.DistributeForces.Request:
    """Prepares a request for calling the DistributeForces service type.

    Args:
        request_data (np.ndarray): Data that is required by the service, given as tuple.

    Returns:
        gwp_services.DistributeForces.Request: DistributeForces service type request.
    """
    request = gwp_services.DistributeForces.Request()
    request.legs_ids = Int8MultiArray(data = request_data)

    return request

def prepare_move_leg_request(request_data: tuple[int, np.ndarray, str, str, float, bool, np.ndarray, bool, bool, bool]) -> gwp_services.MoveLeg.Request:
    """Prepares a request for calling the MoveLeg service type.

    Args:
        request_data (tuple[int, np.ndarray, str, str, float, bool, np.ndarray, bool, bool, bool]): Data that is requeired by the service, given as tuple.

    Returns:
        gwp_services.MoveLeg.Request: MoveLeg service type request.
    """
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

def prepare_get_spider_pose_request(request_data: tuple[Union[list, np.ndarray], np.ndarray]) -> gwp_services.GetSpiderPose.Request:
    """Prepares a request for calling the GetSpiderPose service type.

    Args:
        request_data (tuple[Union[list, np.ndarray], np.ndarray]): Data that is requeired by the service, given as tuple.

    Returns:
        gwp_services.GetSpiderPose.Request: GetSpiderPose service type request.
    """
    legs_ids, legs_global_positions = request_data

    request = gwp_services.GetSpiderPose.Request()
    request.legs_ids = Int8MultiArray(data = legs_ids)
    request.legs_global_positions = create_multiple_2d_array_messages([legs_global_positions])

    return request

def prepare_move_leg_velocity_mode_request(request_data: tuple[Union[list, np.ndarray, np.ndarray, str]]) -> gwp_services.MoveLegVelocityMode.Request:
    """Prepares a request for calling the MoveLegVelocityMode service type.

    Args:
        request_data (tuple[Union[list, np.ndarray, np.ndarray, str]]): Data that is requeired by the service, given as tuple.

    Returns:
        gwp_services.MoveLegVelocityMode.Request: MoveLegVelocityMode service type request.
    """
    legs_ids, velocity_mode_direction, threshold_type = request_data

    request = gwp_services.MoveLegVelocityMode.Request()
    request.legs_ids = Int8MultiArray(data = legs_ids)
    request.velocity_mode_direction = Float32MultiArray(data = velocity_mode_direction)
    request.threshold_type = threshold_type

    return request

def prepare_toggle_controller_mode_request(request_data: tuple[str, str]) -> gwp_services.ToggleAdditionalControllerMode.Request:
    """Prepares a request for calling the ToggleAdditionalControllerMode service type.

    Args:
        request_data (tuple[str, str]): Data that is requeired by the service, given as tuple.

    Returns:
        gwp_services.ToggleAdditionalControllerMode.Request: ToggleAdditionalControllerMode service type request.
    """
    mode, command = request_data

    request = gwp_services.ToggleAdditionalControllerMode.Request()
    request.mode = mode
    request.command = command

    return request

def prepare_toggle_motors_torque_request(request_data: tuple[Union[list, np.ndarray, str]]) -> gwp_services.ToggleMotorsTorque.Request:
    """Prepares a request for calling the ToggleMotorsTorque service type.

    Args:
        request_data (tuple[Union[list, np.ndarray, str]]): Data that is requeired by the service, given as tuple.

    Returns:
        gwp_services.ToggleMotorsTorque.Request: ToggleMotorsTorque service type request.
    """
    motors_ids, command = request_data

    request = gwp_services.ToggleMotorsTorque.Request()
    request.motors_ids = Int8MultiArray(data = motors_ids)
    request.command = command

    return request

def prepare_get_correction_offset_request(request_data: tuple[np.ndarray, Union[np.ndarray, list], Union[np.ndarray, list], Union[np.ndarray, list]]) -> gwp_services.GetCorrectionOffset.Request:
    """Prepares a request for calling the GetCorrectionOffset service type.

    Args:
        request_data (tuple[np.ndarray, Union[np.ndarray, list], Union[np.ndarray, list], Union[np.ndarray, list]]): Data that is requeired by the service, given as tuple.

    Returns:
        gwp_services.GetCorrectionOffset.Request: GetCorrectionOffset service type request.
    """
    legs_current_positions, rpy, leg_goal_position, one_hot_legs = request_data

    request = gwp_services.GetCorrectionOffset.Request()
    request.legs_positions = Float32MultiArray(data = legs_current_positions.flatten())
    request.rpy = Float32MultiArray(data = rpy)
    request.commanded_position = Float32MultiArray(data = leg_goal_position)
    request.one_hot_legs = Int8MultiArray(data = one_hot_legs)

    return request 

def prepare_water_pump_request(request_data: tuple[int, float]) -> gwp_services.ControlWaterPump.Request:
    """Prepares a request for calling the ControlWaterPump service type.

    Args:
        request_data (tuple[int, float]): Data that is requeired by the service, given as tuple.

    Returns:
        gwp_services.ControlWaterPump.Request: ControlWaterPump service type request.
    """
    pump_id, volume = request_data

    request = gwp_services.ControlWaterPump.Request()
    request.pump = pump_id
    request.volume = volume

    return request

def prepare_apply_forces_on_legs_request(request_data: tuple[Union[list, np.ndarray], np.ndarray]) -> gwp_services.ApplyForcesOnLegs.Request:
    """Prepares a request for calling the ApplyForcesOnLegs service type.

    Args:
        request_data (tuple[Union[list, np.ndarray], np.ndarray]): Data that is requeired by the service, given as tuple.

    Returns:
        gwp_services.ApplyForcesOnLegs.Request: ApplyForcesOnLegs service type request.
    """
    legs_ids, desired_forces = request_data

    request = gwp_services.ApplyForcesOnLegs.Request()
    request.legs_ids = Int8MultiArray(data = legs_ids)
    request.desired_forces = create_multiple_2d_array_messages([desired_forces])

    return request

def async_service_call_from_service(client: Client, request: SrvTypeRequest) -> any:
    """Method for calling a service from another service.

    Args:
        client (Client): Service client.
        request (SrvTypeRequest): Service request.

    Returns:
        any: Result of a service's callback.
    """
    event = threading.Event()
    def done_callback(_):
        nonlocal event
        event.set()

    future = client.call_async(request)
    future.add_done_callback(done_callback)
    event.wait()
    response = future.result()

    return response

def async_service_call(client: Client, request: SrvTypeRequest, node: Node) -> any:
    """Method for calling a service.

    Args:
        client (Client): Service client.
        request (SrvTypeRequest): Service request.
        node (Node): Node from which the service is called.

    Returns:
        any: Result of a service's callback.
    """
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    return future.result()