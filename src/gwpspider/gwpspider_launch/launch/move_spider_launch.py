from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'dynamixel_interface',
            executable = 'dynamixel_driver',
            name = 'dynamixel_driver'
        ),
        Node(
            package = 'planning',
            executable = 'trajectory_planner',
            name = 'trajectory_planner'
        ),
        Node(
            package = 'planning',
            executable = 'path_planner',
            name = 'path_planner'
        ),
        Node(
            package = 'arduino_communication',
            executable = 'grippers_controller',
            name = 'grippers_controller'
        ),
        Node(
            package = 'arduino_communication',
            executable = 'water_pumps_bno_controller',
            name = 'water_pumps_bno_controller'
        ),
        Node(
            package = 'controller',
            executable = 'legs_states_publisher',
            name = 'legs_state_publisher'
        ),
        Node(
            package = 'controller',
            executable = 'joint_velocity_controller',
            name = 'joint_velocity_controller'
        ),
        Node(
            package = 'offset_prediction_model',
            executable = 'offset_predictor',
            name = 'offset_predictor'
        ),
        Node(
            package = 'safety_layer',
            executable = 'safety',
            name = 'safety_node'
        ),
        Node(
            package = 'application',
            executable = 'app',
            name = 'app'
        )
    ])