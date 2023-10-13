# Green-Wall-Project-Spider-ROS
Repository with ROS2 workspace for Green-Wall-Project spider robot.

## Custom interfaces documentation
All custom interfaces are defined in gwpspider_interfaces package.
### Topics
|Name|Message type|Publisher node|Description|
|-|-|-|-|
|/battery_voltage_topic|Float32|water_pumps_bno_controller|Current battery voltage.|
|/bno_data_topic|BnoData (custom)|water_pumps_bno_controller|Roll-Pitch-Yaw vector and gravity vector (used only for gravity vector).|
|/commanded_joint_velocities_topic|Float32MultiArray|joint_velocity_controller|Calculated desired joints' velocities.|
|/dynamixel_motors_data|DynamixelMotorsData (custom)|dynamixel_driver|Motors' positions, currents, error codes and temperatures.|
|/grippers_states|GrippersStates (custom)|grippers_controller|Fingers states, microswitches states and grippers' states.|
|/legs_states_topic|LegsStates (custom)|legs_states_publisher|Legs' positions, joints' positions, forces on legs and torques in joints.|
### Services
|Name|Service type|Service server node|Description|
|-|-|-|-|
|/apply_forces_on_legs_service|ApplyForcesOnLegs (custom)|joint_velocity_controller|Apply desired forces on legs' end-effectors.|
|/battery_voltage_service|SetBool|app|Set a flag, which tells a programm whether or not to trigger error in case of battery voltage drops bellow certain threshold.|
|/breaks_service|BreaksControl (custom)|water_pumps_bno_controller|Activate (or release) breaks in third joints.|
|/distribute_forces_service|DistributeForces (custom)|joint_velocity_controller|Optimally distribute forces among desired legs.|
|/get_correction_offset_service|GetCorrectionOffset (custom)|offset_predictor|Get predicted errors in leg's movement, defined as local offsets.|
|/get_leg_trajectory_service|GetLegTrajectory (custom)|trajectory_planner|Get position, velocity and acceleration trajectories for leg's movement.|
|/get_modified_walking_instructions_service|GetModifiedWalkingInstructions (custom)|path_planner|Get walking instructions (poses and holds). First step is modified to consider current legs' positions, which are not necessary the same as calculated by path planner.|
|/get_offsets_to_charging_position_service|GetOffsetsToChargingPosition (custom)|path_planner|Get offsets for legs' movements necessary, to move legs on correct holds for charging.|
|/get_walking_instructions_service|GetWalkingInstructions (custom)|path_planner|Get walking instructions (poses and holds).|
|/idle_state_service|Trigger|app|Immediately stop working procedure and all movements.|
|/monitor_battery_voltage_service|SetBool|safety_node|Set a flag that tells a programm whether or not to monitor battery voltage error.|
|/monitor_hw_errors_service|SetBool|safety_node|Set a flag that tells a programm whether or not to monitor motors' hardware errors.|
|/move_gripper_service|MoveGripper (custom)|grippers_controller|Open or close selected gripper.|
|/move_leg_service|MoveLeg (custom)|joint_velocity_controller|Move selected leg (input is desired position).|
|/move_leg_velocity_mode_service|MoveLegVelocityMode (custom)|joint_velocity_controller|Move selected leg with velocity control (input is desired velocity direction).|
|/move_spider_service|MoveSpider (custom)|joint_velocity_controller|Move spider's body.|
|/states_manager_service|SendStringCommand (custom)|app|Send the command to spider to execute the highest-level action (either working or transition into charging pose).|
|/stop_legs_service|SetBool|joint_velocity_controller|Stop all legs' movements.|
|/stop_water_pump_service|Trigger|water_pumps_bno_controller|Stop all water pumps.|
|/toggle_additional_controller_mode_service|ToggleAdditionalControllerMode (custom)|joint_velocity_controller|Activate or deactivate additional controller modes (velocity or force mode).|
|/toggle_controller_service|ToggleController (custom)|joint_velocity_controller|Activate or deactivate main controller.|
|/toggle_motors_torque_service|ToggleMotorsTorque (custom)|dynamixel_driver|Activate or deactivate torque in selected motors.|
|/tube_holder_service|SetBool|water_pumps_bno_controller|Extract or contract the tube holder on third leg.|
|/update_last_positions_service|Trigger|joint_velocity_controller|Update last legs' positions.|
|/water_pump_service|ControlWaterPump (custom)|water_pumps_bno_controller|Activate water pump (and specify how much water needs to be pumped).|

