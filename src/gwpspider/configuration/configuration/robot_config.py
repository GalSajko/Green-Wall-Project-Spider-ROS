"""File for storing global variables.
"""
# CONTROLLER GAINS
CONTROLLER_FREQUENCY = 100.0
FORCE_DAMPING = 0.01
K_P_FORCE = 0.03
K_P = 25.0
K_D = 1.8
K_ACC = 0.18
F_D_PUSHING = 2.5

START_COMMAND = 'start'
STOP_COMMAND = 'stop'

# ORIGINS NAMES
SPIDER_ORIGIN = 'spider'
LEG_ORIGIN = 'local'
GLOBAL_ORIGIN = 'global'

# TRAJECTORIES NAMES
BEZIER_TRAJECTORY = 'bezier'
MINJERK_TRAJECTORY = 'minJerk'

FORCE_DISTRIBUTION_DURATION = 1.0

# SPIDER'S STATES NAMES
WORKING_STATE = 'working'
RESTING_STATE = 'resting'
TRANSITION_STATE = 'transition'

# ADDITIONAL CONTROLLER MODES NAMES
FORCE_MODE = 'force_mode'
VELOCITY_MODE = 'velocity_mode'

# THREAD NAMES
WORKING_THREAD_NAME = 'working_thread'
TRANSITION_THREAD_NAME = 'transition_thread'
RESTING_THREAD_NAME = 'resting_thread'
SAFETY_THREAD_NAME = 'safety_thread'
CONTROL_THREAD_NAME = 'control_thread'
UPDATE_DATA_THREAD_NAME = 'update_data_thread'
SEND_MESSAGE_DATA_THREAD_NAME = 'send_message_thread'

# SPIDER STATE DICT KEYS
STATE_DICT_POSE_KEY = 'pose'
STATE_DICT_PINS_KEY = 'pins'

# WATERING AND REFILLING INFO
NUMBER_OF_WATERING_BEFORE_REFILL = 6
WATERING_TIME = 12.0
REFILL_TIME = 28.0

PROGRAM_KILL_KEY = 'k'

GRIPPER_BNO_ARDUINO_READING_PERIOD = 0.01

ENABLE_LEGS_COMMAND = 'enable'
DISABLE_LEGS_COMMAND = 'disable'

# GRIPPERS ARDUINO COMMANDS
OPEN_GRIPPER_COMMAND = 'o'
CLOSE_GRIPPER_COMMAND = 'c'
IS_GRIPPER_OPEN_RESPONSE = '1'
IS_GRIPPER_CLOSE_RESPONSE = '0'
GRIPPER_IDENTIFIER = 'g'
SWITCH_IDENTIFIER = 's'
