import numpy as np
from gwpconfig import wall
from src.gwpspider.configuration.configuration import spider

def get_charging_pins(current_pins: np.ndarray) -> np.ndarray:
    left_relative_charging_positions = np.array([
        [[0.0, 0.0], [-0.4, -0.25], [-0.2, -1.0], [0.2, -1.0], [0.2, -0.25]],
        [[0.4, 0.25], [0.0, 0.0], [0.2, -0.75], [0.6, -0.75], [0.6, 0.0]],
        [[0.2, 1.0], [-0.2, 0.75], [0.0, 0.0], [0.4, 0.0], [0.4, 0.75]],
        [[-0.2, 1.0], [-0.6, 0.75], [-0.4, 0.0], [0.0, 0.0], [0.0, 0.75]],
        [[-0.2, 0.25], [-0.6, 0.0], [-0.4, -0.75], [0.0, -0.75], [0.0, 0.0]]
    ], dtype = np.float32)
    right_relative_charging_positions = np.array([
        [[0.0, 0.0], [-0.2, -0.25], [-0.2, -1.0], [0.2, -1.0], [0.4, -0.25]],
        [[0.2, 0.25], [0.0, 0.0], [0.0, -0.75], [0.4, -0.75], [0.6, 0.0]],
        [[0.2, 1.0], [0.0, 0.75], [0.0, 0.0], [0.4, 0.0], [0.6, 0.75]],
        [[-0.2, 1.0], [-0.4, 0.75], [-0.4, 0.0], [0.0, 0.0], [0.2, 0.75]],
        [[-0.4, 0.25], [-0.6, 0.0], [-0.6, -0.75], [-0.2, -0.75], [0.0, 0.0]]
    ], dtype = np.float32)
    relative_charging_positions = np.array([left_relative_charging_positions, right_relative_charging_positions])
    
    pins = wall.create_grid(False)
    current_legs_positions = pins[current_pins]

    current_relative_positions = np.zeros(np.shape(left_relative_charging_positions), dtype = np.float32)
    for leg in spider.LEGS_IDS:
        leg_relative_positions = np.zeros(np.shape(current_legs_positions), dtype = np.float32)
        for i, leg_position in enumerate(current_legs_positions):
            leg_relative_positions[i, :] = leg_position - current_legs_positions[leg]
        # If legs are already in desired positions, no moves are required.
        if (leg_relative_positions == left_relative_charging_positions[leg]).all() or (leg_relative_positions == right_relative_charging_positions[leg]).all():
            return np.zeros(np.shape(current_legs_positions))
        current_relative_positions[leg] = leg_relative_positions

    # Check which legs have to move, to reach the desired relative positions in as few moves as possible.
    are_legs_on_positions = np.zeros((2, spider.NUMBER_OF_LEGS, spider.NUMBER_OF_LEGS), dtype = bool) # Two (5, 5) arrays, first for left configuration, second for right.
    for i, current_relative_position in enumerate(current_relative_positions):
        are_legs_on_positions[0, i, :] = (current_relative_position == left_relative_charging_positions[i]).all(-1)
        are_legs_on_positions[1, i, :] = (current_relative_position == right_relative_charging_positions[i]).all(-1)

    
    # Calculate needed relative offsets.
    new_potential_legs_positions = current_legs_positions
    while True:
        best_left = are_legs_on_positions[0, np.argmax(are_legs_on_positions.sum(axis = 1), axis = 1)[0], :]
        best_right = are_legs_on_positions[1, np.argmax(are_legs_on_positions.sum(axis = 1), axis = 1)[1], :]
        best_configuration_id = np.argmax([np.count_nonzero(best_left), np.count_nonzero(best_right)])

        base_leg = np.argmax([best_left, best_right][best_configuration_id])
        desired_relative_positions_to_selected_base_leg = relative_charging_positions[best_configuration_id][base_leg]
        offsets = desired_relative_positions_to_selected_base_leg - current_relative_positions[base_leg]
        new_potential_legs_positions = current_legs_positions + offsets

        # Solution is valid only, if new positions are inside wall's boundaries.
        if (new_potential_legs_positions[:, 0] >= 0).all() and (new_potential_legs_positions[:, 0] <= 4.0).all() and (new_potential_legs_positions[:, 1] >= 0).all() and (new_potential_legs_positions[:, 1] <= 3.0).all():
            break

        are_legs_on_positions[best_configuration_id][base_leg] = [False] * spider.NUMBER_OF_LEGS

    return offsets

if __name__ == '__main__':
    used_pins = [16, 2, 0, 26, 28]
    i = get_charging_pins(used_pins)