#!/usr/bin/env python
from __future__ import print_function, absolute_import, division

import numpy as np

from utilities.utils import xy_to_rc
from _exploration_cpp import c_check_for_collision


def check_for_collision(state, occupancy_map, footprint_mask, outline_coords, obstacle_values):
    """
    Check if the current state with the given footprint is colliding or not.
    :param state array(3)[float]: pose of the robot
    :param occupancy_map Costmap: object to check the footprint against
    :param footprint_mask array(N,N)[int]: mask of the footprint rotated at the corresponding angle
                           needed for checking, i.e state[2]. N is 2 * mask_radius + 1,
                           the values are -1 for not footprint, 0 for footprint.
    :param outline_coords array(N, 2)[int]: the coordinates that define the outline of the footprint. N is the number
                           of points that define the outline of the footprint
    :param obstacle_values array(N)[uint8]: an array containing values that the collision checker should deem as an obstacle
                             i.e [127, 0]
    :return bool: whether there is a collision or not
    """
    state_px = xy_to_rc(state, occupancy_map)
    c_state = np.array(state_px[:2], dtype=np.int32)
    c_occupancy_map = occupancy_map.data.astype(np.uint8)
    c_footprint_mask = np.logical_not(np.array(footprint_mask, dtype=bool))
    c_outline_coords = np.array(outline_coords, dtype=np.int32)
    c_obstacle_values = np.array(obstacle_values, dtype=np.uint8)

    is_colliding = c_check_for_collision(c_state,
                                         c_occupancy_map,
                                         c_footprint_mask,
                                         c_outline_coords,
                                         c_obstacle_values)

    return is_colliding
