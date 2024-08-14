#!/usr/bin/env python
from __future__ import print_function, absolute_import, division

import numpy as np

from utilities.utils import xy_to_rc, rc_to_xy
from _exploration_cpp import c_astar, c_oriented_astar, c_get_astar_angles


def get_astar_angles():
    """
    Return the angles used for astar.
    :return array(8)[float]: angles used for astar (in the correct order)
    """
    c_angles = np.array(c_get_astar_angles(), dtype=np.float32)
    return c_angles


def astar(goal, start, occupancy_map, obstacle_values, planning_scale=1, delta=0.0, epsilon=1.0, allow_diagonal=False):
    """
    Wrapper for vanilla a-star c++ planning. given a start and end and a map, give a path.
    :param goal array(3)[float]: [x, y, theta] goal pose of the robot
    :param start array(3)[float]: [x, y, theta] start pose of the robot
    :param occupancy_map Costmap: occupancy map used for planning, data must be compatible with uint8
    :param obstacle_values array(N)[uint8]: an array containing values that the collision checker should deem as an obstacle
                             i.e [127, 0]
    :param planning_scale int: value > 1, to plan on a lower resolution than the original occupancy map resolution,
                           this value is round_int(desired resolution / original resolution)
    :param delta float: distance in pixels to extend the goal region for solution (allow to be within delta away from goal)
                  TODO FIX this to be in meters
    :param epsilon float: weighting for the heuristic in the A* algorithm
    :param allow_diagonal bool: whether to allow diagonal movements
    :return Tuple[bool, array(N, 3)[float]]: whether we were able to successfully plan to the goal node,
                                              and the most promising path to the goal node (solution if obtained)
    """
    start_px = xy_to_rc(start, occupancy_map)
    c_start = np.array(start_px[:2], dtype=np.int32)

    goal_px = xy_to_rc(goal, occupancy_map)
    c_goal = np.array(goal_px[:2], dtype=np.int32)

    c_occupancy_map = occupancy_map.data.astype(np.uint8)

    success, path_px = c_astar(c_start,
                               c_goal,
                               c_occupancy_map,
                               obstacle_values,
                               delta,
                               epsilon,
                               planning_scale,
                               allow_diagonal)

    path = rc_to_xy(path_px, occupancy_map)
    return success, np.vstack(([start], path))


def oriented_astar(goal, start, occupancy_map, footprint_masks,
                   outline_coords, obstacle_values, planning_scale=1, delta=0.0, epsilon=1.0, allow_diagonal=True):
    """
        Oriented Astar C++ wrapper for python. Formats input data in required format for c++ function, the calls it,
    returning the path if found.
    :param goal array(3)[float]: [x, y, theta] goal pose of the robot
    :param start array(3)[float]: [x, y, theta] start pose of the robot
    :param occupancy_map Costmap: occupancy map used for planning, data must be compatible with uint8
    :param footprint_masks array(N,M,M)[int]: masks of the footprint rotated at the corresponding angles
                            needed for checking, i.e state[2]. N is 2 * mask_radius + 1,
                            the values are -1 for not footprint, 0 for footprint.
                            N is the dimension across angles, (M, M) is the mask shape
    :param outline_coords array(N, 2)[int]: the coordinates that define the outline of the footprint. N is the number
                           of points that define the outline of the footprint
    :param obstacle_values array(N)[uint8]: an array containing values that the collision checker should deem as an obstacle
                             i.e [127, 0]
    :param planning_scale int: value > 1, to plan on a lower resolution than the original occupancy map resolution,
                           this value is round_int(desired resolution / original resolution)
    :param delta float: distance in pixels to extend the goal region for solution (allow to be within delta away from goal)
                  TODO FIX this to be in meters
    :param epsilon float: weighting for the heuristic in the A* algorithm
    :param allow_diagonal bool: whether to allow diagonal movements
    :return Tuple[bool, array(N, 3)[float]]: whether we were able to successfully plan to the goal,
                                              and the most promising path to the goal node (solution if obtained)
    """
    c_angles = np.array(c_get_astar_angles(), dtype=np.float32)

    start_px = xy_to_rc(start, occupancy_map)
    c_start = np.array(start_px[:2], dtype=np.int32)

    goal_px = xy_to_rc(goal, occupancy_map)
    c_goal = np.array(goal_px[:2], dtype=np.int32)

    c_occupancy_map = occupancy_map.data.astype(np.uint8)

    c_footprint_masks = np.logical_not(np.array(footprint_masks, dtype=bool))
    c_footprint_masks = [c_footprint_mask for c_footprint_mask in c_footprint_masks]

    c_outline_coords = np.array(outline_coords, dtype=np.int32)
    c_outline_coords = [c_outline_coord for c_outline_coord in c_outline_coords]

    c_obstacle_values = np.array(obstacle_values, dtype=np.uint8)

    success, path_px = c_oriented_astar(c_start,
                                        c_goal,
                                        c_occupancy_map,
                                        c_footprint_masks,
                                        c_angles,
                                        c_outline_coords,
                                        c_obstacle_values,
                                        delta,
                                        epsilon,
                                        planning_scale,
                                        allow_diagonal)

    path = rc_to_xy(path_px, occupancy_map)
    return success, np.vstack(([start], path))
