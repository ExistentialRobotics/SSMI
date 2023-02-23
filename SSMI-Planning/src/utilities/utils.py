#!/usr/bin/env python
from __future__ import print_function, absolute_import, division

import cv2
import numpy as np

from matplotlib import pyplot as plt


def clip_range(min_range, max_range, map_shape):
    """
    clips range to stay in map_shape
    :param min_range array(2)[int]: [int, int] min row col range
    :param max_range array(2)[int]: [int, int] max row col range
    :param map_shape array(2)[int]: (int, int) map shape
    :return Tuple[array(2), array(2)]: the min range and max range, clipped.
    """
    clipped_min_range = [max(min_range[0], 0), max(min_range[1], 0)]
    clipped_max_range = [min(max_range[0], map_shape[0]), min(max_range[1], map_shape[1])]
    return clipped_min_range, clipped_max_range


def compute_circumscribed_radius(points):
    """
    Compute the circumscribed radius for the given points
    :param points array(N, 2)[float]: Nx2 points to compute the radius
    :return float: circumscribed radius
    """
    distances = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2)
    return np.max(distances)


def get_rotation_matrix_2d(angle):
    """
    Returns a 2D rotation matrix numpy array corresponding to angle
    :param angle float: angle in radians
    :return array(2, 2)[float]: 2D rotation matrix
    """
    return np.array([[np.cos(angle), -np.sin(angle)],
                     [np.sin(angle), np.cos(angle)]])


def xy_to_rc(pose, occupancy_map):
    """
    Convert x, y points to row, column coordinates
    :param pose Union[array(2)[float], array(3)[float], array(N,2)[float], array(N,3)[float]]: [x, y, theta pose]
    :param occupancy_map Costmap: current map
    :return Union[array(2)[float], array(3)[float], array(N,2)[float], array(N,3)[float]]: [r, c, theta pose]
    """
    new_pose = np.array(pose, dtype=np.float)
    if len(new_pose.shape) == 1:
        new_pose[:2] -= occupancy_map.origin
        new_pose[1] = (occupancy_map.get_size()[1] - occupancy_map.resolution) - new_pose[1]
        new_pose[[0, 1]] = new_pose[[1, 0]]
        new_pose[:2] = np.rint(new_pose[:2] / occupancy_map.resolution)
    else:
        new_pose[:, :2] -= occupancy_map.origin
        new_pose[:, 1] = (occupancy_map.get_size()[1] - occupancy_map.resolution) - new_pose[:, 1]
        new_pose[:, [0, 1]] = new_pose[:, [1, 0]]
        new_pose[:, :2] = np.rint(new_pose[:, :2] / occupancy_map.resolution)
    return new_pose


def rc_to_xy(pose, occupancy_map):
    """
    Convert row, column coordinates to x, y points
    :param pose Union[array(2)[float], array(3)[float], array(N,2)[float], array(N,3)[float]]: x,y pose
    :param occupancy_map Costmap: current map
    :return Union[array(2)[float], array(3)[float], array(N,2)[float], array(N,3)[float]]: x, y pose
    """
    new_pose = np.array(pose, dtype=np.float)
    if len(new_pose.shape) == 1:
        new_pose[0] = (occupancy_map.get_shape()[0] - 1) - new_pose[0]
        new_pose[[0, 1]] = new_pose[[1, 0]]
        new_pose[:2] *= occupancy_map.resolution
        new_pose[:2] += occupancy_map.origin
    else:
        new_pose[:, 0] = (occupancy_map.get_shape()[0] - 1) - new_pose[:, 0]
        new_pose[:, [0, 1]] = new_pose[:, [1, 0]]
        new_pose[:, :2] *= occupancy_map.resolution
        new_pose[:, :2] += occupancy_map.origin

    return new_pose


def which_coords_in_bounds(coords, map_shape):
    """
    Checks the coordinates given to see if they are in bounds
    :param coords Union[array(2)[int], array(N,2)[int]]: [int, int] or [[int, int], ...], Nx2 ndarray
    :param map_shape Tuple[int]: shape of the map to check bounds
    :return Union[bool array(N)[bool]]: corresponding to whether the coord is in bounds (if array is given, then it will be
             array of bool)
    """
    assert isinstance(coords, np.ndarray) and coords.dtype == np.int
    assert np.array(map_shape).dtype == np.int
    if len(coords.shape) == 1:
        return coords[0] >= 0 and coords[0] < map_shape[0] and coords[1] >= 0 and coords[1] < map_shape[1]
    else:
        return np.logical_and(np.logical_and(coords[:, 0] >= 0, coords[:, 0] < map_shape[0]),
                              np.logical_and(coords[:, 1] >= 0, coords[:, 1] < map_shape[1]))


def bresenham2d(p0, p1):
    """
    Implementation of Bresenham's line drawing algorithm
    See en.wikipedia.org/wiki/Bresenham's_line_algorithm

    Yield integer coordinates on the line from p0: (x0, y0) to p1: (x1, y1).
    Input coordinates should be integers.
    The result will contain both the start and the end point.

    :param p0 array(2)[int]: starting point for the algorithm
    :param p1 array(2)[int]: ending point for the algorithm
    :return array(N,2)[int]: ndarray of points from start to end point
    """

    x0 = int(np.round(p0[0]))
    y0 = int(np.round(p0[1]))
    x1 = int(np.round(p1[0]))
    y1 = int(np.round(p1[1]))
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    steep = abs(dy) > abs(dx)
    if steep:
        dx, dy = dy, dx  # swap

    if dy == 0:
        q = np.zeros((dx + 1, 1))
    else:
        q = np.append(0, np.greater_equal(
            np.diff(np.mod(np.arange(np.floor(dx / 2), -dy * dx + np.floor(dx / 2) - 1, -dy), dx)), 0))
    if steep:
        if y0 <= y1:
            y = np.arange(y0, y1 + 1)
        else:
            y = np.arange(y0, y1 - 1, -1)
        if x0 <= x1:
            x = x0 + np.cumsum(q)
        else:
            x = x0 - np.cumsum(q)
    else:
        if x0 <= x1:
            x = np.arange(x0, x1 + 1)
        else:
            x = np.arange(x0, x1 - 1, -1)
        if y0 <= y1:
            y = y0 + np.cumsum(q)
        else:
            y = y0 - np.cumsum(q)
    return np.vstack((x, y)).T


def wrap_angles(angles, is_radians=True):
    """
    Wraps angles between -180 and 180 or -pi and pi based off is_radians
    :param angles array(N)[float]: ndarray containing the angles
    :param is_radians bool: whether to wrap in degrees or radians
    :return array(N)[float]: same shape ndarray where angles are wrapped
    """
    if is_radians:
        wrapped_angles = np.mod(angles + np.pi, 2 * np.pi) - np.pi
    else:
        wrapped_angles = np.mod(angles + 180, 2 * 180) - 180
    return wrapped_angles
