#!/usr/bin/env python
from __future__ import print_function, absolute_import, division

import cv2
import numpy as np

from utils import which_coords_in_bounds


class Costmap:
    """
    Costmap to be used for exploration.
    NOTE THIS IS NOT TO BE CONFUSED WITH CostMap2D
    THIS MAP IS USED FOR EXPLORATION
    CostMap2D was not used in an effort to keep this repo independent
    also the values are different because it makes visualization 10x easier,
    also when saving the map it looks like a map
    """
    FREE = 255
    OCCUPIED = 0
    UNEXPLORED = 127

    def __init__(self, data, resolution, origin):
        """
        Costmap object treated as a struct tying together the map data, resolution, and origin
        :param data array(N, M)[Union[uint8, float, int]]: 2d ndarray corresponding to the costmap data
        :param resolution float: resolution of the map in meters
        :param origin array(2)[float]: [x, y] origin of the costmap
        """
        # todo getters and setters, but for now we will treat as struct
        assert data.dtype == np.uint8
        self.data = data
        self.resolution = resolution
        self.origin = np.array(origin)

    def get_shape(self):
        """
        gets the shape of the data array of the map
        :return Tuple[int]: (row, col)
        """
        return self.data.shape

    def get_size(self):
        """
        gets the size in meters of each dimension of the map
        :return Tuple[float]: (x, y)
        """
        size = self.resolution * np.array(self.data.shape)
        size[[0, 1]] = size[[1, 0]]
        return tuple(size)

    def copy(self):
        """
        Returns a copy of the costmap
        :return Costmap: copy of the current costmap
        """
        return Costmap(self.data.copy(), self.resolution, self.origin.copy())

    def visualize(self, render_size, wait_key):
        """
        Simple visualizer for the costmap
        :param render_size Tuple(int): size to render the visualization window
        :param wait_key int: opencv wait key for visualization
        """
        cv2.namedWindow('map.visualize', cv2.WINDOW_GUI_NORMAL)
        cv2.imshow('map.visualize', self.data)
        cv2.resizeWindow('map.visualize', *render_size)
        cv2.waitKey(wait_key)

    def get_downscaled(self, desired_resolution):
        """
        Return a downscaled version of the costmap, makes sure to preserve obstacles and free space that is marked
        :param desired_resolution float: resolution in meters of the downscaled costmap
        :return Costmap: object
        """
        assert self.resolution <= desired_resolution
        scale_factor = self.resolution / desired_resolution
        scaled_shape = np.rint(np.array(self.data.shape) * scale_factor).astype(np.int)
        scaled_data = np.zeros(scaled_shape)

        scaled_occupied_coords = np.rint(np.argwhere(self.data == Costmap.OCCUPIED) * scale_factor).astype(np.int)
        scaled_unexplored_coords = np.rint(np.argwhere(self.data == Costmap.UNEXPLORED) * scale_factor).astype(np.int)
        scaled_free_coords = np.rint(np.argwhere(self.data == Costmap.FREE) * scale_factor).astype(np.int)

        scaled_occupied_coords = scaled_occupied_coords[which_coords_in_bounds(scaled_occupied_coords,
                                                                               scaled_shape)]
        scaled_unexplored_coords = scaled_unexplored_coords[which_coords_in_bounds(scaled_unexplored_coords,
                                                                                   scaled_shape)]
        scaled_free_coords = scaled_free_coords[which_coords_in_bounds(scaled_free_coords,
                                                                       scaled_shape)]

        # order is important here, we want to make sure to keep the obstacles
        scaled_data[scaled_free_coords[:, 0], scaled_free_coords[:, 1]] = Costmap.FREE
        scaled_data[scaled_unexplored_coords[:, 0], scaled_unexplored_coords[:, 1]] = Costmap.UNEXPLORED
        scaled_data[scaled_occupied_coords[:, 0], scaled_occupied_coords[:, 1]] = Costmap.OCCUPIED

        return Costmap(data=scaled_data.astype(np.uint8),
                       resolution=desired_resolution,
                       origin=self.origin)

