#!/usr/bin/env python
from __future__ import print_function, absolute_import, division

import cv2
import matplotlib.pyplot as plt
import numpy as np
from footprints.collision_cpp import check_for_collision
from mapping.costmap import Costmap
from utilities.utils import rc_to_xy, get_rotation_matrix_2d, which_coords_in_bounds, compute_circumscribed_radius, clip_range, xy_to_rc


class CustomFootprint:
    """
    Footprint object that is created using a ndarray of points. it will draw a hull around them, and collision
    check using the hull.
    """
    def __init__(self, footprint_points, angular_resolution, inflation_scale=1.0):
        """
        Define a footprint as the filled in hull of the points selected. The first point of the hull_points must
        be where 0 radians is. The points must be inorder either counter clockwise or clockwise.
        :param footprint_points array(N,2)[float]: points to "connect" to make the footprint
        :param angular_resolution float: radian resolution at which to precompute the hulls for
        :param inflation_scale float: scaling factor on the the footprint, inflating its size
        """
        assert angular_resolution > 0.001 * np.pi / 180.\
            and "too small resolution. " \
                "should probably implement with degrees (no multiple of irrational numbers with degrees)"

        self._inflation_scale = inflation_scale
        self._footprint_points = footprint_points
        self._inflated_footprint_points = footprint_points * inflation_scale
        self._angular_resolution = angular_resolution

        self._mask_angles, self._rotated_masks = self._precompute_rotated_footprint_points()
        self._rotated_masks_set = dict()
        self._rotated_outline_coords_set = dict()
        self._mask_radius_set = dict()

    def copy(self):
        """
        Returns a copy of the current footprint object
        :return CustomFootprint: a copy of the current object
        """
        return CustomFootprint(footprint_points=self._footprint_points.copy(),
                               angular_resolution=self._angular_resolution,
                               inflation_scale=self._inflation_scale)

    def no_inflation(self):
        """
        Returns a copy of the current footprint object but with no inflation.
        :return CustomFootprint: a copy of the current object but with no inflation.
        """
        return CustomFootprint(footprint_points=self._footprint_points.copy(),
                               angular_resolution=self._angular_resolution,
                               inflation_scale=1.0)

    def _precompute_rotated_footprint_points(self, debug=False):
        """
        Precompute rotated hulls to all angles specified by the angular_resolution
        :param debug bool: show debug plots?
        :return Union[Tuple[array(N)[float], List(array(N, 2)[float]]]]: angles of which the hulls were rotated, the rotated hulls
        """
        rotation_angles = np.arange(-np.pi, np.pi, self._angular_resolution)
        # TODO WHY NOT get_rotation_matrix_2d(-angle))
        rotated_hulls = [self._inflated_footprint_points.dot(get_rotation_matrix_2d(angle)) for angle in rotation_angles]
        if debug:
            for rotated_point in rotated_hulls:
                plt.plot(rotated_point[:, 0], rotated_point[:, 1])
                plt.show()

        return rotation_angles, rotated_hulls

    def _add_new_masks(self, resolution, debug=False):
        """
        With a specified resolution, we can compute and save a mask for the footprint for that resolution.
        This allows us to check footprints very fast.
        :param resolution float: desired resolution to compute the footprint mask
        :param debug bool: show debug plots?
        """
        # compute needed array size of the footprint mask
        first_hull_px = np.ceil(self._rotated_masks[0] / resolution).astype(int)[:, ::-1]

        # add +1 to give a pixel footprint a minimum radius > 0
        mask_radius = np.ceil(compute_circumscribed_radius(first_hull_px)).astype(int) + 1
        mask_shape = np.array([2 * mask_radius + 1, 2 * mask_radius + 1])
        ego_coord = np.floor(mask_shape / 2.).astype(int)

        # loop through all the rotated hulls, and rasterize them onto the mask
        rotated_footprint_masks = []
        rotated_outline_coords = []
        for i, rotated_hull in enumerate(self._rotated_masks):
            rotated_hull_px = np.ceil(rotated_hull / resolution).astype(int)[:, ::-1] + ego_coord
            footprint_mask = -1 * np.ones(mask_shape)
            cv2.drawContours(footprint_mask, [rotated_hull_px[:, ::-1]], contourIdx=0,
                             color=Costmap.OCCUPIED, thickness=-1)
            rotated_footprint_masks.append(footprint_mask)
            rotated_outline_coords.append(rotated_hull_px - ego_coord)
            if debug and i % 10 == 0:
                mask_vis = rotated_footprint_masks[i].copy()
                mask_vis[ego_coord[0], ego_coord[1]] = Costmap.OCCUPIED
                plt.imshow(mask_vis)
                plt.show()

        # save these so we dont have recompute later
        self._mask_radius_set[resolution] = mask_radius
        self._rotated_outline_coords_set[resolution] = np.array(rotated_outline_coords)
        self._rotated_masks_set[resolution] = np.array(rotated_footprint_masks)

    def check_for_collision(self, state, occupancy_map, unexplored_is_occupied=False, use_python=False, debug=False):
        """
        using the state and the map, check for a collision on the map.
        :param state array(3)[float]: state of the robot [x, y, theta]
        :param occupancy_map Costmap: object to check the footprint against
        :param unexplored_is_occupied bool: whether to treat unexplored on the map as occupied
        :param use_python bool: whether to use python collision checking or c++
        :param debug bool: show debug plots?
        :return bool: True for collision
        """
        # check if we have computed a mask for this resolution, if not compute it
        if occupancy_map.resolution not in list(self._rotated_masks_set.keys()):
            self._add_new_masks(occupancy_map.resolution)

        # convert state to pixel coordinates
        state_px = xy_to_rc(pose=state, occupancy_map=occupancy_map)
        position = np.array(state_px[:2]).astype(int)

        if debug:
            map_vis = occupancy_map.copy()
            map_vis.data = np.repeat([occupancy_map.data], repeats=3, axis=0).transpose((1, 2, 0)).copy()
            self.draw(rc_to_xy(state_px, occupancy_map), map_vis, [255, 10, 10])
            occupied_inds = np.argwhere(occupancy_map.data == Costmap.OCCUPIED)
            map_vis.data[occupied_inds[:, 0], occupied_inds[:, 1]] = [0, 0, 0]
            plt.imshow(map_vis.data, interpolation='nearest')
            plt.show()

        # get the mask radius that was saved for this resolution
        mask_radius = self._mask_radius_set[occupancy_map.resolution]

        # compute the closest angle to the current angle in the state, and get that rotated footprint
        closest_angle_ind = np.argmin(np.abs(state_px[2] - self._mask_angles))
        footprint_mask = self._rotated_masks_set[occupancy_map.resolution][closest_angle_ind]
        outline_coords = self._rotated_outline_coords_set[occupancy_map.resolution][closest_angle_ind]

        if not use_python:
            obstacle_values = [Costmap.OCCUPIED, Costmap.UNEXPLORED] if unexplored_is_occupied else [Costmap.OCCUPIED]

            is_colliding = check_for_collision(state,
                                               occupancy_map,
                                               footprint_mask=footprint_mask,
                                               outline_coords=outline_coords,
                                               obstacle_values=obstacle_values)
        else:
            # part of robot off the edge of map, it is a collision, because we assume map is bounded
            if not np.all(which_coords_in_bounds(coords=(outline_coords + state_px[:2]).astype(int),
                                                 map_shape=occupancy_map.get_shape())):
                return True

            # get a small subsection around the state in the map (as big as our mask),
            # and do a masking check with the footprint to see if there is a collision.
            # if part of our subsection is off the map, we need to clip the size
            # to reflect this, in both the subsection and the mask
            min_range = [position[0] - mask_radius, position[1] - mask_radius]
            max_range = [position[0] + mask_radius + 1, position[1] + mask_radius + 1]
            clipped_min_range, clipped_max_range = clip_range(min_range, max_range, occupancy_map.data.shape)

            min_range_delta = clipped_min_range - np.array(min_range)
            max_range_delta = clipped_max_range - np.array(max_range)

            ego_map = occupancy_map.data[clipped_min_range[0]:clipped_max_range[0],
                                         clipped_min_range[1]:clipped_max_range[1]].copy()

            # todo might be - max_range_delta
            footprint_mask = footprint_mask[min_range_delta[0]:footprint_mask.shape[1] + max_range_delta[0],
                                            min_range_delta[1]:footprint_mask.shape[1] + max_range_delta[1]]

            # treat unexplored space as obstacle
            if unexplored_is_occupied:
                ego_map[ego_map == Costmap.UNEXPLORED] = Costmap.OCCUPIED

            is_colliding = np.any(footprint_mask == ego_map)

            if debug:
                plt.imshow(footprint_mask - 0.1 * ego_map)
                plt.show()

        return is_colliding

    def draw(self, state, visualization_map, color):
        """
        Draws the footprint on the map at the state specified, with the color specified.
        :param state array(3)[float]: state of the robot [x, y, theta]
        :param visualization_map Costmap: object to draw the footprint on
        :param color Union[int, array(3)[uint8]]: color based on the dimensionality of the map.data, if map.data is MxNx3 a rgb i.e [255, 0, 0]
                      must be given, otherwise if it is a MxN, then a grayscale value must be given, i.e 70
        """
        if visualization_map.resolution not in list(self._rotated_outline_coords_set.keys()):
            self._add_new_masks(visualization_map.resolution)

        state_px = xy_to_rc(state, visualization_map)
        # TODO WHY NOT -state_px[2]
        closest_angle_ind = int(np.argmin(np.abs(state_px[2] - self._mask_angles)))

        rotated_hull_px = self._rotated_outline_coords_set[visualization_map.resolution][closest_angle_ind] / self._inflation_scale
        rotated_hull_px = (rotated_hull_px + state_px[:2]).astype(int)
        cv2.drawContours(visualization_map.data, [rotated_hull_px[:, ::-1]], contourIdx=0, color=color, thickness=-1)

    def draw_circumscribed(self, state, visualization_map):
        """
        Clears the circumscribed circle of footprint on the map at the state specified

        :param state array(3)[float]: state of the robot [x, y, theta]
        :param visualization_map Costmap: object to draw the footprint on
        """
        if visualization_map.resolution not in list(self._mask_radius_set.keys()):
            self._add_new_masks(visualization_map.resolution)

        mask_radius = self._mask_radius_set[visualization_map.resolution] + 1

        state_px = xy_to_rc(state, visualization_map).astype(int)

        cv2.circle(visualization_map.data, center=tuple(state_px[:2][::-1]), radius=mask_radius,
                   color=Costmap.FREE, thickness=-1)

    def get_ego_points(self, angle, resolution):
        """
        returns the footprint points at the specified angle
        :param angle float: angle of the footprint desired
        :param resolution float: resolution of which to give the points
        :return array(N)[float]: points in ego frame corresponding to the footprint
        """
        if resolution not in list(self._rotated_masks_set.keys()):
            self._add_new_masks(resolution)

        mask_radius = self._mask_radius_set[resolution]
        mask_shape = np.array([2 * mask_radius + 1, 2 * mask_radius + 1])
        ego_center = np.floor(mask_shape / 2.).astype(int)
        closest_angle_ind = int(np.argmin(np.abs(angle - self._mask_angles)))
        ego_coords = np.argwhere(self._rotated_masks_set[resolution][closest_angle_ind] == Costmap.OCCUPIED)\
            - ego_center
        return resolution * ego_coords

    def get_outline_points(self, angle, resolution):
        """
        returns the points that are on the border of the footprint (points on the outline)
        :param angle float: angle of the footprint desired
        :param resolution float: resolution of which to give the points
        :return array(N)[float]: ndarray points that define the outline
        """
        if resolution not in list(self._rotated_masks_set.keys()):
            self._add_new_masks(resolution)

        closest_angle_ind = int(np.argmin(np.abs(angle - self._mask_angles)))
        return resolution * self._rotated_outline_coords_set[resolution][closest_angle_ind]

    def get_clearing_points(self, resolution):
        """
        returns the points that would clear any orientation of the robot (i.e circumscribed circle)
        :param resolution float: resolution of which to give the points
        :return array(N,2)[float]: points that define the clearing footprint of the robot
        """
        if resolution not in list(self._mask_radius_set.keys()):
            self._add_new_masks(resolution)

        mask_radius = self._mask_radius_set[resolution] + 1
        mask_shape = np.array([2 * mask_radius + 1, 2 * mask_radius + 1])
        ego_coord = np.floor(mask_shape / 2.).astype(int)
        mask = np.zeros(mask_shape)
        cv2.circle(mask, center=tuple(ego_coord[::-1]), radius=mask_radius, color=1, thickness=-1)
        return resolution * (np.argwhere(mask == 1) - ego_coord)

    def get_footprint_masks(self, resolution, angles=None):
        """
        Returns the footprint masks used for footprint collision checking
        :param resolution float: resolution of which to give the masks
        :param angles array(N)[float]: if not None then return the masks whose angles are closest to these
        :return array(N,M,M)[int]: there are N of mask with shape (M, M). the angles correspond to self._mask_angles
                 it is -1 for no footprint, and 0 for footprint.
        """
        if resolution not in list(self._rotated_masks_set.keys()):
            self._add_new_masks(resolution)

        if angles is not None:
            angle_inds = np.argmin(np.abs(np.expand_dims(angles, axis=1) -
                                          np.expand_dims(self._mask_angles, axis=0)), axis=1)
            return self._rotated_masks_set[resolution][angle_inds]
        else:
            return self._rotated_masks_set[resolution]

    def get_outline_coords(self, resolution, angles=None):
        """
        returns the ego outline coords of the footprint for the specified resolution
        :param resolution float: resolution of which to give the coords
        :param angles array(N)[float]: if not None then return the masks whose angles are closest to these
        :return array(N,2)[int]: ego coordinates of the footprint at the specified resoltuion
        """
        if resolution not in list(self._rotated_outline_coords_set.keys()):
            self._add_new_masks(resolution)

        if angles is not None:
            angle_inds = np.argmin(np.abs(np.expand_dims(angles, axis=1) -
                                          np.expand_dims(self._mask_angles, axis=0)), axis=1)
            return self._rotated_outline_coords_set[resolution][angle_inds]
        else:
            return self._rotated_outline_coords_set[resolution]

    def get_mask_radius(self, resolution):
        """
        Get the radius of the footprint mask used for the given resolution.
        It is the computed circumscribed radius of the footprint.
        The footprint mask shape is (2*mask_radius+1, 2*mask_radius+1)
        :param resolution float: resolution of which to give the radius
        :return int: computed circumscribed radius of the footprint, or mask_radius
        """
        if resolution not in list(self._mask_radius_set.keys()):
            self._add_new_masks(resolution)
        return self._mask_radius_set[resolution]

    def get_mask_angles(self):
        """
        Return the mask angles that correspond to each of the footprint masks
        :return array(N)[float]: mask angles
        """
        return self._mask_angles
