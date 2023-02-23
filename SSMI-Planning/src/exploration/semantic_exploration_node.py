#!/usr/bin/env python
from __future__ import division
from __future__ import print_function

import sys
import time
import rospy
import tf
import cv2

import numpy as np
import matplotlib.pyplot as plt

import message_filters

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PoseStamped

from costmap import Costmap
from utils import rc_to_xy, wrap_angles, xy_to_rc, bresenham2d
from footprint_points import get_tricky_circular_footprint, get_tricky_oval_footprint
from footprints import CustomFootprint
from astar_cpp import oriented_astar, get_astar_angles

from semantic_octomap.srv import *


class SemanticExplorationAgent:
    def __init__(self):

        self.min_frontier_size = rospy.get_param('/planning/min_frontier_size')

        self.planning_angles = get_astar_angles()
        self.planning_epsilon = rospy.get_param('/planning/planning_epsilon')

        footprint_type = rospy.get_param('/planning/footprint/type')

        if footprint_type == 'tricky_circle':
            footprint_points = get_tricky_circular_footprint()
        elif footprint_type == 'tricky_oval':
            footprint_points = get_tricky_oval_footprint()
        elif footprint_type == 'circle':
            rotation_angles = np.arange(0, 2 * np.pi, 4 * np.pi / 180)
            footprint_radius = rospy.get_param('/planning/footprint/radius')
            footprint_points = \
                footprint_radius * np.array([np.cos(rotation_angles), np.sin(rotation_angles)]).T
        elif footprint_type == 'pixel':
            footprint_points = np.array([[0., 0.]])
        else:
            footprint_points = None
            assert False and "footprint type specified not supported."

        self.footprint = CustomFootprint(footprint_points=footprint_points,
                                          angular_resolution=np.pi / 4,
                                          inflation_scale=rospy.get_param('/planning/footprint/inflation_scale'))
        
        self.kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        
        occ_map_res = rospy.get_param('/octomap/resolution')
        
        self.footprint_masks = self.footprint.get_footprint_masks(occ_map_res, angles=self.planning_angles)
        self.footprint_outline_coords = self.footprint.get_outline_coords(occ_map_res, angles=self.planning_angles)
        self.footprint_mask_radius = self.footprint.get_mask_radius(occ_map_res)

        phi = rospy.get_param('/octomap/phi')
        psi = rospy.get_param('/octomap/psi')
        
        self.l_k = phi * np.ones((5, 5))
        self.l_k[:, 0] = 0
        for i in range(4):
            self.l_k[i, i + 1] = psi
        
        self.path_pub = rospy.Publisher("/planner/path", Path, queue_size = 1)
        
        self.collision_pub = rospy.Publisher("/planner/collision", Bool, queue_size = 1)
        
        self.tf_listener = tf.TransformListener()
        
        self.world_frame_id = rospy.get_param('/octomap/world_frame_id')
        self.sensor_frame_id = rospy.get_param('/semantic_pcl/frame_id')
        self.robot_frame_id = rospy.get_param('/planning/robot_frame_id')
        
        self.goal = None
        self.goal_check_radius = rospy.get_param('/planning/goal_check_radius')
        
        self.path_rc = None
        
        self.skip_pose = rospy.get_param('/planning/skip_pose')
        
        # Get image size
        width, height = rospy.get_param('/camera/width'), rospy.get_param('/camera/height')
        height = int(height / 2) # We only care about the top half of the FoV
        # Camera intrinsic matrix
        fx = rospy.get_param('/camera/fx')
        fy = rospy.get_param('/camera/fy')
        cx = rospy.get_param('/camera/cx')
        cy = rospy.get_param('/camera/cy')
        intrinsic = np.matrix([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype = np.float32)
        
        # Allocate arrays
        x_index = np.array([list(range(width))*height], dtype = '<f4')
        y_index = np.array([[i]*width for i in range(height)], dtype = '<f4').ravel()
        xy_index = np.vstack((x_index, y_index)).T # x,y
        xyd_vect = np.zeros([width*height, 3], dtype = '<f4') # x,y,depth
        
        skip_pixel = rospy.get_param('/planning/skip_pixel')
        max_range = rospy.get_param('/octomap/raycast_range')
        
        xyd_vect[:,0:2] = xy_index * max_range
        xyd_vect[:,2:3] = max_range
        XYZ_vect = xyd_vect.dot(intrinsic.I.T)
        
        XYZ_vect = XYZ_vect[::skip_pixel, :].T
        self.XYZ_vect_hom = np.vstack((XYZ_vect, np.ones((1,XYZ_vect.shape[1]))))
        
        self.T_co = np.array([[0, 0, 1, 0],
                              [-1, 0, 0, 0],
                              [0, -1, 0, 0],
                              [0, 0, 0, 1]])
        
        self.sensor_height = 1 # TODO: Should be read from the sensor extrinsics
        
        time.sleep(1)
        self.map_sub = message_filters.Subscriber('occupancy_map_2D', OccupancyGrid, queue_size = 1)
        self.map_sub.registerCallback(self.map_callback)
        
        rospy.wait_for_service('querry_RLE')
        try:
            self.RLE_query = rospy.ServiceProxy('querry_RLE', GetRLE, persistent=True)
        except rospy.ServiceException as e:
            print("RLE Service initialization failed: %s"%e)
        
        print('Semantic exploration agent initialized!')

    def transform_matrix_2d(self, p_1, p_2, theta):
        return np.array([[np.cos(theta), -1 * np.sin(theta), 0, p_1],
                         [np.sin(theta), np.cos(theta), 0, p_2],
                         [0, 0, 1, self.sensor_height],
                         [0, 0, 0, 1]])

    def pose_msg_to_state(self, pose_msg):
        position = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
        quaternion = np.array([pose_msg.orientation.x, pose_msg.orientation.y,
                               pose_msg.orientation.z, pose_msg.orientation.w])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return np.array([position[0], position[1], euler[2]])
        
    def get_pose_from_tf(self, from_frame_id):
        (translation, rotation) = self.tf_listener.lookupTransform(self.world_frame_id,
                                                                   from_frame_id,
                                                                   rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rotation)
        return np.array([translation[0], translation[1], euler[2]])

    def map_callback(self, occ_map_msg):
        occupancy_map = 255 - np.array(occ_map_msg.data, dtype=np.uint8)
        occupancy_map = occupancy_map.reshape((occ_map_msg.info.height,
                                               occ_map_msg.info.width))
        occupancy_map = np.flipud(occupancy_map)
        occupancy_map[occupancy_map == 0] = Costmap.UNEXPLORED
        occupancy_map[occupancy_map == 155] = Costmap.OCCUPIED
        origin = self.pose_msg_to_state(occ_map_msg.info.origin)[:2]
        resolution = occ_map_msg.info.resolution

        exploration_map = Costmap(occupancy_map, resolution, origin)
        
        if self.path_rc is not None:
	          collision_ahead = np.any(exploration_map.data[self.path_rc[:, 0], self.path_rc[:, 1]] == Costmap.OCCUPIED)
	          if collision_ahead:
	              print("WARNING: Collision Ahead!")
	              collision_msg = Bool()
	              collision_msg.data = True
	              self.collision_pub.publish(collision_msg)
	              robot_pose = self.get_pose_from_tf(self.robot_frame_id)
	              self.path_rc = xy_to_rc(robot_pose, exploration_map)[None, :2].astype(int)
	              self.goal = robot_pose
	              self.publish_path(robot_pose)
	              time.sleep(2)
        
        if self.check_ready():
            print('Planning started!')
            self.plan(exploration_map)
    
    def check_ready(self):
        if self.goal is None:
            return True
        else:
            robot_pose = self.get_pose_from_tf(self.robot_frame_id)
            if np.linalg.norm(robot_pose[:2] - self.goal[:2]) < self.goal_check_radius:
                time.sleep(1)
                self.goal = None
            
            return False
    
    def find_frontiers(self, state, occupancy_map):
        frontiers = extract_frontiers(occupancy_map=occupancy_map, kernel=self.kernel)

        frontier_sizes = np.array([frontier.shape[0] if len(frontier.shape) > 1 else 1 for frontier in frontiers])
        valid_frontier_inds = np.argwhere(frontier_sizes >= self.min_frontier_size).squeeze()

        frontier_goals = []

        if len(valid_frontier_inds.shape) == 0:
            return None

        for v in valid_frontier_inds:
            f = frontiers[v]
            frontier_mean = np.mean(f, axis=0)
            
            frontier_position = f[np.argmin(np.linalg.norm(f - frontier_mean, axis=1))]
            frontier_orientation = wrap_angles(np.arctan2(frontier_position[1] - state[1], frontier_position[0] - state[0]))
            frontier_pose = np.array([frontier_position[0], frontier_position[1], frontier_orientation])

            frontier_goals.append(frontier_pose)

        return frontier_goals
    
    def plan(self, exploration_map):
        
        exploration_map = cleanup_map_for_planning(occupancy_map=exploration_map, kernel=self.kernel, filter_obstacles=False)
        
        robot_pose = self.get_pose_from_tf(self.robot_frame_id)
            
        self.footprint.draw_circumscribed(robot_pose, exploration_map)        
        
        frontiers = self.find_frontiers(robot_pose, exploration_map)
        
        path_list = []
        path_score_list = []
        if frontiers is not None:
            print(str(len(frontiers)) + " frontiers have been detected.")
            print("Planning for each frontier has begun...")
        
            i = 1
            for f in frontiers:
                plan_success, path = oriented_astar(start=robot_pose,
                                                    occupancy_map=exploration_map,
                                                    footprint_masks=self.footprint_masks,
                                                    outline_coords=self.footprint_outline_coords,
                                                    obstacle_values=[Costmap.OCCUPIED, Costmap.UNEXPLORED],
                                                    epsilon=self.planning_epsilon,
                                                    goal=f,
                                                    delta=self.footprint_mask_radius)
                if plan_success and np.sum(np.linalg.norm(path[1:, :2] - path[:-1, :2], axis=1)) > 0.2:
                    path_list.append(path)
                    path_score_list.append(self.compute_path_score(path))

                print(str(i) + " out of " + str(len(frontiers)) + " have been evaluated.")
                i += 1
        
        best_path = None
        if len(path_list) == 0:
            print('Planning failed! Computing a failsafe path.')
            best_path = self.get_failsafe_path(robot_pose, exploration_map)
        else:
            best_path = path_list[path_score_list.index(min(path_score_list))]           

        if best_path is not None:
            print('Path length: ' + str(np.sum(np.linalg.norm(best_path[1:, :2] - best_path[:-1, :2], axis=1))) +
                  '\nNumber of waypoints: ' + str(best_path.shape[0]))
            self.goal = best_path[-1, :]
            
            pose_rc_1 = xy_to_rc(best_path[0, :2], exploration_map)
            pose_rc_2 = xy_to_rc(best_path[1, :2], exploration_map)
            self.path_rc = bresenham2d(pose_rc_1, pose_rc_2)
            for pose_idx in range(1, best_path.shape[0] - 1):
                pose_rc_1 = xy_to_rc(best_path[pose_idx, :2], exploration_map)
                pose_rc_2 = xy_to_rc(best_path[pose_idx + 1, :2], exploration_map)
                self.path_rc = np.vstack((self.path_rc, bresenham2d(pose_rc_1, pose_rc_2)))
                
            self.path_rc = self.path_rc.astype(int)
                
            self.publish_path(best_path)
    
    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = self.world_frame_id
        
        if len(path.shape) == 1:
            quaternion = tf.transformations.quaternion_from_euler(0, 0, path[2])
            
            waypoint = PoseStamped()
            waypoint.header.frame_id = self.world_frame_id
            waypoint.header.stamp = rospy.Time.now()
            waypoint.pose.position.x = path[0]
            waypoint.pose.position.y = path[1]
            waypoint.pose.position.z = 0
            waypoint.pose.orientation.x = quaternion[0]
            waypoint.pose.orientation.y = quaternion[1]
            waypoint.pose.orientation.z = quaternion[2]
            waypoint.pose.orientation.w = quaternion[3]
                
            path_msg.poses.append(waypoint)
        else:
            path = list(path)
        
            path_msg = Path()
            path_msg.header.frame_id = self.world_frame_id
        
            for pose in path:
                quaternion = tf.transformations.quaternion_from_euler(0, 0, pose[2])
              
                waypoint = PoseStamped()
                waypoint.header.frame_id = self.world_frame_id
                waypoint.header.stamp = rospy.Time.now()
                waypoint.pose.position.x = pose[0]
                waypoint.pose.position.y = pose[1]
                waypoint.pose.position.z = 0
                waypoint.pose.orientation.x = quaternion[0]
                waypoint.pose.orientation.y = quaternion[1]
                waypoint.pose.orientation.z = quaternion[2]
                waypoint.pose.orientation.w = quaternion[3]
          
                path_msg.poses.append(waypoint)
        
        self.path_pub.publish(path_msg)
        
        rospy.loginfo("New path published!")
    
    def get_failsafe_path(self, state, occupancy_map):
        path = None
        random_goal, sample_success = self.sample_free_pose(state, occupancy_map)
        if sample_success:
            plan_success, random_path = oriented_astar(start=state,
                                                       occupancy_map=occupancy_map,
                                                       footprint_masks=self.footprint_masks,
                                                       outline_coords=self.footprint_outline_coords,
                                                       obstacle_values=[Costmap.OCCUPIED],
                                                       epsilon=self.planning_epsilon,
                                                       goal=random_goal,
                                                       delta=self.footprint_mask_radius)
        
            if plan_success and random_path.shape[0] > 1:
                path = random_path
        
        return path
    
    def sample_free_pose(self, state, occupancy_map):
        successful = False
        random_pose = None

        free_inds = np.nonzero(occupancy_map.data == Costmap.FREE)
        if free_inds[0].shape[0] == 0:
            return random_pose, successful

        while not successful:
            random_ind = np.random.randint(0, free_inds[0].shape[0])
            random_rc = np.array([free_inds[0][random_ind], free_inds[1][random_ind]])
            random_xy = rc_to_xy(random_rc, occupancy_map)
            random_orientation = wrap_angles(np.arctan2(random_xy[1] - state[1], random_xy[0] - state[0]))
            random_pose = np.array([random_xy[0], random_xy[1], random_orientation])
            if not self.footprint.check_for_collision(random_pose, occupancy_map, unexplored_is_occupied=True):
                successful = True

        return random_pose, successful
    
    def compute_path_score(self, path):
        sampled_path = path[::-self.skip_pose, :]
        score = 0
        for pose in sampled_path:
            origin_point = np.array([pose[0], pose[1], self.sensor_height])
            transfor_mat = np.matmul(self.transform_matrix_2d(pose[0], pose[1], pose[2]), self.T_co)
            end_points = np.matmul(transfor_mat, self.XYZ_vect_hom)[:3, :].T
            RLE_list = self.get_RLE(origin_point, end_points)
            score += self.compute_pose_score_RLE(RLE_list)
        
        # score = score / (1.0 + np.sum(np.linalg.norm(path[1:, :2] - path[:-1, :2], axis=1)))
        score = score / (0.1 + sampled_path.shape[0])
        return score
    
    def get_RLE(self, origin_point, end_points):
        origin_point_ros = Point()
        origin_point_ros.x = origin_point[0]
        origin_point_ros.y = origin_point[1]
        origin_point_ros.z = origin_point[2]
        
        end_points_list_ros = []
        for end_point in end_points:
            end_point_ros = Point()
            end_point_ros.x = end_point[0, 0]
            end_point_ros.y = end_point[0, 1]
            end_point_ros.z = end_point[0, 2]
            end_points_list_ros.append(end_point_ros)
        
        try:
            resp = self.RLE_query(end_points_list_ros, origin_point_ros)
            return resp.RLE_list
        except rospy.ServiceException as e:
            print("RLE Service call failed: %s"%e)
    
    def compute_pose_score_RLE(self, RLE_list):
        pose_score = 0
        for RLE in RLE_list:
            pose_score += self.compute_ray_score(RLE)
        
        return pose_score
    
    def compute_ray_score(self, RLE):
        le_list = RLE.le_list
        ray_score = 0
        a_0 = 1
        b_0 = 0
        for LE in le_list:
            le = LE.le
            w = le[0]
            chi = np.array([0, le[1], le[2], le[3], le[4]])
            pi = softmax(chi)
            pi_w_q = (pi[0]) ** w
            f = self.compute_f(chi)
            
            a = a_0 * pi[1:]
            a_0 *= pi_w_q
            
            b = b_0 + f[:4]
            b_0 += w * f[4]
            
            frac_1 = (1 - pi_w_q) / (1 - pi[0])
            frac_2 = ((w - 1) * pi_w_q * pi[0] - w * pi_w_q + pi[0]) / ((1 - pi[0]) ** 2)
            
            ray_score += np.sum(a * (b * frac_1 + f[4] * frac_2))

        return ray_score
            
    def compute_f(self, chi):
        chi = np.resize(chi, (1, 5))
        chi_rep = np.repeat(chi, 5, axis=0)
        l_tilde = chi_rep + self.l_k
        exp_term = np.exp(l_tilde)
        big_softmax = exp_term / np.sum(exp_term, axis=1)
        f = np.log(np.sum(np.exp(chi)) * big_softmax[:, 0]) + \
            np.sum(big_softmax[:, 1:] * self.l_k[:, 1:], axis=1)
        
        return f


def softmax(l):
    return np.exp(l) / np.sum(np.exp(l))


def extract_frontiers(occupancy_map, approx=True, approx_iters=2,
                      kernel=cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3)),
                      debug=False):
    """
    Given a map of free/occupied/unexplored pixels, identify the frontiers.
    This method is a quicker method than the brute force approach,
    and scales pretty well with large maps. Using cv2.dilate for 1 pixel on the unexplored space
    and comparing with the free space, we can isolate the frontiers.

    :param occupancy_map Costmap: object corresponding to the map to extract frontiers from
    :param approx bool: does an approximation of the map before extracting frontiers. (dilate erode, get rid of single
                   pixel unexplored areas creating a large number fo frontiers)
    :param approx_iters int: number of iterations for the dilate erode
    :param kernel array(N, N)[float]: the kernel of which to use to extract frontiers / approx map
    :param debug bool: show debug windows?
    :return List[array(N, 2][float]: list of frontiers, each frontier is a set of coordinates
    """
    # todo regional frontiers
    # extract coordinates of occupied, unexplored, and free coordinates
    occupied_coords = np.argwhere(occupancy_map.data.astype(np.uint8) == Costmap.OCCUPIED)
    unexplored_coords = np.argwhere(occupancy_map.data.astype(np.uint8) == Costmap.UNEXPLORED)
    free_coords = np.argwhere(occupancy_map.data.astype(np.uint8) == Costmap.FREE)

    if free_coords.shape[0] == 0 or unexplored_coords.shape[0] == 0:
        return []

    # create a binary mask of unexplored pixels, letting unexplored pixels = 1
    unexplored_mask = np.zeros_like(occupancy_map.data)
    unexplored_mask[unexplored_coords[:, 0], unexplored_coords[:, 1]] = 1

    # dilate using a 3x3 kernel, effectively increasing
    # the size of the unexplored space by one pixel in all directions
    dilated_unexplored_mask = cv2.dilate(unexplored_mask, kernel=kernel)
    dilated_unexplored_mask[occupied_coords[:, 0], occupied_coords[:, 1]] = 1

    # create a binary mask of the free pixels
    free_mask = np.zeros_like(occupancy_map.data)
    free_mask[free_coords[:, 0], free_coords[:, 1]] = 1

    # can isolate the frontiers using the difference between the masks,
    # and looking for contours
    frontier_mask = ((1 - dilated_unexplored_mask) - free_mask)
    if approx:
        frontier_mask = cv2.dilate(frontier_mask, kernel=kernel, iterations=approx_iters)
        frontier_mask = cv2.erode(frontier_mask, kernel=kernel, iterations=approx_iters)

    # this indexing will work with opencv 2.x 3.x and 4.x
    frontiers_xy_px = cv2.findContours(frontier_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[-2:][0]
    frontiers = [rc_to_xy(np.array(frontier).squeeze(1)[:, ::-1], occupancy_map) for frontier in frontiers_xy_px]

    if debug:
        frontier_map = np.repeat([occupancy_map.data], repeats=3, axis=0).transpose((1, 2, 0))
        # frontiers = [frontiers[rank] for i, rank in enumerate(frontier_ranks)
        #              if i < self.num_frontiers_considered]
        for frontier in frontiers:
            # if frontier.astype(int).tolist() in self.frontier_blacklist:
            #     continue
            frontier_px = xy_to_rc(frontier, occupancy_map).astype(int)
            frontier_px = frontier_px[which_coords_in_bounds(frontier_px, occupancy_map.get_shape())]
            frontier_map[frontier_px[:, 0], frontier_px[:, 1]] = [255, 0, 0]

        plt.imshow(frontier_map, cmap='gray', interpolation='nearest')
        plt.show()

    return frontiers


def cleanup_map_for_planning(occupancy_map, kernel, filter_obstacles=False, debug=False):
    """
    We are not allowed to plan in unexplored space, (treated as collision), so what we do is dilate/erode the free
    space on the map to eat up the small little unexplored pixels, allows for quicker planning.
    :param occupancy_map Costmap: object
    :param kernel array(N, N)[float]: kernel to use to cleanup map (dilate/erode)
    :param filter_obstacles bool: whether to filter obstacles with a median filter, potentially cleaning up single
                              pixel noise in the environment.
    :param debug bool: show debug plot?
    :return Costmap: cleaned occupancy map
    """
    occupied_coords = np.argwhere(occupancy_map.data == Costmap.OCCUPIED)
    free_coords = np.argwhere(occupancy_map.data == Costmap.FREE)

    free_mask = np.zeros_like(occupancy_map.data)
    free_mask[free_coords[:, 0], free_coords[:, 1]] = 1
    free_mask = cv2.dilate(free_mask, kernel=kernel, iterations=2)
    free_mask = cv2.erode(free_mask, kernel=kernel, iterations=2)
    new_free_coords = np.argwhere(free_mask == 1)

    if filter_obstacles:
        occupied_mask = np.zeros_like(occupancy_map.data)
        occupied_mask[occupied_coords[:, 0], occupied_coords[:, 1]] = 1
        occupied_mask = cv2.medianBlur(occupied_mask, kernel.shape[0])
        occupied_coords = np.argwhere(occupied_mask == 1)

    cleaned_occupancy_map = occupancy_map.copy()
    cleaned_occupancy_map.data[new_free_coords[:, 0], new_free_coords[:, 1]] = Costmap.FREE
    cleaned_occupancy_map.data[occupied_coords[:, 0], occupied_coords[:, 1]] = Costmap.OCCUPIED

    if debug:
        plt.imshow(cleaned_occupancy_map.data, cmap='gray', interpolation='nearest')
        plt.show()

    return cleaned_occupancy_map


def main(args):
    rospy.init_node('semantic_exploration', anonymous=True)
    sem_exp_agent = SemanticExplorationAgent()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Exploration stopped!")


if __name__ == '__main__':
    main(sys.argv)

