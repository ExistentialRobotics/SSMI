#!/usr/bin/env python
from __future__ import division
from __future__ import print_function

import sys
import time
import rospy
import tf
import message_filters

import numpy as np

from nav_msgs.msg import Path
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped


class PathNavigation:

    def __init__(self):
        self.alpha = rospy.get_param('/planning/interpolation/alpha')
        self.SE2_radius = rospy.get_param('/planning/interpolation/radius')
        self.world_frame_id = rospy.get_param('/octomap/world_frame_id')
        
        self.dist = rospy.get_param('/planning/goal_check_radius')
        
        self.path_sub = message_filters.Subscriber('/planner/path', Path, queue_size = 1)
        self.path_sub.registerCallback(self.path_callback)
        
        self.collision_sub = message_filters.Subscriber('/planner/collision', Bool, queue_size = 1)
        self.collision_sub.registerCallback(self.collision_callback)
        
        self.collision = False
        self.is_tracking = False
        
        self.position_cmd_pub = rospy.Publisher("/planner/position_cmd", PoseStamped, queue_size = 1)
        
        self.world_frame_id = 'world'
        self.robot_frame_id = '/husky/base'
        self.tf_listener = tf.TransformListener()

    def get_pose_from_tf(self, from_frame_id):
        (translation, rotation) = self.tf_listener.lookupTransform(self.world_frame_id,
                                                                   from_frame_id,
                                                                   rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rotation)
        return np.array([translation[0], translation[1], euler[2]])

    def path_callback(self, path_msg):
        rospy.loginfo("Received a path from exploration algorithm!")
        if self.is_tracking:
            self.collision = True
        
        traj = [path_msg.poses[0].pose]
        
        for path_waypoint in path_msg.poses[1:]:
            interpolated_pose = self.interpolate(traj[-1], path_waypoint.pose)
            while not self.close_enough(interpolated_pose, path_waypoint.pose):
                traj.append(interpolated_pose)
                interpolated_pose = self.interpolate(traj[-1], path_waypoint.pose)
            
            traj.append(path_waypoint.pose)
        
        self.publish_traj(traj)

    def collision_callback(self, collision_msg):
	      self.collision = collision_msg.data

    def publish_traj(self, traj):
        self.is_tracking = True
        self.collision = False
        for pose in traj:
            while True:
                if self.collision:
                    break
                robot_pose = self.get_pose_from_tf(self.robot_frame_id)
                dist = np.sqrt((robot_pose[0] - pose.position.x)**2 + (robot_pose[1] - pose.position.y)**2)
                if dist < self.dist:
                    break
                
                position_cmd_msg = PoseStamped()
                position_cmd_msg.pose = pose
                position_cmd_msg.header.frame_id = self.world_frame_id
                position_cmd_msg.header.stamp = rospy.Time.now()

                self.position_cmd_pub.publish(position_cmd_msg)
                rospy.sleep(0.5)
        self.is_tracking = False
    
    def interpolate(self, pose_1, pose_2):
        xi_1 = self.get_se2_from_pose_msg(pose_1)
        xi_2 = self.get_se2_from_pose_msg(pose_2)
        
        T_1 = self.se2_to_SE2(xi_1)
        T_2 = self.se2_to_SE2(xi_2)
        
        T_12 = np.matmul(np.linalg.inv(T_1), T_2)
        xi_12 = self.SE2_to_se2(T_12)
        T = np.matmul(T_1, self.se2_to_SE2(self.alpha * xi_12))
        xi = self.SE2_to_se2(T)
        
        interploted_pose = Pose()
        interploted_pose.position.x = xi[0]
        interploted_pose.position.y = xi[1]
        interploted_pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, xi[2])
        interploted_pose.orientation.x = quaternion[0]
        interploted_pose.orientation.y = quaternion[1]
        interploted_pose.orientation.z = quaternion[2]
        interploted_pose.orientation.w = quaternion[3]
        
        return interploted_pose
        
    def close_enough(self, pose_1, pose_2):
        xi_1 = self.get_se2_from_pose_msg(pose_1)
        xi_2 = self.get_se2_from_pose_msg(pose_2)
        
        T_1 = self.se2_to_SE2(xi_1)
        T_2 = self.se2_to_SE2(xi_2)
        
        T_12 = np.matmul(np.linalg.inv(T_1), T_2)
        xi_12 = self.SE2_to_se2(T_12)
        
        distance = np.linalg.norm(xi_12)
        
        if distance < self.SE2_radius:
            return True
        else:
            return False
        
    @staticmethod
    def get_se2_from_pose_msg(pose_msg):
        x = pose_msg.position.x
        y = pose_msg.position.y
        theta = tf.transformations.euler_from_quaternion([pose_msg.orientation.x,
                                                          pose_msg.orientation.y,
                                                          pose_msg.orientation.z,
                                                          pose_msg.orientation.w])[2]
        return np.array([x, y, theta])
        
    @staticmethod
    def se2_to_SE2(xi):
        sin = np.sin(xi[2])
        cos = np.cos(xi[2])
        
        if xi[2] != 0:
            V = 1 / xi[2] * np.array([[sin, cos - 1], [1 - cos, sin]])
        else:
            V = np.eye(2)
        
        Vu = np.matmul(V, xi[:2, None]).squeeze()
        
        T = np.array([[cos, -sin, Vu[0]],
                      [sin,  cos, Vu[1]],
                      [0,    0,   1]])
        
        return T
    
    @staticmethod
    def SE2_to_se2(T):
        theta = np.arctan2(T[1, 0], T[0, 0])
        if theta != 0:
            A = T[1, 0] / theta
            B = (1 - T[0, 0]) / theta
            V_inv = np.array([[A, B], [-B, A]]) / (A**2 + B**2)
        else:
            V_inv = np.eye(2)
        
        u = np.matmul(V_inv, T[:2, 2])
        
        xi = np.array([u[0], u[1], theta])
        
        return xi
            

def main(args):
    rospy.init_node('path_navigation', anonymous=True)
    path_navigation = PathNavigation()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Navigation stopped!")


if __name__ == '__main__':
    main(sys.argv)
