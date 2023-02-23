#!/usr/bin/env python
from __future__ import division
from __future__ import print_function

import sys
import rospy
import numpy as np
import cv2
import message_filters
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from skimage.transform import resize
from sensor_msgs.msg import PointCloud2
from semantic_sensor import PointType, SemanticPclGenerator


class SemanticCloud:
    """
    Class for ros node to take in a color image (bgr) and a semantic segmentation image (bgr)
    Then produce point cloud based on depth information
    """
    def __init__(self):
        # Get point type
        point_type = rospy.get_param('/semantic_pcl/point_type')
        if point_type == 0:
            self.point_type = PointType.SEMANTIC
            print('Generate semantic point cloud.')
        else:
            print("Invalid point type.")
            return
        # Get image size
        self.img_width, self.img_height = rospy.get_param('/camera/width'), rospy.get_param('/camera/height')
        # Set up ROS
        self.bridge = CvBridge() # CvBridge to transform ROS Image message to OpenCV image
        # Set up ros image subscriber
        # Set buff_size to average msg size to avoid accumulating delay
        # Point cloud frame id
        frame_id = rospy.get_param('/semantic_pcl/frame_id')
        # Camera intrinsic matrix
        fx = rospy.get_param('/camera/fx')
        fy = rospy.get_param('/camera/fy')
        cx = rospy.get_param('/camera/cx')
        cy = rospy.get_param('/camera/cy')
        intrinsic = np.matrix([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype = np.float32)
        
        # Noise configuration
        self.noisy_obs = False
        self.depth_noise_std = rospy.get_param('/semantic_pcl/depth_noise_std')
        self.true_class_prob = rospy.get_param('/semantic_pcl/true_class_prob')
        if self.depth_noise_std > 0 or self.true_class_prob < 1:
            self.noisy_obs = True
            num_classes = rospy.get_param('/class_labels/num_classes')
            self.class_colors = []
            for i in range(num_classes):
                color = rospy.get_param('/class_labels/color_' + str(i + 1))
                self.class_colors.append(255 * np.array([color['b'], color['g'], color['r']]))
            self.class_colors = np.array(self.class_colors).astype(np.uint8)
        
        self.pcl_pub = rospy.Publisher("/semantic_pcl/semantic_pcl", PointCloud2, queue_size = 1)

        # increase buffer size to avoid delay (despite queue_size = 1)
        self.color_sub = message_filters.Subscriber(rospy.get_param('/semantic_pcl/color_image_topic'), Image,
                                                    queue_size = 1, buff_size = 30*self.img_width*self.img_height)
        self.semantic_sub = message_filters.Subscriber(rospy.get_param('/semantic_pcl/semantic_image_topic'), Image,
                                                       queue_size = 1, buff_size = 30*self.img_width*self.img_height)
        self.depth_sub = message_filters.Subscriber(rospy.get_param('/semantic_pcl/depth_image_topic'), Image,
                                                    queue_size = 1, buff_size = 30*self.img_width*self.img_height)

        # Take in color image, semantic image, and depth image with a limited time gap between message time stamps
        self.ts = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.semantic_sub, self.depth_sub],
                                                              queue_size = 1, slop = 0.3)
        self.ts.registerCallback(self.color_semantic_depth_callback)
        self.cloud_generator = SemanticPclGenerator(intrinsic, self.img_width,self.img_height, frame_id,
                                                    self.point_type)
        print('Semantic point cloud ready!')

    def color_semantic_depth_callback(self, color_img_ros, semantic_img_ros , depth_img_ros):
        """
        Callback function to produce point cloud registered with semantic class color based
        on input color image and depth image
        """
        # Convert ros Image message to numpy array
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_img_ros, "bgr8")
            semantic_img = self.bridge.imgmsg_to_cv2(semantic_img_ros, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_img_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Resize depth
        if depth_img.shape[0] is not self.img_height or depth_img.shape[1] is not self.img_width:
            depth_img = resize(depth_img, (self.img_height, self.img_width), order = 0, mode = 'reflect',
                               anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
            depth_img = depth_img.astype(np.float32)

        # Resize semantic
        if semantic_img.shape[0] is not self.img_height or semantic_img.shape[1] is not self.img_width:
            semantic_img = resize(semantic_img, (self.img_height, self.img_width), order = 0, mode = 'reflect',
                                  anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
            semantic_img = semantic_img.astype(np.uint8)
        
        # Add noise
        if self.noisy_obs is True:
            depth_img, semantic_img = self.add_noise(depth_img, semantic_img)
        
        if self.point_type == PointType.SEMANTIC:
            cloud_ros = self.cloud_generator.generate_cloud_semantic(color_img, semantic_img, depth_img,
                                                                     color_img_ros.header.stamp)
        else:
            print('Point type not supported!')

        # Publish point cloud
        self.pcl_pub.publish(cloud_ros)

    def add_noise(self, depth_img, semantic_img):
        noisy_depth_img = depth_img + np.random.normal(0, self.depth_noise_std, depth_img.shape).astype(np.float32)
        np.place(noisy_depth_img, depth_img == 0, 0)
        error_mask = np.random.sample(size=semantic_img.shape[:2]) > self.true_class_prob
        random_classes = np.random.choice(self.class_colors.shape[0],size=np.count_nonzero(error_mask))
        error_mask = np.repeat(error_mask[:,:,None],repeats=3,axis=2)
        np.place(semantic_img, error_mask, self.class_colors[random_classes, :])
        
        return noisy_depth_img, semantic_img
        

def main(args):
    rospy.init_node('semantic_cloud', anonymous=True)
    sem_cloud = SemanticCloud()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Semantic cloud shutting down!")


if __name__ == '__main__':
    main(sys.argv)
