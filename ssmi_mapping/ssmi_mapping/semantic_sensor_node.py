#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from skimage.transform import resize
from sensor_msgs.msg import Image, PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ssmi_mapping.semantic_sensor import PointType, SemanticPclGenerator

class SemanticCloudNode(Node):
    def __init__(self):
        super().__init__('semantic_cloud')

        self.declare_parameter('semantic_pcl.point_type', 0)
        pt = self.get_parameter('semantic_pcl.point_type').value
        if pt == 0:
            self.point_type = PointType.SEMANTIC
            self.get_logger().info('Generate semantic point cloud.')
        else:
            self.get_logger().error('Invalid point type.')
            return

        self.declare_parameter('semantic_pcl.unit_conversion', 1.0)
        unit_conversion = self.get_parameter('semantic_pcl.unit_conversion').value

        self.declare_parameter('semantic_pcl.color_image_topic', '/camera/color/image_raw')
        self.declare_parameter('semantic_pcl.semantic_image_topic', '/camera/semantic/image_raw')
        self.declare_parameter('semantic_pcl.depth_image_topic', '/camera/depth/image_raw')
        self.declare_parameter('semantic_pcl.frame_id', 'camera_optic')
        frame_id = self.get_parameter('semantic_pcl.frame_id').value

        self.declare_parameter('camera.width', 640)
        self.declare_parameter('camera.height', 480)
        self.declare_parameter('camera.fx', 320.0)
        self.declare_parameter('camera.fy', 320.0)
        self.declare_parameter('camera.cx', 320.0)
        self.declare_parameter('camera.cy', 240.0)

        w = self.get_parameter('camera.width').value
        h = self.get_parameter('camera.height').value
        fx = self.get_parameter('camera.fx').value
        fy = self.get_parameter('camera.fy').value
        cx = self.get_parameter('camera.cx').value
        cy = self.get_parameter('camera.cy').value
        self.img_width, self.img_height = w, h

        intrinsic = np.matrix([[fx, 0, cx],
                              [0, fy, cy],
                              [0,  0,  1]], dtype=np.float32)

        self.declare_parameter('octomap.pointcloud_topic', '/pointcloud')
        self.declare_parameter('octomap.world_frame_id', 'env_small')
        pointcloud_topic = self.get_parameter('octomap.pointcloud_topic').value

        # —— Publisher & Subscriber —— 
        self.pcl_pub = self.create_publisher(
            PointCloud2,
            pointcloud_topic,
            QoSProfile(depth=1)
        )

        qos = QoSProfile(depth=1)

        # If has bug here please set max_message_size to 30 * w * h 
        # NOTE: ROS2 does not support setting max_message_size in qos_profile, must use other method
        color_sub = Subscriber(
            self, Image,
            self.get_parameter('semantic_pcl.color_image_topic').value,
            qos_profile=qos)

        sem_sub = Subscriber(
            self, Image,
            self.get_parameter('semantic_pcl.semantic_image_topic').value,
            qos_profile=qos)

        depth_sub = Subscriber(
            self, Image,
            self.get_parameter('semantic_pcl.depth_image_topic').value,
            qos_profile=qos)

        ats = ApproximateTimeSynchronizer([color_sub, sem_sub, depth_sub],
                                          queue_size=1, slop=0.3)
        ats.registerCallback(self.cb)

        self.bridge = CvBridge()
        self.cloud_gen = SemanticPclGenerator(
            intrinsic, w, h, unit_conversion, frame_id, self.point_type)

        self.get_logger().info('SemanticCloudNode ready!')

    def cb(self, color_msg, sem_msg, depth_msg):
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            sem   = self.bridge.imgmsg_to_cv2(sem_msg,   'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        if depth.shape[:2] != (self.img_height, self.img_width):
            depth = resize(depth, (self.img_height, self.img_width),
                           order=0, mode='reflect',
                           anti_aliasing=False,
                           preserve_range=True).astype(np.float32)
        if sem.shape[:2] != (self.img_height, self.img_width):
            sem = resize(sem, (self.img_height, self.img_width),
                         order=0, mode='reflect',
                         anti_aliasing=False,
                         preserve_range=True).astype(np.uint8)

        cloud = self.cloud_gen.generate_cloud_semantic(
            color, sem, depth, color_msg.header.stamp)
        self.pcl_pub.publish(cloud)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down SemanticCloudNode')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
