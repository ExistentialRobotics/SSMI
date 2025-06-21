#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class PoseToTF(Node):
    def __init__(self):
        super().__init__('pose_to_tf_py')
        self.declare_parameter('agent_name', 'husky_1')
        self.agent_name = self.get_parameter('agent_name').get_parameter_value().string_value
        self.declare_parameter('pose_topic_suffix', '/pose')
        topic = self.agent_name + self.get_parameter('pose_topic_suffix').get_parameter_value().string_value
        self.declare_parameter('frame_suffix', '/base_link')
        self.child_frame = self.agent_name + self.get_parameter('frame_suffix').get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            PoseStamped,
            topic,
            self.pose_callback,
            1
        )
        self.get_logger().info(f"Subscribed to '{topic}' → broadcasting as TF frame '{self.child_frame}'")

    def pose_callback(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = self.child_frame
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation    = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
