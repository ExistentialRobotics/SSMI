#ifndef SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_ROS2_HPP
#define SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_ROS2_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "ssmi_mapping/semantic_octomap_node/octomap_generator.h"
#include "std_srvs/srv/empty.hpp"
#include "ssmi_interface/srv/get_rle.hpp"
#include "ssmi_interface/msg/ray_rle.hpp"
#include "octomap/octomap_types.h"
#include "octomap/Pointcloud.h"
#include "octomap/octomap.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "message_filters/subscriber.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "tf2_ros/message_filter.h"
// #include "message_filters/chain.h"

class OctomapGeneratorNode : public rclcpp::Node
{
public:
  explicit OctomapGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~OctomapGeneratorNode() override = default;

  void reset();

  void insertCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud);

  void publish2DOccupancyMap(const SemanticOctree * octomap,
                             const rclcpp::Time & stamp,
                             const std::string & frame_id);

  bool save(const char * filename) const;

protected:
  OctomapGeneratorBase<SemanticOctree>* octomap_generator_; ///<Octomap instance pointer

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr toggle_color_service_;
  rclcpp::Service<ssmi_interface::srv::GetRLE>::SharedPtr RLE_service_;

  // Service callbacks
  void toggleUseSemanticColor(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void querry_RLE(
    const std::shared_ptr<ssmi_interface::srv::GetRLE::Request> request,
    std::shared_ptr<ssmi_interface::srv::GetRLE::Response> response);

  // Publishers
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr fullmap_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr colormap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_map_pub_;

  // TF and filters
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> * pointcloud_sub_;
  tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2> * tf_pointcloud_filter_;

  // Parameters
  std::string world_frame_id_;
  std::string pointcloud_topic_;
  float max_range_;
  float raycast_range_;
  float clamping_thres_max_;
  float clamping_thres_min_;
  float psi_;
  float phi_;
  float resolution_;
  float occupancy_thres_;
  float prob_hit_;
  float prob_miss_;
  bool publish_2d_map_;
  double min_ground_z_;
  double max_ground_z_;

  octomap_msgs::msg::Octomap map_msg_;
};

#endif // SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_ROS2_HPP

  // {
    // Initialize octomap generator
    // Services
    // toggle_color_service_ = this->create_service<std_srvs::srv::Empty>(
    //   "/toggle_semantic_color",
    //   std::bind(&OctomapGeneratorNode::toggleUseSemanticColor, this, std::placeholders::_1, std::placeholders::_2));

    // RLE_service_ = this->create_service<ssmi_interface::srv::GetRle>(
    //   "/get_rle",
    //   std::bind(&OctomapGeneratorNode::querry_RLE, this, std::placeholders::_1, std::placeholders::_2));

    // Subscriber with TF filter

    // pointcloud_sub_ = new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(this, pointcloud_topic_);
    // tf_pointcloud_filter_ = new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>(
    //   *pointcloud_sub_, tf_buffer_, world_frame_id_, 10, this);
    // tf_pointcloud_filter_->registerCallback(
      // std::bind(&OctomapGeneratorNode::insertCloudCallback, this, std::placeholders::_1));
  // }