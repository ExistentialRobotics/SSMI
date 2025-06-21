#include <ssmi_mapping/semantic_octomap_node/octomap_generator_ros.h>
#include <pcl_ros/impl/transforms.hpp>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/msg/occupancy_grid.h>
#include <pcl/conversions.h>
#include <chrono>
#include <cmath>
#include <sstream>
#include <cstring> // For std::memcpy
#include <tf2_eigen/tf2_eigen.hpp>
#include "octomap_msgs/msg/octomap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


OctomapGeneratorNode::OctomapGeneratorNode(const rclcpp::NodeOptions & options): Node("octomap_generator", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
{
    // Initiate octree
    RCLCPP_INFO(this->get_logger(), "Semantic octomap generated!");
    octomap_generator_ = new OctomapGenerator<PCLSemantics, SemanticOctree>();
    toggle_color_service_ = this->create_service<std_srvs::srv::Empty>("toggle_use_semantic_color", std::bind(&OctomapGeneratorNode::toggleUseSemanticColor, this, std::placeholders::_1, std::placeholders::_2));
    RLE_service_ = this->create_service<ssmi_interface::srv::GetRLE>("querry_RLE", std::bind(&OctomapGeneratorNode::querry_RLE, this, std::placeholders::_1, std::placeholders::_2));

    reset();
    fullmap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_full", 1);
    colormap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_color", 1);
    occ_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_map_2D", 1);
    pointcloud_sub_ = new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(this, pointcloud_topic_);
    pointcloud_sub_->registerCallback(std::bind(&OctomapGeneratorNode::insertCloudCallback, this, std::placeholders::_1));
}
/// Clear octomap and reset values to paramters from parameter server
void OctomapGeneratorNode::reset()
{

    // this->declare_parameter("octomap.pointcloud_topic", rclcpp::PARAMETER_STRING);
    // this->declare_parameter("octomap.world_frame_id",   rclcpp::PARAMETER_STRING);
    // this->declare_parameter("octomap.resolution",       rclcpp::PARAMETER_DOUBLE);
    // this->declare_parameter("octomap.max_range",        rclcpp::PARAMETER_DOUBLE);
    // this->declare_parameter("octomap.raycast_range",    rclcpp::PARAMETER_DOUBLE);
    // this->declare_parameter("octomap.clamping_thres_min", rclcpp::PARAMETER_DOUBLE);
    // this->declare_parameter("octomap.clamping_thres_max", rclcpp::PARAMETER_DOUBLE);
    // this->declare_parameter("octomap.occupancy_thres",  rclcpp::PARAMETER_DOUBLE);
    // this->declare_parameter("octomap.prob_hit",         rclcpp::PARAMETER_DOUBLE);
    // this->declare_parameter("octomap.prob_miss",        rclcpp::PARAMETER_DOUBLE);
    // this->declare_parameter("octomap.psi",              rclcpp::PARAMETER_DOUBLE);
    // this->declare_parameter("octomap.phi",              rclcpp::PARAMETER_DOUBLE);
    // this->declare_parameter("octomap.publish_2d_map",   rclcpp::PARAMETER_BOOL);
    // this->declare_parameter("octomap.min_ground_z",     rclcpp::PARAMETER_DOUBLE);
    // this->declare_parameter("octomap.max_ground_z",     rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("octomap.pointcloud_topic", "pointcloud");
    this->declare_parameter("octomap.world_frame_id",   "env_small");
    this->declare_parameter("octomap.resolution",       0.5);
    this->declare_parameter("octomap.max_range",        15.0);
    this->declare_parameter("octomap.raycast_range",    10.0);
    this->declare_parameter("octomap.clamping_thres_min", 1e-4);  
    this->declare_parameter("octomap.clamping_thres_max", 1-1e-4);
    this->declare_parameter("octomap.occupancy_thres", 0.5);
    this->declare_parameter("octomap.prob_hit",         0.7);
    this->declare_parameter("octomap.prob_miss",        0.4);
    this->declare_parameter("octomap.psi",              1.0);
    this->declare_parameter("octomap.phi",              -0.1);
    this->declare_parameter("octomap.publish_2d_map",   true);
    this->declare_parameter("octomap.min_ground_z",     1.0);
    this->declare_parameter("octomap.max_ground_z",     3.5);

    this->get_parameter("octomap.pointcloud_topic", pointcloud_topic_);
    this->get_parameter("octomap.world_frame_id",   world_frame_id_);
    this->get_parameter("octomap.resolution",       resolution_);
    this->get_parameter("octomap.max_range",        max_range_);
    this->get_parameter("octomap.raycast_range",    raycast_range_);
    this->get_parameter("octomap.clamping_thres_min", clamping_thres_min_);
    this->get_parameter("octomap.clamping_thres_max", clamping_thres_max_);
    this->get_parameter("octomap.occupancy_thres",  occupancy_thres_);
    this->get_parameter("octomap.prob_hit",         prob_hit_);
    this->get_parameter("octomap.prob_miss",        prob_miss_);
    this->get_parameter("octomap.psi",              psi_);
    this->get_parameter("octomap.phi",              phi_);
    this->get_parameter("octomap.publish_2d_map",   publish_2d_map_);
    this->get_parameter("octomap.min_ground_z",     min_ground_z_);
    this->get_parameter("octomap.max_ground_z",     max_ground_z_);

    octomap_generator_->setClampingThresMin(clamping_thres_min_);
    octomap_generator_->setClampingThresMax(clamping_thres_max_);
    octomap_generator_->setResolution(resolution_);
    octomap_generator_->setOccupancyThres(occupancy_thres_);
    octomap_generator_->setProbHit(prob_hit_);
    octomap_generator_->setProbMiss(prob_miss_);
    octomap_generator_->setPsi(psi_);
    octomap_generator_->setPhi(phi_);
    octomap_generator_->setRayCastRange(raycast_range_);
    octomap_generator_->setMaxRange(max_range_);
}

void OctomapGeneratorNode::toggleUseSemanticColor(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    octomap_generator_->setUseSemanticColor(!octomap_generator_->isUseSemanticColor());
    if(octomap_generator_->isUseSemanticColor())
        RCLCPP_INFO(this->get_logger(), "Using semantic color");
    else
        RCLCPP_INFO(this->get_logger(), "Using rgb color");
    
    octomap_generator_->setWriteSemantics(true);
    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
        fullmap_pub_->publish(map_msg_);
    else
        RCLCPP_ERROR(this->get_logger(), "Error serializing full OctoMap");
    
    octomap_generator_->setWriteSemantics(false);
    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
        colormap_pub_->publish(map_msg_);
    else
        RCLCPP_ERROR(this->get_logger(), "Error serializing color OctoMap");
}

void OctomapGeneratorNode::querry_RLE(const std::shared_ptr<ssmi_interface::srv::GetRLE::Request> request,
    std::shared_ptr<ssmi_interface::srv::GetRLE::Response> response)
{
    const octomap::point3d origin(request->origin.x, request->origin.y, request->origin.z);

    for (int i = 0; i < (int)request->end_points.size(); ++i)
    {   
        const octomap::point3d endPoint(request->end_points[i].x, request->end_points[i].y, request->end_points[i].z);
        ssmi_interface::msg::RayRLE rayRLE_msg;
        // octomap_generator_->get_ray_RLE(origin, endPoint, rayRLE_msg);
        // response->rle_list.push_back(rayRLE_msg);
        if (octomap_generator_->get_ray_RLE(origin, endPoint, rayRLE_msg))
        {          
            response->rle_list.push_back(rayRLE_msg);
        } 
    }
}


void OctomapGeneratorNode::insertCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg) 
{
    // Downsample / convert to PCL internal type
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Lookup transform from sensor frame to world frame
    geometry_msgs::msg::TransformStamped sensorToWorldTf;
    try {
      sensorToWorldTf = tf_buffer_.lookupTransform(
        world_frame_id_,                    
        cloud_msg->header.frame_id,         
        cloud_msg->header.stamp,            
        rclcpp::Duration(std::chrono::seconds(1))                
      );
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(),
                   "Transform error of sensor data: %s", ex.what());
      return;
    }

    // Convert TransformStamped to Eigen::Matrix4f
    Eigen::Isometry3d iso = tf2::transformToEigen(sensorToWorldTf);
    Eigen::Matrix4f sensorToWorld = iso.matrix().cast<float>();

    // Insert into OctoMap
    octomap_generator_->insertPointCloud(cloud, sensorToWorld);

    // Publish full-probability OctoMap
    octomap_msgs::msg::Octomap map_msg;
    map_msg.header.frame_id = world_frame_id_;
    map_msg.header.stamp = cloud_msg->header.stamp;

    octomap_generator_->setWriteSemantics(true);
    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg)) {
      fullmap_pub_->publish(map_msg);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error serializing full OctoMap");
    }

    // Optionally publish color OctoMap
    octomap_generator_->setWriteSemantics(false);
    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg)) {
      colormap_pub_->publish(map_msg);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error serializing color OctoMap");
    }
    // Publish 2D occupancy map
    if (publish_2d_map_) {
        rclcpp::Time ts(cloud_msg->header.stamp);
        publish2DOccupancyMap(octomap_generator_->getOctree(), ts, world_frame_id_);
    }
}

void OctomapGeneratorNode::publish2DOccupancyMap(const SemanticOctree* octomap,
                                                 const rclcpp::Time& stamp,
                                                 const std::string& frame_id)
{
    // get dimensions of octree
    double minX, minY, minZ, maxX, maxY, maxZ;
    octomap->getMetricMin(minX, minY, minZ);
    octomap->getMetricMax(maxX, maxY, maxZ);
    octomap::point3d minPt = octomap::point3d(minX, minY, minZ);

    unsigned int tree_depth = octomap->getTreeDepth();

    octomap::OcTreeKey paddedMinKey = octomap->coordToKey(minPt);

    nav_msgs::msg::OccupancyGrid::Ptr occupancy_map (new nav_msgs::msg::OccupancyGrid());

    unsigned int width, height;
    double res;

    unsigned int ds_shift = tree_depth-16;

    occupancy_map->header.stamp = stamp;
    occupancy_map->header.frame_id = frame_id;
    occupancy_map->info.resolution = res = octomap->getNodeSize(16);
    occupancy_map->info.width = width = (maxX-minX) / res + 1;
    occupancy_map->info.height = height = (maxY-minY) / res + 1;
    occupancy_map->info.origin.position.x = minX  - (res / (float)(1<<ds_shift) ) + res;
    occupancy_map->info.origin.position.y = minY  - (res / (float)(1<<ds_shift) );

    occupancy_map->data.clear();
    occupancy_map->data.resize(width*height, -1);

      // traverse all leafs in the tree:
    unsigned int treeDepth = std::min<unsigned int>(16, octomap->getTreeDepth());
    for (typename SemanticOctree::iterator it = octomap->begin(treeDepth), end = octomap->end(); it != end; ++it)
    {
    
      double node_z = it.getZ();
      double node_half_side = pow(it.getSize(), 1/3) / 2;
      double top_side = node_z + node_half_side;
      double bottom_side = node_z - node_half_side;
      
      if((bottom_side >= min_ground_z_ && bottom_side <= max_ground_z_) ||
        (top_side >= min_ground_z_ && top_side <= max_ground_z_) ||
        (bottom_side <= min_ground_z_ && top_side >= max_ground_z_))
      {
        bool occupied = octomap->isNodeOccupied(*it);
        int intSize = 1 << (16 - it.getDepth());

        octomap::OcTreeKey minKey=it.getIndexKey();

        for (int dx = 0; dx < intSize; dx++)
        {
          for (int dy = 0; dy < intSize; dy++)
          {
            int posX = std::max<int>(0, minKey[0] + dx - paddedMinKey[0]);
            posX>>=ds_shift;

            int posY = std::max<int>(0, minKey[1] + dy - paddedMinKey[1]);
            posY>>=ds_shift;

            int idx = width * posY + posX;

            if (occupied)
              occupancy_map->data[idx] = 100;
            else if (occupancy_map->data[idx] == -1)
            {
              occupancy_map->data[idx] = 0;
            }

          }
        }
      }
    }

    occ_map_pub_->publish(*occupancy_map);
}

bool OctomapGeneratorNode::save(const char* filename) const
{
    return octomap_generator_->save(filename);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctomapGeneratorNode>());
    rclcpp::shutdown();

    return 0;
}
