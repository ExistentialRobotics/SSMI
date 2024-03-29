#ifndef SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_ROS_H
#define SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <semantic_octomap_node/octomap_generator.h>
#include <std_srvs/Empty.h>
#include <semantic_octomap/GetRLE.h>
#include <semantic_octomap/RayRLE.h>
#include <octomap/octomap_types.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <string>
#include <octomap_msgs/Octomap.h>

class OctomapGeneratorNode{
public:
    /**
     * \brief Constructor
     * \param nh The ros node handler to be used in OctomapGenerator
     */
    OctomapGeneratorNode(ros::NodeHandle& nh);
    /// Desturctor
    virtual ~OctomapGeneratorNode();
    /// Reset values to paramters from parameter server
    void reset();
    /**
     * \brief Callback to point cloud topic. Update the octomap and publish it in ROS
     * \param cloud ROS Pointcloud2 message in arbitrary frame (specified in the clouds header)
     */
    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    
    void publish2DOccupancyMap(const SemanticOctree* octomap,
                               const ros::Time& stamp,
                               const std::string& frame_id);
    
    /**
     * \brief Save octomap to a file. NOTE: Not tested
     * \param filename The output filename
     */
    bool save(const char* filename) const;

protected:
    OctomapGeneratorBase<SemanticOctree>* octomap_generator_; ///<Octomap instance pointer
    ros::ServiceServer toggle_color_service_;  ///<ROS service to toggle semantic color display
    ros::ServiceServer RLE_service_;  ///<ROS service to querry RLE values
    bool toggleUseSemanticColor(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response); ///<Function to toggle whether write semantic color or rgb color as when serializing octree
    bool querry_RLE(semantic_octomap::GetRLE::Request& request, semantic_octomap::GetRLE::Response& response);
    ros::NodeHandle nh_; ///<ROS handler
    ros::Publisher fullmap_pub_; ///<ROS publisher for full octomap message
    ros::Publisher colormap_pub_; ///<ROS publisher for color octomap message
    ros::Publisher occ_map_pub_; ///<ROS publisher for 2D occupancy map message
    message_filters::Subscriber<sensor_msgs::PointCloud2>* pointcloud_sub_; ///<ROS subscriber for pointcloud message
    tf::MessageFilter<sensor_msgs::PointCloud2>* tf_pointcloud_sub_; ///<ROS tf message filter to sychronize the tf and pointcloud messages
    tf::TransformListener tf_listener_; ///<Listener for the transform between the camera and the world coordinates
    std::string world_frame_id_; ///<Id of the world frame
    std::string pointcloud_topic_; ///<Topic name for subscribed pointcloud message
    float max_range_; ///<Max range for points to be inserted into octomap
    float raycast_range_; ///<Max range for points to perform raycasting to free unoccupied space
    float clamping_thres_max_; ///<Upper bound of occupancy probability for a node
    float clamping_thres_min_; ///<Lower bound of occupancy probability for a node
    float psi_; ///<Increment update value for a semantic class
    float phi_; ///<Decrement update value for a semantic class
    float resolution_; ///<Resolution of octomap
    float occupancy_thres_; ///<Minimum occupancy probability for a node to be considered as occupied
    float prob_hit_;  ///<Hit probability of sensor
    float prob_miss_; ///<Miss probability of sensor
    bool publish_2d_map;
    double min_ground_z;
    double max_ground_z;
    octomap_msgs::Octomap map_msg_; ///<ROS octomap message
};

#endif //SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_ROS_H
