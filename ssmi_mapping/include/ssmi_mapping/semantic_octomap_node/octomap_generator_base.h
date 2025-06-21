#ifndef SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_BASE_H
#define SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_BASE_H

#include <octomap/octomap.h>
#include "ssmi_interface/msg/ray_rle.hpp"

template<class OCTREE>
class OctomapGeneratorBase
{
public:

    /// Destructor
    virtual ~OctomapGeneratorBase(){};

    /// Set max range for point cloud insertion
    virtual void setMaxRange(float max_range) = 0;

    /// Set max range to perform raycasting on inserted points
    virtual void setRayCastRange(float raycast_range) = 0;

    /// Set clamping_thres_min, parameter for octomap
    virtual void setClampingThresMin(float clamping_thres_min) = 0;

    /// Set clamping_thres_max, parameter for octomap
    virtual void setClampingThresMax(float clamping_thres_max) = 0;

    /// Set resolution, parameter for octomap
    virtual void setResolution(float resolution) = 0;

    /// Set occupancy_thres, parameter for octomap
    virtual void setOccupancyThres(float occupancy_thres) = 0;

    /// Set prob_hit, parameter for octomap
    virtual void setProbHit(float prob_hit) = 0;

    /// Set prob_miss, parameter for octomap
    virtual void setProbMiss(float prob_miss) = 0;

    /// Set phi, parameter for semantic octomap
    virtual void setPhi(float phi) = 0;

    /// Set psi, parameter for semantic octomap
    virtual void setPsi(float psi) = 0;

    /**
     * \brief Insert point cloud into octree
     * \param cloud converted ros cloud to be inserted
     * \param sensorToWorld transform from sensor frame to world frame
     */
    virtual void insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensorToWorld) = 0;
    
    virtual bool get_ray_RLE(const octomap::point3d& origin, const octomap::point3d& end, ssmi_interface::msg::RayRLE& rayRLE_msg) = 0;

    /// Set whether use semantic color for serialization
    virtual void setUseSemanticColor(bool use) = 0;

    /// Get whether use semantic color for serialization
    virtual bool isUseSemanticColor() = 0;
    
    /// Set whether write semantics for serialization
    virtual void setWriteSemantics(bool write) = 0;

    /// Get whether write semantics for serialization
    virtual bool doesWriteSemantics() = 0;


    /// Get octree
    virtual OCTREE* getOctree() = 0;

    /// Save octomap to a file. NOTE: Not tested
    virtual bool save(const char* filename) const = 0;
};

#endif //SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_BASE_H
