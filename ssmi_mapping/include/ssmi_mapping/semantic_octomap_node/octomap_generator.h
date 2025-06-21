#ifndef SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_H
#define SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_H

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/projection_matrix.h>
#include <ssmi_mapping/semantic_octree/SemanticOcTree.h>
#include <ssmi_mapping/semantic_octree/Semantics.h>
#include <ssmi_mapping/semantic_point_cloud/semantic_point_type.h>
#include <ssmi_mapping/semantic_octomap_node/octomap_generator_base.h>

typedef pcl::PointCloud<PointXYZRGBSemantic> PCLSemantics;
typedef octomap::SemanticOcTree<octomap::SemanticsLogOdds> SemanticOctree;
typedef octomap::SemanticOcTreeNode<octomap::SemanticsLogOdds> SemanticsOcTreeNode;

template<class CLOUD, class OCTREE>
class OctomapGenerator: public OctomapGeneratorBase<OCTREE>
{
public:
    /**
     * \brief Constructor
     * \param nh The ros node handler to be used in OctomapGenerator
     */
    OctomapGenerator();

    virtual ~OctomapGenerator();

    virtual void setMaxRange(float max_range){max_range_ = max_range;}

    virtual void setRayCastRange(float raycast_range){raycast_range_ = raycast_range;}

    virtual void setClampingThresMin(float clamping_thres_min)
    {
        octomap_.setClampingThresMin(clamping_thres_min);
        octomap_.setMinLogOdds(clamping_thres_min);
    }

    virtual void setClampingThresMax(float clamping_thres_max)
    {
        octomap_.setClampingThresMax(clamping_thres_max);
        octomap_.setMaxLogOdds(clamping_thres_max);
    }

    virtual void setResolution(float resolution)
    {
        octomap_.setResolution(resolution);
    }

    virtual void setOccupancyThres(float occupancy_thres)
    {
        octomap_.setOccupancyThres(occupancy_thres);
    }

    virtual void setProbHit(float prob_hit)
    {
        octomap_.setProbHit(prob_hit);
    }

    virtual void setProbMiss(float prob_miss)
    {
        octomap_.setProbMiss(prob_miss);
    }

    /// Set phi, parameter for semantic octomap
    virtual void setPhi(float phi)
    {
        octomap_.setPhi(phi);
    }

    /// Set psi, parameter for semantic octomap
    virtual void setPsi(float psi)
    {
        octomap_.setPsi(psi);
    }

    /**
     * \brief Callback to point cloud topic. Update the octomap and publish it in ROS
     * \param cloud ROS Pointcloud2 message in arbitrary frame (specified in the clouds header)
     */
    virtual void insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensorToWorld);
    
    virtual bool get_ray_RLE(const octomap::point3d& origin, const octomap::point3d& end, ssmi_interface::msg::RayRLE& rayRLE_msg);

    virtual void setUseSemanticColor(bool use);

    virtual bool isUseSemanticColor();
    
    virtual void setWriteSemantics(bool write);

    virtual bool doesWriteSemantics();

    virtual OCTREE* getOctree(){return &octomap_;}

    /**
     * \brief Save octomap to a file. NOTE: Not tested
     * \param filename The output filename
     */
    virtual bool save(const char* filename) const;

protected:
    OCTREE octomap_; ///<Templated octree instance
    float max_range_; ///<Max range for points to be inserted into octomap
    float raycast_range_; ///<Max range for points to perform raycasting to free unoccupied space

};

#endif //SEMANTIC_OCTOMAP_OCTOMAP_GENERATOR_H
