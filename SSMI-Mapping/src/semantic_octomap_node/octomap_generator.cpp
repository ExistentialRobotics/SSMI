#include <semantic_octomap_node/octomap_generator.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <cmath>
#include <sstream>
#include <cstring> // For std::memcpy

template<class CLOUD, class OCTREE>
OctomapGenerator<CLOUD, OCTREE>::OctomapGenerator(): octomap_(0.05), max_range_(1.), raycast_range_(1.){}

template<class CLOUD, class OCTREE>
OctomapGenerator<CLOUD, OCTREE>::~OctomapGenerator(){}

template<class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::setUseSemanticColor(bool use)
{
    octomap_.setUseSemanticColor(use);
}

template<class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::isUseSemanticColor()
{
    return octomap_.isUseSemanticColor();
}

template<class CLOUD, class OCTREE>
void OctomapGenerator<CLOUD, OCTREE>::insertPointCloud(const pcl::PCLPointCloud2::Ptr& cloud, const Eigen::Matrix4f& sensorToWorld)
{
    // Voxel filter to down sample the point cloud
    // Create the filtering object
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2 ());
    // Perform voxel filter
    float voxel_flt_size = octomap_.getResolution();
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (voxel_flt_size, voxel_flt_size, voxel_flt_size);
    sor.filter (*cloud_filtered);
    // Convert to PCL pointcloud
    CLOUD pcl_cloud;
    pcl::fromPCLPointCloud2(*cloud_filtered, pcl_cloud);
    //std::cout << "Voxel filtered cloud size: "<< pcl_cloud.size() << std::endl;
    // Transform coordinate
    pcl::transformPointCloud(pcl_cloud, pcl_cloud, sensorToWorld);

    //tf::Vector3 originTf = sensorToWorldTf.getOrigin();
    //octomap::point3d origin(originTf[0], originTf[1], originTf[2]);
    octomap::point3d origin(static_cast<float>(sensorToWorld(0,3)),static_cast<float>(sensorToWorld(1,3)),static_cast<float>(sensorToWorld(2,3)));
    octomap::Pointcloud raycast_cloud; // Point cloud to be inserted with ray casting
    int endpoint_count = 0; // total number of endpoints inserted
    for(typename CLOUD::const_iterator it = pcl_cloud.begin(); it != pcl_cloud.end(); ++it)
    {
        // Check if the point is invalid
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
        {
            float dist = sqrt((it->x - origin.x())*(it->x - origin.x()) + (it->y - origin.y())*(it->y - origin.y()) + (it->z - origin.z())*(it->z - origin.z()));
            // Check if the point is in max_range
            if(dist <= max_range_)
            {
                octomap::ColorOcTreeNode::Color color_obs(it->r, it->g, it->b);
                octomap::ColorOcTreeNode::Color class_obs;
                // Get semantics
                uint32_t rgb;
                std::memcpy(&rgb, &it->semantic_color, sizeof(uint32_t));
                class_obs.r = (rgb >> 16) & 0x0000ff;
                class_obs.g = (rgb >> 8)  & 0x0000ff;
                class_obs.b = (rgb)       & 0x0000ff;
            
                octomap_.updateNode(it->x, it->y, it->z, true, class_obs, color_obs, false);
            
                endpoint_count++;
            }
            
            raycast_cloud.push_back(it->x, it->y, it->z);
        }
    }
    // Do ray casting for points in raycast_range_
    if(raycast_cloud.size() > 0)
        octomap_.insertPointCloud(raycast_cloud, origin, raycast_range_, true);
    
    /* updates inner node occupancy and colors
    if(endpoint_count > 0)
        octomap_.updateInnerOccupancy();*/
}

template<class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::get_ray_RLE(const octomap::point3d& origin, const octomap::point3d& end, semantic_octomap::RayRLE& rayRLE_msg)
{
    if (octomap_.get_ray_RLE(origin, end, rayRLE_msg))
    {
        return true;
    } else {
        return false;
    }
}

template<class CLOUD, class OCTREE>
bool OctomapGenerator<CLOUD, OCTREE>::save(const char* filename) const
{
    std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
    if (outfile.is_open()){
        std::cout << "Writing octomap to " << filename << std::endl;
        octomap_.write(outfile);
        outfile.close();
        std::cout << "Color tree written " << filename << std::endl;
        return true;
    }
    else {
        std::cout << "Could not open " << filename  << " for writing" << std::endl;
        return false;
    }
}

//Explicit template instantiation
template class OctomapGenerator<PCLSemantics, SemanticOctree>;
