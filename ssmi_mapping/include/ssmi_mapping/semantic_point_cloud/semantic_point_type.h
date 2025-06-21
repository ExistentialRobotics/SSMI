#ifndef SEMANTIC_OCTOMAP_SEMANTIC_POINT_TYPE_H
#define SEMANTIC_OCTOMAP_SEMANTIC_POINT_TYPE_H

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

struct PointXYZRGBSemantic
{
    PCL_ADD_POINT4D;                  // Preferred way of adding a XYZ+padding
    PCL_ADD_RGB;
    union  // Semantic color
    {
        float semantic_color;
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                      // enforce SSE padding for correct memory alignment

// here we assume a XYZ + RGB + semantic_color as fields
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBSemantic,
(float, x, x)
(float, y, y)
(float, z, z)
(float, rgb, rgb)
(float, semantic_color, semantic_color)
)

#endif //SEMANTIC_OCTOMAP_SEMANTIC_POINT_TYPE_H
