from __future__ import division
from __future__ import print_function
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from enum import Enum


class PointType(Enum):
    SEMANTIC = 0


class SemanticPclGenerator:
    def __init__(self, intrinsic, width = 80, height = 60,unit_conversion = 1, frame_id = "/camera",
                 point_type = PointType.SEMANTIC):
        '''
        width: (int) width of input images
        height: (int) height of input images
        unit_conversion: (float) unit conversion factor 1 for m and 1e3 for mm
        '''
        self.point_type = point_type
        self.intrinsic = intrinsic
        self.unit_conversion = unit_conversion # Unit conversion factor 1 for m and 1e3 for mm
        # Allocate arrays
        x_index = np.array([list(range(width))*height], dtype = '<f4')
        y_index = np.array([[i]*width for i in range(height)], dtype = '<f4').ravel()
        self.xy_index = np.vstack((x_index, y_index)).T # x,y
        self.xyd_vect = np.zeros([width*height, 3], dtype = '<f4') # x,y,depth
        self.XYZ_vect = np.zeros([width*height, 3], dtype = '<f4') # real world coord
        self.ros_data = np.ones([width*height, 6], dtype = '<f4') # [x,y,z,0,bgr0,semantic_color]
        self.bgr0_vect = np.zeros([width*height, 4], dtype = '<u1') #bgr0
        self.semantic_color_vect = np.zeros([width*height, 4], dtype = '<u1') #bgr0
        # Prepare ros cloud msg
        # Cloud data is serialized into a contiguous buffer, set fields to specify offsets in buffer
        self.cloud_ros = PointCloud2()
        self.cloud_ros.header.frame_id = frame_id
        self.cloud_ros.height = 1
        self.cloud_ros.width = width*height
        self.cloud_ros.fields.append(PointField(
            name = "x",
            offset = 0,
            datatype = PointField.FLOAT32, count = 1))
        self.cloud_ros.fields.append(PointField(
            name = "y",
            offset = 4,
            datatype = PointField.FLOAT32, count = 1))
        self.cloud_ros.fields.append(PointField(
            name = "z",
            offset = 8,
            datatype = PointField.FLOAT32, count = 1))
        self.cloud_ros.fields.append(PointField(
            name = "rgb",
            offset = 16,
            datatype = PointField.FLOAT32, count = 1))
        self.cloud_ros.fields.append(PointField(
            name = "semantic_color",
            offset = 20,
            datatype = PointField.FLOAT32, count = 1))
        self.cloud_ros.is_bigendian = False
        self.cloud_ros.point_step = 6 * 4 # In bytes
        self.cloud_ros.row_step = self.cloud_ros.point_step * self.cloud_ros.width * self.cloud_ros.height
        self.cloud_ros.is_dense = False

    def generate_cloud_data_common(self, bgr_img, depth_img):
        """
        Do depth registration, suppose that rgb_img and depth_img has the same intrinsic
        \param bgr_img (numpy array bgr8)
        \param depth_img (numpy array float32 2d)
        [x, y, Z] = [X, Y, Z] * intrinsic.T
        """
        np.place(depth_img, depth_img == 0, 100000) # Handle maximum range measurements
        
        bgr_img = bgr_img.view('<u1')
        depth_img = depth_img.view('<f4')
        # Add depth information
        self.xyd_vect[:,0:2] = self.xy_index * depth_img.reshape(-1,1) / self.unit_conversion
        self.xyd_vect[:,2:3] = depth_img.reshape(-1,1) / self.unit_conversion # Convert to meters
        self.XYZ_vect = self.xyd_vect.dot(self.intrinsic.I.T)
        # Convert to ROS point cloud message in a vectorialized manner
        # ros msg data: [x,y,z,0,bgr0,semantic_color] (little endian float32)
        # Transform color
        self.bgr0_vect[:,0:1] = bgr_img[:,:,0].reshape(-1,1)
        self.bgr0_vect[:,1:2] = bgr_img[:,:,1].reshape(-1,1)
        self.bgr0_vect[:,2:3] = bgr_img[:,:,2].reshape(-1,1)
        # Concatenate data
        self.ros_data[:,0:3] = self.XYZ_vect
        self.ros_data[:,4:5] = self.bgr0_vect.view('<f4')

    def make_ros_cloud(self, stamp):
        # Assign data to ros msg
        # We should send directly in bytes, send in as a list is too slow, numpy tobytes is too slow, takes 0.3s.
        self.cloud_ros.data = self.ros_data.ravel().tobytes()
        self.cloud_ros.header.stamp = stamp
        return self.cloud_ros

    def generate_cloud_semantic(self, bgr_img, semantic_color, depth_img, stamp):
        self.generate_cloud_data_common(bgr_img, depth_img)
        #Transform semantic color
        self.semantic_color_vect[:,0:1] = semantic_color[:,:,0].reshape(-1,1)
        self.semantic_color_vect[:,1:2] = semantic_color[:,:,1].reshape(-1,1)
        self.semantic_color_vect[:,2:3] = semantic_color[:,:,2].reshape(-1,1)
        # Concatenate data
        self.ros_data[:,5:6] = self.semantic_color_vect.view('<f4')
        return self.make_ros_cloud(stamp)
