# Semantic-Octomap
Semantic 3-D OctoMap implementation for building probabilistic multi-class maps of an environment using semantic pointcloud input.

## Dependencies
* ROS Melodic
* Catkin
* PCL
* OctoMap
* octomap_msgs
* octomap_rviz_plugins
* scikit-image

## How to use
1. `launch/semantic_octomap.launch` starts ROS nodes `semantic_cloud` and `semantic_octomap`.
2. `semantic_cloud` takes three **aligned** images, i.e. RBG image, depth image, and semantic segmentation image. The output is a sematic pointcloud topic of type `sensor_msgs/PointCloud2`.
3. `semantic_octomap` receives the generated semantic pointcloud and updates the semantic OctoMap. This node internally maintains a semantic OcTree where each node stores the probability of each object class. However, only the maximmum likelihood semantic OcTree (with probabilistic occupancies) is published as an `octomap_msgs/Octomap` message, as well as a probabilistic 2-D occupancy map projected on the ground plane.
4. `params/semantic_cloud.yaml` stores camera intrinsic parameters. This should be set to the values used by your camera. You can also add artificial range and category noise to your sensor by choosing non-zero `depth_noise_std` and `true_class_prob` less than 1.
5. `params/octomap_generator.yaml` stores parameters for the semantic OctoMap such as minimum grid size (`resolution`), log-odds increments (`psi` and `phi`), and path for saving the final map.
