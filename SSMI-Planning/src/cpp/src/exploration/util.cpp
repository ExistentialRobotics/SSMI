#include "exploration/util.h"


int index_2d_to_1d(const int* coord, const int* map_shape) {
  return coord[0] * map_shape[1] + coord[1];
}

int index_2d_to_1d(const int i, const int j, const int* map_shape) {
  return i * map_shape[1] + j;
}

void index_1d_to_2d(const int idx, const int* map_shape, int* result) {
  result[0] = idx / map_shape[1];
  result[1] = idx % map_shape[1];
}

float euclidean(const int* coord1, const int* coord2) {
  return (float) std::sqrt(std::pow((coord2[1] - coord1[1]), 2) + std::pow((coord2[0] - coord1[0]), 2));
}

void clip_range(const int* min_range, const int* max_range, const int* map_shape,
                int* clipped_min_range, int* clipped_max_range) {
  clipped_min_range[0] = std::max(min_range[0], 0);
  clipped_min_range[1] = std::max(min_range[1], 0);
  clipped_max_range[0] = std::min(max_range[0], map_shape[0]);
  clipped_max_range[1] = std::min(max_range[1], map_shape[1]);
}

int get_angle_idx(const float angle, const float* angles, const int num_angles) {
  float angular_distance = std::numeric_limits<float>::infinity();;
  int angle_idx = -1;
  for (int i = 0; i < num_angles; i++) {
    float d = std::abs(angle - angles[i]);
    if (d < angular_distance) {
      angle_idx = i;
      angular_distance = d;
    }
  }

  return angle_idx;
}

void print_coord(const int* coord) {
  std::cout << "[" << coord[0] << ", " << coord[1] << "]" << std::endl;
}
