#include "exploration/collision.h"
#include "exploration/util.h"


bool check_for_collision(pybind11::safe_array<int, 1> position,
                         pybind11::safe_array<uint8_t, 2> occupancy_map,
                         pybind11::safe_array<bool, 2> footprint_mask,
                         pybind11::safe_array<int, 2> outline_coords,
                         pybind11::safe_array<uint8_t, 1> obstacle_values) {
  /* Collision checking for custom footprints.
   *
   * :param position: 1x2 array with robot coordinate [row, column]
   * :param occupancy_map: 1xN*M array of flattened occupancy map
   * :param map_shape: 1x2 array with (N, M), the shape of the occupancy map
   * :param footprint_mask: (array(N*N)[int]) mask of the footprint rotated at the corresponding angle
   *                        needed for checking, i.e state[2]. N is 2 * mask_radius + 1,
   *                        the values are -1 for not footprint, 0 for footprint. (note the (N,N) mask is flattened)
   * :param mask_radius: (int) (footprint_mask.shape[0] - 1) / 2 the radius of the mask in pixels
   * :param outline_coords: (array(N*2)[int]) the coordinates that define the outline of the footprint. N is the number
   *                        of points that define the outline of the footprint. (it is a (N,2) array but flattened)
   * :param num_coords: number of coordinates in that define the outline of the footprint. i.e N from the description
   *                    of outline_coords
   * :param obstacle_values: (array(N)[uint8]) an array containing values that the collision checker should deem as an obstacle
   *                         i.e [127, 0]
   * :param num_obstacle values: the number of obstacle values given in the obstacle_values array.
   *
   * :return: (bool) whether there is a collision or not
   */
  int map_shape[2] = {(int) occupancy_map.shape()[0] , (int) occupancy_map.shape()[1]};

  // check if footprint it out of bounds -- if so, it is a collision
  // if in bounds, check if the outline coord is colliding
  for(int i = 0; i < outline_coords.shape()[0]; i++) {
    const int row = outline_coords(i, 0) + position[0];
    const int col = outline_coords(i, 1) + position[1];
    if (row < 0 || row >= map_shape[0]) {
      return true;
    }

    if (col < 0 || col >= map_shape[1]) {
      return true;
    }

    for (int k = 0; k < obstacle_values.shape()[0]; k++){
      if (occupancy_map(row, col) == obstacle_values[k]) {
        return true;
      }
    }
  }

  int mask_radius = (footprint_mask.shape()[0] - 1) / 2;
  int clipped_min_range[2], clipped_max_range[2];
  const int min_range[2] = {position[0] - mask_radius, position[1] - mask_radius};
  const int max_range[2] = {position[0] + mask_radius, position[1] + mask_radius};
  clip_range(min_range, max_range, map_shape, clipped_min_range, clipped_max_range);

  for (int m = clipped_min_range[0] - min_range[0], i = clipped_min_range[0]; i < clipped_max_range[0] + 1; m++, i++) {
    for(int n = clipped_min_range[1] - min_range[1], j = clipped_min_range[1]; j < clipped_max_range[1] + 1; n++, j++) {
      if (footprint_mask(m, n)) {
        for (int k = 0; k < obstacle_values.shape()[0]; k++) {
          if (occupancy_map(i, j) == obstacle_values[k]) {
            return true;
          }
        }
      }
    }
  }
  return false;
}
