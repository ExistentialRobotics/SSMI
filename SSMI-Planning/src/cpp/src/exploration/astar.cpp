#include "exploration/astar.h"
#include "exploration/util.h"

bool operator>(const Node &n1, const Node &n2) {
  return n1.f > n2.f;
}

PlanningResult astar(pybind11::safe_array<int, 1> start,
                     pybind11::safe_array<int, 1> goal,
                     pybind11::safe_array<uint8_t, 2> occupancy_map,
                     pybind11::safe_array<uint8_t, 1> obstacle_values,
                     float delta,
                     float epsilon,
                     int planning_scale,
                     bool allow_diagonal) {
  /* A* algorithm
   *
   * :param start array(2)[int32]: start coordinate [row, column]
   * :param goal array(2)[int32]: goal coordinate [row, column]
   * :param occupancy_map array(N, M)[uint8]: 2d occupancy map
   * :param obstacle_values array(N)[uint8]: values in the occupancy map we consider to be obstacles
   * :param delta float: distance (in pixels) of which we define around the goal coordinate, to consider as the goal
   *                     if it is 0, we look to plan exactly to the goal coord, if its > 0 then we define the goal region
   *                     to be that many pixels bigger
   * :param epsilon float: weighting for the heuristic in the A* algorithm
   * :param planning_scale int: value > 1, to plan on a lower resolution than the original occupancy map resolution,
   *                            this value is round_int(desired resolution / original resolution)
   * :param allow_diagonal bool: whether to allow diagonal movements
   * :return Tuple[bool, array(N, 2)[int32]]: is_successful - whether the goal pose was reached,
   *                                          path_px - array of coordinates for the most promising path if not successful
   *                                                    otherwise it is the path to the goal.
   */
  std::cout << "start plan, ";

  // verify planning scale is correct
  if (planning_scale < 1) {
    throw std::logic_error("ERROR: parameter planning_scale of c++ function oriented_astar() must be greater than 1 ");
  }

  const float inf = std::numeric_limits<float>::infinity();

  int map_shape[2] = {(int) occupancy_map.shape()[0] , (int) occupancy_map.shape()[1]};
  const int start_idx = index_2d_to_1d(&start(0), map_shape);

  for(int i = 0; i < obstacle_values.shape()[0]; i++){
    if (occupancy_map(start(0), start(1)) == obstacle_values[i] || occupancy_map(goal(0), goal(1)) == obstacle_values[i]) {
      return std::make_tuple(false, pybind11::safe_array<int, 2>());
    }
  }

  Node start_node = Node(0.0, start_idx);

  std::priority_queue<Node, std::vector<Node>, std::greater<> > open_set;
  open_set.push(start_node);

  std::vector<float> costs((ulong) map_shape[0]*map_shape[1]);
  for (int i = 0; i < map_shape[0]*map_shape[1]; i++) {
    costs[i] = inf;
  }
  costs[start_idx] = 0.0;

  std::vector<int> paths((ulong) map_shape[0]*map_shape[1]);
  for (int i = 0; i < map_shape[0]*map_shape[1]; i++) {
    paths[i] = -1;
  }

  int children[8][2];
  int parent_coord[2];
  int solution_idx = -1;

  int num_nodes_expanded = 0;
  bool is_successful = false;
  while(!open_set.empty()) {
    Node parent = open_set.top();
    open_set.pop();

    index_1d_to_2d(parent.idx, map_shape, parent_coord);

    // todo planning_scale + delta isnt quite right, maybe max(planning_scale, delta) is correct
    float distance_to_goal = euclidean(parent_coord, &goal(0));
    if (distance_to_goal <= delta || (planning_scale != 1 && distance_to_goal <= planning_scale + delta)) {
      is_successful = true;
      solution_idx = parent.idx;
      break;
    }

    children[0][0] = parent_coord[0];                  children[0][1] = parent_coord[1] - planning_scale;
    children[1][0] = parent_coord[0] + planning_scale; children[1][1] = parent_coord[1] - planning_scale;
    children[2][0] = parent_coord[0] + planning_scale; children[2][1] = parent_coord[1];
    children[3][0] = parent_coord[0] + planning_scale; children[3][1] = parent_coord[1] + planning_scale;
    children[4][0] = parent_coord[0];                  children[4][1] = parent_coord[1] + planning_scale;
    children[5][0] = parent_coord[0] - planning_scale; children[5][1] = parent_coord[1] + planning_scale;
    children[6][0] = parent_coord[0] - planning_scale; children[6][1] = parent_coord[1];
    children[7][0] = parent_coord[0] - planning_scale; children[7][1] = parent_coord[1] - planning_scale;

    for (int c = 0; c < 8; c++) {
      // if diagonal is not allowed, skip those children
      if (!allow_diagonal && c % 2 != 0) {
            continue;
      }

      // skip child if out of bounds
      if (children[c][0] < 0 || children[c][0] >= map_shape[0] \
              || children[c][1] < 0 || children[c][1] >= map_shape[1]) {
        continue;
      }

      // skip child if it lies on an obstacle
      bool on_obstacle = false;
      for(int i = 0; i < obstacle_values.shape()[0]; i++) {
        if (occupancy_map(children[c][0], children[c][1]) == obstacle_values[i]) {
          on_obstacle = true;
        }
      }

      if (on_obstacle) {
        continue;
      }

      float g = costs[parent.idx] + euclidean(parent_coord, children[c]);

      int child_idx = index_2d_to_1d(children[c], map_shape);
      if (costs[child_idx] > g) {
        costs[child_idx] = g;
        paths[child_idx] = parent.idx;

        float f = g + epsilon * euclidean(children[c], &goal(0));
        open_set.push(Node(f, child_idx));
      }
    }
    num_nodes_expanded++;
  }

  int current_idx = solution_idx;
  std::stack<int> path_stack;
  while(start_idx != current_idx) {
    path_stack.push(current_idx);
    current_idx = paths[current_idx];
  }

  auto path_px = pybind11::zeros<int>(path_stack.size(), 2);

  int i = 0;
  int coord[2];
  while(!path_stack.empty()) {
    int idx = path_stack.top();
    index_1d_to_2d(idx, map_shape, coord);
    path_px(i, 0) = coord[0];
    path_px(i, 1) = coord[1];
    path_stack.pop();
    i++;
  }

  std::cout << "expanded " << num_nodes_expanded << " nodes. successful: " << is_successful << std::endl;
  return std::make_tuple(is_successful, path_px);
}


std::vector<float> get_astar_angles() {
  // these angles correspond to the x,y world converted
  // angles of moving in the corresponding
  // children direction in the astar algorithm
  // see astar assignment of children.
  std::vector<float> angles(8);
  angles[0] = (float) -M_PI;
  angles[1] = (float) (-3.*M_PI_4);
  angles[2] = (float) -M_PI_2;
  angles[3] = (float) -M_PI_4;
  angles[4] = (float) 0.;
  angles[5] = (float) M_PI_4;
  angles[6] = (float) M_PI_2;
  angles[7] = (float) (3.*M_PI_4);
  return angles;
}


OrientedPlanningResult oriented_astar(pybind11::safe_array<int, 1> start,
                                      pybind11::safe_array<int, 1> goal,
                                      pybind11::safe_array<uint8_t, 2> occupancy_map,
                                      std::vector<pybind11::safe_array_mut<bool, 2> > footprint_masks,
                                      pybind11::safe_array<float, 1> mask_angles,
                                      std::vector<pybind11::safe_array_mut<int, 2> > outline_coords,
                                      pybind11::safe_array<uint8_t, 1> obstacle_values,
                                      const float delta,
                                      const float epsilon,
                                      const int planning_scale,
                                      const bool allow_diagonal) {
  /* Oriented A* algorithm, does not plan on angular space, rather has assigned angles for each movement direction.
   *
   * :param start array(2)[float32]: start coordinate [row, column]
   * :param goal array(2)[float32]: goal coordinate [row, column]
   * :param occupancy_map array(N, M)[uint8]: 2d occupancy map
   * :param footprint_masks List[array(U, U)]: U is the size of a UxU mask for checking footprint collision,
   *                                           where U must be odd, such that U / 2 + 1 is the center pixel,
   *                                           the mask is true for footprint and false for not footprint,
   *                                           which is centered among the center pixel in the mask.
   *                                           Each mask in the list corresponds to a footprint mask for each angle
   *                                           defined in mask_angles.
   * :param mask_angles array(8)[float32]: 1x8 array of these angles (in order)
   *                                       [-pi, -3pi/4, -pi/2, -pi/4, 0, pi/4, pi/2, 3pi/4]
   *                                       can use the get_astar_angles() function to get these angles.
   * :param outline_coords array(N, 2)[int32]: coordinates corresponding to the outline of the footprint in ego coordinates
   *                                           used for quick checking of out of bounds / collisions
   * :param obstacle_values array(N)[uint8]: values in the occupancy map we consider to be obstacles
   * :param delta float: distance (in pixels) of which we define around the goal coordinate, to consider as the goal
   *                     if it is 0, we look to plan exactly to the goal coord, if its > 0 then we define the goal region
   *                     to be that many pixels bigger
   * :param epsilon float: weighting for the heuristic in the A* algorithm
   * :param planning_scale int: value > 1, to plan on a lower resolution than the original occupancy map resolution,
   *                       this value is round_int(desired resolution / original resolution)
   * :param allow_diagonal bool: whether to allow diagonal movements
   * :return Tuple[bool, array(N, 2)[int32]]: is_successful - whether the goal pose was reached,
   *                                          path_px - array of coordinates for the most promising path if not successful
   *                                                    otherwise it is the path to the goal.
   */

  std::cout << "start plan, ";

  // todo: we arent going to goal angle.. we need to make sure we collision check it if we want to.
  const float inf = std::numeric_limits<float>::infinity();

  // verify angles are correct (i.e human knows what he is doing when he calls this function)
  std::vector<float> correct_angles = get_astar_angles();
  for (int i = 0; i < 8; i++) {
    if (correct_angles[i] != mask_angles[i]) {
      throw std::logic_error("ERROR: parameter mask_angles of c++ function oriented_astar() does not match required angles. "
                             "See get_astar_angles() for the correct angles/order. "
                             "Note, the footprint masks must match these angles, or you will get undesired behavior!");
    }
  }

  // verify planning scale is correct
  if (planning_scale < 1) {
    throw std::logic_error("ERROR: parameter planning_scale of c++ function oriented_astar() must be greater than 1 ");
  }

  int map_shape[2] = {(int) occupancy_map.shape()[0] , (int) occupancy_map.shape()[1]};
  const int start_idx = index_2d_to_1d(&start(0), map_shape);

  Node start_node = Node(0.0, start_idx);

  std::priority_queue<Node, std::vector<Node>, std::greater<> > open_set;
  open_set.push(start_node);

  std::vector<float> costs((ulong) map_shape[0]*map_shape[1]);
  for (int i = 0; i < map_shape[0]*map_shape[1]; i++) {
    costs[i] = inf;
  }
  costs[start_idx] = 0.0;

  std::vector<int> paths((ulong) map_shape[0]*map_shape[1]);
  for (int i = 0; i < map_shape[0]*map_shape[1]; i++) {
    paths[i] = -1;
  }

  std::vector<int> paths_angle_inds((ulong) map_shape[0]*map_shape[1]);
  for (int i = 0; i < map_shape[0]*map_shape[1]; i++) {
    paths_angle_inds[i] = -1;
  }

  int children[8][2];
  int parent_coord[2];

  int solution_idx = -1;
  float closest_distance_to_goal = inf;

  int num_nodes_expanded = 0;
  bool is_successful = false;
  while(!open_set.empty()) {
    Node parent = open_set.top();
    open_set.pop();

    index_1d_to_2d(parent.idx, map_shape, parent_coord);

    float distance_to_goal = euclidean(parent_coord, &goal(0));

    if (distance_to_goal < closest_distance_to_goal) {
      closest_distance_to_goal = distance_to_goal;
      solution_idx = parent.idx;
    }

    // todo planning_scale + delta isnt quite right, maybe max(planning_scale, delta) is correct
    if (distance_to_goal <= delta || (planning_scale != 1 && distance_to_goal <= planning_scale + delta)) {
      is_successful = true;
      solution_idx = parent.idx;
      break;
    }

    // todo Note that if planning_scale is too large, in theory this will cause us to jump over thin obstacles.
    // todo In reality, this will never happen, since we won't be planning on a large enough resolutions.
    // todo can I somehow efficiently check the line between the pose and the neighbor to make sure the move is valid?
    children[0][0] = parent_coord[0];                  children[0][1] = parent_coord[1] - planning_scale;
    children[1][0] = parent_coord[0] + planning_scale; children[1][1] = parent_coord[1] - planning_scale;
    children[2][0] = parent_coord[0] + planning_scale; children[2][1] = parent_coord[1];
    children[3][0] = parent_coord[0] + planning_scale; children[3][1] = parent_coord[1] + planning_scale;
    children[4][0] = parent_coord[0];                  children[4][1] = parent_coord[1] + planning_scale;
    children[5][0] = parent_coord[0] - planning_scale; children[5][1] = parent_coord[1] + planning_scale;
    children[6][0] = parent_coord[0] - planning_scale; children[6][1] = parent_coord[1];
    children[7][0] = parent_coord[0] - planning_scale; children[7][1] = parent_coord[1] - planning_scale;

    for (int c = 0; c < 8; c++) {
      // if diagonal is not allowed, skip those children
      if (!allow_diagonal && c % 2 != 0) {
        continue;
      }

      // skip child if out of bounds
      if (children[c][0] < 0 || children[c][0] >= map_shape[0] \
          || children[c][1] < 0 || children[c][1] >= map_shape[1]) {
        continue;
      }

      auto child = pybind11::zeros<int>(2);
      child(0) = children[c][0]; child(1) = children[c][1];

      if (check_for_collision(child,
                              occupancy_map,
                              footprint_masks[c],
                              outline_coords[c],
                              obstacle_values)) {
        continue;
      }

      float g = costs[parent.idx] + euclidean(parent_coord, children[c]);

      int child_idx = index_2d_to_1d(children[c], map_shape);
      if (costs[child_idx] > g) {
        costs[child_idx] = g;
        paths[child_idx] = parent.idx;
        paths_angle_inds[child_idx] = c;

        float f = g + epsilon * euclidean(children[c], &goal(0));
        open_set.push(Node(f, child_idx));
      }
    }
    num_nodes_expanded++;
  }

  int current_idx = solution_idx;
  float current_angle = mask_angles[paths_angle_inds[current_idx]];
  std::stack<std::pair<int, float> > path_stack;
  while(start_idx != current_idx) {
    path_stack.push(std::make_pair(current_idx, current_angle));
    current_idx = paths[current_idx];
    current_angle = mask_angles[paths_angle_inds[current_idx]];
  }

  auto path_px = pybind11::zeros<float>(path_stack.size(), 3);

  int i = 0;
  int coord[2];
  while(!path_stack.empty()) {
    std::pair<int, float> item = path_stack.top();
    index_1d_to_2d(item.first, map_shape, coord);
    path_px(i, 0) = coord[0];
    path_px(i, 1) = coord[1];
    path_px(i, 2) = item.second;
    path_stack.pop();
    i++;
  }

  std::cout << "expanded " << num_nodes_expanded << " nodes. successful: " << is_successful << std::endl;
  return std::make_tuple(is_successful, path_px);
}
