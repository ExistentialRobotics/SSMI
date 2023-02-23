#pragma once

#include <iostream>
#include <queue>
#include <limits>
#include <cmath>
#include <stack>
#include <vector>
#include <stdexcept>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include "collision.h"

using PlanningResult = std::tuple<bool, pybind11::safe_array<int, 2> >;
using OrientedPlanningResult = std::tuple<bool, pybind11::safe_array<float, 2> >;

struct Node {
  float f;
  int idx;

  Node(const float f, const int idx) : f(f), idx(idx) {}
};

bool operator>(const Node &n1, const Node &n2);

PlanningResult astar(pybind11::safe_array<int, 1> start,
                     pybind11::safe_array<int, 1> goal,
                     pybind11::safe_array<uint8_t, 2> occupancy_map,
                     pybind11::safe_array<uint8_t, 1> obstacle_values,
                     float delta,
                     float epsilon,
                     int planning_scale,
                     bool allow_diagonal);

std::vector<float> get_astar_angles();

OrientedPlanningResult oriented_astar(pybind11::safe_array<int, 1> start,
                                      pybind11::safe_array<int, 1> goal,
                                      pybind11::safe_array<uint8_t, 2> occupancy_map,
                                      std::vector<pybind11::safe_array_mut<bool, 2> > footprint_masks,
                                      pybind11::safe_array<float, 1> mask_angles,
                                      std::vector<pybind11::safe_array_mut<int, 2> >  outline_coords,
                                      pybind11::safe_array<uint8_t, 1> obstacle_values,
                                      float delta,
                                      float epsilon,
                                      int planning_scale,
                                      bool allow_diagonal);
