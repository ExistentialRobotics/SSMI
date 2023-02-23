#pragma once

#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include "safe_array.h"


bool check_for_collision(pybind11::safe_array<int, 1> position,
                         pybind11::safe_array<uint8_t, 2> occupancy_map,
                         pybind11::safe_array<bool, 2> footprint_mask,
                         pybind11::safe_array<int, 2> outline_coords,
                         pybind11::safe_array<uint8_t, 1> obstacle_values);
