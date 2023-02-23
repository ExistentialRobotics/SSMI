#pragma once
#include <iostream>
#include <cmath>
#include <limits>

int index_2d_to_1d(const int* coord, const int* map_shape);
int index_2d_to_1d(int i, int j, const int* map_shape);

void index_1d_to_2d(int idx, const int* map_shape, int* result);
float euclidean(const int* coord1, const int* coord2);
void clip_range(const int* min_range, const int* max_range, const int* map_shape,
                int* clipped_min_range, int* clipped_max_range);

int get_angle_idx(float angle, const float* angles, int num_angles);
void print_coord(const int* coord);
