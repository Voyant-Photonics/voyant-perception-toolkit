// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <tuple>
#include <vector>
#include <voyant_frame_wrapper.hpp>

/*
 * @class PointCloudUtils
 * @brief A class that performs coordinate transforms and other utilities
 */

namespace PointCloudUtils {
// Convert Range to Frequency
double rngToFreq(const double &rng, double bw, double T);

// Convert Doppler to Frequency
double dopToFreq(const double &d, double lam);

// Convert Frequency to Range
double freqToRng(double fr, double bw, double T);

// Convert Frequency to Doppler
double freqToDop(double fd, double lam);

// Cartesian to Spherical conversion
std::tuple<double, double, double> cart2sphere(const double &x, const double &y, const double &z);

// Efficient median finder of a vector
double findMedian(std::vector<double> arr);

// Spherical to cartesian conversion
std::tuple<double, double, double> sphere2cart(const double &r, const double &az, const double &el);

// Check if the point is valid
bool validatePointCoordinates(const PointDataWrapper &pt);

constexpr double C = 3e8;  // Speed of light in m/s

constexpr double MAX_COORD = 1000.0;
}  // namespace PointCloudUtils
