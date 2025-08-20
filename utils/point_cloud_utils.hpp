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

class PointCloudUtils {
 public:
  PointCloudUtils();   // Constructor
  ~PointCloudUtils();  // Destructor

  double rngToFreq(const double &rng, double bw, double T);
  double dopToFreq(const double &d, double lam);
  double freqToRng(double fr, double bw, double T);
  double freqToDop(double fd, double lam);
  std::tuple<double, double, double> cart2sphere(const double &x, const double &y, const double &z);
  double findMedian(std::vector<double> arr);
  std::tuple<double, double, double> sphere2cart(const double &r, const double &az,
                                                 const double &el);
  bool validatePointCoordinates(const PointDataWrapper &pt);

  double C = 3e8;  // Speed of light in m/s
};
