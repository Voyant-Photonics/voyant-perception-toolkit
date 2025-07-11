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

/*
 * @class PointCloudUtils
 * @brief A class that performs coordinate transforms and other utilities
 */

class PointCloudUtils
{
public:
  PointCloudUtils();  // Constructor
  ~PointCloudUtils(); // Destructor

  std::vector<double> rngToFreq(const std::vector<double> &rng, double bw, double c, double T);
  std::vector<double> dopToFreq(const std::vector<double> &d, double lam);
  double freqToRng(double fr, double bw, double c, double T);
  double freqToDop(double fd, double lam);
  std::pair<std::vector<double>, std::vector<double>> getUpDownFreqs(std::vector<double> &rng,
                                                                     std::vector<double> &d,
                                                                     double bw,
                                                                     double c,
                                                                     double T,
                                                                     double lam);
  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> cart2sphere(
      const std::vector<double> &x,
      const std::vector<double> &y,
      const std::vector<double> &z);
  double findMedian(std::vector<double> arr);
  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> sphere2cart(
      const std::vector<double> &r,
      const std::vector<double> &az,
      const std::vector<double> &el);
};
