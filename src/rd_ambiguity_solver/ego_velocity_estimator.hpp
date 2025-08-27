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
#include <set>
#include <tuple>
#include <vector>

#include "point_cloud_utils.hpp"
/*
 * @class EgoInlierEstimator
 * @brief A class that estimates the inlier points that account for true ego velocity
 */

class EgoInlierEstimator {
 public:
  EgoInlierEstimator();   // Constructor
  ~EgoInlierEstimator();  // Destructor

  // A simple ego velocity estimator based on probabilistic RANSAC.
  double p_ransac_ego_velocity(const std::vector<double> &azimuth,
                               const std::vector<double> &elevation,
                               const std::vector<double> &doppler, double confidence,
                               double threshold, int max_iterations, double rnsc_inlier_ratio);

  // A simple median based ego velocity estimator
  std::pair<double, std::vector<double>> simpleMedianEstimator(const std::vector<double> &doppler,
                                                               const std::vector<double> &cosaz,
                                                               const std::vector<double> &cosel);

  // Find the ego velocity of top elevation points, where number of points is total_points
  double getTopEleEgo(const std::vector<double> &doppler, const std::vector<double> &az,
                      const std::vector<double> &el, const double vertical_res,
                      size_t total_points);
};
