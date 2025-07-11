// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once
#include "voyant-perception-toolkit/point_cloud_utils.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
// #include <map>
#include <numeric>
#include <random>
#include <set>
#include <tuple>
#include <vector>
/*
 * @class EgoInlierEstimator
 * @brief A class that estimates the inlier points that account for true ego velocity
 */

class EgoInlierEstimator
{
  public:
    EgoInlierEstimator(PointCloudUtils &utils); // Constructor
    ~EgoInlierEstimator();                      // Destructor
    double p_ransac_ego_velocity(const std::vector<double> &azimuth, const std::vector<double> &elevation,
                                 const std::vector<double> &doppler, double confidence, double threshold,
                                 int max_iterations, double rnsc_inlier_ratio);

    std::pair<double, std::vector<double>> simpleMedianEstimator(const std::vector<double> &doppler,
                                                                 const std::vector<double> &cosaz,
                                                                 const std::vector<double> &cosel);

    double getTopEleEgo(const std::vector<double> &doppler, const std::vector<double> &az,
                        const std::vector<double> &el, const double vertical_res, int total_points);

  private:
    PointCloudUtils &pc_utils_;
};
