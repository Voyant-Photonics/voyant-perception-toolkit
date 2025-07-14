// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <voyant_frame_wrapper.hpp>
#include <voyant_playback.hpp>

#include "ego_velocity_estimator.hpp"

/*
 * @brief Parameters for the Doppler ambiguity resolver
 */
struct Params {
  std::string input_file;
  std::string output_file;
  double T;
  double BW;
  double LAMBDA;
  double VERTICAL_RES_DEG;
  int min_pts;
  double rnsc_confidence;
  double rnsc_thres;
  double rnsc_inlier_ratio;
  double rnsc_max_iter;
  double rnsc_alpha;
  double med_alpha_up;
  double med_alpha_down;
  double med_alpha_up_agg;
  double med_alpha_down_agg;
  int num_pts_init;
  int num_pts_recollect;
  double ego_recollect_threshold;
  double min_ego_velocity_for_correction;
  double ransac_preference_threshold;
  double med_diff_aggr_thres;
  double top_ego_conf_th;
};

/*
 * @class AmbiguitySolver
 * @brief A class that solves specific range-doppler ambiguity
 */
class AmbiguitySolver {
 public:
  AmbiguitySolver(const std::string &yaml_path);
  ~AmbiguitySolver();
  Params loadParams(const std::string &yaml_path);
  bool validatePointCoordinates(const PointDataWrapper &pt);
  std::vector<std::array<double, 12>> Solver(const VoyantFrameWrapper &frame, size_t frame_id);
  void writeRecoveredPointsToCSV(const std::string &filename,
                                 const std::vector<std::array<double, 12>> &points);
  std::pair<std::vector<double>, std::vector<double>> recoverRangeDoppler(
      std::vector<double> amb_rng, std::vector<double> amb_dop);
  Params config;

 private:
  PointCloudUtils pc_utils_;
  EgoInlierEstimator ego_solver;
};
