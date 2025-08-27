// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <voyant_frame_wrapper.hpp>

#include "point_types.hpp"

/*
 * @class PointCloudLogger
 * @brief This class provide tools to log data. Currently only csv is supported.
 */
class PointCloudLogger {
 public:
  PointCloudLogger();
  ~PointCloudLogger();

  // Initialize the csv headers
  bool initializeCSV(const std::string &filename);

  // Write points immediately (sequential)
  void writePointsSequential(const std::vector<VoyantPoint> &points,
                             const VoyantFrameWrapper &frame);

  // Close the csv properly
  void finalizeCSV();

 private:
  std::ofstream csv_file;
  bool csv_initialized = false;
};
