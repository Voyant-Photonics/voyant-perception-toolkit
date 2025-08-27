// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "point_cloud_logger.hpp"

PointCloudLogger::PointCloudLogger() { std::cout << "[+] Starting the logger" << std::endl; }
PointCloudLogger::~PointCloudLogger() { std::cout << "[+] Shutting down the logger" << std::endl; }

// Initialize CSV file with header
bool PointCloudLogger::initializeCSV(const std::string &filename) {
  csv_file.open(filename);
  if (!csv_file.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return false;
  }

  // Write header
  csv_file << "proto_version,api_version,firmware_version,hdl_version,"
              "device_id,frame_timestamp_seconds,frame_timestamp_nanoseconds,frame_index,nanosecs_"
              "since_frame,point_index,azimuth_index,elevation_index,x,y,z,radial_vel"
              ",snr_linear,calibrated_reflectance,"
              "noise_mean_estimate,min_ramp_snr,drop_reason\n";

  csv_initialized = true;
  return true;
}

// Close CSV file
void PointCloudLogger::finalizeCSV() {
  if (csv_file.is_open()) {
    csv_file.flush();
    csv_file.close();
    csv_initialized = false;
    std::cout << "CSV file closed successfully." << std::endl;
  } else {
    std::cerr << "Warning: Attempted to finalize CSV, but file is already closed." << std::endl;
  }
}

// Write points immediately (sequential)
void PointCloudLogger::writePointsSequential(const std::vector<VoyantPoint> &points,
                                             const VoyantFrameWrapper &frame) {
  if (!csv_initialized || !csv_file.is_open()) {
    std::cerr << "CSV file not initialized!" << std::endl;
    return;
  }
  for (const auto &pt : points) {
    csv_file << frame.header().protoVersion().toU32Hash() << ","
             << frame.header().apiVersion().toU32Hash() << ","
             << frame.header().firmwareVersion().toU32Hash() << ","
             << frame.header().hdlVersion().toU32Hash() << "," << frame.header().deviceId() << ","
             << pt.frame_timestamp_seconds << "," << pt.frame_timestamp_nanoseconds << ","
             << pt.frame_idx << "," << pt.nanosecs_since_frame << "," << pt.point_idx << "," << 0
             << "," << 0 << "," << pt.x << "," << pt.y << "," << pt.z << "," << pt.radial_vel << ","
             << pt.snr_linear << "," << pt.calibrated_reflectance << "," << pt.noise_mean_estimate
             << "," << pt.min_ramp_snr << "," << pt.drop_reason << "\n";
  }

  // Optional: flush to ensure data is written immediately
  csv_file.flush();
}
