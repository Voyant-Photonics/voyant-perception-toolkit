// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "point_cloud_utils.hpp"

PointCloudUtils::PointCloudUtils() {
  std::cout << "[+] Starting the utils" << std::endl;
}  // Constructor
PointCloudUtils::~PointCloudUtils() {
  std::cout << "[+] Shutting the utils..." << std::endl;
}  // Destructor

double PointCloudUtils::rngToFreq(const double &rng, double bw, double T) {
  double frequency = (2 * bw * rng) / (C * T);
  return frequency;
}
double PointCloudUtils::dopToFreq(const double &d, double lam) {
  double frequency = (d * 2) / lam;
  return frequency;
}
double PointCloudUtils::freqToRng(double fr, double bw, double T) {
  return (fr * C * T) / (2 * bw);
}
double PointCloudUtils::freqToDop(double fd, double lam) { return (lam * fd) / 2; }

bool PointCloudUtils::validatePointCoordinates(const PointDataWrapper &pt) {
  if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z()) || std::isinf(pt.x()) ||
      std::isinf(pt.y()) || std::isinf(pt.z()) ||
      (pt.x() == 0.0 && pt.y() == 0.0 && pt.z() == 0.0)) {
    return false;
  }

  if (pt.drop_reason() != DropReason::SUCCESS) {
    return false;
  }

  // Check for unreasonably large coordinates
  constexpr double MAX_COORD = 1000.0;
  if (std::abs(pt.x()) > MAX_COORD || std::abs(pt.y()) > MAX_COORD ||
      std::abs(pt.z()) > MAX_COORD) {
    return false;
  }
  return true;
}

std::tuple<double, double, double> PointCloudUtils::cart2sphere(const double &x, const double &y,
                                                                const double &z) {
  double rng = std::sqrt((x * x) + (y * y) + (z * z));
  double az = std::atan2(y, x);
  double el = 0.0;
  if (std::abs(rng) > 1e-8) {  // r is zero or very close or small epsilon
    double ratio = z / rng;
    el = std::asin(ratio);
  } else {
    el = 0.0;
  }
  return {rng, az, el};
}
double PointCloudUtils::findMedian(std::vector<double> arr) {
  if (arr.empty()) {
    throw std::invalid_argument("Cannot compute median of an empty array.");
  }

  size_t size = arr.size();
  size_t mid = size / 2;

  // Use nth_element to partially sort the array up to the middle
  std::nth_element(arr.begin(), arr.begin() + mid, arr.end());

  if (size % 2 != 0) {
    return arr[mid];  // Odd size
  } else {
    // Need to find the other middle element
    double first_middle = *std::max_element(arr.begin(), arr.begin() + mid);
    return (first_middle + arr[mid]) / 2.0;
  }
}
std::tuple<double, double, double> PointCloudUtils::sphere2cart(const double &r, const double &az,
                                                                const double &el) {
  double X = r * std::cos(el) * std::cos(az);
  double Y = r * std::cos(el) * std::sin(az);
  double Z = r * std::sin(el);

  return {X, Y, Z};
}
