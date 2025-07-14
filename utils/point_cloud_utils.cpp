// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "voyant-perception-toolkit/point_cloud_utils.hpp"

PointCloudUtils::PointCloudUtils() {
  std::cout << "[+] Starting the utils" << std::endl;
}  // Constructor
PointCloudUtils::~PointCloudUtils() {
  std::cout << "[+] Shutting the utils..." << std::endl;
}  // Destructor

std::vector<double> PointCloudUtils::rngToFreq(const std::vector<double> &rng, double bw, double c,
                                               double T) {
  size_t n = rng.size();
  std::vector<double> frequencies;
  for (size_t i = 0; i < n; ++i) {
    double f = (2 * bw * rng[i]) / (c * T);
    frequencies.push_back(f);
  }
  return frequencies;
}
std::vector<double> PointCloudUtils::dopToFreq(const std::vector<double> &d, double lam) {
  size_t n = d.size();
  std::vector<double> frequencies;
  for (size_t i = 0; i < n; ++i) {
    double f = (d[i] * 2) / lam;
    frequencies.push_back(f);
  }
  return frequencies;
}
double PointCloudUtils::freqToRng(double fr, double bw, double c, double T) {
  return (fr * c * T) / (2 * bw);
}
double PointCloudUtils::freqToDop(double fd, double lam) { return (lam * fd) / 2; }
std::pair<std::vector<double>, std::vector<double>> PointCloudUtils::getUpDownFreqs(
    std::vector<double> &rng, std::vector<double> &d, double bw, double c, double T, double lam) {
  std::vector<double> f_up, f_down, f_rng, f_dop;
  f_rng = rngToFreq(rng, bw, c, T);  // Range beat frequency
  f_dop = dopToFreq(d, lam);         // Doppler beat frequency

  for (size_t i = 0; i < f_rng.size(); ++i) {
    double fr = f_rng[i];
    double fd = f_dop[i];
    f_up.push_back(fr + fd);    // up-chirp beat is range + doppler:
    f_down.push_back(fr - fd);  // down-chirp beat is range – doppler:
  }
  return {f_up, f_down};
}
std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
PointCloudUtils::cart2sphere(const std::vector<double> &x, const std::vector<double> &y,
                             const std::vector<double> &z) {
  if (x.size() != y.size() || y.size() != z.size()) {
    throw std::runtime_error("Vectors r, az, and el must have the same size");
  }
  // Convert cartesian coordinates to spherical coordinates.
  std::vector<double> rng, az, el;
  size_t n = x.size();

  for (size_t i = 0; i < n; ++i) {
    double X = x[i];
    double Y = y[i];
    double Z = z[i];

    double r = std::sqrt((X * X) + (Y * Y) + (Z * Z));
    double azi = std::atan2(Y, X);
    double ele = 0.0;
    if (std::abs(r) > 1e-8) {  // or some small epsilon
      double ratio = Z / r;
      ele = std::asin(ratio);
    } else {
      // r is zero or very close
      ele = 0.0;
    }

    rng.push_back(r);
    az.push_back(azi);
    el.push_back(ele);
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
std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
PointCloudUtils::sphere2cart(const std::vector<double> &r, const std::vector<double> &az,
                             const std::vector<double> &el) {
  if (r.size() != az.size() || az.size() != el.size()) {
    throw std::runtime_error("Vectors r, az, and el must have the same size");
  }

  std::vector<double> x, y, z;
  size_t n = r.size();

  for (size_t i = 0; i < n; ++i) {
    double Ra = r[i];
    double Az = az[i];
    double El = el[i];

    double X = Ra * std::cos(El) * std::cos(Az);
    double Y = Ra * std::cos(El) * std::sin(Az);
    double Z = Ra * std::sin(El);

    x.push_back(X);
    y.push_back(Y);
    z.push_back(Z);
  }

  return {x, y, z};
}
