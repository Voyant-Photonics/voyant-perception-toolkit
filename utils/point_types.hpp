// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#pragma once
#include <pcl/point_types.h>

struct EIGEN_ALIGN16 VoyantPoint {
  PCL_ADD_POINT4D;
  double radial_vel;
  uint32_t frame_idx;
  uint32_t point_idx;
  int64_t frame_timestamp_seconds;
  int32_t frame_timestamp_nanoseconds;
  int32_t nanosecs_since_frame;
  double snr_linear;
  double calibrated_reflectance;
  double noise_mean_estimate;
  double min_ramp_snr;
  uint16_t drop_reason;

  VoyantPoint()
      : x(0.f),
        y(0.f),
        z(0.f),
        radial_vel(0.f),
        frame_idx(0),
        point_idx(0),
        frame_timestamp_seconds(0),
        frame_timestamp_nanoseconds(0),
        nanosecs_since_frame(0),
        snr_linear(0.0),
        calibrated_reflectance(0.0),
        noise_mean_estimate(0.0),
        min_ramp_snr(0.0),
        drop_reason(0) {}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    VoyantPoint,
    (double, x, x)(double, y, y)(double, z, z)(double, radial_vel, radial_vel)(uint32_t, frame_idx,
                                                                               frame_idx)(
        uint32_t, point_idx, point_idx)(int64_t, frame_timestamp_seconds, frame_timestamp_seconds)(
        int32_t, frame_timestamp_nanoseconds,
        frame_timestamp_nanoseconds)(int32_t, nanosecs_since_frame, nanosecs_since_frame)(
        double, snr_linear, snr_linear)(double, calibrated_reflectance, calibrated_reflectance)(
        double, noise_mean_estimate, noise_mean_estimate)(double, min_ramp_snr,
                                                          min_ramp_snr)(uint16_t, drop_reason,
                                                                        drop_reason))

struct EIGEN_ALIGN16 VoyantLarkPoint {
  PCL_ADD_POINT4D;
  double radial_vel;
  uint32_t frame_idx;
  uint32_t point_idx;
  int64_t frame_timestamp_seconds;
  int32_t frame_timestamp_nanoseconds;
  int32_t nanosecs_since_frame;
  double snr_linear;
  double reflectivity;
  double noise_mean_estimate;
  double polarization;
  uint16_t drop_reason;

  VoyantLarkPoint()
      : x(0.f),
        y(0.f),
        z(0.f),
        radial_vel(0.f),
        frame_idx(0),
        point_idx(0),
        frame_timestamp_seconds(0),
        frame_timestamp_nanoseconds(0),
        nanosecs_since_frame(0),
        snr_linear(0.0),
        reflectivity(0.0),
        noise_mean_estimate(0.0),
        polarization(0.0),
        drop_reason(0) {}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    VoyantLarkPoint,
    (double, x, x)(double, y, y)(double, z, z)(double, radial_vel, radial_vel)(uint32_t, frame_idx,
                                                                               frame_idx)(
        uint32_t, point_idx, point_idx)(int64_t, frame_timestamp_seconds, frame_timestamp_seconds)(
        int32_t, frame_timestamp_nanoseconds,
        frame_timestamp_nanoseconds)(int32_t, nanosecs_since_frame,
                                     nanosecs_since_frame)(double, snr_linear, snr_linear)(
        double, reflectivity, reflectivity)(double, noise_mean_estimate, noise_mean_estimate)(
        double, polarization, polarization)(uint16_t, drop_reason, drop_reason))
