// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "doppler_ambiguity_resolver.hpp"

AmbiguitySolver::AmbiguitySolver(const std::string &yaml_path)
    : pc_utils_(), ego_solver(pc_utils_) {
  std::cout << "[+] Starting the solver" << std::endl;
  config = loadParams(yaml_path);
}
AmbiguitySolver::~AmbiguitySolver() {
  std::cout << "[+] Shutting down the solver ..." << std::endl;
}

Params AmbiguitySolver::loadParams(const std::string &yaml_path) {
  Params solver_params_;

  try {
    YAML::Node config = YAML::LoadFile(yaml_path);
    auto sensor_config = config["parameters"];

    if (!sensor_config) {
      throw std::runtime_error(
          "[+] There seems to be an issue with the YAML file. "
          "Please check the file path and format.");
    }
    solver_params_.input_file = sensor_config["input_file"].as<std::string>();
    solver_params_.output_file = sensor_config["output_file"].as<std::string>();
    solver_params_.T = sensor_config["T"].as<double>();
    solver_params_.BW = sensor_config["BW"].as<double>();
    solver_params_.LAMBDA = sensor_config["LAMBDA"].as<double>();
    solver_params_.min_pts = sensor_config["min_pts"].as<int>();
    solver_params_.rnsc_confidence = sensor_config["rnsc_confidence"].as<double>();
    solver_params_.rnsc_thres = sensor_config["rnsc_thres"].as<double>();
    solver_params_.rnsc_inlier_ratio = sensor_config["rnsc_inlier_ratio"].as<double>();
    solver_params_.rnsc_max_iter = sensor_config["rnsc_max_iter"].as<double>();
    solver_params_.rnsc_alpha = sensor_config["rnsc_alpha"].as<double>();
    solver_params_.med_alpha_up = sensor_config["med_alpha_up"].as<double>();
    solver_params_.med_alpha_down = sensor_config["med_alpha_down"].as<double>();
    solver_params_.med_alpha_up_agg = sensor_config["med_alpha_up_agg"].as<double>();
    solver_params_.med_alpha_down_agg = sensor_config["med_alpha_down_agg"].as<double>();
    solver_params_.num_pts_init = sensor_config["num_pts_init"].as<int>();
    solver_params_.num_pts_recollect = sensor_config["num_pts_recollect"].as<int>();
    solver_params_.ego_recollect_threshold = sensor_config["ego_recollect_threshold"].as<double>();
    solver_params_.min_ego_velocity_for_correction =
        sensor_config["min_ego_velocity_for_correction"].as<double>();
    solver_params_.ransac_preference_threshold =
        sensor_config["ransac_preference_threshold"].as<double>();
    solver_params_.med_diff_aggr_thres = sensor_config["med_diff_aggr_thres"].as<double>();
    solver_params_.top_ego_conf_th = sensor_config["top_ego_conf_th"].as<double>();
  } catch (const std::exception &e) {
    std::cerr << "Error parsing YAML: " << e.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }

  return solver_params_;
}

std::pair<std::vector<double>, std::vector<double>> AmbiguitySolver::recoverRangeDoppler(
    std::vector<double> amb_rng, std::vector<double> amb_dop) {
  std::vector<double> new_rng, new_dop, fr_up, fr_down;
  std::pair<std::vector<double>, std::vector<double>> chirps =
      pc_utils_.getUpDownFreqs(amb_rng, amb_dop, config.BW, config.T, config.LAMBDA);
  fr_up = std::get<0>(chirps);
  fr_down = std::get<1>(chirps);

  for (size_t i = 0; i < fr_up.size(); ++i) {
    double fup = fr_up[i];
    double fdnw = fr_down[i];

    if (fup < fdnw) {
      fup = -fup;
    } else if (fdnw < fup) {
      fdnw = -fdnw;
    }

    // Calculate the new Range and Doppler frequencies from new Up and Down Chirps.
    double r_new = pc_utils_.freqToRng(((fup + fdnw) / 2), config.BW, config.T);
    double d_new = pc_utils_.freqToDop(((fup - fdnw) / 2), config.LAMBDA);

    new_rng.push_back(r_new);
    new_dop.push_back(d_new);
  }

  return {new_rng, new_dop};
}

void AmbiguitySolver::writeRecoveredPointsToCSV(const std::string &filename,
                                                const std::vector<std::array<double, 12>> &points) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return;
  }

  // Write header
  file << "x,y,z,doppler,point_idx,frame_idx,timestamp,drop_reason,snr_linear,"
          "calibrated_reflectance,noise_mean_estimate,min_ramp_snr\n";

  // Write data
  for (const auto &row : points) {
    for (size_t i = 0; i < row.size(); ++i) {
      file << row[i];
      if (i != row.size() - 1) {
        file << ",";
      }
    }
    file << "\n";
  }

  file.close();
  std::cout << "Saved file at: " << filename << std::endl;
}

bool AmbiguitySolver::validatePointCoordinates(const PointDataWrapper &pt) {
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

std::vector<std::array<double, 12>> AmbiguitySolver::Solver(const VoyantFrameWrapper &frame,
                                                            size_t frame_id) {
  std::vector<double> xs, ys, zs, vels, snrs, calibrated_reflectances, noise_mean_estimates,
      min_ramp_snrs;
  std::vector<uint32_t> timestamps, pnt_idxs;
  std::vector<uint8_t> drop_rsns;
  std::vector<std::array<double, 12>> recovered_points;

  const auto &points = frame.points();

  for (const auto &pt : points) {
    bool is_valid = validatePointCoordinates(pt);  // Check if the point is valid.
    if (is_valid) {
      xs.push_back(pt.x());
      ys.push_back(pt.y());
      zs.push_back(pt.z());
      vels.push_back(pt.radial_vel());
      timestamps.push_back(pt.timestamp_nanosecs());
      drop_rsns.push_back(static_cast<uint8_t>(pt.drop_reason()));
      pnt_idxs.push_back(pt.point_index());
      snrs.push_back(pt.snr_linear());
      calibrated_reflectances.push_back(pt.calibrated_reflectance());
      noise_mean_estimates.push_back(pt.noise_mean_estimate());
      min_ramp_snrs.push_back(pt.min_ramp_snr());
    }
  }

  if (!xs.empty()) {
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<double> rng, az, el, cos_az, cos_el;
    std::vector<double> amb_rng, amb_az, amb_el, amb_dop;
    std::vector<double> nx, ny, nz;
    double inlier_v_ego = 0.0;
    std::vector<bool> amb_mask;

    // Convert to Polar Coordinates
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> polar_coors =
        pc_utils_.cart2sphere(xs, ys, zs);
    rng = std::get<0>(polar_coors);
    az = std::get<1>(polar_coors);
    el = std::get<2>(polar_coors);

    // Reserve space for safety
    cos_az.reserve(az.size());
    cos_el.reserve(el.size());
    for (size_t i = 0; i < az.size(); ++i) {
      cos_az.push_back(cos(az[i]));
      cos_el.push_back(cos(el[i]));
    }

    // First get the RANSAC based ego vel
    double rnsc_v_ego =
        ego_solver.p_ransac_ego_velocity(az, el, vels, config.rnsc_confidence, config.rnsc_thres,
                                         config.rnsc_max_iter, config.rnsc_inlier_ratio);

    // Get top elevation's ego
    double top_ele_v_ego =
        ego_solver.getTopEleEgo(vels, az, el, config.VERTICAL_RES_DEG, config.num_pts_init);

    // Get the simple median ego velocity
    std::vector<double> ego_velocities;
    double med_v_ego;
    std::pair<double, std::vector<double>> med_estimator =
        ego_solver.simpleMedianEstimator(vels, cos_az, cos_el);
    med_v_ego = std::get<0>(med_estimator);
    ego_velocities = std::get<1>(med_estimator);

    double rnsc_diff = abs(top_ele_v_ego - rnsc_v_ego);
    double med_diff = abs(top_ele_v_ego - med_v_ego);

    // Recollection step
    if (med_diff >= config.ego_recollect_threshold && rnsc_diff >= config.ego_recollect_threshold) {
      // Recollect more points as this probably means the previously collected points
      // were noisy
      top_ele_v_ego =
          ego_solver.getTopEleEgo(vels, az, el, config.VERTICAL_RES_DEG, config.num_pts_recollect);
    }
    rnsc_diff = abs(top_ele_v_ego - rnsc_v_ego);
    med_diff = abs(top_ele_v_ego - med_v_ego);
    bool use_median_method = (med_diff < rnsc_diff);
    double rnd_top_ego = std::round(top_ele_v_ego * 10.0) / 10.0;
    double rnd_med_ego = std::round(med_v_ego * 10.0) / 10.0;
    double rnd_rnsc_ego = std::round(rnsc_v_ego * 10.0) / 10.0;

    double best_ego = std::round((top_ele_v_ego + med_v_ego + rnsc_v_ego) / 3.0 * 10.0) / 10.0;
    // Initialize amb mask with correct size before using it.
    amb_mask.resize(ego_velocities.size(), false);
    if (best_ego <= config.min_ego_velocity_for_correction) {
      // std::cout << "Skipping correction, no ambiguous points" << best_ego << std::endl;
      inlier_v_ego = rnsc_v_ego;
      amb_mask.assign(ego_velocities.size(), false);
    }

    else if (med_diff <= config.ransac_preference_threshold &&
             rnsc_diff <= config.ransac_preference_threshold) {
      inlier_v_ego = rnsc_v_ego;
      double alpha = config.rnsc_alpha;
      double az_thres_min = rnsc_v_ego - alpha;
      double az_thres_max = rnsc_v_ego + alpha;

      amb_mask.resize(ego_velocities.size());
      for (size_t i = 0; i < ego_velocities.size(); ++i) {
        amb_mask[i] = !((ego_velocities[i] <= az_thres_max) && (ego_velocities[i] >= az_thres_min));
      }
      // std::cout << "[+] Using Ransac logic" << std::endl;
    }

    else if (rnd_top_ego == rnd_med_ego && rnd_med_ego == rnd_rnsc_ego) {
      // std::cout << "[+] Looks like all the vels are same, prefer RANSAC" << std::endl;
      inlier_v_ego = rnsc_v_ego;
      double alpha = config.rnsc_alpha;
      double az_thres_min = rnsc_v_ego - alpha;
      double az_thres_max = rnsc_v_ego + alpha;

      amb_mask.resize(ego_velocities.size());
      for (size_t i = 0; i < ego_velocities.size(); ++i) {
        amb_mask[i] = !((ego_velocities[i] <= az_thres_max) && (ego_velocities[i] >= az_thres_min));
      }
      // std::cout << "[+] Using Ransac logic AGAIN" << std::endl;
    }

    else if (use_median_method) {
      inlier_v_ego = med_v_ego;
      // std::cout << "[+] Using median logic" << inlier_v_ego << std::endl;
      std::vector<double> abs_devs;
      for (double v : ego_velocities) {
        abs_devs.push_back(std::abs(v - inlier_v_ego));
      }
      double mad = pc_utils_.findMedian(abs_devs);  // Use Median Absolute Deviation when
                                                    // using median or Std when using mean

      double alpha_up = config.med_alpha_up;
      double alpha_dnw = config.med_alpha_down;
      if (med_diff >= config.med_diff_aggr_thres && top_ele_v_ego >= config.top_ego_conf_th) {
        // std::cout << "Aggressive alpha" << std::endl;
        alpha_dnw = config.med_alpha_down_agg;
        alpha_up = config.med_alpha_up_agg;
      }

      double az_thres_min = inlier_v_ego - alpha_dnw * mad;
      double az_thres_max = inlier_v_ego + alpha_up * mad;

      amb_mask.resize(ego_velocities.size());
      for (size_t i = 0; i < ego_velocities.size(); ++i) {
        /*
        TODO: Find a better logic such that we don't have to
        invert the mask logic for RANSAC and Median based ego selectors
        */
        amb_mask[i] = (ego_velocities[i] <= az_thres_max) && (ego_velocities[i] >= az_thres_min);
      }
    } else if (!use_median_method) {
      // std::cout << "[+] Using Ransac logic" << std::endl;
      inlier_v_ego = rnsc_v_ego;
      double alpha = config.rnsc_alpha;
      double az_thres_min = rnsc_v_ego - alpha;
      double az_thres_max = rnsc_v_ego + alpha;

      amb_mask.resize(ego_velocities.size());
      for (size_t i = 0; i < ego_velocities.size(); ++i) {
        amb_mask[i] = !((ego_velocities[i] <= az_thres_max) && (ego_velocities[i] >= az_thres_min));
      }
    }

    // Reserve space for safety
    amb_rng.clear();
    amb_az.clear();
    amb_el.clear();
    amb_dop.clear();

    size_t amb_count = std::count(amb_mask.begin(), amb_mask.end(), true);
    amb_rng.reserve(amb_count);
    amb_az.reserve(amb_count);
    amb_el.reserve(amb_count);
    amb_dop.reserve(amb_count);

    std::vector<size_t> amb_indices;
    for (size_t i = 0; i < amb_mask.size(); ++i) {
      if (amb_mask[i] && vels[i] > 0.0) {
        // Here filter out the points that are in opposite doppler, hence a portion
        // of the dynamic object problem is solved
        amb_indices.push_back(i);
        amb_rng.push_back(rng[i]);
        amb_az.push_back(az[i]);
        amb_el.push_back(el[i]);
        amb_dop.push_back(vels[i]);
      } else {
        std::array<double, 12> row = {xs[i],
                                      ys[i],
                                      zs[i],
                                      vels[i],
                                      static_cast<double>(pnt_idxs[i]),
                                      static_cast<double>(frame_id),
                                      static_cast<double>(timestamps[i]),
                                      static_cast<double>(drop_rsns[i]),
                                      snrs[i],
                                      calibrated_reflectances[i],
                                      noise_mean_estimates[i],
                                      min_ramp_snrs[i]};
        recovered_points.push_back(row);
      }
    }
    if (!amb_rng.empty()) {
      std::cout << "Processing " << amb_rng.size() << " ambiguous points" << std::endl;

      std::pair<std::vector<double>, std::vector<double>> recovered_rngs_dops =
          recoverRangeDoppler(amb_rng, amb_dop);
      std::vector<double> new_rng = std::get<0>(recovered_rngs_dops);
      std::vector<double> new_dop = std::get<1>(recovered_rngs_dops);
      std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> new_carts =
          pc_utils_.sphere2cart(new_rng, amb_az, amb_el);
      nx = std::get<0>(new_carts);
      ny = std::get<1>(new_carts);
      nz = std::get<2>(new_carts);

      for (size_t j = 0; j < amb_indices.size(); ++j) {
        size_t original_idx = amb_indices[j];
        std::array<double, 12> row = {
            nx[j], ny[j], nz[j],
            new_dop[j],                                   // Use j for recovered data
            static_cast<double>(pnt_idxs[original_idx]),  // Use
                                                          // original_idx
                                                          // for metadata
            static_cast<double>(frame_id), static_cast<double>(timestamps[original_idx]),
            static_cast<double>(drop_rsns[original_idx]), snrs[original_idx],
            calibrated_reflectances[original_idx], noise_mean_estimates[original_idx],
            min_ramp_snrs[original_idx]};
        recovered_points.push_back(row);
      }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "[+] Time taken to process a frame: " << duration.count() / 1e6 << " seconds"
              << std::endl;
  }
  // Clear for next frame
  xs.clear();
  ys.clear();
  zs.clear();
  vels.clear();
  timestamps.clear();
  drop_rsns.clear();
  pnt_idxs.clear();
  // }
  return recovered_points;
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <input_yaml_file_path>\n";
    return EXIT_FAILURE;
  }

  const std::string yaml_file_path = argv[1];  // yaml_file_path should be passed as a command line
                                               // argument
  // Create the playback instance
  AmbiguitySolver rngDopSolver(yaml_file_path);
  VoyantPlayback player;
  if (!player.isValid()) {
    std::cerr << "Failed to create VoyantPlayback instance: " << player.getLastError() << std::endl;
    return EXIT_FAILURE;
  }

  // Open the file
  std::cout << "Opening file: " << rngDopSolver.config.input_file << std::endl;
  if (!player.openFile(rngDopSolver.config.input_file)) {
    std::cerr << "Failed to open file: " << player.getLastError() << std::endl;
    return EXIT_FAILURE;
  }

  if (rngDopSolver.config.output_file.empty()) {
    std::cerr << "Output path cannot be empty" << std::endl;
    return EXIT_FAILURE;
  }

  // Create full file path using std::filesystem
  std::filesystem::path filePath = std::filesystem::path(rngDopSolver.config.input_file);
  auto start = std::chrono::high_resolution_clock::now();

  std::vector<std::array<double, 12>> recovered_cloud;
  while (player.nextFrame()) {
    size_t frameIndex = player.currentFrameIndex();
    std::cout << "[+] Processing frame: " << frameIndex << std::endl;
    const VoyantFrameWrapper &frame = player.currentFrame();
    // Start the solver
    std::vector<std::array<double, 12>> recovered_points = rngDopSolver.Solver(frame, frameIndex);
    recovered_cloud.insert(recovered_cloud.end(), recovered_points.begin(), recovered_points.end());
  }
  // Save the output to desired file
  std::filesystem::path OutfilePath = std::filesystem::path(rngDopSolver.config.output_file);
  rngDopSolver.writeRecoveredPointsToCSV(OutfilePath, recovered_cloud);
  // Check if we exited the loop due to an error
  if (!player.getLastError().empty()) {
    std::cerr << "[-] Error during playback: " << player.getLastError() << std::endl;
    return EXIT_FAILURE;
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  std::cout << "[+] Time taken to read the file: " << duration.count() / 1e6 << " seconds"
            << std::endl;

  // Print some details about the playback
  std::cout << "\nConversion complete!" << std::endl;
  std::cout << "[+] Processed " << player.getFramesProcessed() << " frames" << std::endl;
  std::cout << "[+] Saved output at: " << rngDopSolver.config.output_file << std::endl;
}
