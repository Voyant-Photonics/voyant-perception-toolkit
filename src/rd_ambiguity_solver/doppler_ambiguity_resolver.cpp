// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "doppler_ambiguity_resolver.hpp"

AmbiguitySolver::AmbiguitySolver(const std::string &yaml_path)
    : pc_utils_(), ego_solver_(pc_utils_) {
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
    solver_params_.VERTICAL_RES_DEG = sensor_config["VERTICAL_RES_DEG"].as<double>();
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

std::pair<double, double> AmbiguitySolver::recoverSingleRangeDoppler(double amb_rng,
                                                                     double amb_dop) {
  double f_rng = pc_utils_.rngToFreq(amb_rng, config.BW, config.T);  // Convert Range to frequency
  double f_dop = pc_utils_.dopToFreq(amb_dop, config.LAMBDA);  // Convert Doppler to Frequency.

  double f_up = (f_rng + f_dop);
  double f_down = (f_rng - f_dop);

  if (f_up < f_down) {
    f_up = -f_up;
  } else if (f_down < f_up) {
    f_down = -f_down;
  }
  double r_new = pc_utils_.freqToRng(((f_up + f_down) / 2), config.BW, config.T);
  double d_new = pc_utils_.freqToDop(((f_up - f_down) / 2), config.LAMBDA);

  return {r_new, d_new};
}

std::vector<VoyantPoint> AmbiguitySolver::Solver(const VoyantFrameWrapper &frame, size_t frame_id) {
  const auto &points = frame.points();
  std::vector<VoyantPoint> recovered_points;

  if (points.empty()) {
    return recovered_points;
  }
  auto start = std::chrono::high_resolution_clock::now();

  // Create filtered view of valid points
  std::vector<size_t> valid_indices;
  valid_indices.reserve(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    if (pc_utils_.validatePointCoordinates(points[i])) {  // Check if the point is valid.
      valid_indices.emplace_back(i);
    }
  }
  if (valid_indices.empty()) {
    return recovered_points;
  }

  const size_t valid_count = valid_indices.size();

  // Process the coordinates directly from the pointcloud
  std::vector<double> rng(valid_count), az(valid_count), el(valid_count);
  std::vector<double> vels(valid_count), cos_az(valid_count), cos_el(valid_count);

  for (size_t i = 0; i < valid_count; ++i) {
    const auto &pt = points[valid_indices[i]];
    double x = pt.x(), y = pt.y(), z = pt.z();
    double doppler = pt.radial_vel();

    // Convert to Polar Coordinates
    std::tuple<double, double, double> polar_coors = pc_utils_.cart2sphere(x, y, z);

    rng[i] = std::get<0>(polar_coors);
    az[i] = std::get<1>(polar_coors);
    el[i] = std::get<2>(polar_coors);
    vels[i] = doppler;

    cos_az[i] = std::cos(az[i]);
    cos_el[i] = std::cos(el[i]);
  }

  double inlier_v_ego = 0.0;
  std::vector<bool> amb_mask(valid_count, false);

  // First get the RANSAC based ego vel
  double rnsc_v_ego =
      ego_solver_.p_ransac_ego_velocity(az, el, vels, config.rnsc_confidence, config.rnsc_thres,
                                        config.rnsc_max_iter, config.rnsc_inlier_ratio);

  // Get top elevation's ego
  double top_ele_v_ego =
      ego_solver_.getTopEleEgo(vels, az, el, config.VERTICAL_RES_DEG, config.num_pts_init);

  // Get the simple median ego velocity
  std::vector<double> ego_velocities;
  double med_v_ego;
  std::pair<double, std::vector<double>> med_estimator =
      ego_solver_.simpleMedianEstimator(vels, cos_az, cos_el);
  med_v_ego = std::get<0>(med_estimator);
  ego_velocities = std::get<1>(med_estimator);

  double rnsc_diff = abs(top_ele_v_ego - rnsc_v_ego);
  double med_diff = abs(top_ele_v_ego - med_v_ego);

  // Recollection step
  if (med_diff >= config.ego_recollect_threshold && rnsc_diff >= config.ego_recollect_threshold) {
    // Recollect more points as this probably means the previously collected points
    // were noisy
    top_ele_v_ego =
        ego_solver_.getTopEleEgo(vels, az, el, config.VERTICAL_RES_DEG, config.num_pts_recollect);
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
  }

  else if (rnd_top_ego == rnd_med_ego && rnd_med_ego == rnd_rnsc_ego) {
    inlier_v_ego = rnsc_v_ego;
    double alpha = config.rnsc_alpha;
    double az_thres_min = rnsc_v_ego - alpha;
    double az_thres_max = rnsc_v_ego + alpha;

    amb_mask.resize(ego_velocities.size());
    for (size_t i = 0; i < ego_velocities.size(); ++i) {
      amb_mask[i] = !((ego_velocities[i] <= az_thres_max) && (ego_velocities[i] >= az_thres_min));
    }
  }

  else if (use_median_method) {
    inlier_v_ego = med_v_ego;
    std::vector<double> abs_devs;
    for (double v : ego_velocities) {
      abs_devs.push_back(std::abs(v - inlier_v_ego));
    }
    double mad = pc_utils_.findMedian(abs_devs);  // Use Median Absolute Deviation when
                                                  // using median or Std when using mean

    double alpha_up = config.med_alpha_up;
    double alpha_dnw = config.med_alpha_down;
    if (med_diff >= config.med_diff_aggr_thres && top_ele_v_ego >= config.top_ego_conf_th) {
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

  recovered_points.reserve(valid_count);

  for (size_t i = 0; i < valid_count; ++i) {
    const auto &original_pt = points[valid_indices[i]];

    if (amb_mask[i] && vels[i] > 0.0) {
      auto [new_rng, new_dop] = recoverSingleRangeDoppler(rng[i], vels[i]);

      // Convert back to cartesian
      std::tuple<double, double, double> new_cart = pc_utils_.sphere2cart(new_rng, az[i], el[i]);
      double nx = std::get<0>(new_cart);
      double ny = std::get<1>(new_cart);
      double nz = std::get<2>(new_cart);

      VoyantPoint recovered_pt;
      recovered_pt.x = nx;
      recovered_pt.y = ny;
      recovered_pt.z = nz;
      recovered_pt.radial_vel = new_dop;
      recovered_pt.frame_idx = frame_id;
      recovered_pt.point_idx = original_pt.point_index();
      recovered_pt.frame_timestamp_seconds = frame.header().timestampSeconds();
      recovered_pt.frame_timestamp_nanoseconds = frame.header().timestampNanoseconds();
      recovered_pt.nanosecs_since_frame = original_pt.timestamp_nanosecs();
      recovered_pt.snr_linear = original_pt.snr_linear();
      recovered_pt.calibrated_reflectance = original_pt.calibrated_reflectance();
      recovered_pt.noise_mean_estimate = original_pt.noise_mean_estimate();
      recovered_pt.min_ramp_snr = original_pt.min_ramp_snr();
      recovered_pt.drop_reason = static_cast<uint16_t>(original_pt.drop_reason());
      recovered_points.emplace_back(std::move(recovered_pt));
    } else {
      // Copy original point
      VoyantPoint recovered_pt;
      recovered_pt.x = original_pt.x();
      recovered_pt.y = original_pt.y();
      recovered_pt.z = original_pt.z();
      recovered_pt.radial_vel = original_pt.radial_vel();
      recovered_pt.frame_idx = frame_id;
      recovered_pt.point_idx = original_pt.point_index();
      recovered_pt.frame_timestamp_seconds = frame.header().timestampSeconds();
      recovered_pt.frame_timestamp_nanoseconds = frame.header().timestampNanoseconds();
      recovered_pt.nanosecs_since_frame = original_pt.timestamp_nanosecs();
      recovered_pt.snr_linear = original_pt.snr_linear();
      recovered_pt.calibrated_reflectance = original_pt.calibrated_reflectance();
      recovered_pt.noise_mean_estimate = original_pt.noise_mean_estimate();
      recovered_pt.min_ramp_snr = original_pt.min_ramp_snr();
      recovered_pt.drop_reason = static_cast<uint16_t>(original_pt.drop_reason());
      recovered_points.emplace_back(std::move(recovered_pt));
    }
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  std::cout << "[+] Time taken to process a frame: " << duration.count() / 1e6 << " seconds"
            << std::endl;

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
  PointCloudLogger pc_logger;  // Initialize the logger
  VoyantPlayback player(0.0);  // Setting this to 0.0 to playback as fast as possible.
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

  std::filesystem::path OutfilePath = std::filesystem::path(rngDopSolver.config.output_file);
  if (!pc_logger.initializeCSV(OutfilePath)) {
    std::cerr << "Failed to initialize CSV file!" << std::endl;
    return EXIT_FAILURE;
  }

  if (rngDopSolver.config.output_file.empty()) {
    std::cerr << "Output path cannot be empty" << std::endl;
    return EXIT_FAILURE;
  }

  // Create full file path using std::filesystem
  std::filesystem::path filePath = std::filesystem::path(rngDopSolver.config.input_file);
  auto start = std::chrono::high_resolution_clock::now();

  std::vector<VoyantPoint> recovered_cloud;
  while (player.nextFrame()) {
    size_t frameIndex = player.currentFrameIndex();
    std::cout << "[+] Processing frame: " << frameIndex << std::endl;
    const VoyantFrameWrapper &frame = player.currentFrame();

    // Start the solver
    std::vector<VoyantPoint> recovered_points = rngDopSolver.Solver(frame, frameIndex);

    // Write points immediately to CSV
    pc_logger.writePointsSequential(recovered_points, frame);
  }
  // Close the CSV file
  pc_logger.finalizeCSV();

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
