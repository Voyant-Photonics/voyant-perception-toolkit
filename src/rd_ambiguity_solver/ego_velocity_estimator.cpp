// Copyright (c) 2024-2025 Voyant Photonics, Inc.
//
// This example code is licensed under the MIT License.
// See the LICENSE file in the repository root for full license text.

#include "ego_velocity_estimator.hpp"

EgoInlierEstimator::EgoInlierEstimator(PointCloudUtils &utils) : pc_utils_(utils)
{
    std::cout << "[+] Starting Ego Velocity Estimator" << std::endl;
}

EgoInlierEstimator::~EgoInlierEstimator()
{
    std::cout << "[+] Shutting Down Ego Velocity Estimator ..." << std::endl;
}

double EgoInlierEstimator::p_ransac_ego_velocity(const std::vector<double> &azimuth,
                                                 const std::vector<double> &elevation,
                                                 const std::vector<double> &doppler, double confidence = 0.99,
                                                 double threshold = 0.5, int max_iterations = 1000,
                                                 double rnsc_inlier_ratio = 0.1)
{
    size_t n = azimuth.size();
    std::vector<double> projection_factors;
    std::vector<double> doppler_valid;

    for (size_t i = 0; i < n; ++i)
    {
        double proj = std::cos(azimuth[i]) * std::cos(elevation[i]);
        if (std::abs(proj) > 1e-3)
        {
            projection_factors.push_back(proj);
            doppler_valid.push_back(doppler[i]);
        }
    }

    size_t n_points = projection_factors.size();
    if (n_points < 10)
    {
        std::vector<double> estimates;
        for (size_t i = 0; i < n; ++i)
        {
            double proj = std::cos(azimuth[i]) * std::cos(elevation[i]);
            if (std::abs(proj) > 1e-3)
            {
                estimates.push_back(doppler[i] / proj);
            }
        }
        std::nth_element(estimates.begin(), estimates.begin() + estimates.size() / 2, estimates.end());
        return estimates[estimates.size() / 2];
    }

    double best_velocity = 0.0;
    int best_inliers = 0;
    double best_inlier_ratio = 0.0;
    int iteration = 0;

    std::mt19937 rng(42);
    std::uniform_int_distribution<> dist(0, n_points - 1);

    while (iteration < max_iterations)
    {
        std::vector<int> sample_idx;
        while (sample_idx.size() < 3)
        {
            int idx = dist(rng);
            if (std::find(sample_idx.begin(), sample_idx.end(), idx) == sample_idx.end())
            {
                sample_idx.push_back(idx);
            }
        }

        std::vector<double> estimates;
        for (int idx : sample_idx)
        {
            estimates.push_back(doppler_valid[idx] / projection_factors[idx]);
        }
        std::nth_element(estimates.begin(), estimates.begin() + 1, estimates.end());
        double sample_velocity = estimates[1]; // median of 3

        int inliers = 0;
        for (size_t i = 0; i < n_points; ++i)
        {
            double predicted = sample_velocity * projection_factors[i];
            double residual = std::abs(doppler_valid[i] - predicted);
            if (residual < threshold)
            {
                inliers++;
            }
        }

        double inlier_ratio = static_cast<double>(inliers) / n_points;
        if (inliers > best_inliers)
        {
            best_inliers = inliers;
            best_velocity = sample_velocity;
            best_inlier_ratio = inlier_ratio;

            if (inlier_ratio > rnsc_inlier_ratio)
            {
                double prob_all_outliers = 1.0 - std::pow(inlier_ratio, 3);
                if (prob_all_outliers > 0.0)
                {
                    max_iterations = std::min(
                        max_iterations, static_cast<int>(std::log(1.0 - confidence) / std::log(prob_all_outliers)));
                }
            }
        }

        iteration++;
    }

    if (best_inlier_ratio > 0.2)
    {
        std::vector<double> refined;
        for (size_t i = 0; i < n_points; ++i)
        {
            double predicted = best_velocity * projection_factors[i];
            double residual = std::abs(doppler_valid[i] - predicted);
            if (residual < threshold)
            {
                refined.push_back(doppler_valid[i] / projection_factors[i]);
            }
        }

        if (refined.size() > 5)
        {
            std::nth_element(refined.begin(), refined.begin() + refined.size() / 2, refined.end());
            best_velocity = refined[refined.size() / 2];
        }
    }

    return best_velocity;
}

std::pair<double, std::vector<double>> EgoInlierEstimator::simpleMedianEstimator(const std::vector<double> &doppler,
                                                                                 const std::vector<double> &cosaz,
                                                                                 const std::vector<double> &cosel)
{
    std::vector<double> ego_velocities;
    size_t n = cosaz.size();

    for (size_t i = 0; i < n; ++i)
    {
        double ego_vel = doppler[i] / (cosaz[i] * cosel[i]);
        ego_velocities.push_back(ego_vel);
    }
    double med_ego_vel = pc_utils_.findMedian(ego_velocities);
    return {med_ego_vel, ego_velocities};
}

double EgoInlierEstimator::getTopEleEgo(const std::vector<double> &doppler, const std::vector<double> &az,
                                        const std::vector<double> &el, const double vertical_res,
                                        int total_points = 500)
{
    double VERTICAL_RES_DEG = (vertical_res) * 180.0 / M_PI; // Radians

    std::vector<int> elevation_bins(el.size());
    std::set<int, std::greater<int>> unique_bins;

    // Step 1: Compute elevation bins and store unique bins in reverse order
    for (size_t i = 0; i < el.size(); ++i)
    {
        int bin = static_cast<int>(std::round(el[i] * 180.0 / M_PI / VERTICAL_RES_DEG));
        elevation_bins[i] = bin;
        unique_bins.insert(bin);
    }
    std::vector<double> top_el_pts;
    for (int bin : unique_bins)
    {
        for (size_t i = 0; i < el.size(); ++i)
        {
            if (elevation_bins[i] == bin)
            {
                double proj = cos(az[i]) * cos(el[i]);
                if (std::abs(proj) > 1e-3)
                {
                    double ego = doppler[i] / proj;
                    top_el_pts.push_back(ego);
                }
            }
        }
        if (top_el_pts.size() >= total_points)
        {
            break;
        }
    }
    double best_ego = pc_utils_.findMedian(top_el_pts);

    return best_ego;
}
