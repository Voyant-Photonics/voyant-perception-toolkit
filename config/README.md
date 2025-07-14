
# Parameter Guide for Range-Doppler Ambiguity Solver (RDAS)

This configuration file controls the behavior of the Range-Doppler ambiguity resolution algorithm using a combination of Probabilistic RANSAC, Median-based filtering, and top-elevation point sampling.

## 1. P-RANSAC Parameters

### a. rnsc_confidence:
Sets the confidence level for P-RANSAC. Higher values demand stronger inlier support to accept a model.
Effect: Affects the dynamic adjustment of the number of RANSAC iterations.

### b. rnsc_thres:
The residual error threshold used to determine inliers.
Effect: Controls how tightly a point must fit the model to be considered valid.

### c. rnsc_inlier_ratio:
The minimum ratio of inliers needed to consider a RANSAC model valid.
Effect: Prevents early convergence if too few inliers are found.

### d. rnsc_max_iter:
The hard cap on the number of RANSAC iterations.
Effect: Prevents infinite loops and caps computation.

### e. rnsc_alpha:
Sets the ± bounds around the RANSAC ego velocity for determining ambiguous Doppler values.
Effect: Defines the size of the acceptance band for valid points. Anything outside this range is considered ambiguous.

## 2. Median-Based Outlier Filtering

### a. med_alpha_up / med_alpha_down:
Define the upper and lower multipliers on the Mean Absolute Deviation (MAD) when using median-based filtering.
Effect: Controls how strict the outlier rejection is. Anything inside this range is considered valid.

### b. med_alpha_up_agg / med_alpha_down_agg:
Used when operating under aggressive filtering (e.g., high-speed scenarios).
Effect: Loosens the valid Doppler range when ambiguity is high.

## 3. Top Elevation Point Sampling

### a. num_pts_init:
Number of points initially sampled from the top elevation layers of the point cloud.
Effect: These points are assumed to be most trustworthy for estimating ego velocity.

### b. num_pts_recollect:
Number of points recollected from top elevations if initial sampling leads to disagreement between methods.
Effect: Improves reliability of ego velocity estimation when noise is present.

## Dynamic Method Selection Logic

### a. ego_recollect_threshold:
If the ego velocity from top-elevation points differs from both RANSAC and median estimates by more than this value, recollection is triggered.
Effect: Helps improve estimation by increasing sample size when the scene is ambiguous.

### b. min_ego_velocity_for_correction:
If the average ego velocity is below this, ambiguity correction is skipped.
Effect: Avoids overcorrecting at very low speeds (e.g., static scenes).

### c. ransac_preference_threshold:
If the difference between median and RANSAC ego velocities is less than this, RANSAC is preferred.
Effect: Acts as a tie-breaker for similar estimates.

### d. med_diff_aggr_thres:
If the median-based estimate deviates from top-point ego velocity by more than this, aggressive median thresholds are used.
Effect: Triggers relaxed validation in ambiguous or high-speed cases.

### e. top_ego_conf_th:
If the ego velocity from top elevation points exceeds this, it's considered highly reliable, enabling aggressive median mode.
Effect: Strengthens trust in top elevation readings in fast motion.
