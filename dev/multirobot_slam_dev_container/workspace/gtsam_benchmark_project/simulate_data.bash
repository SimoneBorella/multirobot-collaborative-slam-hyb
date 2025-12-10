#!/bin/bash

# Benchmark 0: Error on initial estimates

python3 scripts/simulate_data.py \
    --benchmark 0 \
    --exp 0 \
    --n_steps 10000 \
    \
    --landmark_err 0.0 \
    \
    --pose_pos_err 0.0 \
    --pose_theta_err 0.0 \
    \
    --obs_range_err_perc_min 0.0 \
    --obs_range_err_perc_max 0.0 \
    --obs_bearing_err 0.0 \
    \
    --odo_pos_err 0.0 \
    --odo_theta_err 0.0


# python3 scripts/simulate_data.py \
#     --benchmark 0 \
#     --exp 1 \
#     --n_steps 10000 \
#     \
#     --landmark_err 0.2 \
#     \
#     --pose_pos_err 0.2 \
#     --pose_theta_err 0.02 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.0 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.0 \
#     --odo_theta_err 0.0

# python3 scripts/simulate_data.py \
#     --benchmark 0 \
#     --exp 2 \
#     --n_steps 10000 \
#     \
#     --landmark_err 0.5 \
#     \
#     --pose_pos_err 0.5 \
#     --pose_theta_err 0.05 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.0 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.0 \
#     --odo_theta_err 0.0


# python3 scripts/simulate_data.py \
#     --benchmark 0 \
#     --exp 3 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.0 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.0 \
#     --odo_theta_err 0.0

# python3 scripts/simulate_data.py \
#     --benchmark 0 \
#     --exp 4 \
#     --n_steps 10000 \
#     \
#     --landmark_err 6.0 \
#     \
#     --pose_pos_err 6.0 \
#     --pose_theta_err 0.392 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.0 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.0 \
#     --odo_theta_err 0.0

# python3 scripts/simulate_data.py \
#     --benchmark 0 \
#     --exp 5 \
#     --n_steps 10000 \
#     \
#     --landmark_err 20.0 \
#     \
#     --pose_pos_err 20.0 \
#     --pose_theta_err 0.785 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.0 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.0 \
#     --odo_theta_err 0.0


# python3 scripts/simulate_data.py \
#     --benchmark 0 \
#     --exp 6 \
#     --n_steps 10000 \
#     \
#     --landmark_err 100.0 \
#     \
#     --pose_pos_err 100.0 \
#     --pose_theta_err 3.14 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.0 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.0 \
#     --odo_theta_err 0.0

# python3 scripts/simulate_data.py \
#     --benchmark 0 \
#     --exp 7 \
#     --n_steps 10000 \
#     \
#     --landmark_err 200.0 \
#     \
#     --pose_pos_err 200.0 \
#     --pose_theta_err 4.712 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.0 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.0 \
#     --odo_theta_err 0.0






























# # Benchmark 1: Error on measures

# python3 scripts/simulate_data.py \
#     --benchmark 1 \
#     --exp 8 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.0 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.0 \
#     --odo_theta_err 0.0


# python3 scripts/simulate_data.py \
#     --benchmark 1 \
#     --exp 9 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.05 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.025 \
#     --odo_theta_err 0.0

# python3 scripts/simulate_data.py \
#     --benchmark 1 \
#     --exp 10 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.05 \
#     --obs_range_err_perc_max 0.10 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.05 \
#     --odo_theta_err 0.0

# python3 scripts/simulate_data.py \
#     --benchmark 1 \
#     --exp 11 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.10 \
#     --obs_range_err_perc_max 0.15 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.075 \
#     --odo_theta_err 0.0

# python3 scripts/simulate_data.py \
#     --benchmark 1 \
#     --exp 12 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.15 \
#     --obs_range_err_perc_max 0.20 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.1 \
#     --odo_theta_err 0.0

# python3 scripts/simulate_data.py \
#     --benchmark 1 \
#     --exp 13 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.20 \
#     --obs_range_err_perc_max 0.25 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.125 \
#     --odo_theta_err 0.0


































# # Benchmark 2: Outliers

# python3 scripts/simulate_data.py \
#     --benchmark 2 \
#     --exp 14 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.05 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.025 \
#     --odo_theta_err 0.0 \
#     \
#     --obs_outlier_rate 0.0 \
#     --obs_outlier_range_err_perc_min 0.8 \
#     --obs_outlier_range_err_perc_max 1.2 \
#     --obs_outlier_bearing_err 0.0

# python3 scripts/simulate_data.py \
#     --benchmark 2 \
#     --exp 15 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.05 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.025 \
#     --odo_theta_err 0.0 \
#     \
#     --obs_outlier_rate 0.02 \
#     --obs_outlier_range_err_perc_min 0.8 \
#     --obs_outlier_range_err_perc_max 1.2 \
#     --obs_outlier_bearing_err 0.0

# python3 scripts/simulate_data.py \
#     --benchmark 2 \
#     --exp 16 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.05 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.025 \
#     --odo_theta_err 0.0 \
#     \
#     --obs_outlier_rate 0.04 \
#     --obs_outlier_range_err_perc_min 0.8 \
#     --obs_outlier_range_err_perc_max 1.2 \
#     --obs_outlier_bearing_err 0.0

# python3 scripts/simulate_data.py \
#     --benchmark 2 \
#     --exp 17 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.05 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.025 \
#     --odo_theta_err 0.0 \
#     \
#     --obs_outlier_rate 0.06 \
#     --obs_outlier_range_err_perc_min 0.8 \
#     --obs_outlier_range_err_perc_max 1.2 \
#     --obs_outlier_bearing_err 0.0

# python3 scripts/simulate_data.py \
#     --benchmark 2 \
#     --exp 18 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.05 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.025 \
#     --odo_theta_err 0.0 \
#     \
#     --obs_outlier_rate 0.08 \
#     --obs_outlier_range_err_perc_min 0.8 \
#     --obs_outlier_range_err_perc_max 1.2 \
#     --obs_outlier_bearing_err 0.0


# python3 scripts/simulate_data.py \
#     --benchmark 2 \
#     --exp 19 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.05 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.025 \
#     --odo_theta_err 0.0 \
#     \
#     --obs_outlier_rate 0.10 \
#     --obs_outlier_range_err_perc_min 0.8 \
#     --obs_outlier_range_err_perc_max 1.2 \
#     --obs_outlier_bearing_err 0.0


# python3 scripts/simulate_data.py \
#     --benchmark 2 \
#     --exp 20 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.05 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.025 \
#     --odo_theta_err 0.0 \
#     \
#     --obs_outlier_rate 0.12 \
#     --obs_outlier_range_err_perc_min 0.8 \
#     --obs_outlier_range_err_perc_max 1.2 \
#     --obs_outlier_bearing_err 0.0


# python3 scripts/simulate_data.py \
#     --benchmark 2 \
#     --exp 21 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.05 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.025 \
#     --odo_theta_err 0.0 \
#     \
#     --obs_outlier_rate 0.14 \
#     --obs_outlier_range_err_perc_min 0.8 \
#     --obs_outlier_range_err_perc_max 1.2 \
#     --obs_outlier_bearing_err 0.0



# python3 scripts/simulate_data.py \
#     --benchmark 2 \
#     --exp 22 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.05 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.025 \
#     --odo_theta_err 0.0 \
#     \
#     --obs_outlier_rate 0.16 \
#     --obs_outlier_range_err_perc_min 0.8 \
#     --obs_outlier_range_err_perc_max 1.2 \
#     --obs_outlier_bearing_err 0.0


# python3 scripts/simulate_data.py \
#     --benchmark 2 \
#     --exp 23 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.05 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.025 \
#     --odo_theta_err 0.0 \
#     \
#     --obs_outlier_rate 0.18 \
#     --obs_outlier_range_err_perc_min 0.8 \
#     --obs_outlier_range_err_perc_max 1.2 \
#     --obs_outlier_bearing_err 0.0

# python3 scripts/simulate_data.py \
#     --benchmark 2 \
#     --exp 24 \
#     --n_steps 10000 \
#     \
#     --landmark_err 1.5 \
#     \
#     --pose_pos_err 1.5 \
#     --pose_theta_err 0.1 \
#     \
#     --obs_range_err_perc_min 0.0 \
#     --obs_range_err_perc_max 0.05 \
#     --obs_bearing_err 0.0 \
#     \
#     --odo_pos_err 0.025 \
#     --odo_theta_err 0.0 \
#     \
#     --obs_outlier_rate 0.20 \
#     --obs_outlier_range_err_perc_min 0.8 \
#     --obs_outlier_range_err_perc_max 1.2 \
#     --obs_outlier_bearing_err 0.0
