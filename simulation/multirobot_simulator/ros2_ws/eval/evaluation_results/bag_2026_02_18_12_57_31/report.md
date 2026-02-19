# Multirobot Collaborative SLAM Evaluation Report

**Bag ID:** bag_2026_02_18_12_57_31

**Experiment Date:** 2026-02-18

**Experiment Time:** 12:57:31

**Evaluation Date:** 2026-02-18

**Robots:**
`robot_0`

## Timing evaluation:

**Bag record time:** 915066 ms

| Robot ID | Success | Last cmd (linear_x, angular_z) | Execution time |
|----------|---------|--------------------------------|----------------|
| `robot_0` | No | 0.000 m/s, 0.000 rad/s | **913235 ms** (913.235 s) |

**Exploration gap:** 0 ms

**Total time:** 913235 ms

## Localization evaluation:

| Robot ID | Success | Ground truth | Traveled distance | Final distance error | Distance MAE | Distance RMSE | Distance STD | Final yaw error | Yaw MAE | Yaw RMSE | Yaw STD |
|----------|---------|--------------|-------------------|----------------------|--------------|---------------|--------------|-----------------|---------|----------|---------|
| `robot_0` | No | Yes | 192.82 m | 0.38 m | 0.03 m | 0.06 m | 0.05 m | 0.07 rad | 0.01 rad | 0.02 rad | 0.02 rad |

### Localization results:

**Localization result:**

![Localization](localization/localization.png)

**Distance error:**

![Distance error](localization/distance_error.png)

**Yaw error:**

![Yaw error](localization/yaw_error.png)

## Mapping evaluation:

**Map resolution:** 0.05 m/pixel

**Dimension:** 800 x 800 cells (40.00 x 40.00 m)

**Map origin:** (-20.0, -20.0, 0.0)

**Explored area:** 434.03 m²

**Explored area ratio:** 94.86%

| Robot ID | Exploration area | Exploration percentage |
|----------|------------------|------------------------|
| `robot_0` | 433.16 m² | 99.80% |

**Mapping accuracy**: 0.9444 (4082138/4322500 compared cells)
| Quantity | Value |
|--------|-------|
| True Positives (TP)  | 189451 |
| True Negatives (TN)  | 3892687 |
| False Positives (FP) | 83649 |
| False Negatives (FN) | 156713 |

| Metric | Value |
|--------|-------|
| True Positive Rate (TPR)  | 0.5472868351417247 |
| True Negative Rate (TNR)  | 0.9789632968642489 |
| False Positive Rate (FPR) | 0.021036703135751105 |
| False Negative Rate (FNR) | 0.4527131648582753 |
### Mapping results:

**Map ground truth:**

![Map](mapping/map_ground_truth.png)

**Map result:**

![Map](mapping/resampled_map_on_gt.png)

**Map error:**

![Map](mapping/error_map.png)

**Map confusion:**

![Map](mapping/confusion_map.png)

Grey = not_evaluated, Green = TP, White = TN, Blue = FP, RED = FN

**Map robot_0:**

![Map robot_0](mapping/map_robot_0.png)

**Map merged:**

![Map merged](mapping/map_merged.png)

**Map merged with robot trajectories:**

![Map merged with robot trajectories](mapping/localization_on_map.png)

