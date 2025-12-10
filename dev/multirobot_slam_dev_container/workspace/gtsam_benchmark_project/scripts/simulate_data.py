import argparse
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
import os
import yaml

def generate_landmarks(space_range):
    return [np.random.uniform(-space_range/2, space_range/2, 2) for _ in range(6)]

def generate_landmarks_init_estimates(landmarks, landmark_err):
    landmarks_init_estimates = []

    for l in landmarks:
        x_init = l[0] + np.random.uniform(-landmark_err, landmark_err)
        y_init = l[1] + np.random.uniform(-landmark_err, landmark_err)
        landmarks_init_estimates.append(np.array([x_init, y_init]))

    return landmarks_init_estimates

def generate_poses(space_range, n_steps, delta_s):
    t_steps_per_turn = 10000
    delta_t = 0.5
    turns = 1
    
    path_length = 0
    
    for _ in range(2):
        n_t_steps = t_steps_per_turn*turns

        t = np.arange(0, n_t_steps, delta_t)
        x = (space_range/2)*np.sin(2*(2*np.pi/(n_t_steps/turns))*t) + (space_range/4)*np.sin(7*(2*np.pi/(n_t_steps/turns))*t)
        y = (space_range/2)*np.cos(3*(2*np.pi/(n_t_steps/turns))*t + np.pi/2)

        distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
        arc_length = np.insert(np.cumsum(distances), 0, 0)

        path_length = arc_length[-1]
        turns = (n_steps*delta_s // path_length) + 1


    interp_x = interp1d(arc_length, x, kind='cubic')
    interp_y = interp1d(arc_length, y, kind='cubic')

    s = np.arange(0, n_steps*delta_s, delta_s)

    x = interp_x(s)
    y = interp_y(s)
    theta = np.arctan2(np.diff(y, prepend=y[0]), np.diff(x, prepend=x[0]))
    theta[0] = theta[1]

    poses = [np.array([i, j, k]) for i, j, k in zip(x, y, theta)]

    return poses

def generate_poses_init_estimates(poses, pose_pos_err, pose_theta_err):
    poses_init_estimates = []

    for p in poses:
        x_init = p[0] + np.random.uniform(-pose_pos_err, pose_pos_err)
        y_init = p[1] + np.random.uniform(-pose_pos_err, pose_pos_err)
        theta_init = p[2] + np.random.uniform(-pose_theta_err, pose_theta_err)
        poses_init_estimates.append(np.array([x_init, y_init, theta_init]))

    return poses_init_estimates


def generate_observations(poses, landmarks, obs_range_err_perc_min, obs_range_err_perc_max, obs_bearing_err,
                          obs_outlier_rate, obs_outlier_range_err_perc_min, obs_outlier_range_err_perc_max, obs_outlier_bearing_err):
    observations = []

    for p_id, p in enumerate(poses):
        for l_id, l in enumerate(landmarks):
            x_obs = l[0] - p[0]
            y_obs = l[1] - p[1]

            range = np.sqrt(x_obs**2 + y_obs**2)
            bearing = np.arctan2(y_obs, x_obs) - p[2]

            range_err_perc_min = obs_range_err_perc_min
            range_err_perc_max = obs_range_err_perc_max
            bearing_error = obs_bearing_err

            if np.random.random() < obs_outlier_rate:
                range_err_perc_min = obs_outlier_range_err_perc_min
                range_err_perc_max = obs_outlier_range_err_perc_max
                bearing_error = obs_outlier_bearing_err

            range_min_error = range * range_err_perc_min
            range_max_error = range * range_err_perc_max
            
            if np.random.random() < 0.5:
                range += np.random.uniform(range_min_error, range_max_error)
            else:
                range -= np.random.uniform(range_min_error, range_max_error)

            bearing += np.random.uniform(-bearing_error, bearing_error)

            if range < 0:
                range = abs(range)
                bearing += np.pi
                
            # Theta in range (-np.pi, np.pi)
            bearing = (bearing + np.pi) % (2 * np.pi) - np.pi

            observations.append((p_id, l_id, np.array([bearing, range])))

    return observations

def generate_odometries(poses, odo_pos_err, odo_theta_err):
    odometries = []

    for p_id in range(len(poses)):
        if p_id == 0:
            continue

        p0 = poses[p_id-1]
        p1 = poses[p_id]
        odo = p1 - p0

        odo[0] += np.random.uniform(-odo_pos_err, odo_pos_err)
        odo[1] += np.random.uniform(-odo_pos_err, odo_pos_err)

        odo[2] += np.random.uniform(-odo_theta_err, odo_theta_err)

        # Theta in range (-np.pi, np.pi)
        odo[2] = (odo[2] + np.pi) % (2 * np.pi) - np.pi

        odometries.append((p_id-1, p_id, odo))

    return odometries


def store_ground_truth(gt_path, landmarks, poses):
    with open(f"{gt_path}/landmarks.csv", "w") as f:
        f.write("l_id,x,y\n")
        for i, l in enumerate(landmarks):
            f.write(f"{i},{l[0]},{l[1]}\n")

    with open(f"{gt_path}/poses.csv", "w") as f:
        f.write("p_id,x,y,theta\n")
        for i, p in enumerate(poses):
            f.write(f"{i},{p[0]},{p[1]},{p[2]}\n")

def store_data(exp_path, args, landmarks_init_estimates, poses_init_estimates, observations, odometries):
    with open(f"{exp_path}/params.yaml", "w") as f:
        yaml.dump(vars(args), f, sort_keys=False)

    with open(f"{exp_path}/landmarks_initial_estimates.csv", "w") as f:
        f.write("l_id,x,y\n")
        for i, l in enumerate(landmarks_init_estimates):
            f.write(f"{i},{l[0]},{l[1]}\n")
    
    with open(f"{exp_path}/poses_initial_estimates.csv", "w") as f:
        f.write("p_id,x,y,theta\n")
        for i, p in enumerate(poses_init_estimates):
            f.write(f"{i},{p[0]},{p[1]},{p[2]}\n")

    with open(f"{exp_path}/observations.csv", "w") as f:
        f.write("p_id,l_id,bearing,range\n")
        for p_id, l_id, obs in observations:
            f.write(f"{p_id},{l_id},{obs[0]},{obs[1]}\n")

    with open(f"{exp_path}/odometries.csv", "w") as f:
        f.write("p_id0,p_id1,x,y,theta\n")
        for p_id0, p_id1, odo in odometries:
            f.write(f"{p_id0},{p_id1},{odo[0]},{odo[1]},{odo[2]}\n")
    


def plot(poses, landmarks, poses_init_estimates, landmarks_init_estimates):
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    
    ax = axes[0]
    x = [p[0] for p in poses]
    y = [p[1] for p in poses]
    theta = [p[2] for p in poses]

    ax.plot(x, y, label="Poses")
    dx = 0.3 * np.cos(theta)  
    dy = 0.3 * np.sin(theta)
    ax.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1., color='red', width=0.01)
    ax.scatter([l[0] for l in landmarks], [l[1] for l in landmarks], c='green', label="Landmarks")
    ax.set_title("Ground Truth")
    # ax.legend()
    ax.grid()
    ax.set_aspect('equal')

    ax = axes[1]
    x_init = [p[0] for p in poses_init_estimates]
    y_init = [p[1] for p in poses_init_estimates]
    theta_init = [p[2] for p in poses_init_estimates]

    ax.plot(x_init, y_init, label="Initial Pose Estimates")
    dx_init = 0.3 * np.cos(theta_init)  
    dy_init = 0.3 * np.sin(theta_init)
    ax.quiver(x_init, y_init, dx_init, dy_init, angles='xy', scale_units='xy', scale=1., color='red', width=0.01)
    ax.scatter([l[0] for l in landmarks_init_estimates], [l[1] for l in landmarks_init_estimates], c='green', label="Initial Landmark Estimates")
    ax.set_title("Initial Estimates")
    # ax.legend()
    ax.grid()
    ax.set_aspect('equal')

    plt.tight_layout()
    plt.show()

# def plot(poses, landmarks, poses_init_estimates, landmarks_init_estimates):
#     fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    
#     x = [p[0] for p in poses]
#     y = [p[1] for p in poses]
#     theta = [p[2] for p in poses]

#     ax.plot(x, y, label="Poses")
#     dx = 0.3 * np.cos(theta)  
#     dy = 0.3 * np.sin(theta)
#     # ax.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1., color='red', width=0.01)
#     ax.scatter([l[0] for l in landmarks], [l[1] for l in landmarks], c='green', label="Landmarks")
#     ax.set_title("Ground Truth")
#     # ax.legend()
#     ax.grid()
#     ax.set_aspect('equal')

#     plt.tight_layout()
#     plt.show()




def main(args):

    exp_path = f"./data/benchmark_{args.benchmark}/exp_{args.exp}"
    if not os.path.exists(exp_path):
        os.makedirs(exp_path)
    else:
        raise Exception(f"Directory {exp_path} already exists")


    print(f"Generating data for benchmark {args.benchmark} experiment {args.exp}...")

    landmarks = generate_landmarks(args.space_range)
    # print(landmarks)

    # landmarks = [
    #     np.array([ 49.10878951, -19.96772516]),
    #     np.array([-14.71252512,   2.86688899]),
    #     np.array([ 45.46266376, -27.60415914]),
    #     np.array([  2.65279246, -13.18585996]),
    #     np.array([-1.66187825, 33.13027665]),
    #     np.array([-41.15576638, -49.28131293])
    # ]

    landmarks_init_estimates = generate_landmarks_init_estimates(landmarks, args.landmark_err)

    poses = generate_poses(args.space_range, args.n_steps, args.delta_s)

    poses_init_estimates = generate_poses_init_estimates(poses, args.pose_pos_err, args.pose_theta_err)

    observations = generate_observations(poses, landmarks, args.obs_range_err_perc_min, args.obs_range_err_perc_max, args.obs_bearing_err,
                                            args.obs_outlier_rate, args.obs_outlier_range_err_perc_min, args.obs_outlier_range_err_perc_max, args.obs_outlier_bearing_err)
    odometries = generate_odometries(poses, args.odo_pos_err, args.odo_theta_err)

    gt_path = f"./data/ground_truth"
    if not os.path.exists(gt_path):
        os.makedirs(gt_path)
        store_ground_truth(gt_path, landmarks, poses)

    store_data(exp_path, args, landmarks_init_estimates, poses_init_estimates, observations, odometries)
    
    print(f"Data generated corectly.")
    print()

    if args.plot:
        plot(poses, landmarks, poses_init_estimates, landmarks_init_estimates)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Simulate data to perform a GTSAM benchmark.')
    parser.add_argument('--benchmark', type=int, required=True, help='Benchmark number')
    parser.add_argument('--exp', type=int, required=True, help='Experiment number')
    parser.add_argument('--space_range', type=float, default=100, help='Range of the 2D space')
    parser.add_argument('--n_steps', type=int, default=10000, help='Number of steps')
    parser.add_argument('--delta_s', type=float, default=0.5, help='Step size (m)')

    parser.add_argument('--landmark_err', type=float, default=0.0, help='Max absolute error from uniform distribution on landmark (x, y) position (m)')

    parser.add_argument('--pose_pos_err', type=float, default=0.0, help='Max absolute error from uniform distribution on poses (x, y) position (m)')
    parser.add_argument('--pose_theta_err', type=float, default=0.0, help='Max absolute error from uniform distribution on poses yaw (rad)')

    parser.add_argument('--obs_range_err_perc_min', type=float, default=0.0, help='Min absolute error from uniform distribution on observation range (%)')
    parser.add_argument('--obs_range_err_perc_max', type=float, default=0.0, help='Max absolute error from uniform distribution on observation range (%)')
    parser.add_argument('--obs_bearing_err', type=float, default=0.0, help='Max absolute error from uniform distribution on oservation bearing (rad)')

    parser.add_argument('--odo_pos_err', type=float, default=0.0, help='Max absolute error from uniform distribution on odometry (x, y) position measure (m)')
    parser.add_argument('--odo_theta_err', type=float, default=0.0, help='Max absolute error from uniform distribution on odometry yaw measure (rad)')

    parser.add_argument('--obs_outlier_rate', type=float, default=0.0, help='Outlier rate (%)')
    parser.add_argument('--obs_outlier_range_err_perc_min', type=float, default=0.0, help='Min absolute error from uniform distribution on oulier observation range (%)')
    parser.add_argument('--obs_outlier_range_err_perc_max', type=float, default=0.0, help='Max absolute error from uniform distribution on oulier observation range (%)')
    parser.add_argument('--obs_outlier_bearing_err', type=float, default=0.0, help='Max absolute error from uniform distribution on oulier oservation bearing (rad)')

    parser.add_argument('--plot', action="store_true", help='Plot flag')

    args = parser.parse_args()
    main(args)