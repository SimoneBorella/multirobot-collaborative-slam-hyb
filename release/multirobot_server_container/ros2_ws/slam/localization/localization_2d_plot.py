import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Wedge

def parse_args():
    parser = argparse.ArgumentParser(description="Plot SLAM results with edges (2D projection)")
    parser.add_argument('--filename', type=str, default="slam/graphs/graph_x1.txt", help="Input data file")
    parser.add_argument('--show', action='store_true', default=False, help="Show plot")
    return parser.parse_args()

def parse_pose_line(line):
    line_split = line.split()
    pose_sym = line_split[1]
    pose_values = [float(v) for v in line_split[2:8]]
    covariance_values = [float(v) for v in line_split[8:]]
    return pose_sym, pose_values, np.array(covariance_values).reshape(6, 6)

def parse_point_line(line):
    line_split = line.split()
    point_sym = line_split[1]
    point_values = [float(v) for v in line_split[2:5]]
    covariance_values = [float(v) for v in line_split[5:]]
    return point_sym, point_values, np.array(covariance_values).reshape(3, 3)

def parse_factor_measure(line):
    line_split = line.split()
    f1 = line_split[1]
    f2 = line_split[2]
    return f1, f2

def plot_covariance_ellipse(ax, mean, covariance, color='red', alpha=0.2, n_std=2):
    """ Plot a 2D covariance ellipse (projected on XY) """
    cov_2d = covariance[:2, :2]
    vals, vecs = np.linalg.eigh(cov_2d)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))
    width, height = 2 * n_std * np.sqrt(vals)
    ellipse = Ellipse(xy=mean[:2], width=width, height=height, angle=theta, color=color, alpha=alpha)
    ax.add_patch(ellipse)


def plot_theta_variance(ax, pose, covariance, color='red', alpha=0.2, n_std=2):
    x, y, _, _, _, theta = pose
    theta_var = covariance[5, 5]
    
    if theta_var > 0:
        theta_std = np.sqrt(theta_var)
        theta_min = theta - n_std * theta_std
        theta_max = theta + n_std * theta_std

        theta_min_deg = np.degrees(theta_min)
        theta_max_deg = np.degrees(theta_max)

        wedge = Wedge((x, y), 0.33, theta_min_deg, theta_max_deg, color=color, alpha=alpha)
        ax.add_patch(wedge)

def main():
    args = parse_args()
    poses = {}
    poses_priors = []
    landmarks = {}
    odometry_edges = []
    measure_edges = []

    with open(args.filename, "r") as file:
        for line in file:
            if line.startswith("POSE3"):
                pose_sym, pose_values, covariance = parse_pose_line(line)
                poses[pose_sym] = (pose_values, covariance)
            elif line.startswith("POINT3"):
                point_sym, point_values, covariance = parse_point_line(line)
                landmarks[point_sym] = (point_values, covariance)
            elif line.startswith("PRIORPOSE3"):
                prior_sym = line.split()[1]
                poses_priors.append(prior_sym)
            elif line.startswith("BETWEENFACTOR3"):
                pose1, pose2 = parse_factor_measure(line)
                odometry_edges.append((pose1, pose2))
            elif line.startswith("BEARINGRANGEFACTOR3"):
                pose, landmark = parse_factor_measure(line)
                measure_edges.append((pose, landmark))

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect('equal')
    ax.set_title("SLAM Poses and Landmarks with Edges (2D projection)")

    # Plot poses
    for pose_sym, (pose_values, covariance) in poses.items():
        x, y, _, _, _, yaw = pose_values
        plot_covariance_ellipse(ax, (x, y), covariance[:3, :3], color='red', alpha=0.05)
        plot_theta_variance(ax, pose_values, covariance, color='red', alpha=0.2)
        if pose_sym in poses_priors:
            ax.scatter(x, y, color='green', marker='o')
            ax.text(x + 0.05, y + 0.05, pose_sym, color='green', fontsize=12)
        else:
            ax.scatter(x, y, color='red', marker='o')
            ax.text(x + 0.05, y + 0.05, pose_sym, color='red', fontsize=9)
        dx = 0.04 * np.cos(pose_values[5])  
        dy = 0.04 * np.sin(pose_values[5])
        ax.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1., color='red', width=0.003)

    # Plot landmarks
    for point_sym, (point_values, covariance) in landmarks.items():
        x, y, _ = point_values
        plot_covariance_ellipse(ax, (x, y), covariance, color='blue', alpha=0.2)
        ax.scatter(x, y, color='blue', marker='x')
        ax.text(x + 0.05, y + 0.05, point_sym, color='blue', fontsize=9)

    # Plot odometry edges
    for pose1, pose2 in odometry_edges:
        if pose1 in poses and pose2 in poses:
            x1, y1, *_ = poses[pose1][0]
            x2, y2, *_ = poses[pose2][0]
            ax.plot([x1, x2], [y1, y2], color='red', linestyle='--', linewidth=0.5, alpha=0.2)

    # Plot measure edges
    for pose, landmark in measure_edges:
        if pose in poses and landmark in landmarks:
            x1, y1, *_ = poses[pose][0]
            x2, y2, _ = landmarks[landmark][0]
            ax.plot([x1, x2], [y1, y2], color='blue', linestyle='--', linewidth=0.5, alpha=0.05)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.8)
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.savefig(args.filename.replace(".txt", "_2d.pdf"))
    if args.show:
        plt.show()

if __name__ == "__main__":
    main()