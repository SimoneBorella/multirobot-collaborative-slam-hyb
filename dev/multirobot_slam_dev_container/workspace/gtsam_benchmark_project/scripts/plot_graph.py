import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Wedge


def parse_pose_line(line):
    line_split = line.split()
    pose_sym = line_split[1]
    pose_values = [float(v) for v in line_split[2:5]]
    covariance_values = [float(v) for v in line_split[5:]]
    return pose_sym, pose_values, np.array(covariance_values).reshape(3, 3)

def parse_point_line(line):
    line_split = line.split()
    point_sym = line_split[1]
    point_values = [float(v) for v in line_split[2:4]]
    covariance_values = [float(v) for v in line_split[4:]]
    return point_sym, point_values, np.array(covariance_values).reshape(2, 2)

def parse_factor_measure(line):
    line_split = line.split()
    f1 = line_split[1]
    f2 = line_split[2]
    return f1, f2

def plot_covariance_ellipse(ax, mean, covariance, color, alpha=0.3, n_std=2):
    eigvals, eigvecs = np.linalg.eigh(covariance)
    angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))
    width, height = 2 * n_std * np.sqrt(eigvals)
    ellipse = Ellipse(mean, width=width, height=height, angle=angle, color=color, alpha=alpha)
    ax.add_patch(ellipse)
    
def plot_theta_variance(ax, pose, covariance, color='red', alpha=0.2, n_std=2):
    x, y, theta = pose
    theta_var = covariance[2, 2]
    
    if theta_var > 0:
        theta_std = np.sqrt(theta_var)
        theta_min = theta - n_std * theta_std
        theta_max = theta + n_std * theta_std

        theta_min_deg = np.degrees(theta_min)
        theta_max_deg = np.degrees(theta_max)

        wedge = Wedge((x, y), 0.33, theta_min_deg, theta_max_deg, color=color, alpha=alpha)
        ax.add_patch(wedge)

def main(args):
    poses = {}
    landmarks = {}
    odometry_edges = []
    measure_edges = []
    
    with open(args.filename, "r") as file:
        for line in file:
            if line.startswith("POSE2"):
                pose_sym, pose_values, covariance = parse_pose_line(line)
                poses[pose_sym] = (pose_values, covariance)
            elif line.startswith("POINT2"):
                point_sym, point_values, covariance = parse_point_line(line)
                landmarks[point_sym] = (point_values, covariance)
            elif line.startswith("BETWEENFACTOR"):
                pose1, pose2 = parse_factor_measure(line)
                odometry_edges.append((pose1, pose2))
            elif line.startswith("BEARINGRANGEFACTOR"):
                pose, landmark = parse_factor_measure(line)
                measure_edges.append((pose, landmark))
                
    
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect('equal')
    ax.set_title("SLAM Poses and Landmarks with Edges")
    
    # Plot poses
    for pose_sym, (pose_values, covariance) in poses.items():
        x, y, theta = pose_values
        # plot_covariance_ellipse(ax, (x, y), covariance[:2, :2], color='red', alpha=0.2)
        # plot_theta_variance(ax, pose_values, covariance, color='red', alpha=0.2)
        ax.scatter(x, y, color='red', marker='o', s=7)
        # ax.text(x + 0.05, y + 0.05, pose_sym, color='red', fontsize=9)
        dx = 0.33 * np.cos(theta)  
        dy = 0.33 * np.sin(theta)
        ax.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1., color='red', width=0.003)


    # Plot landmarks
    for point_sym, (point_values, covariance) in landmarks.items():
        x, y = point_values
        # plot_covariance_ellipse(ax, (x, y), covariance, color='blue', alpha=0.2)
        ax.scatter(x, y, color='blue', marker='x', s=12)
        ax.text(x + 0.05, y + 0.05, point_sym, color='blue', fontsize=9)
    
    # Plot odometry edges
    # for pose1, pose2 in odometry_edges:
    #     if pose1 in poses and pose2 in poses:
    #         x1, y1, _ = poses[pose1][0]
    #         x2, y2, _ = poses[pose2][0]
    #         ax.plot([x1, x2], [y1, y2], color='red', linestyle='--', linewidth=1)

    # Plot measure edges
    # for pose, landmark in measure_edges:
    #     if pose in poses and landmark in landmarks:
    #         x1, y1, _ = poses[pose][0]
    #         x2, y2 = landmarks[landmark][0]
    #         ax.plot([x1, x2], [y1, y2], color='blue', linestyle='--', linewidth=1)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.8)
    plt.tight_layout()
    plt.savefig(args.filename.replace(".txt", ".pdf"))
    if args.show:
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot planar SLAM results with edges")
    parser.add_argument('--filename', type=str, default="res/exp_0/after.txt", help="Input data file")
    parser.add_argument('--show', action='store_true', default=False, help="Show plot")

    args = parser.parse_args()
    main(args)
