import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Wedge

def parse_args():
    parser = argparse.ArgumentParser(description="Plot SLAM results with edges")
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

def plot_covariance_ellipsoid(ax, mean, covariance, color='red', alpha=0.2, n_std=2):
    """ Plot a 3D covariance ellipsoid using eigen decomposition. """
    U, S, _ = np.linalg.svd(covariance)  # Eigen decomposition
    radii = np.sqrt(S) * n_std

    # Generate points on a unit sphere
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 10)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones_like(u), np.cos(v))

    # Scale and rotate
    for i in range(len(x)):
        for j in range(len(x[i])):
            xyz = np.array([x[i, j], y[i, j], z[i, j]]) * radii
            xyz = U @ xyz  # Rotate using eigenvectors
            x[i, j], y[i, j], z[i, j] = xyz + mean

    ax.plot_wireframe(x, y, z, color=color, alpha=alpha, linewidth=0.5)
    
def plot_orientation(ax, pose, length=0.5):
    """ Plot orientation using roll, pitch, and yaw """
    x, y, z, roll, pitch, yaw = pose
    dx = length * np.cos(yaw) * np.cos(pitch)
    dy = length * np.sin(yaw) * np.cos(pitch)
    dz = length * np.sin(pitch)
    ax.quiver(x, y, z, dx, dy, dz, color='red', linewidth=1, arrow_length_ratio=0.2)


def main():
    args = parse_args()
    poses = {}
    poses_priors = []
    landmarks = {}
    landmarks_priors = {}
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
    ax = fig.add_subplot(111, projection='3d')
    # ax.set_aspect('equal')
    ax.set_title("SLAM Poses and Landmarks with Edges")
    
    # Plot poses
    for pose_sym, (pose_values, covariance) in poses.items():
        x, y, z, _, _, _ = pose_values
        plot_covariance_ellipsoid(ax, (x, y, z), covariance[:3, :3], color='red', alpha=0.2)
        plot_orientation(ax, pose_values)

        if pose_sym in poses_priors:
            ax.scatter(x, y, z, color='green', marker='o', s=30)
            ax.text(x + 0.05, y + 0.05, z + 0.05, pose_sym, color='green', fontsize=12)
        else:
            ax.scatter(x, y, z, color='red', marker='o', s=30)
            ax.text(x + 0.05, y + 0.05, z + 0.05, pose_sym, color='red', fontsize=9)

    # Plot landmarks
    for point_sym, (point_values, covariance) in landmarks.items():
        x, y, z = point_values
        plot_covariance_ellipsoid(ax, (x, y, z), covariance, color='blue', alpha=0.2)
        ax.scatter(x, y, z, color='blue', marker='x', s=40)
        ax.text(x + 0.1, y + 0.1, z + 0.1, point_sym, color='blue', fontsize=9)

    # Plot odometry edges
    for pose1, pose2 in odometry_edges:
        if pose1 in poses and pose2 in poses:
            x1, y1, z1, _, _, _ = poses[pose1][0]
            x2, y2, z2, _, _, _ = poses[pose2][0]
            ax.plot([x1, x2], [y1, y2], [z1, z2], color='red', linestyle='--', linewidth=0.1)

    # Plot measure edges
    for pose, landmark in measure_edges:
        if pose in poses and landmark in landmarks:
            x1, y1, z1, _, _, _ = poses[pose][0]
            x2, y2, z2 = landmarks[landmark][0]
            ax.plot([x1, x2], [y1, y2], [z1, z2], color='blue', linestyle='--', linewidth=0.1)





    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel("Z")
    ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.8)
    plt.tight_layout()
    plt.savefig(args.filename.replace(".txt", ".pdf"))
    if args.show:
        plt.show()

if __name__ == "__main__":
    main()
