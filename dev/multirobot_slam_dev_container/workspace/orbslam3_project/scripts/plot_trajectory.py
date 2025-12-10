import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_trajectory(file_path):
    trajectory = []
    with open(file_path, 'r') as f:
        for line in f:
            if line.strip() == "":
                continue
            parts = line.strip().split()
            if len(parts) != 8:
                continue
            timestamp, tx, ty, tz, qx, qy, qz, qw = map(float, parts)
            # trajectory.append([tx, ty, tz])
            trajectory.append([-ty, -tx, tz])
    return np.array(trajectory)

def plot_trajectory(trajectory):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], label='Camera Trajectory', color='blue')
    ax.scatter(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2], color='green', label='Start')
    ax.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2], color='red', label='End')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.set_title('Camera Trajectory')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot camera trajectory from a file.")
    
    parser.add_argument(
        '--file_path', 
        type=str, 
        default='./res/frame_trajectory_0.txt', 
        help="Path to the trajectory file (default: './res/frame_trajectory_0.txt')"
    )
    
    args = parser.parse_args()
    
    trajectory = load_trajectory(args.file_path)
    plot_trajectory(trajectory)