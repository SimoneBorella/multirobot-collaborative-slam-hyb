import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Wedge

def parse_args():
    parser = argparse.ArgumentParser(description="Plot Factor Graph Local Planning results")
    parser.add_argument('--filename', type=str, default="graphs/graph_x1.txt", help="Input data file")
    parser.add_argument('--show', action='store_true', default=False, help="Show plot")
    return parser.parse_args()

def parse_state_line(line):
    line_split = line.split()
    state_sym = line_split[1]
    state_values = [float(v) for v in line_split[2:6]]
    covariance_values = [float(v) for v in line_split[6:]]
    return state_sym, state_values, np.array(covariance_values).reshape(4, 4)


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
    
def plot_state_velocity(ax, state, covariance, color='red', alpha=0.2, n_std=2):
    x, y, vx, vy = state
    v_var = covariance[2:4, 2:4]

    theta = np.arctan2(vy, vx)

    dx = 0.04 * np.cos(theta)  
    dy = 0.04 * np.sin(theta)
    ax.quiver(x, y, dx, dy, angles='xy', scale_units='xy', scale=1., color='red', width=0.003)
    
    # if v_var > 0:
    #     theta_std = np.sqrt(theta_var)
    #     theta_min = theta - n_std * theta_std
    #     theta_max = theta + n_std * theta_std

    #     theta_min_deg = np.degrees(theta_min)
    #     theta_max_deg = np.degrees(theta_max)

    #     wedge = Wedge((x, y), 0.33, theta_min_deg, theta_max_deg, color=color, alpha=alpha)
    #     ax.add_patch(wedge)

def main():
    args = parse_args()
    states = {}
    states_priors = []


    dynamic_factor_edges = []
    measure_edges = []
    inter_robot_factor_edges = []

    with open(args.filename, "r") as file:
        for line in file:
            if line.startswith("STATE"):
                state_sym, state_values, covariance = parse_state_line(line)
                states[state_sym] = (state_values, covariance)
            elif line.startswith("PRIORSTATE"):
                prior_sym = line.split()[1]
                states_priors.append(prior_sym)
            elif line.startswith("DYNAMICFACTOR"):
                x1, x2 = line.split()[1:3]
                dynamic_factor_edges.append((x1, x2))
            elif line.startswith("INTERROBOTFACTOR"):
                x1, x2 = line.split()[1:3]
                inter_robot_factor_edges.append((x1, x2))


    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect('equal')
    ax.set_title("Factor Graph Local Planning")

    # Plot states
    for state_sym, (state_values, covariance) in states.items():
        state_sym_text = f"{state_sym[0]}_{int(int(state_sym[1:])//1e6)}_{int(int(state_sym[1:])%1e6)}"
        x, y, vx, vy = state_values
        plot_covariance_ellipse(ax, (x, y), covariance[:2, :2], color='red', alpha=0.01)

        if state_sym in states_priors:
            ax.scatter(x, y, color='green', marker='o')
            ax.text(x + 0.05, y + 0.05, state_sym_text, color='green', fontsize=12)
            plot_state_velocity(ax, state_values, covariance, color='green', alpha=0.2)
        else:
            ax.scatter(x, y, color='red', marker='o')
            ax.text(x + 0.05, y + 0.05, state_sym_text, color='red', fontsize=9)
            plot_state_velocity(ax, state_values, covariance, color='red', alpha=0.2)

    # Plot dynamic factor edges
    for sym1, sym2 in dynamic_factor_edges:
        if sym1 in states and sym2 in states:
            x1, y1, *_ = states[sym1][0]
            x2, y2, *_ = states[sym2][0]
            ax.plot([x1, x2], [y1, y2], color='red', linestyle='--', linewidth=0.5, alpha=0.2)

    # Plot inter-robot factor edges
    for sym1, sym2 in inter_robot_factor_edges:
        if sym1 in states and sym2 in states:
            x1, y1, *_ = states[sym1][0]
            x2, y2, *_ = states[sym2][0]
            ax.plot([x1, x2], [y1, y2], color='blue', linestyle='--', linewidth=0.5, alpha=0.6)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.8)
    plt.tight_layout()
    plt.savefig(args.filename.replace(".txt", ".pdf"))
    if args.show:
        plt.show()

if __name__ == "__main__":
    main()
