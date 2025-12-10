import argparse
import matplotlib.pyplot as plt
import numpy as np

def moving_average(data, window_size):
    ma_values = []
    
    for i in range(len(data)):
        # For the first few elements, use smaller windows (starting from 1)
        current_window_size = min(i + 1, window_size)
        ma_values.append(np.mean(data[max(0, i - current_window_size + 1):i + 1]))
    
    return ma_values

def main(args):
    res_dir = args.dir

    filename = res_dir + "/results.csv"

    update_times = []
    optimization_times = []
    maes = []
    rmses = []

    with open(filename, "r") as file:
        file.readline()
        for line in file:
            iteration, update_time, optimization_time, mae, rmse = line.strip().split(",")

            update_times.append(float(update_time))
            optimization_times.append(float(optimization_time))
            maes.append(float(mae))
            rmses.append(float(rmse))
    
    window_size = 100
    update_times_ma = moving_average(update_times, window_size)
    optimization_times_ma = moving_average(optimization_times, window_size)



    fig, axs = plt.subplots(2, 2, figsize=(12, 10))

    fig.suptitle("ISAM2 metrics")

    axs[0, 0].plot(update_times, color='#4169E1', alpha=0.6, label="True time")
    axs[0, 0].plot(update_times_ma, color='#228B22', label="Moving average")
    axs[0, 0].set_title('Update time')
    axs[0, 0].set_xlabel('Iteration')
    axs[0, 0].set_ylabel('Time (ms)')
    axs[0, 0].legend()
    axs[0, 0].grid(True)

    axs[0, 1].plot(optimization_times, color='#4169E1', alpha=0.6, label="True time")
    axs[0, 1].plot(optimization_times_ma, color='#228B22', label="Moving average")
    axs[0, 1].set_title('Optimization time')
    axs[0, 1].set_xlabel('Iteration')
    axs[0, 1].set_ylabel('Time (ms)')
    axs[0, 1].legend()
    axs[0, 1].grid(True)

    axs[1, 0].plot(maes, color='#B00000', label="MAE")
    axs[1, 0].set_title('Mean Absolute Error (MAE)')
    axs[1, 0].set_xlabel('Iteration')
    axs[1, 0].set_ylabel('Error (m)')
    axs[1, 0].legend()
    axs[1, 0].set_ylim(-0.05, (maes[-1]+0.005)*10)
    axs[1, 0].grid(True)

    axs[1, 1].plot(rmses, color='#B00000', label="RMSE")
    axs[1, 1].set_title('Root Mean Squared Error (RMSE)')
    axs[1, 1].set_xlabel('Iteration')
    axs[1, 1].set_ylabel('Error (m)')
    axs[1, 1].legend()
    axs[1, 1].set_ylim(-0.05, (rmses[-1]+0.005)*10)
    axs[1, 1].grid(True)

    plt.tight_layout()

    plt.savefig(filename.replace(".csv", ".pdf"))
    if args.show:
        plt.show()




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot planar SLAM results with edges")
    parser.add_argument('--dir', type=str, help="Experiment directory")
    parser.add_argument('--show', action="store_true", help="Show plot")

    args = parser.parse_args()
    main(args)
