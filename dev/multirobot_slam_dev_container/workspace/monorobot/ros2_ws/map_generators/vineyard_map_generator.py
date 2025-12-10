import os
import re
import argparse
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


def create_occupancy_grid(width, height, occupancy_value=0):
    grid = np.full((height, width), occupancy_value, dtype=np.uint8)
    return grid

def save_pgm(occupancy_grid, filename="map.pgm"):
    occupancy_grid = np.flipud(occupancy_grid)
    grayscale = np.where(occupancy_grid == -1, 205, 255 - (occupancy_grid * 2.55)).astype(np.uint8)
    
    image = Image.fromarray(grayscale, mode="L")
    image.save(filename, format="PPM")
    print(f"Saved PGM map to {filename}")

def save_yaml(resolution, origin, occupied_thresh, free_thresh, map_filename="map.pgm", filename="map.yaml"):
    
    with open(filename, "w") as file:
        file.write(f'image: {map_filename}\n')
        file.write(f'mode: trinary\n')
        file.write(f'resolution: {resolution}\n')
        file.write(f'origin: {list(origin)}\n')
        file.write(f'occupied_thresh: {occupied_thresh}\n')
        file.write(f'free_thresh: {free_thresh}\n')
        file.write(f'negate: 0')

    print(f"Saved YAML map metadata to {filename}")

def plot(occupancy_grid, origin, width_grid, height_grid, resolution):
    plt.figure(figsize=(12, 10))
    grayscale = np.where(occupancy_grid == -1, 205, 255 - (occupancy_grid * 2.55)).astype(np.uint8)
    plt.imshow(grayscale, cmap='gray', extent=[origin[0], origin[0] + width_grid * resolution, origin[1], origin[1] + height_grid * resolution])
    plt.title("Vineyard Occupancy Grid")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.show()



def main(args):

    height_grid = args.height_grid
    width_grid = args.width_grid
    resolution = args.resolution

    height_origin = -(height_grid*resolution)/2
    width_origin = -(width_grid*resolution)/2

    origin=(width_origin, height_origin,0.0)

    rows = args.rows

    rows_width = args.rows_width
    rows_distance_mean = args.rows_distance_mean
    rows_distance_std = args.rows_distance_std

    rows_border_distance_mean = args.rows_border_distance_mean
    rows_border_distance_std = args.rows_border_distance_std

    rows_space_width_mean = args.rows_space_width_mean
    rows_space_width_std = args.rows_space_width_std

    spaces_values = [0, 1, 2, 3]
    spaces_probabilities = [0.6, 0.22, 0.12, 0.06]
    




    occupancy_grid = create_occupancy_grid(width_grid, height_grid)

    border_grid = 10
    occupancy_grid[0:border_grid, :] = 100
    occupancy_grid[:, 0:border_grid] = 100
    occupancy_grid[height_grid-border_grid:height_grid, :] = 100
    occupancy_grid[:, width_grid-border_grid:width_grid] = 100


    curr_height = height_origin
    for _ in range(rows):
        curr_height += np.random.normal(rows_distance_mean, rows_distance_std)

        curr_height_grid = round((curr_height - height_origin) / resolution)
        row_height_grid = round(rows_width/resolution)
        
        row_trend = np.random.normal(0.7, 0.1)
        row_trend_grid = round(row_trend/resolution)
        row_trend_grid_f0 = np.random.normal(1.5, 0.05)*np.pi/width_grid

        row_noise = np.random.normal(0.06, 0.02)
        row_noise_grid = round(row_noise/resolution)
        row_noise_grid_f1 = np.random.normal(300, 20)*np.pi/width_grid
        row_noise_grid_f2 = np.random.normal(400, 20)*np.pi/width_grid
        
        row_border_distance_left = np.random.normal(rows_border_distance_mean, rows_border_distance_std)
        row_border_distance_left_grid = round(row_border_distance_left/resolution)
        row_border_distance_right = np.random.normal(rows_border_distance_mean, rows_border_distance_std)
        row_border_distance_right_grid = round(row_border_distance_right/resolution)


        spaces_number = np.random.choice(spaces_values, p=spaces_probabilities)
        spaces_centers = np.random.uniform(row_border_distance_left_grid, width_grid-row_border_distance_right_grid, size=spaces_number)
        spaces = []
        for center in spaces_centers:
            space_width = np.random.normal(rows_space_width_mean, rows_space_width_std)
            space_width_grid = round(space_width/resolution)
            spaces.append((center-space_width_grid//2, center+space_width_grid//2))
        

        for cell in range(row_border_distance_left_grid, width_grid-row_border_distance_right_grid):
            skip = False
            for space in spaces:
                if cell > space[0] and cell < space[1]:
                    skip = True
            
            if skip:
                continue

            trend = round((row_trend_grid)*np.sin(cell*row_trend_grid_f0))
            noise = round((row_noise_grid)*np.sin(cell*row_noise_grid_f1) + (row_noise_grid/2)*np.cos(cell*row_noise_grid_f2))
            row_min_grid = curr_height_grid - row_height_grid//2 + trend + noise
            row_max_grid = curr_height_grid + row_height_grid//2 + trend + noise

            occupancy_grid[row_min_grid:row_max_grid, cell] = 100


    if args.show:
        plot(occupancy_grid, origin, width_grid, height_grid, resolution)


    source_file_path = os.path.dirname(os.path.abspath(__file__))

    maps_path = f"{source_file_path}/vineyard_maps"
    if not os.path.exists(maps_path):
        os.makedirs(maps_path)
    
    files = os.listdir(maps_path)
    numbers = []
    for file in files:
        match = re.match(r"vineyard_map_(\d+)\.pgm", file)
        if match:
            numbers.append(int(match.group(1)))

    map_id = max(numbers)+1 if numbers else 0

    save_pgm(occupancy_grid, filename=f"{maps_path}/vineyard_map_{map_id}.pgm")
    save_yaml(
        resolution=resolution,
        origin=origin,
        occupied_thresh=0.65,
        free_thresh=0.25,
        filename=f"{maps_path}/vineyard_map_{map_id}.yaml",
        map_filename=f"vineyard_map_{map_id}.pgm"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Vineyard map generator.')
    parser.add_argument('--width_grid', type=int, default=5000, help='Map width (number of grid cells)')
    parser.add_argument('--height_grid', type=int, default=5000, help='Map height (number of grid cells)')
    parser.add_argument('--resolution', type=float, default=0.01, help='Map resolution (m)')
    parser.add_argument('--rows', type=int, default=15, help='Number of vineyard rows')
    parser.add_argument('--rows_width', type=float, default=0.5, help='Width of vineyard rows (m)')
    parser.add_argument('--rows_distance_mean', type=float, default=3.0, help='Mean distance of vineyard rows (m)')
    parser.add_argument('--rows_distance_std', type=float, default=0.2, help='Std distance of vineyard rows (m)')
    parser.add_argument('--rows_border_distance_mean', type=float, default=4.0, help='Mean distance of vineyard rows from border (m)')
    parser.add_argument('--rows_border_distance_std', type=float, default=0.7, help='Std distance of vineyard rows from border (m)')
    parser.add_argument('--rows_space_width_mean', type=float, default=2.0, help='Mean space width in the middle of vineyard rows (m)')
    parser.add_argument('--rows_space_width_std', type=float, default=0.5, help='Std space width in the middle of vineyard rows (m)')

    parser.add_argument('--show', action="store_true", help='Show plot')


    args = parser.parse_args()
    main(args)