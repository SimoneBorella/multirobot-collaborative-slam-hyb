import osmnx as ox
import numpy as np
import argparse
from PIL import Image
import os
import re

def latlon_to_xy(lat, lon, min_lat, min_lon, R=6378137):
    x = R * np.radians(lon - min_lon) * np.cos(np.radians(min_lat))
    y = R * np.radians(lat - min_lat)
    return x, y

def draw_line(grid, x0, y0, x1, y1, value=255, road_px_width=1):
    """Bresenham line algorithm to draw a line with thickness on a 2D numpy array"""
    x0 = int(round(x0))
    y0 = int(round(y0))
    x1 = int(round(x1))
    y1 = int(round(y1))
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        for dx_off in range(-road_px_width // 2, road_px_width // 2 + 1):
            for dy_off in range(-road_px_width // 2, road_px_width // 2 + 1):
                xi = x0 + dx_off
                yi = y0 + dy_off
                if 0 <= yi < grid.shape[0] and 0 <= xi < grid.shape[1]:
                    grid[yi, xi] = value
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

def generate_pgm_from_center(lat_center, lon_center, half_size_m, resolution_m, pgm_filename, road_width_m=0.5):
    R = 6378137

    # Bounding box
    delta_lat = (half_size_m / R) * (180 / np.pi)
    delta_lon = (half_size_m / (R * np.cos(np.radians(lat_center)))) * (180 / np.pi)
    min_lat = lat_center - delta_lat
    max_lat = lat_center + delta_lat
    min_lon = lon_center - delta_lon
    max_lon = lon_center + delta_lon

    print(f"Bounding box: {min_lat},{min_lon} -> {max_lat},{max_lon}")

    # Download roads
    G = ox.graph_from_point((lat_center, lon_center), dist=half_size_m, network_type='drive')
    edges = ox.graph_to_gdfs(G, nodes=False, edges=True)
    if edges.empty:
        print("Warning: no roads found in this area")
        return

    # Compute map size
    max_x, max_y = latlon_to_xy(max_lat, max_lon, min_lat, min_lon)
    width = int(max_x / resolution_m)
    height = int(max_y / resolution_m)
    print(f"Map size: {width} x {height} pixels, resolution {resolution_m} m/pixel")

    # Road width in pixels
    road_px_width = max(1, int(round(road_width_m / resolution_m)))

    # Initialize grid
    grid = np.zeros((height, width), dtype=np.uint8)

    # Draw roads
    for geom in edges['geometry']:
        if geom.is_empty:
            continue
        coords = list(geom.coords)
        for i in range(len(coords) - 1):
            x0, y0 = latlon_to_xy(coords[i][1], coords[i][0], min_lat, min_lon)
            x1, y1 = latlon_to_xy(coords[i+1][1], coords[i+1][0], min_lat, min_lon)
            px0, py0 = x0 / resolution_m, y0 / resolution_m
            px1, py1 = x1 / resolution_m, y1 / resolution_m
            # Flip y axis for image coordinates
            draw_line(grid, px0, height - py0 - 1, px1, height - py1 - 1, value=255, road_px_width=road_px_width)





    source_file_path = os.path.dirname(os.path.abspath(__file__))

    maps_path = f"{source_file_path}/openstreetmap_maps"
    if not os.path.exists(maps_path):
        os.makedirs(maps_path)
    
    files = os.listdir(maps_path)
    numbers = []
    for file in files:
        match = re.match(r"openstreetmap_map_(\d+)\.pgm", file)
        if match:
            numbers.append(int(match.group(1)))

    map_id = max(numbers)+1 if numbers else 0

    # Save PGM
    pgm_filename = f"{maps_path}/openstreetmap_map_{map_id}.pgm"
    with open(pgm_filename, 'wb') as f:
        f.write(f"P5\n{width} {height}\n255\n".encode())
        f.write(grid.tobytes())
    print(f"Saved PGM map to {pgm_filename}")


    # Save YAML
    map_filename = f"openstreetmap_map_{map_id}.pgm"
    filename = f"{maps_path}/openstreetmap_map_{map_id}.yaml"    
    new_resolution = 0.05
    free_thresh = 0.25
    occupied_thresh = 0.65

    origin = (-(half_size_m/resolution_m)*new_resolution, -(half_size_m/resolution_m)*new_resolution)

    print(origin)

    with open(filename, "w") as file:
        file.write(f'image: {map_filename}\n')
        file.write(f'mode: trinary\n')
        file.write(f'resolution: {new_resolution}\n')
        file.write(f'origin: {list(origin)}\n')
        file.write(f'occupied_thresh: {occupied_thresh}\n')
        file.write(f'free_thresh: {free_thresh}\n')
        file.write(f'negate: 0')

    print(f"Saved yaml to {filename}")

    # Save PNG for visualization
    # png_filename = pgm_filename.replace('.pgm', '.png')
    # img = Image.fromarray(grid)
    # img.save(png_filename)
    # print(f"Saved PNG map to {png_filename}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate PGM map from central location with road width")
    parser.add_argument('--lat', type=float, default=45.061430, help="Center latitude")
    parser.add_argument('--lon', type=float, default=7.657586, help="Center longitude")
    parser.add_argument('--half_size', type=float, default=500.0, help="Half size of map in meters")
    parser.add_argument('--resolution', type=float, default=0.5, help="Resolution in meters/pixel")
    parser.add_argument('--road_width', type=float, default=5.5, help="Width of roads in meters")
    parser.add_argument('--pgm', default='map.pgm', help="Output PGM filename")
    args = parser.parse_args()

    generate_pgm_from_center(args.lat, args.lon, args.half_size, args.resolution, args.pgm, road_width_m=args.road_width)


# Locations:
# 1. PIC4SeR - 45.061430,7.657586