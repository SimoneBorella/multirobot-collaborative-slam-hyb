import os
import re
import argparse
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import numpy as np
from PIL import Image
import networkx as nx
from scipy.spatial import Delaunay


class Room:
    def __init__(self, pos=(0, 0), width=0, height=0):
        self.pos = pos
        self.width = width
        self.height = height\
        
    def area(self):
        return self.height * self.width

    def __str__(self):
        return f"Room(position={self.pos}, height={self.height}, width={self.width}, area={self.area()})"

    def __lt__(self, other):
        if isinstance(other, Room):
            return self.area() < other.area()
        return NotImplemented





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

def plot(occupancy_grid, origin, width_grid, height_grid, resolution, main_rooms=None, tri=None, graph=None, mst=None):
    plt.figure(figsize=(12, 10))

    occupancy_grid = np.flipud(occupancy_grid)
    grayscale = np.where(occupancy_grid == -1, 205, 255 - (occupancy_grid * 2.55)).astype(np.uint8)
    plt.imshow(grayscale, cmap='gray', extent=[origin[0], origin[0] + width_grid * resolution, origin[1], origin[1] + height_grid * resolution])

    if main_rooms:
        transparent_red = LinearSegmentedColormap.from_list("transparent_red", [(1, 0, 0, 0), (1, 0, 0, 1)])
        occupancy_grid_main_rooms = create_occupancy_grid(width_grid, height_grid, occupancy_value=0)
        for room in main_rooms:
            occupancy_grid_main_rooms[(room.pos[0] - room.height//2):(room.pos[0] + room.height//2), (room.pos[1] - room.width//2):(room.pos[1] + room.width//2)] = 255
        occupancy_grid_main_rooms = np.flipud(occupancy_grid_main_rooms)
        plt.imshow(occupancy_grid_main_rooms, cmap=transparent_red, extent=[origin[0], origin[0] + width_grid * resolution, origin[1], origin[1] + height_grid * resolution], alpha=0.5)

    if tri:
        plt.triplot((tri.points[:, 1]*resolution + origin[1]), (tri.points[:, 0]*resolution + origin[0]), tri.simplices, color='blue', linewidth=1.2)
        plt.scatter((tri.points[:, 1]*resolution + origin[1]), (tri.points[:, 0]*resolution + origin[0]), color='red', s=36)

    if graph:
        pos = {node: (data['x']*resolution + origin[0], data['y']*resolution + origin[1]) for node, data in graph.nodes(data=True)}
        nx.draw(graph, pos, node_size=36, node_color='red', edge_color='blue')

    if graph and mst:
        pos = {node: (data['x'] * resolution + origin[0], data['y'] * resolution + origin[1]) for node, data in graph.nodes(data=True)}
        nx.draw(mst, pos, node_size=36, node_color='red', edge_color='green', width=2)


    plt.title("Indoor Occupancy Grid with Connectivity Graph")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.show()


def generate_graph(main_rooms_points, tri):
    graph = nx.Graph()

    for idx, pos in enumerate(main_rooms_points):
        graph.add_node(idx, x=pos[1], y=pos[0])

    for simplex in tri.simplices:
        for i in range(3):
            for j in range(i + 1, 3):
                graph.add_edge(simplex[i], simplex[j])

    return graph




def main(args):
    width_grid = args.width_grid
    height_grid = args.height_grid
    resolution = args.resolution

    height_origin = -(height_grid*resolution)/2
    width_origin = -(width_grid*resolution)/2

    origin=(width_origin, height_origin, 0.0)

    num_rooms = args.num_rooms
    num_main_rooms = args.num_main_rooms
    room_min_size = args.room_min_size
    room_max_size = args.room_max_size

    room_space_border = args.room_space_border
    room_space_border_grid = round(room_space_border / resolution)

    redundant_connection_rate = args.redundant_connection_rate

    hallway_width = args.hallway_width


    occupancy_grid = create_occupancy_grid(width_grid, height_grid, occupancy_value=100)


    rooms = []
    for id in range(num_rooms):
        room = Room()

        placed = False
        trial = 0
        while not placed:
            room.height = round(np.random.uniform(room_min_size, room_max_size)/resolution)
            room.width = round(np.random.uniform(room_min_size, room_max_size)/resolution)
            room.pos = np.array([
                np.random.randint(room_space_border_grid + room.height//2, height_grid - room_space_border_grid - room.height//2),
                np.random.randint(room_space_border_grid + room.width//2, width_grid - room_space_border_grid - room.width//2),
            ])

            placed = np.all(occupancy_grid[(room.pos[0] - room.height//2 - room_space_border_grid):(room.pos[0] + room.height//2 + room_space_border_grid), (room.pos[1] - room.width//2 - room_space_border_grid):(room.pos[1] + room.width//2 + room_space_border_grid)] == 100)

            trial+=1
            if trial >= 1000:
                raise Exception(f"Unable to place room after {trial} trials")

        occupancy_grid[(room.pos[0] - room.height//2):(room.pos[0] + room.height//2), (room.pos[1] - room.width//2):(room.pos[1] + room.width//2)] = 0

        rooms.append(room)

    rooms.sort(reverse=True)

    print("Inserted rooms:")
    for room in rooms:
        print(room)

    main_rooms = rooms[0:num_main_rooms]

    main_rooms_points = np.array([np.array(room.pos) for room in main_rooms])
    # plot(occupancy_grid, origin, width_grid, height_grid, resolution, main_rooms=main_rooms)

    tri = Delaunay(main_rooms_points)
    # plot(occupancy_grid, origin, width_grid, height_grid, resolution, main_rooms=main_rooms, tri=tri)

    graph = generate_graph(main_rooms_points, tri)
    # plot(occupancy_grid, origin, width_grid, height_grid, resolution, main_rooms=main_rooms, graph=graph)

    mst = nx.minimum_spanning_tree(graph, weight='weight')
    # plot(occupancy_grid, origin, width_grid, height_grid, resolution, main_rooms=main_rooms, graph=graph, mst=mst)

    final_graph = mst.copy()
    edges = np.array([edge for edge in graph.edges if edge not in mst.edges])
    random_edges = edges[np.random.choice(len(edges), round(len(edges)*redundant_connection_rate), replace=False)]
    for u, v in random_edges:
        final_graph.add_edge(u, v)

    # plot(occupancy_grid, origin, width_grid, height_grid, resolution, main_rooms=main_rooms, graph=final_graph)
    


    main_rooms_occupancy_grid = create_occupancy_grid(width_grid, height_grid, occupancy_value=100)
    for room in main_rooms:
        main_rooms_occupancy_grid[(room.pos[0] - room.height//2):(room.pos[0] + room.height//2), (room.pos[1] - room.width//2):(room.pos[1] + room.width//2)] = 0


    for edge in final_graph.edges:
        u, v = edge

        u_pos = final_graph.nodes[u]
        v_pos = final_graph.nodes[v]

        min_pos_y = np.min([u_pos["y"], v_pos["y"]])
        max_pos_y = np.max([u_pos["y"], v_pos["y"]])

        min_pos_x = np.min([u_pos["x"], v_pos["x"]])
        max_pos_x = np.max([u_pos["x"], v_pos["x"]])

        hv = round((hallway_width/2)/resolution)

        # if np.abs(u_pos["x"] - v_pos["x"]) < (room_min_size/resolution):
        #     mean_pos_x = (u_pos["x"] + v_pos["x"])//2
        #     main_rooms_occupancy_grid[min_pos_y:max_pos_y, mean_pos_x-hv:mean_pos_x+hv] = 0

        # elif np.abs(u_pos["y"] - v_pos["y"]) < (room_min_size/resolution):
        #     mean_pos_y = (u_pos["y"] + v_pos["y"])//2
        #     main_rooms_occupancy_grid[mean_pos_y-hv:mean_pos_y+hv, min_pos_x:max_pos_x] = 0

        # else:
        if np.abs(u_pos["y"] - v_pos["y"]) > np.abs(u_pos["x"] - v_pos["x"]):
            x_first = u_pos["x"] if min_pos_y == u_pos["y"] else v_pos["x"]
            y_second = u_pos["y"] if x_first == v_pos["x"] else v_pos["y"]

            main_rooms_occupancy_grid[min_pos_y-hv:max_pos_y+hv, x_first-hv:x_first+hv] = 0
            main_rooms_occupancy_grid[y_second-hv:y_second+hv, min_pos_x-hv:max_pos_x+hv] = 0
        else:
            y_first = u_pos["y"] if min_pos_x == u_pos["x"] else v_pos["y"]
            x_second = u_pos["x"] if y_first == v_pos["y"] else v_pos["x"]

            main_rooms_occupancy_grid[y_first-hv:y_first+hv, min_pos_x-hv:max_pos_x+hv] = 0
            main_rooms_occupancy_grid[min_pos_y-hv:max_pos_y+hv, x_second-hv:x_second+hv] = 0

    if args.show:
        plot(main_rooms_occupancy_grid, origin, width_grid, height_grid, resolution, graph=final_graph)
    

    


    


    
    source_file_path = os.path.dirname(os.path.abspath(__file__))

    maps_path = f"{source_file_path}/indoor_maps"
    if not os.path.exists(maps_path):
        os.makedirs(maps_path)
    
    files = os.listdir(maps_path)
    numbers = []
    for file in files:
        match = re.match(r"indoor_map_(\d+)\.pgm", file)
        if match:
            numbers.append(int(match.group(1)))

    map_id = max(numbers)+1 if numbers else 0

    
    save_pgm(main_rooms_occupancy_grid, filename=f"{maps_path}/indoor_map_{map_id}.pgm")
    save_yaml(
        resolution=resolution,
        origin=origin,
        occupied_thresh=0.65,
        free_thresh=0.25,
        filename=f"{maps_path}/indoor_map_{map_id}.yaml",
        map_filename=f"indoor_map_{map_id}.pgm"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Indoor environment map generator with connectivity graph.')
    parser.add_argument('--width_grid', type=int, default=10000, help='Map width (number of grid cells)')
    parser.add_argument('--height_grid', type=int, default=10000, help='Map height (number of grid cells)')
    parser.add_argument('--resolution', type=float, default=0.01, help='Map resolution (m)')
    parser.add_argument('--num_rooms', type=int, default=20, help='Number of rooms to generate')
    parser.add_argument('--num_main_rooms', type=int, default=15, help='Number of main rooms to generate')
    parser.add_argument('--room_min_size', type=float, default=8.0, help='Minimum room size (m)')
    parser.add_argument('--room_max_size', type=float, default=22.0, help='Maximum room size (m)')
    parser.add_argument('--room_space_border', type=float, default=1.0, help='Room space from other rooms or the map border (m)')
    parser.add_argument('--redundant_connection_rate', type=float, default=0.15, help='Rate of connection to reintegrate into mst')
    parser.add_argument('--hallway_width', type=float, default=2.0, help='Hallway width (m)')
    parser.add_argument('--show', action="store_true", help='Show plot')
    args = parser.parse_args()
    main(args)
