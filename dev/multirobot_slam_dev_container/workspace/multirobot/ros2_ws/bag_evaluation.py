import argparse
import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import os


GREEN = "#29B266"
ORANGE = "#E68021"
RED = "#FF0000"
YELLOW = "#F2C40F"
BLUE = "#2673A6"
DARK_GREY = "#424A4A"

PRIMARY_CYAN = "#00FFFF"
PRIMARY_MAGENTA = "#FF00FF"
PRIMARY_YELLOW = "#FFFF00"

WHITE  = "#FFFFFF"
GREY = "#CDCDCD"
BLACK  = "#000000"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Extract last map from ros bags"
    )
    parser.add_argument(
        "bag",
        type=str,
        help="Specify bag .db3 file path",
    )

    return parser.parse_args()


class BagFileParser:
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        # Create a message type map
        topics_data = self.cursor.execute(
            "SELECT id, name, type FROM topics"
        ).fetchall()
        self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {
            name_of: get_message(type_of) for id_of, name_of, type_of in topics_data
        }

    def __del__(self):
        self.conn.close()

    def get_messages(self, topic_name):
        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)
        ).fetchall()
        # Deserialise all and timestamp them
        return [
            {
                "timestamp":timestamp,
                "data": deserialize_message(data, self.topic_msg_message[topic_name])
            }
            for timestamp, data in rows
        ]
    
def log(s, file_handle):
    # print(s)
    file_handle.write(str(s) + "\n")


def hex_to_rgb(hex_color: str):
    hex_color = hex_color.lstrip("#")
    return [int(hex_color[i:i+2], 16) for i in (0, 2, 4)]

def blend_rgb(color_rgb, base_rgb, alpha=0.5):
    return [
        int(alpha * color_rgb[c] + (1 - alpha) * base_rgb[c]) for c in range(3)
    ]

def blend_rgb_images(rgb1, rgb2, alpha=0.5):
    return (rgb1.astype(np.float32) * alpha + rgb2.astype(np.float32) * (1 - alpha)).astype(np.uint8)

    
def save_map_png(occupancy_grid, filename="map.png"):
    # Flip vertically to match ROS convention
    occupancy_grid = np.flipud(occupancy_grid)

    # Create RGB image
    h, w = occupancy_grid.shape
    rgb_image = np.zeros((h, w, 3), dtype=np.uint8)

    # Assign colors
    rgb_image[occupancy_grid == -1] = hex_to_rgb(GREY)
    rgb_image[occupancy_grid == 0]  = hex_to_rgb(WHITE)
    rgb_image[occupancy_grid == 100] = hex_to_rgb(BLACK)

    # If there are in-between values (0–100), scale as grayscale
    mask = (occupancy_grid > 0) & (occupancy_grid < 100)
    rgb_image[mask] = np.stack([255 - occupancy_grid[mask]*255//100]*3, axis=-1)

    # Save as png
    image = Image.fromarray(rgb_image, mode="RGB")
    image.save(filename, format="PNG")
    print(f"Saved PNG map to {filename}")

def save_map_merged_png(occupancy_grids, colors, filenames, filename):
    # Flip vertically to match ROS convention
    occupancy_grids = [np.flipud(grid) for grid in occupancy_grids]

    # Create RGB image
    h, w = occupancy_grids[0].shape
    rgb_images = [np.zeros((h, w, 3), dtype=np.uint8) for _ in range(len(occupancy_grids))]

    # Assign colors
    for i, occupancy_grid in enumerate(occupancy_grids):
        rgb_images[i][occupancy_grid == -1] = hex_to_rgb(GREY)
        rgb_images[i][occupancy_grid == 0]  = blend_rgb(hex_to_rgb(colors[i]), hex_to_rgb(WHITE), alpha=0.5)
        rgb_images[i][occupancy_grid == 100] = blend_rgb(hex_to_rgb(colors[i]), hex_to_rgb(BLACK), alpha=0.5)

        # If there are in-between values (0–100), scale as grayscale
        mask = (occupancy_grid > 0) & (occupancy_grid < 100)
        rgb_images[i][mask] = np.stack([255 - occupancy_grid[mask]*255//100]*3, axis=-1)

        # Save as png
        image = Image.fromarray(rgb_images[i], mode="RGB")
        image.save(filenames[i], format="PNG")
        print(f"Saved PNG map to {filenames[i]}")
    
    # Blend all maps together
    merged_rgb_image = rgb_images[0].copy()
    for rgb_image in rgb_images[1:]:
        merged_rgb_image = blend_rgb_images(merged_rgb_image, rgb_image, alpha=0.5)

    # Save as png
    image = Image.fromarray(merged_rgb_image, mode="RGB")
    image.save(filename, format="PNG")
    print(f"Saved PNG map to {filename}")

def save_map_pgm(occupancy_grid, filename="map.pgm"):
    occupancy_grid = np.flipud(occupancy_grid)
    grayscale = np.where(occupancy_grid == -1, 205, 255 - (occupancy_grid * 2.55)).astype(np.uint8)
    image = Image.fromarray(grayscale, mode="L")
    image.save(filename, format="PPM")
    print(f"Saved PGM map to {filename}")

def save_map_yaml(resolution, origin, occupied_thresh, free_thresh, map_filename="map.pgm", filename="map.yaml"):
    
    with open(filename, "w") as file:
        file.write(f'image: {map_filename}\n')
        file.write(f'mode: trinary\n')
        file.write(f'resolution: {resolution}\n')
        file.write(f'origin: {list(origin)}\n')
        file.write(f'occupied_thresh: {occupied_thresh}\n')
        file.write(f'free_thresh: {free_thresh}\n')
        file.write(f'negate: 0')

    print(f"Saved YAML map metadata to {filename}")





if __name__ == "__main__":
    args = parse_args()

    bag_parser = BagFileParser(args.bag)

    
    # Initialize results folder and log file
    source_file_path = os.path.dirname(os.path.abspath(__file__))
    results_path = f"{source_file_path}/bag_evaluation_data/{args.bag.split('/')[-2]}"
    if not os.path.exists(results_path):
        os.makedirs(results_path)

    f = open(os.path.join(results_path, "log.md"), "w")

    log(f"# Evaluation {args.bag.split('/')[-2]}\n", f)


    # Retrieve tf messages
    tf_msgs = bag_parser.get_messages('/tf')
    static_tf_msgs = bag_parser.get_messages('/tf_static')


    # Retrieve robots list
    robots = set()

    for msg in static_tf_msgs:
        for transform in msg['data'].transforms:
            parent_frame = transform.header.frame_id
            child_frame  = transform.child_frame_id

            if parent_frame == 'all' and '/map' in child_frame:
                robot_name = child_frame.split('/')[0]
                robots.add(robot_name)

    robots = sorted(list(robots))

    log(f"## Robots:\n", f)
    for robot in robots:
        log(f"- {robot}\n", f)


    # Retrieve messages
    cmd_vel_msgs = {robot: bag_parser.get_messages(f'/{robot}/cmd_vel') for robot in robots}
    map_msgs = bag_parser.get_messages(f'/map')
    robot_map_msgs = {robot: bag_parser.get_messages(f'/{robot}/map') for robot in robots}


    # Timing evaluation    
    start_timestamp = None
    last_timestamp = None
    success = {}
    end_timestamp = {}
    last_cmd_vel = {}

    linear_x_threshold = 0.1
    angular_z_threshold = 1.0

    for robot in robots:
        msgs = cmd_vel_msgs[robot]

        # Start and last timestamp common to all robots
        if start_timestamp is None or msgs[0]["timestamp"] < start_timestamp:
            start_timestamp = msgs[0]["timestamp"]
        
        if last_timestamp is None or msgs[-1]["timestamp"] > last_timestamp:
            last_timestamp = msgs[-1]["timestamp"]

        # Success check
        last = msgs[-1]["data"]
        last_cmd_vel[robot] = last
        success[robot] = (abs(last.linear.x) < linear_x_threshold and abs(last.angular.z) < angular_z_threshold)

        # Find last stop timestamp
        end_timestamp[robot] = msgs[-1]["timestamp"]
        for msg in reversed(msgs):
            if (abs(msg["data"].linear.x) < linear_x_threshold and
                abs(msg["data"].angular.z) < angular_z_threshold):
                end_timestamp[robot] = msg["timestamp"]
            else:
                break


    log("## Timing evaluation:\n", f)
    log("### Last velocity commands:\n", f)
    for robot in robots:
        log(f"- {robot}: [linear_x={last_cmd_vel[robot].linear.x:.3f}, ang_z={last_cmd_vel[robot].angular.z:.3f}]\n", f)

    log("### Success:\n", f)
    for robot in robots:
        log(f"- {robot}: {success[robot]}\n", f)

    log("### Execution time:\n", f)
    for robot in robots:
        log(f"- {robot}: {(end_timestamp[robot] - start_timestamp) * 1e-6:.0f} ms\n", f)

    log(f"Total: {(max([item[1] for item in end_timestamp.items()]) - start_timestamp) * 1e-6:.0f} ms\n", f)
    log(f"Bag record time: {(last_timestamp - start_timestamp) * 1e-6:.0f} ms\n", f)







    # Mapping
    unknown_replace_value = -1

    last_map_msg = map_msgs[-1]

    resolution = last_map_msg['data'].info.resolution
    width = last_map_msg['data'].info.width
    height = last_map_msg['data'].info.height
    origin = (last_map_msg['data'].info.origin.position.x, last_map_msg['data'].info.origin.position.y, last_map_msg['data'].info.origin.position.z)

    log(f"## Mapping evaluation:\n", f)
    log(f"### Mapping metadata:\n", f)
    log(f"Map resolution: {resolution:.3f}\n", f)
    log(f"Map width: {width}\n", f)
    log(f"Map height: {height}\n", f)
    log(f"Map origin: {origin}\n", f)



    log(f"### Mapping results:\n", f)
    
    map_grid = np.array(last_map_msg['data'].data, dtype=np.int8).reshape(((height, width)))
    map_grid[map_grid==-1] = unknown_replace_value

    save_map_png(map_grid, filename=f"{results_path}/map.png")

    log(f"Map:\n", f)
    log(f"![Map](map.png)\n", f)

    map_name = args.bag.split("/")[-2].replace("bag", "map")
    save_map_pgm(map_grid, filename=f"{results_path}/{map_name}.pgm")
    save_map_yaml(
        resolution=resolution,
        origin=origin,
        occupied_thresh=0.65,
        free_thresh=0.25,
        filename=f"{results_path}/{map_name}.yaml",
        map_filename=f"{map_name}.pgm"
    )



    
    map_grids = []
    for robot in robots:
        map_msg = robot_map_msgs[robot][-1]

        map_grid = np.array(map_msg['data'].data, dtype=np.int8).reshape(((height, width)))
        map_grid[map_grid==-1] = unknown_replace_value

        map_grids.append(map_grid)

    colors = [PRIMARY_CYAN, PRIMARY_MAGENTA, PRIMARY_YELLOW]
    map_merged_name = args.bag.split("/")[-2].replace("bag", "map_merged")
    filenames = [f"{results_path}/map_{robot}.png" for robot in robots]
    filename = f"{results_path}/map_merged.png"

    save_map_merged_png(map_grids, colors, filenames, filename)

    for robot in robots:
        log(f"Map {robot}:\n", f)
        log(f"![Map {robot}](map_{robot}.png)\n", f)
    
    log(f"Map merged:\n", f)
    log(f"![Map merged](map_merged.png)\n", f)




    print(f"Saved log to {results_path}/log.md")
    f.close()
