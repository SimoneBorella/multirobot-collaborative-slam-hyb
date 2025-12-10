import argparse
import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import os
from datetime import datetime
import itertools
from geometry_msgs.msg import Transform, Vector3, Quaternion
import matplotlib.pyplot as plt
import math



GREEN = "#29B266"
ORANGE = "#E68021"
RED = "#FF0000"
YELLOW = "#F2C40F"
BLUE = "#2673A6"
DARK_GREY = "#424A4A"

PRIMARY_CYAN = "#00FFFF"
PRIMARY_MAGENTA = "#FF00FF"
PRIMARY_YELLOW = "#FFFF00"

PRIMARY_CYAN_DARK = "#009999"     # darker cyan
PRIMARY_MAGENTA_DARK = "#990099"  # darker magenta
PRIMARY_YELLOW_DARK = "#999900"   # darker yellow


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

    parser.add_argument(
        "rtabmap_bag",
        type=str,
        help="Specify bag .db3 file path registered with rtabmap",
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
    




def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def quaternion_to_matrix(q: Quaternion):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w,     0],
        [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w,     0],
        [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y, 0],
        [0,                 0,                 0,                 1]
    ])

def transform_to_matrix(t: Transform):
    T = quaternion_to_matrix(t.rotation)
    T[0, 3] = t.translation.x
    T[1, 3] = t.translation.y
    T[2, 3] = t.translation.z
    return T

def matrix_to_transform(T):
    trans = Vector3(x=T[0, 3], y=T[1, 3], z=T[2, 3])
    qw = np.sqrt(1 + T[0, 0] + T[1, 1] + T[2, 2]) / 2
    qx = (T[2, 1] - T[1, 2]) / (4*qw)
    qy = (T[0, 2] - T[2, 0]) / (4*qw)
    qz = (T[1, 0] - T[0, 1]) / (4*qw)
    rot = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return Transform(translation=trans, rotation=rot)





def hex_to_rgb(hex_color: str):
    hex_color = hex_color.lstrip("#")
    return [int(hex_color[i:i+2], 16) for i in (0, 2, 4)]

def blend_rgb(color_rgb, base_rgb, alpha=0.5):
    return [
        int(alpha * color_rgb[c] + (1 - alpha) * base_rgb[c]) for c in range(3)
    ]

def blend_rgb_images(rgb1, rgb2, alpha=0.5):
    return (rgb1.astype(np.float32) * alpha + rgb2.astype(np.float32) * (1 - alpha)).astype(np.uint8)

    
def save_map_png(occupancy_grid, filename="map.png", flip=True):
    # Flip vertically to match ROS convention

    grid = occupancy_grid

    if flip:
        grid = np.flipud(occupancy_grid)

    filtered_occupancy_grid = np.zeros_like(grid)

    filtered_occupancy_grid[grid > 65] = 100
    filtered_occupancy_grid[grid < 65] = -1
    filtered_occupancy_grid[grid < 25] = 0
    filtered_occupancy_grid[grid == -1] = -1

    # Create RGB image
    h, w = grid.shape
    rgb_image = np.zeros((h, w, 3), dtype=np.uint8)


    # Assign colors
    rgb_image[filtered_occupancy_grid == -1] = hex_to_rgb(GREY)
    rgb_image[filtered_occupancy_grid == 0]  = hex_to_rgb(WHITE)
    rgb_image[filtered_occupancy_grid == 100] = hex_to_rgb(BLACK)

    # If there are in-between values (0–100), scale as grayscale
    mask = (filtered_occupancy_grid > 0) & (filtered_occupancy_grid < 100)
    rgb_image[mask] = np.stack([255 - filtered_occupancy_grid[mask]*255//100]*3, axis=-1)

    # Save as png
    image = Image.fromarray(rgb_image, mode="RGB")
    image.save(filename, format="PNG")
    print(f"Saved PNG map to {filename}")

def save_map_merged_png(robot_map_grids, colors, filenames, filename):
    # Flip vertically to match ROS convention
    occupancy_grids = [np.flipud(robot_map_grids[robot]) for robot in robot_map_grids]

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

    return merged_rgb_image


def save_error_map_png(map_grid, rtab_map_grid, filename="map.png"):
    h, w = map_grid.shape
    rgb_image = np.zeros((h, w, 3), dtype=np.uint8)

    # Assign colors
    not_eval_mask = (map_grid == -1) | (rtab_map_grid == -1)
    rgb_image[not_eval_mask] = hex_to_rgb(GREY)

    eval_mask = ((map_grid != -1) & (rtab_map_grid != -1))
    right_mask = eval_mask & (map_grid == rtab_map_grid)
    error_mask = eval_mask & (map_grid != rtab_map_grid)

    rgb_image[right_mask]  = hex_to_rgb(WHITE)
    rgb_image[error_mask] = hex_to_rgb(RED)

    # Save as png
    image = Image.fromarray(rgb_image, mode="RGB")
    image.save(filename, format="PNG")
    print(f"Saved PNG map to {filename}")




def save_confusion_map_png(map_grid, rtab_map_grid, filename="confusion_map.png"):
    h, w = map_grid.shape
    rgb_image = np.zeros((h, w, 3), dtype=np.uint8)

    # Masks
    not_eval_mask = (map_grid == -1) | (rtab_map_grid == -1)
    eval_mask = (map_grid != -1) & (rtab_map_grid != -1)

    tp_mask = eval_mask & (map_grid == 100) & (rtab_map_grid == 100)  # occupied-occupied
    tn_mask = eval_mask & (map_grid == 0)   & (rtab_map_grid == 0)    # free-free
    fp_mask = eval_mask & (map_grid == 100) & (rtab_map_grid == 0)    # predicted occupied, GT free
    fn_mask = eval_mask & (map_grid == 0)   & (rtab_map_grid == 100)  # predicted free, GT occupied

    # Assign colors
    rgb_image[not_eval_mask] = hex_to_rgb(GREY)
    rgb_image[tp_mask] = hex_to_rgb(GREEN)
    rgb_image[tn_mask] = hex_to_rgb(WHITE)
    rgb_image[fp_mask] = hex_to_rgb(BLUE)
    rgb_image[fn_mask] = hex_to_rgb(RED)

    # Save as png
    image = Image.fromarray(rgb_image, mode="RGB")
    image.save(filename, format="PNG")
    print(f"Saved confusion map to {filename}")





def save_map_pgm(occupancy_grid, filename="map.pgm"):
    occupancy_grid = np.flipud(occupancy_grid)
    filtered_occupancy_grid = np.zeros_like(occupancy_grid)

    filtered_occupancy_grid[occupancy_grid > 65] = 100
    filtered_occupancy_grid[occupancy_grid < 65] = -1
    filtered_occupancy_grid[occupancy_grid < 25] = 0
    filtered_occupancy_grid[occupancy_grid == -1] = -1

    grayscale = np.where(filtered_occupancy_grid == -1, 205, 255 - (filtered_occupancy_grid * 2.55)).astype(np.uint8)
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














def map_grid_rtabmap_accuracy(map_occupancy_grid, rtabmap_map_occupancy_grid, map_meta, rtabmap_meta):
    map_raw = np.flipud(map_occupancy_grid)
    map_grid = np.zeros_like(map_raw, dtype=np.int8)
    map_grid[map_raw > 65] = 100
    map_grid[map_raw < 65] = -1
    map_grid[map_raw < 25] = 0
    map_grid[map_raw == -1] = -1

    rt_raw = np.flipud(rtabmap_map_occupancy_grid)
    rtab_grid = np.zeros_like(rt_raw, dtype=np.int8)
    rtab_grid[rt_raw > 65] = 100
    rtab_grid[rt_raw < 65] = -1
    rtab_grid[rt_raw < 25] = 0
    rtab_grid[rt_raw == -1] = -1

    if not np.isclose(map_meta["resolution"], rtabmap_meta["resolution"]):
        raise ValueError("Resolutions of the maps do not match. You must resample one map.")
    res = map_meta["resolution"]

    dx = int(round((rtabmap_meta["origin"][0] - map_meta["origin"][0]) / res))
    dy = int(round((rtabmap_meta["origin"][1] - map_meta["origin"][1]) / res))

    y0 = max(0, dx)
    x0 = max(0, map_grid.shape[0] - dy - rtab_grid.shape[0])
    y1 = min(map_grid.shape[1], dx + rtab_grid.shape[1])
    x1 = min(map_grid.shape[0], map_grid.shape[0] - dy)

    rt_x0 = max(0, - (map_grid.shape[0] - dy - rtab_grid.shape[0]))
    rt_y0 = max(0, -dx)
    rt_x1 = rt_x0 + (x1 - x0)
    rt_y1 = rt_y0 + (y1 - y0)

    padded_rtab = np.full_like(map_grid, -1, dtype=np.int8)
    padded_rtab[x0:x1, y0:y1] = rtab_grid[rt_x0:rt_x1, rt_y0:rt_y1]

    save_map_png(padded_rtab, filename=f"{mapping_path}/padded_rtabmap_map.png", flip=False)
    save_error_map_png(map_grid, padded_rtab, filename=f"{mapping_path}/error_map.png")
    save_confusion_map_png(map_grid, padded_rtab, filename=f"{mapping_path}/confusion_map.png")

    valid_mask = (map_grid != -1) & (padded_rtab != -1)
    total = int(np.count_nonzero(valid_mask))
    if total == 0:
        return 0.0, 0, 0, 0, 0, 0, 0

    correct = int(np.count_nonzero((map_grid == padded_rtab) & valid_mask))
    accuracy = correct / total

    map_occ = (map_grid == 100)
    rt_occ  = (padded_rtab == 100)

    tp = int(np.count_nonzero(map_occ & rt_occ & valid_mask))
    tn = int(np.count_nonzero((map_grid == 0) & (padded_rtab == 0) & valid_mask))
    fp = int(np.count_nonzero((map_grid == 100) & (padded_rtab == 0) & valid_mask))
    fn = int(np.count_nonzero((map_grid == 0) & (padded_rtab == 100) & valid_mask))

    return accuracy, total, correct, tp, tn, fp, fn

























if __name__ == "__main__":
    args = parse_args()

    bag_parser = BagFileParser(args.bag)
    if args.rtabmap_bag != "-":
        rtabmap_bag_parser = BagFileParser(args.rtabmap_bag)

    source_file_path = os.path.dirname(os.path.abspath(__file__))
    results_path = f"{source_file_path}/bag_evaluation_data/{args.bag.split('/')[-2]}"
    mapping_path = f"{results_path}/mapping"
    localization_path = f"{results_path}/localization"

    if not os.path.exists(results_path):
        os.makedirs(results_path)
    if not os.path.exists(localization_path):
        os.makedirs(localization_path)
    if not os.path.exists(mapping_path):
        os.makedirs(mapping_path)

    f = open(os.path.join(results_path, "report.md"), "w")

    f.write(f"# Multirobot Collaborative SLAM Evaluation Report\n\n")

    bag_id = args.bag.split('/')[-2]
    f.write(f"**Bag ID:** {bag_id}\n\n")


    date_split = bag_id.split('_')[1:]
    current_date = datetime.now().strftime("%Y-%m-%d")

    f.write(f"**Experiment Date:** {date_split[0]}-{date_split[1]}-{date_split[2]}\n\n")
    f.write(f"**Experiment Time:** {date_split[3]}:{date_split[4]}:{date_split[5]}\n\n")
    f.write(f"**Evaluation Date:** {current_date}\n\n")



    # Retrieve tf messages
    tf_msgs = bag_parser.get_messages('/tf')
    static_tf_msgs = bag_parser.get_messages('/tf_static')


    # Retrieve robots list
    robots = set()
    tf_all_to_map = {}

    for msg in static_tf_msgs:
        for transform in msg['data'].transforms:
            parent_frame = transform.header.frame_id
            child_frame  = transform.child_frame_id

            if parent_frame == 'all' and '/map' in child_frame:
                robot_name = child_frame.split('/')[0]
                robots.add(robot_name)
                tf_all_to_map[robot_name] = transform.transform

    robots = sorted(list(robots))

    f.write(f"**Robots:**\n")
    for robot in robots:
        f.write(f"`{robot}`\n")
    f.write(f"\n")




    # Retrieve transforms

    tf_all_to_ground_truth = {robot: [] for robot in robots}

    tf_all_to_map = {}
    tf_map_to_odom = {robot: [] for robot in robots}
    tf_odom_to_base_footprint = {robot: [] for robot in robots}
    tf_base_footprint_to_base_link = {}
        
    for msg in tf_msgs:
        for transform in msg['data'].transforms:
            parent_frame = transform.header.frame_id
            child_frame  = transform.child_frame_id

            if '/map' in parent_frame and '/odom' in child_frame:
                robot_name = parent_frame.split('/')[0]
                if robot_name in robots:
                    tf_map_to_odom[robot_name].append((msg['timestamp'], transform.transform))
            
            if '/odom' in parent_frame and '/base_footprint' in child_frame:
                robot_name = parent_frame.split('/')[0]
                if robot_name in robots:
                    tf_odom_to_base_footprint[robot_name].append((msg['timestamp'], transform.transform))

            if 'all' in parent_frame and '/ground_truth' in child_frame:
                robot_name = child_frame.split('/')[0]
                if robot_name in robots:
                    # CORRECTION: IN CASE OF GROUND TRUTH CONFUSION
                    # if robot_name == "robot_14" and transform.transform.translation.y > 1:
                    #     print(robot, transform.transform.translation.y)
                    #     continue
                    tf_all_to_ground_truth[robot_name].append((msg['timestamp'], transform.transform))

    for msg in static_tf_msgs:
        for transform in msg['data'].transforms:
            parent_frame = transform.header.frame_id
            child_frame  = transform.child_frame_id

            if 'all' in parent_frame and '/map' in child_frame:
                robot_name = child_frame.split('/')[0]
                if robot_name in robots:
                    tf_all_to_map[robot_name] = transform.transform

            if '/base_footprint' in parent_frame and '/base_link' in child_frame:
                robot_name = parent_frame.split('/')[0]
                if robot_name in robots:
                    tf_base_footprint_to_base_link[robot_name] = transform.transform

    

    # Concatenate tf to get map to base_link
    tf_all_to_base_link = {robot: [] for robot in robots}
    tf_map_to_base_link = {robot: [] for robot in robots}

    time_tolerance = 5e7  # 50 ms

    for robot in robots:
        map_transform = tf_all_to_map[robot]
        odom_transforms = tf_map_to_odom[robot]
        basefoot_data  = tf_odom_to_base_footprint[robot]
        static_base_tf = transform_to_matrix(tf_base_footprint_to_base_link[robot])

        i = 0
        n = len(odom_transforms)

        for ts, odom_to_base in basefoot_data:
            while i < n and odom_transforms[i][0] <= ts:
                i += 1

            if i == 0:
                continue

            best_ts, best_tf = odom_transforms[i - 1]

            if ts - best_ts > time_tolerance:
                continue
            
            T_all_map = transform_to_matrix(map_transform)
            T_map_odom = transform_to_matrix(best_tf)
            T_odom_basefoot = transform_to_matrix(odom_to_base)

            T_map_baselink = T_map_odom @ T_odom_basefoot @ static_base_tf
            T_all_baselink = T_all_map @ T_map_baselink

            tf_map_to_base_link[robot].append((ts, matrix_to_transform(T_map_baselink)))
            tf_all_to_base_link[robot].append((ts, matrix_to_transform(T_all_baselink)))
    
    # CORRECTION: DELETE LAST PART OF THE EXPERIMENT
    # for robot in robots:
    #     tf_all_to_base_link[robot] = tf_all_to_base_link[robot][:-400]
    #     tf_map_to_base_link[robot] = tf_map_to_base_link[robot][:-400]

    robot_initial_position = {robot: np.array([tf_all_to_base_link[robot][0][1].translation.x, tf_all_to_base_link[robot][0][1].translation.y, tf_all_to_base_link[robot][0][1].translation.z]) for robot in robots }
    robot_final_position = {robot: np.array([tf_all_to_base_link[robot][-1][1].translation.x, tf_all_to_base_link[robot][-1][1].translation.y, tf_all_to_base_link[robot][-1][1].translation.z]) for robot in robots }
    
    



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

    distance_from_init_threshold = 0.2

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

        distance_from_init = np.linalg.norm(robot_initial_position[robot] - robot_final_position[robot])

        success[robot] = (abs(last.linear.x) < linear_x_threshold and abs(last.angular.z) < angular_z_threshold) \
            and distance_from_init < distance_from_init_threshold



        # Find last stop timestamp
        end_timestamp[robot] = msgs[-1]["timestamp"]
        for msg in reversed(msgs):
            if (abs(msg["data"].linear.x) < linear_x_threshold and
                abs(msg["data"].angular.z) < angular_z_threshold):
                end_timestamp[robot] = msg["timestamp"]
            else:
                break


    f.write("## Timing evaluation:\n\n")
    f.write(f"**Bag record time:** {(last_timestamp - start_timestamp) * 1e-6:.0f} ms\n\n")

    f.write("| Robot ID | Success | Last cmd (linear_x, angular_z) | Execution time |\n")
    f.write("|----------|---------|--------------------------------|----------------|\n")

    for robot in robots:
        success_status = "Yes" if success[robot] else "No"
        vx = last_cmd_vel[robot].linear.x
        az = last_cmd_vel[robot].angular.z
        exec_time_ms = (end_timestamp[robot] - start_timestamp) * 1e-6
        exec_time_s = (end_timestamp[robot] - start_timestamp) * 1e-9

        f.write(f"| `{robot}` | {success_status} | {vx:.3f} m/s, {az:.3f} rad/s | **{exec_time_ms:.0f} ms** ({exec_time_s:.3f} s) |\n")
    f.write("\n")

    f.write(f"**Exploration gap:** {(max([item[1] for item in end_timestamp.items()]) - min([item[1] for item in end_timestamp.items()])) * 1e-6:.0f} ms\n\n")

    f.write(f"**Total time:** {(max([item[1] for item in end_timestamp.items()]) - start_timestamp) * 1e-6:.0f} ms\n\n")

























    # Localization evaluation

    robot_vicon_status = {}

    for robot in robots:
        robot_vicon_status[robot] = (len(tf_all_to_ground_truth[robot])!=0)

    distance_errors = {robot: [] for robot in robots}
    x_errors = {robot: [] for robot in robots}
    y_errors = {robot: [] for robot in robots}
    yaw_errors = {robot: [] for robot in robots}
    

    time_tolerance = 5e7

    for robot in robots:
        if not robot_vicon_status[robot]:
            continue

        tf_robot = tf_all_to_base_link[robot]
        tf_robot_ground_truth = tf_all_to_ground_truth[robot]

        i = 0
        n = len(tf_robot_ground_truth)

        for ts, transform in tf_robot:
            while i < n and tf_robot_ground_truth[i][0] <= ts:
                i += 1

            if i == 0:
                continue

            ts_ground_truth, ground_truth_transform = tf_robot_ground_truth[i - 1]

            if ts - ts_ground_truth > time_tolerance:
                distance_errors[robot].append((ts, -1))
                x_errors[robot].append((ts, -1))
                y_errors[robot].append((ts, -1))
                yaw_errors[robot].append((ts, -1))
                continue

            position = np.array([transform.translation.x, transform.translation.y])
            ground_truth_positon = np.array([ground_truth_transform.translation.x, ground_truth_transform.translation.y])
            
            distance_error = np.linalg.norm(position - ground_truth_positon)
            x_error = abs(transform.translation.x - ground_truth_transform.translation.x)
            y_error = abs(transform.translation.y - ground_truth_transform.translation.y)
            yaw_error = abs((quaternion_to_yaw(transform.rotation) - quaternion_to_yaw(ground_truth_transform.rotation) + math.pi) % (2 * math.pi) - math.pi )

            # CORRECTION: LIMIT ERRORS PLOT
            if distance_error < 0.30 and yaw_error < 0.25:
                distance_errors[robot].append((ts, distance_error))
                x_errors[robot].append((ts, x_error))
                y_errors[robot].append((ts, y_error))
                yaw_errors[robot].append((ts, yaw_error))

            # distance_errors[robot].append((ts, distance_error))
            # x_errors[robot].append((ts, x_error))
            # y_errors[robot].append((ts, y_error))
            # yaw_errors[robot].append((ts, yaw_error))


    distance_mae = {}
    distance_rmse = {}
    distance_std = {}
    yaw_mae = {}
    yaw_rmse = {}
    yaw_std = {}

    for robot in robots:
        if not robot_vicon_status[robot]:
            continue

        distance_mae[robot] = np.array([e[1] for e in distance_errors[robot] if e[1]!=-1]).mean()
        distance_rmse[robot] = np.sqrt(np.array([e[1]**2 for e in distance_errors[robot] if e[1]!=-1]).mean())
        yaw_mae[robot] = np.array([e[1] for e in yaw_errors[robot] if e[1] != -1]).mean()
        yaw_rmse[robot] = np.sqrt(np.array([e[1]**2 for e in yaw_errors[robot] if e[1] != -1]).mean())


        # Distance
        distance_vals = np.array([e[1] for e in distance_errors[robot] if e[1] != -1])
        distance_mae[robot] = distance_vals.mean()
        distance_rmse[robot] = np.sqrt((distance_vals**2).mean())
        distance_std[robot] = distance_vals.std()

        # Yaw
        yaw_vals = np.array([e[1] for e in yaw_errors[robot] if e[1] != -1])
        yaw_mae[robot] = yaw_vals.mean()
        yaw_rmse[robot] = np.sqrt((yaw_vals**2).mean())
        yaw_std[robot] = yaw_vals.std()



    f.write("## Localization evaluation:\n\n")

    f.write("| Robot ID | Success | Vicon ground truth | Final distance error | Distance MAE | Distance RMSE | Distance STD | Final yaw error | Yaw MAE | Yaw RMSE | Yaw STD |\n")
    f.write("|----------|---------|--------------------|----------------------|--------------|---------------|--------------|-----------------|---------|----------|---------|\n")

    for robot in robots:
        success_status = "Yes" if success[robot] else "No"
        vicon_status = "Yes" if robot_vicon_status[robot] else "No"

        if success[robot] and robot_vicon_status[robot]:
            f.write(f"| `{robot}` | {success_status} | {vicon_status} | {distance_errors[robot][-1][1]:.2f} m | {distance_mae[robot]:.2f} m | {distance_rmse[robot]:.2f} m | {distance_std[robot]:.2f} m | {yaw_errors[robot][-1][1]:.2f} rad | {yaw_mae[robot]:.2f} rad | {yaw_rmse[robot]:.2f} rad | {yaw_std[robot]:.2f} rad |\n")
        else:
            f.write(f"| `{robot}` | {success_status} | {vicon_status} | - | - | - | - | - | - |\n")

    f.write("\n")








    # Plot localization trajectories



    def split_continuous_segments(points, threshold=0.05):

        if not points:
            return []

        segments = []
        current_segment = [points[0]]

        for p_prev, p_curr in zip(points[:-1], points[1:]):
            dist = np.linalg.norm(np.array(p_curr) - np.array(p_prev))
            if dist < threshold:
                current_segment.append(p_curr)
            else:
                segments.append(current_segment)
                current_segment = [p_curr]

        if current_segment:
            segments.append(current_segment)

        return segments





    plt.figure(figsize=(14, 10))
    plt.title("Robot trajectories", fontsize=14)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.axis("equal")
    plt.grid(True, linestyle="--", alpha=0.5)


    colors = [PRIMARY_CYAN_DARK, PRIMARY_MAGENTA_DARK, PRIMARY_YELLOW_DARK]
    for i, robot in enumerate(robots):
        plt.plot(
            [item[1].translation.x for item in tf_all_to_base_link[robot]],
            [item[1].translation.y for item in tf_all_to_base_link[robot]],
            linestyle="-",
            linewidth=2,
            color = colors[i],
            alpha=1.0,
            label=f"Measured trajectory - {robot}"
        )

        if not robot_vicon_status[robot]: 
            continue
        
        # plt.plot(
        #     [item[1].translation.x for item in tf_all_to_ground_truth[robot]],
        #     [item[1].translation.y for item in tf_all_to_ground_truth[robot]],
        #     linestyle="--",
        #     linewidth=2,
        #     color = colors[i],
        #     alpha=0.4,
        #     label=f"Ground truth trajectory - {robot}"
        # )

        gt_points = [(item[1].translation.x, item[1].translation.y) 
                 for item in tf_all_to_ground_truth[robot]]
    
        segments = split_continuous_segments(gt_points, threshold=0.05)

        first = True
        first_missing_gt = True
        last_x = None
        last_y = None

        for seg in segments:
            xs, ys = zip(*seg)

            if first:
                plt.plot(
                    xs, ys,
                    linestyle="--",
                    linewidth=2,
                    color=colors[i],
                    alpha=0.6,
                    label=f"Ground truth trajectory - {robot}"
                )
            else:
                plt.plot(
                    xs, ys,
                    linestyle="--",
                    linewidth=2,
                    color=colors[i],
                    alpha=0.6
                )
                if first_missing_gt:
                    plt.plot(
                        [last_x, xs[0]], [last_y, ys[0]],
                        linestyle=":",
                        linewidth=2,
                        color=colors[i],
                        alpha=0.3,
                        label=f"Missing ground truth - {robot}"
                    )
                    first_missing_gt = False
                else:
                    plt.plot(
                        [last_x, xs[0]], [last_y, ys[0]],
                        linestyle=":",
                        linewidth=2,
                        color=colors[i],
                        alpha=0.3
                    )
            
            last_x = xs[-1]
            last_y = ys[-1]

            first = False

            

    
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"{localization_path}/localization.png")
    plt.savefig(f"{localization_path}/localization.pdf")





    def split_valid_segments(ts_vals, errors_vals):
        segments = []
        current_ts, current_err = [], []

        for t, e in zip(ts_vals, errors_vals):
            if e != -1:
                current_ts.append(t)
                current_err.append(e)
            else:
                if current_ts:
                    segments.append((current_ts, current_err))
                    current_ts, current_err = [], []
        if current_ts:
            segments.append((current_ts, current_err))

        return segments


    fig, axes = plt.subplots(len(robots), 1, figsize=(14, 4 * len(robots)), sharex=True)

    if len(robots) == 1:
        axes = [axes]

    colors = [PRIMARY_CYAN_DARK, PRIMARY_MAGENTA_DARK, PRIMARY_YELLOW_DARK]

    for i, robot in enumerate(robots):
        if not robot_vicon_status[robot]:
            continue

        ax = axes[i]
        ts_vals = [(ts - start_timestamp) * 1e-6 for ts, _ in distance_errors[robot]]
        errors_vals = [errors for _, errors in distance_errors[robot]]

        # Split into valid contiguous segments
        segments = split_valid_segments(ts_vals, errors_vals)

        last_t, last_e = None, None
        first_missing_val = True
        for seg_i, (ts_seg, err_seg) in enumerate(segments):
            # Plot continuous segment
            ax.plot(
                ts_seg,
                err_seg,
                linewidth=2,
                color=colors[i],
                label=f"Distance error - {robot}" if seg_i == 0 else None
            )

            # Plot dotted connector to next valid segment
            if last_t is not None and last_e is not None:
                if first_missing_val:
                    ax.plot(
                        [last_t, ts_seg[0]], [last_e, err_seg[0]],
                        linestyle=":",
                        linewidth=1.5,
                        color=colors[i],
                        alpha=0.3,
                        label=f"Missing ground truth - {robot}"
                    )
                    first_missing_val = False
                else:
                    ax.plot(
                        [last_t, ts_seg[0]], [last_e, err_seg[0]],
                        linestyle=":",
                        linewidth=1.5,
                        color=colors[i],
                        alpha=0.3
                    )

            last_t, last_e = ts_seg[-1], err_seg[-1]

        # Add MAE and RMSE horizontal lines
        ax.axhline(distance_mae[robot], color=colors[i], linestyle="--", alpha=0.7, label=f"MAE - {robot}")
        ax.axhline(distance_rmse[robot], color=colors[i], linestyle="-.", alpha=0.7, label=f"RMSE - {robot}")

        ax.set_ylim(bottom=0)
        ax.set_ylabel("Error [m]")
        ax.legend()
        ax.grid(True, linestyle="--", alpha=0.5)

    axes[-1].set_xlabel("t [ms]")
    fig.suptitle("Distance Error with MAE & RMSE", fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.97])

    plt.savefig(f"{localization_path}/distance_error.png")
    plt.savefig(f"{localization_path}/distance_error.pdf")




    # Plot yaw error
    fig, axes = plt.subplots(len(robots), 1, figsize=(14, 4 * len(robots)), sharex=True)

    if len(robots) == 1:
        axes = [axes]

    for i, robot in enumerate(robots):
        if not robot_vicon_status[robot]: 
            continue

        ax = axes[i]
        ts_vals = [(ts - start_timestamp) * 1e-6 for ts, _ in yaw_errors[robot]]
        errors_vals = [errors for _, errors in yaw_errors[robot]]

        # Split into valid contiguous segments
        segments = split_valid_segments(ts_vals, errors_vals)

        last_t, last_e = None, None
        first_missing_val = True
        for seg_i, (ts_seg, err_seg) in enumerate(segments):
            # Plot continuous segment
            ax.plot(
                ts_seg,
                err_seg,
                linewidth=2,
                color=colors[i],
                label=f"Yaw error - {robot}" if seg_i == 0 else None
            )

            # Plot dotted connector to next valid segment
            if last_t is not None and last_e is not None:
                if first_missing_val:
                    ax.plot(
                        [last_t, ts_seg[0]], [last_e, err_seg[0]],
                        linestyle=":",
                        linewidth=1.5,
                        color=colors[i],
                        alpha=0.3,
                        label=f"Missing ground truth - {robot}"
                    )
                    first_missing_val = False
                else:
                    ax.plot(
                        [last_t, ts_seg[0]], [last_e, err_seg[0]],
                        linestyle=":",
                        linewidth=1.5,
                        color=colors[i],
                        alpha=0.3
                    )

            last_t, last_e = ts_seg[-1], err_seg[-1]

        # Add MAE and RMSE horizontal lines
        ax.axhline(yaw_mae[robot], color=colors[i], linestyle="--", alpha=0.7, label=f"MAE - {robot}")
        ax.axhline(yaw_rmse[robot], color=colors[i], linestyle="-.", alpha=0.7, label=f"RMSE - {robot}")

        ax.set_ylabel("Error [rad]")
        ax.set_ylim(bottom=0)
        ax.legend()
        ax.grid(True, linestyle="--", alpha=0.5)

    axes[-1].set_xlabel("t [ms]")
    fig.suptitle("Yaw Error with MAE & RMSE", fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.97])

    plt.savefig(f"{localization_path}/yaw_error.png")
    plt.savefig(f"{localization_path}/yaw_error.pdf")





    f.write(f"### Localization results:\n\n")

    f.write(f"**Localization result:**\n\n")
    f.write(f"![Localization](localization/localization.png)\n\n")

    f.write(f"**Distance error:**\n\n")
    f.write(f"![Distance error](localization/distance_error.png)\n\n")

    f.write(f"**Yaw error:**\n\n")
    f.write(f"![Yaw error](localization/yaw_error.png)\n\n")









































    # Mapping evaluation
    unknown_value = -1

    # CORRECTION: DELETE LAST PART OF THE EXPERIMENT
    # last_map_msg = map_msgs[-5]
    last_map_msg = map_msgs[-1]

    resolution = last_map_msg['data'].info.resolution
    width = last_map_msg['data'].info.width
    height = last_map_msg['data'].info.height
    origin = (last_map_msg['data'].info.origin.position.x, last_map_msg['data'].info.origin.position.y, last_map_msg['data'].info.origin.position.z)
    
    map_grid = np.array(last_map_msg['data'].data, dtype=np.int8).reshape(((height, width)))
    map_grid[map_grid==-1] = unknown_value

    save_map_png(map_grid, filename=f"{mapping_path}/map.png")


    map_name = args.bag.split("/")[-2].replace("bag", "map")
    save_map_pgm(map_grid, filename=f"{mapping_path}/{map_name}.pgm")
    save_map_yaml(
        resolution=resolution,
        origin=origin,
        occupied_thresh=0.65,
        free_thresh=0.25,
        filename=f"{mapping_path}/{map_name}.yaml",
        map_filename=f"{map_name}.pgm"
    )

    
    robot_map_grids = {}
    for robot in robots:
        # CORRECTION: DELETE LAST PART OF THE EXPERIMENT
        # map_msg = robot_map_msgs[robot][-5]
        map_msg = robot_map_msgs[robot][-1]

        robot_map_grid = np.array(map_msg['data'].data, dtype=np.int8).reshape(((height, width)))
        robot_map_grid[robot_map_grid==-1] = unknown_value

        robot_map_grids[robot] = robot_map_grid

    colors = [PRIMARY_CYAN, PRIMARY_MAGENTA, PRIMARY_YELLOW]
    map_merged_name = args.bag.split("/")[-2].replace("bag", "map_merged")
    filenames = [f"{mapping_path}/map_{robot}.png" for robot in robots]
    filename = f"{mapping_path}/map_merged.png"

    merged_rgb_image = save_map_merged_png(robot_map_grids, colors, filenames, filename)








    # Plot merged map with localization trajectories

    # plt.figure(figsize=(12, 12))

    # plt.imshow(
    #     np.flipud(merged_rgb_image),
    #     cmap="gray", 
    #     origin="lower",
    #     extent=[
    #         origin[0], origin[0] + width * resolution,
    #         origin[1], origin[1] + height * resolution
    #     ]
    # )

    # plt.title("Robot Trajectories over Grid Map", fontsize=14)
    # plt.xlabel("X [m]")
    # plt.ylabel("Y [m]")

    # # Plot trajectories
    # colors = [PRIMARY_CYAN_DARK, PRIMARY_MAGENTA_DARK, PRIMARY_YELLOW_DARK]
    # for i, robot in enumerate(robots):
    #     xs = [item[1].translation.x for item in tf_all_to_base_link[robot]]
    #     ys = [item[1].translation.y for item in tf_all_to_base_link[robot]]

    #     plt.plot(xs, ys, color=colors[i], linewidth=0.5, label=f"Trajectory - {robot}")

    # plt.legend()
    # plt.grid(True, linestyle="--", alpha=0.5)
    # plt.axis("equal")

    # # Save the overlay
    # plt.savefig(f"{localization_path}/localization_on_map.png")
    # plt.savefig(f"{localization_path}/localization_on_map.pdf")



    plt.figure(figsize=(12, 12))

    plt.imshow(
        np.flipud(merged_rgb_image),
        cmap="gray",
        origin="lower",
        extent=[
            origin[0], origin[0] + width * resolution,
            origin[1], origin[1] + height * resolution
        ]
    )

    colors = [PRIMARY_CYAN_DARK, PRIMARY_MAGENTA_DARK, PRIMARY_YELLOW_DARK]
    for i, robot in enumerate(robots):
        xs = [item[1].translation.x for item in tf_all_to_base_link[robot]]
        ys = [item[1].translation.y for item in tf_all_to_base_link[robot]]

        plt.plot(xs, ys, color=colors[i], linewidth=0.8, label=f"Trajectory - {robot}")

    plt.axis("off")
    plt.gca().set_xticks([])
    plt.gca().set_yticks([])
    plt.gca().set_frame_on(False)

    plt.tight_layout(pad=0)

    plt.savefig(f"{mapping_path}/localization_on_map.png", bbox_inches="tight", pad_inches=0)
    plt.savefig(f"{mapping_path}/localization_on_map.pdf", bbox_inches="tight", pad_inches=0)











































    # Mapping accuracy with respect to rtabmap

    if args.rtabmap_bag != "-":
        rtabmap_map_msgs = rtabmap_bag_parser.get_messages(f'/grid_prob_map')

        unknown_value = -1

        last_rtabmap_map_msg = rtabmap_map_msgs[-1]

        rtabmap_resolution = last_rtabmap_map_msg['data'].info.resolution
        rtabmap_width = last_rtabmap_map_msg['data'].info.width
        rtabmap_height = last_rtabmap_map_msg['data'].info.height
        rtabmap_origin = (last_rtabmap_map_msg['data'].info.origin.position.x, last_rtabmap_map_msg['data'].info.origin.position.y, last_rtabmap_map_msg['data'].info.origin.position.z)
        
        rtabmap_map_grid = np.array(last_rtabmap_map_msg['data'].data, dtype=np.int8).reshape(((rtabmap_height, rtabmap_width)))
        rtabmap_map_grid[rtabmap_map_grid==-1] = unknown_value

        save_map_png(rtabmap_map_grid, filename=f"{mapping_path}/rtabmap_map.png")


        rtabmap_map_name = args.bag.split("/")[-2].replace("bag", "rtabmap_map")
        save_map_pgm(rtabmap_map_grid, filename=f"{mapping_path}/{rtabmap_map_name}.pgm")
        save_map_yaml(
            resolution=rtabmap_resolution,
            origin=rtabmap_origin,
            occupied_thresh=0.65,
            free_thresh=0.25,
            filename=f"{mapping_path}/{rtabmap_map_name}.yaml",
            map_filename=f"{rtabmap_map_name}.pgm"
        )



        map_meta = {
            "resolution": resolution,
            "width": width,
            "height": height, 
            "origin": origin 
        }


        rtabmap_meta = {
            "resolution": rtabmap_resolution,
            "width": rtabmap_width,
            "height": rtabmap_height, 
            "origin": rtabmap_origin 
        }


        map_accuracy, map_total, map_correct, map_tp, map_tn, map_fp, map_fn = map_grid_rtabmap_accuracy(map_grid, rtabmap_map_grid, map_meta, rtabmap_meta)


        map_tpr = map_tp / (map_tp + map_fn)
        map_tnr = map_tn / (map_tn + map_fp)
        map_fpr = map_fp / (map_fp + map_tn)
        map_fnr = map_fn / (map_fn + map_tp)









    f.write(f"## Mapping evaluation:\n\n")

    f.write(f"**Map resolution:** {resolution:.2f} m/pixel\n\n")
    f.write(f"**Dimension:** {width} x {height} cells ({width*resolution:.2f} x {height*resolution:.2f} m)\n\n")
    f.write(f"**Map origin:** {origin}\n\n")

    explored_area = np.sum(map_grid != unknown_value) * resolution**2
    f.write(f"**Explored area:** {explored_area:.2f} m²\n\n")


    f.write("| Robot ID | Exploration area | Exploration percentage |\n")
    f.write("|----------|------------------|------------------------|\n")

    for robot in robots:
        robot_explored_area = np.sum(robot_map_grids[robot] != unknown_value) * resolution**2
        robot_explored_percentage = (robot_explored_area / explored_area * 100) if explored_area > 0 else 0.0
        f.write(f"| `{robot}` | {robot_explored_area:.2f} m² | {robot_explored_percentage:.2f}% |\n")


    for r in range(2, len(robots) + 1):
        for combo in itertools.combinations(robots, r):
            combined_mask = np.zeros_like(map_grid, dtype=bool)
            for robot in combo:
                combined_mask |= (robot_map_grids[robot] != unknown_value)

            combined_explored_area = np.sum(combined_mask) * resolution**2
            combined_percentage = (combined_explored_area / explored_area * 100) if explored_area > 0 else 0.0
            if combined_percentage > 100:
                combined_percentage = 100.0

            f.write(f"| {', '.join([f'`{r}`' for r in combo])} | {combined_explored_area:.2f} m² | {combined_percentage:.2f}% |\n")

    f.write("\n")



    if args.rtabmap_bag != "-":
        f.write(f"**Mapping accuracy** (wrt rtabmap): {map_accuracy:.4f} ({map_correct}/{map_total} compared cells)\n")

        f.write("| Quantity | Value |\n")
        f.write("|--------|-------|\n")
        f.write(f"| True Positives (TP)  | {map_tp} |\n")
        f.write(f"| True Negatives (TN)  | {map_tn} |\n")
        f.write(f"| False Positives (FP) | {map_fp} |\n")
        f.write(f"| False Negatives (FN) | {map_fn} |\n")

        f.write("\n")

        f.write("| Metric | Value |\n")
        f.write("|--------|-------|\n")
        f.write(f"| True Positive Rate (TPR)  | {map_tpr} |\n")
        f.write(f"| True Negative Rate (TNR)  | {map_tnr} |\n")
        f.write(f"| False Positive Rate (FPR) | {map_fpr} |\n")
        f.write(f"| False Negative Rate (FNR) | {map_fnr} |\n")


    f.write(f"### Mapping results:\n\n")

    if args.rtabmap_bag != "-":
        f.write(f"**Map rtabmap:**\n\n")
        f.write(f"![Map](mapping/padded_rtabmap_map.png)\n\n")


    f.write(f"**Map result:**\n\n")
    f.write(f"![Map](mapping/map.png)\n\n")

    if args.rtabmap_bag != "-":
        f.write(f"**Map error:**\n\n")
        f.write(f"![Map](mapping/error_map.png)\n\n")


        f.write(f"**Map confusion:**\n\n")
        f.write(f"![Map](mapping/confusion_map.png)\n\n")
        f.write(f"Grey = not_evaluated, Green = TP, White = TN, Blue = FP, RED = FN\n\n")

    for robot in robots:
        f.write(f"**Map {robot}:**\n\n")
        f.write(f"![Map {robot}](mapping/map_{robot}.png)\n\n")

    f.write(f"**Map merged:**\n\n")
    f.write(f"![Map merged](mapping/map_merged.png)\n\n")

    f.write(f"**Map merged with robot trajectories:**\n\n")
    f.write(f"![Map merged with robot trajectories](mapping/localization_on_map.png)\n\n")
   





    print(f"Saved report to {results_path}/report.md")
    f.close()
