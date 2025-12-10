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
GREY = "#424A4A"
YELLOW = "#F2C40F"
BLUE = "#2673A6"



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
        "--topic",
        type=str,
        default="/map",
        help="Specify map topic",
    )

    parser.add_argument(
        "--unknown_replace_value",
        type=int,
        default=100,
        help="Unknown cells value",
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


if __name__ == "__main__":
    args = parse_args()

    map_topic = args.topic
    bag_parser = BagFileParser(args.bag)
    unknown_replace_value = args.unknown_replace_value

    map_msgs = bag_parser.get_messages(map_topic)

    map_msg = map_msgs[-1]

    print()

    print(f"Message header: {map_msg['data'].header}")
    print()

    resolution = map_msg['data'].info.resolution
    width = map_msg['data'].info.width
    height = map_msg['data'].info.height
    origin = (map_msg['data'].info.origin.position.x, map_msg['data'].info.origin.position.y, map_msg['data'].info.origin.position.z)

    print(f"Map resolution: {resolution}")
    print(f"Map width: {width}")
    print(f"Map height: {height}")
    print(f"Map origin: {origin}")
    print()

    data = map_msg['data'].data

    grid = np.array(data, dtype=np.int8).reshape(((height, width)))

    grid[grid==-1] = unknown_replace_value


    source_file_path = os.path.dirname(os.path.abspath(__file__))

    maps_path = f"{source_file_path}/bag_extracted_maps"
    if not os.path.exists(maps_path):
        os.makedirs(maps_path)
    
    map_name = args.bag.split("/")[-2].replace("bag", "map")

    
    save_pgm(grid, filename=f"{maps_path}/{map_name}.pgm")
    save_yaml(
        resolution=resolution,
        origin=origin,
        occupied_thresh=0.65,
        free_thresh=0.25,
        filename=f"{maps_path}/{map_name}.yaml",
        map_filename=f"{map_name}.pgm"
    )
    

    
    