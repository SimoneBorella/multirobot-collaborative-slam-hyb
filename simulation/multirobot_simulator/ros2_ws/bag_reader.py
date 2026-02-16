import argparse
import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import matplotlib.pyplot as plt

GREEN = "#29B266"
ORANGE = "#E68021"
RED = "#FF0000"
GREY = "#424A4A"
YELLOW = "#F2C40F"
BLUE = "#2673A6"


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


if __name__ == "__main__":
    # Argument parsing
    parser = argparse.ArgumentParser(
        description="Plot logged data retrieved by rosbags."
    )
    parser.add_argument(
        "bag",
        type=str,
        help="Specify rosbag2 .db3 (sqlite3 database) file path",
    )

    parser.add_argument(
        "--namespace",
        type=str,
        default="robot_12",
        help="Specify namespace",
    )

    args = parser.parse_args()

    namespace = args.namespace
    parser = BagFileParser(args.bag)

    print(f"{namespace}_tf_static:")
    tf_static_msgs = parser.get_messages(f"/{namespace}/tf_static")

    for msg in tf_static_msgs:
        for tf in msg["data"].transforms:
            print(f"\tframe_id: {tf.header.frame_id}")
            print(f"\tchild_frame_id: {tf.child_frame_id}")
            print(f"\ttranslation: {tf.transform.translation}")
            print(f"\trotation: {tf.transform.rotation}")
            print()
    print()
    

    print(f"{namespace}_tf:")

    transform_description_list = [
        (f'map', f'odom'),
        (f'odom', f'base_footprint'),
        (f'base_link', f'wheel_left_link'),
        (f'base_link', f'wheel_right_link'),
    ]

    tf_msgs = parser.get_messages(f"/{namespace}/tf")
    tf_msgs.reverse()

    for msg in tf_msgs:
        if(len(transform_description_list)==0):
                break
        for tf in msg["data"].transforms:
            for transform_description in transform_description_list:
                if (tf.header.frame_id == transform_description[0] and tf.child_frame_id == transform_description[1]):
                    print(f"\tframe_id: {tf.header.frame_id}")
                    print(f"\tchild_frame_id: {tf.child_frame_id}")
                    print(f"\ttranslation: {tf.transform.translation}")
                    print(f"\trotation: {tf.transform.rotation}")
                    print()
                    transform_description_list.remove(transform_description)
    print()

    exit()