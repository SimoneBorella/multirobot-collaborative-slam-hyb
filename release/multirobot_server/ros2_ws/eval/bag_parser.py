import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

class BagParser:
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        # Load topics metadata
        topics_data = self.cursor.execute(
            "SELECT id, name, type FROM topics"
        ).fetchall()

        # {topic_name: {id: ..., type: ...}}
        self.topics = {
            name: {"id": id_, "type": type_}
            for id_, name, type_ in topics_data
        }

    def __del__(self):
        try:
            self.conn.close()
        except Exception:
            pass

    def get_messages(self, topic_name):
        if topic_name not in self.topics:
            available = "\n  ".join(self.topics.keys())
            raise KeyError(
                f"Topic '{topic_name}' not in bag. Available topics:\n  {available}"
            )

        topic_meta = self.topics[topic_name]
        topic_id = topic_meta["id"]
        msg_type_identifier = topic_meta["type"]

        try:
            msg_cls = get_message(msg_type_identifier)
        except ModuleNotFoundError:
            raise ModuleNotFoundError(
                f"Cannot import message type '{msg_type_identifier}'. "
                "Is the required ROS package installed?"
            )

        rows = self.cursor.execute(
            f"SELECT timestamp, data FROM messages WHERE topic_id = {topic_id}"
        ).fetchall()

        messages = []
        for ts, raw in rows:
            msg = deserialize_message(raw, msg_cls)
            messages.append({"timestamp": ts, "data": msg})

        return messages

    def get_n_first_messages(self, topic_name, n=1):
        """
        Recupera i primi n messaggi (i più vecchi) per un determinato topic.
        Ritorna una lista di dizionari ordinati dal più vecchio al più recente.
        """
        if topic_name not in self.topics:
            available = "\n  ".join(self.topics.keys())
            raise KeyError(
                f"Topic '{topic_name}' not in bag. Available topics:\n  {available}"
            )

        topic_meta = self.topics[topic_name]
        topic_id = topic_meta["id"]
        msg_type_identifier = topic_meta["type"]

        try:
            msg_cls = get_message(msg_type_identifier)
        except ModuleNotFoundError:
            raise ModuleNotFoundError(
                f"Cannot import message type '{msg_type_identifier}'."
            )

        # Cambiamo ORDER BY timestamp da DESC a ASC per prendere i primi messaggi
        results = self.cursor.execute(
            "SELECT timestamp, data FROM messages "
            "WHERE topic_id = ? "
            "ORDER BY timestamp ASC LIMIT ?",
            (topic_id, n)
        ).fetchall()

        if not results:
            return []

        messages = []
        for timestamp, raw_data in results:
            msg = deserialize_message(raw_data, msg_cls)
            messages.append({"timestamp": timestamp, "data": msg})
        
        return messages

    def get_n_last_message(self, topic_name, n=1):
        """
        Recupera il singolo n-esimo messaggio più recente.
        n=1 è l'ultimo, n=2 è il penultimo, ecc.
        """
        if n < 1:
            raise ValueError("Il parametro n deve essere maggiore o uguale a 1.")

        if topic_name not in self.topics:
            available = "\n  ".join(self.topics.keys())
            raise KeyError(
                f"Topic '{topic_name}' not in bag. Available topics:\n  {available}"
            )

        topic_meta = self.topics[topic_name]
        topic_id = topic_meta["id"]
        msg_type_identifier = topic_meta["type"]

        try:
            msg_cls = get_message(msg_type_identifier)
        except ModuleNotFoundError:
            raise ModuleNotFoundError(
                f"Cannot import message type '{msg_type_identifier}'."
            )

        result = self.cursor.execute(
            "SELECT timestamp, data FROM messages "
            "WHERE topic_id = ? "
            "ORDER BY timestamp DESC LIMIT 1 OFFSET ?",
            (topic_id, n - 1)
        ).fetchone()

        if not result:
            return None

        timestamp, raw_data = result
        msg = deserialize_message(raw_data, msg_cls)
        
        return {"timestamp": timestamp, "data": msg}