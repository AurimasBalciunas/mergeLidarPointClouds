#!/usr/bin/env python3

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--source", type=str, help="Source for rosbag")
parser.add_argument("--destination", type=str, help="Destination for bin and txt file output")
args = parser.parse_args()

class BagFileParser():
    frontdict = {}
    leftdict = {}
    result = []
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    def process_messages(self, topic_name, skipCount = 0, showCount = 5): # was 1000000
        
        topic_id = self.topic_id[topic_name]
        # Get from the db
        self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? LIMIT ?,?""",(topic_id, skipCount, showCount),)
       
        done = False
        while not done:
            row = self.cursor.fetchone()
            if row is None:
                done = True
                break
            
            # Deserialise all and timestamp them
            timestamp = row[0]
            pointCloud = deserialize_message(row[1], self.topic_msg_message[topic_name])
            if topic=="/luminar_front_points":
                frontdict[timestamp] = topic_name
            elif topic=="/luminar_left_points"
                leftdict[timestamp] = topic_name
            elif topic=="/luminar_right_point"
                if frondict.has_key(timestamp) and leftdict.has_key(timestamp):
                    newLine = timestamp + "***" + leftdict[timestamp] + frontdict[timestamp]


            print(f"Got {topic_name} for timestep {timestamp}")
        
        return True

if __name__ == "__main__":

        bag_file = args.source

        parser = BagFileParser(bag_file)

        topics = ["/luminar_front_points", "/luminar_left_points","/luminar_right_points"]
        for topic in topics:
            status = parser.process_messages(topic)
            print("="*80)
            if (status):
                print(f"Completed point processing for {topic}")
            else:
                print(f"FAILED point processing for {topic}")
                