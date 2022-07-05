#!/usr/bin/env python3

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import argparse
import numpy as np
import struct

import argparse
import cv2
import numpy as np
import os
from pathlib import Path
import struct


parser = argparse.ArgumentParser()
parser.add_argument("--source", type=str, help="Source for rosbag")
parser.add_argument("--destination", type=str, help="Destination for bin and txt file output")
args = parser.parse_args()
frontdict = {};
leftdict = {};

frontcount = 0;
leftcount = 0;
rightcount = 0;

class BagFileParser():

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

    def process_messages(self, topic_name, skipCount = 0, showCount = 100): # was 1000000
        global leftdict
        global frontdict
        global leftcount
        global frontcount
        global rightcount
        notprinted = True
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
            #looking for matching timestamps
            rf = -8 #rounding factor
            pc = 1
            if topic=="/luminar_front_points":
                if frontcount <= pc:
                    #print(str(timestamp))
                    print(str(round(timestamp, rf)))
                    #print("Initial front timestamp is " + str(timestamp))
                    frontcount+=1
                    print("width: " + str(pointCloud.width))
                    print("height: " + str(pointCloud.height))
                    print("row_step: " + str(pointCloud.row_step))
                    print("point_step: " + str(pointCloud.point_step))
                frontdict[round(timestamp, rf)] = pointCloud
            elif topic=="/luminar_left_points":
                if leftcount <= pc:
                    #print(str(timestamp))
                    print(str(round(timestamp, rf)))
                    #print("Initial left timestamp is " + str(timestamp))
                    leftcount+=1
                    print("width: " + str(pointCloud.width))
                    print("height: " + str(pointCloud.height))
                    print("row_step: " + str(pointCloud.row_step))
                    print("point_step: " + str(pointCloud.point_step))
                leftdict[round(timestamp, rf)] = pointCloud
            elif topic=="/luminar_right_points":
                if rightcount <= pc:
                    #print(str(timestamp))
                    print(str(round(timestamp, rf)))
                    #print("Initial right timestamp is "  + str(timestamp))
                    rightcount+=1
                    print("width: " + str(pointCloud.width))
                    print("height: " + str(pointCloud.height))
                    print("row_step: " + str(pointCloud.row_step))
                    print("point_step: " + str(pointCloud.point_step))
                rounded_timestamp = round(timestamp, rf)
                if rounded_timestamp in frontdict.keys() and rounded_timestamp in leftdict.keys():
                    self.bin_data(timestamp, leftdict[rounded_timestamp], "left")
                    self.bin_data(timestamp, frontdict[rounded_timestamp], "front")
                    self.bin_data(timestamp, pointCloud, "right")
                    print("Binned 1 set")
                    exit()
                '''
                if round(timestamp, rf) in frontdict.keys():
                    ...#print("hit front at timestamp: " + str(round(timestamp, rf)))
                if round(timestamp, rf) in leftdict.keys():
                    ...#print("hit left at timestamp: " + str(round(timestamp, rf)))
                    #newLine = timestamp + "***" + leftdict[timestamp] + frontdict[timestamp] + pointcloud
                    #result = result + "\n"  +newLine
                '''
            #print(f"Got {topic_name} for timestep {timestamp}")        
        return True

    def bin_data(self, timestamp, msg, direction):
        width = msg.width
        height = msg.height
        point_step = msg.point_step
        row_step = msg.row_step
        data = msg.data
        is_bigendian = msg.is_bigendian
        # TODO: If this assertion fails and we start seeing ordered point
        # clouds, account for LiDAR messages that aren't 1 line
        assert (height == 1), "PointCloud2 is of type ordered: expected unordered"
        size = width

        fmt = '>' if is_bigendian else '<'
        fmt += 'f'

        # print(struct.unpack_from('BBBB', msg.data))
        offset = 0
        points = np.zeros(shape=(size, 4))
        # TODO: If we start seeing ordered (multi-line) point clouds this will
        # need to change
        for u in range(size):
            # x y z r (r=intensity)
            # print(struct.unpack_from(fmt, data, offset=offset))
            points[u][0] = (struct.unpack_from(fmt, data, offset=offset))[0]
            points[u][1] = (struct.unpack_from(fmt, data, offset=offset+4))[0]
            points[u][2] = (struct.unpack_from(fmt, data, offset=offset+8))[0]
            points[u][3] = (struct.unpack_from(fmt, data, offset=offset+12))[0]
            offset += point_step
        #with open("testfile.bin") as file:
        with open(str(timestamp) + "_" + str(direction) + ".bin", 'w') as file:
            points.astype('float32').tofile(file)


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
                