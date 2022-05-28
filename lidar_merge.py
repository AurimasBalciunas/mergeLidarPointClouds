#!/usr/bin/env python3

import sqlite3
import cv2
import numpy as np
import argparse
import os
from multiprocessing import Process
import glob

parser = argparse.ArgumentParser()
parser.add_argument(
    "-v", action="store_true", help="Give more information about outputs"
)
parser.add_argument("--source", type=str, help="Source for rosbag")
parser.add_argument("--destination", type=str, help="Destination for video output")
args = parser.parse_args()

def readBlobData(
    topicId, source, destination, frame, verbose, skipCount=0, showCount=1000000
):
    try:
        #topicName = "/camera/" + frame + "/image/compressed"
        topicName = frame
        os.mkdir(destination + "/{}".format(frame))

        sqliteConnection = None
        img = None
        row_id = 0
        (rows, cols) = (0, 0)

        file_path = source + "*.db3"
        files = sorted(
            glob.glob(file_path), key=lambda x: int(x.split("_")[-1].split(".")[0])
        )

        print("Writing " + frame)
        for dbName in files:
            print("New dbname")
            sqliteConnection = sqlite3.connect(dbName)
            cursor = sqliteConnection.cursor()
            if verbose:
                print("Connected to SQLite")

            if topicName is not None:
                cursor.execute("""SELECT id from topics where name = ?""", (topicName,))
                row = cursor.fetchone()
                if row is None:
                    print("topicName", topicName, "not found in", dbName)
                topicId = int(row[0])
                if verbose:
                    print("topicName: ", topicName, ", topicId: ", topicId)

            cursor.execute(
                """SELECT * from messages where topic_id = ? LIMIT ?,?""",
                (topicId, skipCount, showCount),
            )

            done = False
            num_frame = 0
            while not done:
                row = cursor.fetchone()
                if row is None:
                    done = True
                    break
                num_frame = num_frame + 1
                row_id = row[0]
                topic_id = row[1]
                print(topic_id)

                counter2 = 0
                if topic_id == topicId:
                    timestamp = row[2]
                    photo = row[3]
                    counter2+=1
                    print(row[3])
                    if(counter2 > 1):
                        exit()
                    #print(row[2])
                    #print(row[3]
                    
                    """
                    final_name = destination + "/" + frame + "/" + frame + "_{}".format(num_frame) + ".bin"
                    with open(final_name, "w") as f:
                        f.write(str(photo))
                    print("one file written")
                    """





        print("Finished writing " + frame)
        cursor.close()
    except sqlite3.Error as error:
        print("Failed to read blob data from sqlite table", error)
    finally:
        if sqliteConnection:
            sqliteConnection.close()
            if verbose:
                print("sqlite connection is closed")


if __name__ == "__main__":
    frames = [
        "/luminar_left_points",
        "/luminar_right_points",
        "/luminar_front_points",
    ]
    p = Process(target=readBlobData, args=(-1, args.source, args.destination, frames[1], args.v))
    p.start()
    """
    processes = []
    for frame in frames:
        p = Process(
            target=readBlobData, args=(-1, args.source, args.destination, frame, args.v)
        )
        p.start()
        processes.append(p)

    for p in processes:
        p.join()
    """
