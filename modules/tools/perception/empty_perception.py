#!/usr/bin/env python3

"""
this module creates a node and fake perception data
"""

import time
import argparse
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
from modules.common_msgs.perception_msgs.perception_obstacle_pb2 import PerceptionObstacles


def perception_publisher(perception_channel, rate):
    """publisher"""
    cyber.init()
    node = cyber.Node("perception")
    writer = node.create_writer(perception_channel, PerceptionObstacles)
    sleep_time = 1.0 / rate
    seq_num = 1
    while not cyber.is_shutdown():
        perception_osbtacles = PerceptionObstacles()
        perception_osbtacles.header.sequence_num = seq_num
        perception_osbtacles.header.timestamp_sec = cyber_time.Time.now().to_sec()
        perception_osbtacles.header.module_name = "perception"
        perception_osbtacles.header.frame_id = "map"
        print(str(perception_osbtacles))
        writer.write(perception_osbtacles)
        seq_num += 1
        time.sleep(sleep_time)
    cyber.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="create empty perception obstacles message")
    parser.add_argument("-c", "--channel", action="store", type=str, default="/apollo/perception/obstacles",
                        help="set the perception obstacles channel")
    parser.add_argument("-r", "--rate", action="store", type=int, default=10,
                        help="set the perception obstacles channel publish time duration")
    args = parser.parse_args()
    perception_publisher(args.channel, args.rate)
