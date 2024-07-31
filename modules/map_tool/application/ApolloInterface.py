import os
import subprocess

import google.protobuf.text_format as text_format
from modules.common_msgs.map_msgs import map_pb2


class ApolloInterface():
    
    def __init__(self):
        self._map = map_pb2.Map()

    def write_pb_to_text_file(self, file_name):
        with open(file_name, 'w') as f:
            f.write(str(self._map))
            print("File success saved in: {}".format(file_name))

    def get_pb_from_text_file(self, file_name):
        with open(file_name, 'r') as file_in:
            text_format.Merge(file_in.read(), self._map)
            print("File success loaded from: {}".format(file_name))

    def map(self):
        return self._map
    
    def print_debug(self):
        print("-------------------------------------------------")
        print(f"Header:         \n\n{self._map.header}")
        print(f"Crosswalk:      \t{len(self._map.crosswalk)}")
        print(f"Junction:       \t{len(self._map.junction)}")
        print(f"Lane:           \t{len(self._map.lane)}")
        print(f"StopSign:       \t{len(self._map.stop_sign)}")
        print(f"Signal:         \t{len(self._map.signal)}")
        print(f"YieldSign:      \t0 (Undefined)")
        print(f"Overlap:        \t{len(self._map.overlap)}")
        print(f"ClearArea:      \t{len(self._map.clear_area)}")
        print(f"SpeedBump:      \t{len(self._map.speed_bump)}")
        print(f"Road:           \t{len(self._map.road)}")
        print(f"ParkingSpace:   \t{len(self._map.parking_space)}")
        print(f"PNCJunction:    \t{len(self._map.pnc_junction)}")
        print(f"RSU:            \t{len(self._map.rsu)}")
        print("-------------------------------------------------")

