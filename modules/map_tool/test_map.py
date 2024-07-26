import os
import subprocess

import google.protobuf.text_format as text_format
from modules.common_msgs.map_msgs import map_pb2

TOOL_PATH = "modules/map_tool"
MAPS_PATH = "modules/map/data"

IN_FILE = os.path.join(TOOL_PATH, "data", "line_spb.txt")
MAP_DIR = os.path.join(MAPS_PATH, "spb_map")
OUT_BASE_FILE = os.path.join(MAP_DIR, "base_map.txt")


def write_pb_to_text_file(file_name, topic_pb):
    """write pb message to file"""
    with open(file_name, 'w') as f:
        f.write(str(topic_pb))
        print("File success saved in: {}".format(file_name))


def get_pb_from_text_file(filename, pb_value):
    """Get a proto from given text file."""
    with open(filename, 'r') as file_in:
        return text_format.Merge(file_in.read(), pb_value)


map = map_pb2.Map()

get_pb_from_text_file(IN_FILE, map)


if not os.path.exists(MAP_DIR):
    os.mkdir(MAP_DIR)


write_pb_to_text_file(OUT_BASE_FILE, map)

routing_cmd = f"./scripts/generate_routing_topo_graph.sh --map_dir=/apollo/{MAP_DIR}"
sim_cmd = f"./bazel-bin/modules/map/tools/sim_map_generator \
        --map_dir=/apollo/{MAP_DIR} \
        --output_dir=/apollo/{MAP_DIR}"

subprocess.call([routing_cmd], shell=True)
subprocess.call([sim_cmd], shell=True)
subprocess.call([f'chmod 777 {MAP_DIR}'], shell=True)
subprocess.call([f'chmod 777 {MAP_DIR}/*'], shell=True)

print("done")