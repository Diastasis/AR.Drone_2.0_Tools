#!/usr/bin/env python3
# This script automates the generation of aruco markers 
# Original project: https://github.com/fdcl-gwu/aruco-markers
#
#

import subprocess

# ./generate_marker --bb=1 -d=6 --id=6 --ms=800 6_mark
program_path = "/home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers/build/generate_marker" 	# Use your path
output_path = "/home/ros/parrot2_ws/src/parrot_ardrone/aruco-markers/create_markers/build/5x5_markers/"		# Use your path

for i in range(2):
    marker_id =  "--id={}".format(i)
    output_marker_name = "{}_marker.png".format(i)

    args = [program_path,"--bb=1","-d=6", marker_id, "--ms=800", output_path + output_marker_name]
    # subprocess allow the execution of linux commands and it captures the output
    output = subprocess.Popen( args, stdout=subprocess.PIPE,stderr=subprocess.PIPE ).communicate()
print("finished")
