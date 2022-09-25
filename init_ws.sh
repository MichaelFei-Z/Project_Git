#!/bin/bash

## Author: Michael
## Data: 2022-09-19
## Usage: ./init_ws.sh folder_path

ws_name="catkin_ws" # your workspace name
folder_path="$1"

mkdir -p $folder_path/$ws_name/src
cd $folder_path/$ws_name/src
catkin_init_workspace

cd $folder_path/$ws_name/
catkin_make

rossource="source $folder_path/$ws_name/devel/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in ~/.bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource