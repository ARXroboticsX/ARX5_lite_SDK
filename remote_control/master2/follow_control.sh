#!/bin/bash
# workspace=$(pwd)

source ~/.bashrc

gnome-terminal -t "master1" -x sudo bash -c "cd /home/razer/桌面/Ecat/ecat_arx_follow_source/master1;source ./devel/setup.bash && roslaunch arm_control arx5v.launch; exec bash;"
sleep 1
