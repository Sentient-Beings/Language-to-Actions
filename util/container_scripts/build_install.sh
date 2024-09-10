#!/usr/bin/env bash
source ~/.bashrc
source /opt/ros/humble/setup.bash

cd /home/agentic_workflow/mnt/ws
colcon build 
source install/setup.bash
