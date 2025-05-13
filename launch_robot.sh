#!/bin/bash

# Activating Conda Environment
echo "Activating Conda Environment: polymetis"
source $(conda info --base)/etc/profile.d/conda.sh

conda activate polymetis
# Disable Firewall
echo "Disabling Firewall..."
sudo ufw disable

# Killing any Existing Servers
echo "Killing any existing servers"
sudo pkill -9 run_server

# Running the Franka Server in the background
echo "Starting the Franka Server..."
launch_robot.py robot_client=franka_hardware