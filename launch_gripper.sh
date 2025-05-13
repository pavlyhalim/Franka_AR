#!/bin/bash

# Activating Conda Environment
echo "Activating Conda Environment: polymetis"
source $(conda info --base)/etc/profile.d/conda.sh
conda activate polymetis

# Disable Firewall
echo "Disabling Firewall..."
sudo ufw disable

# Running the Gripper Server in the background
echo "Starting the Gripper Server..."
launch_gripper.py gripper=franka_hand