import requests
from scipy.spatial.transform import Rotation as R
import numpy as np
from loop_rate_limiters import RateLimiter
from mujoco_ar import MujocoARConnector
from FrankaClient import FrankaClient
import time
import random

robot = FrankaClient(
    server_ip="127.0.0.1"
)

robot.start_cartesian_impedance(
    Kx = (np.array([750.0, 750.0, 750.0, 15.0, 15.0, 15.0]) * 0.4),
    Kxd = (np.array([37.0, 37.0, 37.0, 2.0, 2.0, 2.0]) * 0.5)
)

# while True:
#     robot.set_gripper_width(0.055)


# rate = RateLimiter(frequency=20, warn=False)

# while True:

#     current_pose = robot.get_ee_pose() # x, y, z, rx, ry, rz

#     current_pose[2] -= 0.01

#     robot.update_desired_ee_pose(current_pose)

#     rate.sleep()

# while True:
#         robot.set_gripper_width(0.055)

#     else:
#         interface.set_gripper_width(0.085)


# current_pose[2] -= 0.05

# robot.update_desired_ee_pose(current_pose)
# robot.set_gripper_width(0.0005)

print(robot.get_ee_pose())
# print(robot.get_joint_positions())

# print(current_pose[3])