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

# robot.set_gripper_width(0.0005)

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

# print(robot.get_ee_pose())
# print(robot.get_joint_positions())

# print(current_pose[3])

poses = [
    [0.33156064, -0.01603172, 0.79745853, -1.76751667, 0.54284367, -1.90647879],
    [0.38822079, 0.32070839, 0.47164139, -2.43147625, -0.19967937, -1.15933037],
    [0.34696248, -0.11200628, 0.79474294, -1.75794628, 0.73992233, -2.06166423],
    [0.35902002, -0.00602789, 0.98636353, -1.01442095, 0.81177634, -1.30876504],
    [0.37012458, -0.29279864, 0.59744781, -1.059807, 1.79468055, -1.14428223],
    [0.38043791, -0.44176033, 0.69270021, -0.64359446, 2.01502479, -1.39960962],
    [0.38124463, -0.28479406, 0.99853593, -0.48466614, 1.32456626, -1.83766387],
    [0.36032018, 0.34523964, 0.81660122, -1.68169808, 0.34155519, -1.27790099],
    [0.31422082, -0.01210051, 0.85161239, -1.69203558, 0.80911189, -2.06997761],
    [0.36122319, -0.1241796, 1.00724995, -1.40942591, 0.72255244, -2.62112178],
    [0.3326827, -0.22184773, 0.95942956, -1.05792559, 1.13279891, -2.15931433],
    [0.33076555, -0.02533704, 0.509911, -1.73433858, 1.43070732, -1.1156476],
    [0.32085368, 0.24629392, 0.95931989, -1.39285154, 0.44936788, -1.37287883],
    [0.32199094, 0.06236095, 0.89892572, -1.33501369, 0.76470208, -1.52255318],
    [0.32831132, -0.09705982, 0.79982287, -1.19633614, 1.20818784, -1.35439046],
    [0.32166088, -0.18573754, 0.75993508, -1.21658143, 1.31803507, -1.44843059],
    [0.30856007, -0.07161313, 0.96916991, -1.06295882, 0.82823239, -1.58781964],
    [0.32001621, 0.2472963, 0.64729738, -2.23116789, -0.87958146, -1.83061499],
    [0.34897503, -0.04463822, 0.42570806, -2.32191212, 0.40578916, -1.57230371],
    # [ 0.34795085, -0.00738653,  0.56436121, -2.21846513,  0.58424639, -1.64900467],
    [ 0.30276144,  0.00810913 , 0.52167195 ,-2.25313821 , 0.10338608 ,-1.74913836],
    [ 0.19047891,  0.02104671,  0.98909324, -0.99779258,  0.14904579, -2.1933166 ],
    [ 0.32955277, -0.03149165,  0.61245275, -1.73002143 , 0.74150974, -1.58129807],
]


# Set up rate limiter for smooth control
rate = RateLimiter(frequency=200, warn=False)
transition_time = 4.0  # seconds per movement

# Get the initial pose
current_pose = robot.get_ee_pose()  # x, y, z, rx, ry, rz
print(f"Starting pose: {current_pose}")

# Move through each pose in sequence
for idx, target_pose in enumerate(poses):
    print(f"Moving to pose {idx+1}/{len(poses)}")
    
    # Get the current pose again to ensure accuracy
    start_pose = robot.get_ee_pose()
    start_pos = start_pose[:3]  # position components
    start_rot = start_pose[3:]  # rotation components
    
    # Start timing for this transition
    start_time = time.time()
    
    # Loop until transition completes
    while True:
        current_time = time.time()
        elapsed_time = current_time - start_time
        
        # Calculate progress (0.0 to 1.0)
        progress = min(elapsed_time / transition_time, 1.0)
        
        # Linear interpolation for position
        new_pos = start_pos + progress * (target_pose[:3] - start_pos)
        # Fix the X position to 0.3317
        # new_pos[0] = 0.3317
        new_pos[0] = 0.33

        
        # Linear interpolation for rotation
        new_rot = start_rot + progress * (target_pose[3:] - start_rot)
        
        # Combine position and rotation components
        updated_pose = np.concatenate([new_pos, new_rot])
        
        # Update robot position
        robot.update_desired_ee_pose(updated_pose)
        
        # Print progress every few iterations
        if int(elapsed_time * 10) % 5 == 0:
            print(f"Progress: {progress*100:.1f}%")
            
        # Exit loop once transition is complete
        if progress >= 1.0:
            break
            
        # Sleep to maintain control frequency
        rate.sleep()
    
    # Wait a moment at the target pose
    print(f"Reached pose {idx+1}")
    time.sleep(0.1)

print("Motion sequence completed")




