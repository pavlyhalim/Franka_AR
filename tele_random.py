import requests
from scipy.spatial.transform import Rotation as R
import numpy as np
from loop_rate_limiters import RateLimiter
from mujoco_ar import MujocoARConnector
from FrankaClient import FrankaClient
import time

infront_of_robot = False

# Connect to the server
interface = FrankaClient(
    server_ip='127.0.0.1',
)

# Start the AR connector
connector = MujocoARConnector(port=8888, debug=False) 
connector.start()

# Cartesian impedance controller gains
Kx = (np.array([750.0, 750.0, 750.0, 15.0, 15.0, 15.0]) * 0.4)
Kxd = (np.array([37.0, 37.0, 37.0, 2.0, 2.0, 2.0]) * 0.5)
rate = RateLimiter(frequency=1000, warn=False)

MIN_Z_HEIGHT = 0.3
MAX_Z_HEIGHT = 0.9

MIN_Y_HEIGHT = -0.5
MAX_Y_HEIGHT = 0.38

# Define poses sequence from test.py
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
def execute_pose_sequence():
    """Execute the predefined pose sequence"""
    print("Phone disconnected. Starting pose sequence...")
    
    # Set up a rate limiter for smoother control during sequence
    sequence_rate = RateLimiter(frequency=200, warn=False)
    transition_time = 4.0  # seconds per movement
    
    # Get the initial pose
    current_pose = interface.get_ee_pose()  # x, y, z, rx, ry, rz
    # print(f"Starting sequence from pose: {current_pose}")
    
    # Move through each pose in sequence
    for idx, target_pose in enumerate(poses):
        # print(f"Moving to pose {idx+1}/{len(poses)}")
        
        # Get the current pose again to ensure accuracy
        start_pose = interface.get_ee_pose()
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
            # Fix the X position
            new_pos[0] = 0.33
            
            # Linear interpolation for rotation
            new_rot = start_rot + progress * (target_pose[3:] - start_rot)
            
            # Combine position and rotation components
            updated_pose = np.concatenate([new_pos, new_rot])
            
            # Update robot position
            try:
                interface.update_desired_ee_pose(updated_pose)
            except:
                interface.start_cartesian_impedance(Kx=Kx, Kxd=Kxd)
            
            # Exit loop once transition is complete
            if progress >= 1.0:
                break
                
            # Sleep to maintain control frequency
            sequence_rate.sleep()
        
        # Wait a moment at the target pose
        # print(f"Reached pose {idx+1}")
        time.sleep(0.1)
    
    print("Motion sequence completed")

# Start the cartesian impedance controller
interface.start_cartesian_impedance(Kx=Kx, Kxd=Kxd)

# Get the initial pose
start_rot = interface.get_ee_pose()[3:]
start_rot_matrix = R.from_rotvec(start_rot).as_matrix()
start_pos = interface.get_ee_pose()[:3]

# Wait for the AR connector to get the first data
while connector.get_latest_data()["position"] is None:
    pass

last_episode_stop = None

while True:

    phone_pose = np.identity(4)
    phone_pose[:3, :3] = connector.get_latest_data()["rotation"]
    # print(phone_pose)
    phone_pose[:3, 3] = connector.get_latest_data()["position"]
    
    if infront_of_robot:
        z_fix_pose = np.identity(4)
        z_fix_pose[:3, :3] = R.from_euler('z', np.pi).as_matrix()
        phone_pose = z_fix_pose @ phone_pose
        phone_pose = phone_pose @ z_fix_pose
    
    new_pos = start_pos + phone_pose[0:3,3]  * 0.8
    # # add safe bounds

    # new_pos[2] = max(0.155, new_pos[2])
    # new_pos[2] = max(0.127, new_pos[2])
    new_pos[2] = max(MIN_Z_HEIGHT, new_pos[2])
    new_pos[2] = min(MAX_Z_HEIGHT, new_pos[2])
    new_pos[1] = max(MIN_Y_HEIGHT, new_pos[1])
    new_pos[1] = min(MAX_Y_HEIGHT, new_pos[1])
    # new_pos[0] = min(0.5, new_pos[0])
    new_pos[0] = 0.33
    # new_pos[3] = -1.69830349
    # new_pos[1] = max(-0.09, new_pos[1]) 

    # print(new_pos)

    transformation_matrix = phone_pose[:3, :3]
    transformed_matrix = transformation_matrix @ start_rot_matrix
    transformed_rot_vec = R.from_matrix(transformed_matrix).as_rotvec()
    # transformed_rot_vec[0] = -1.69830349

    updated_pose = np.concatenate([new_pos, transformed_rot_vec])

    try:
        interface.update_desired_ee_pose(updated_pose)
    except:
        interface.start_cartesian_impedance(Kx=Kx, Kxd=Kxd)


    # if connector.get_latest_data()["toggle"] is True:
    #     interface.set_gripper_width(0.055)
    # else:
    #     interface.set_gripper_width(0.085)

    if connector.get_latest_data()["button"] is True:
        execute_pose_sequence()
        if last_episode_stop is None or time.time() - last_episode_stop > 5:
            response = requests.post("http://localhost:5000/trigger")
            last_episode_stop = time.time()

    

    rate.sleep()