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
    print(phone_pose)
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
    new_pos[0] = 0.29
    # new_pos[3] = -1.69830349
    # new_pos[1] = max(-0.09, new_pos[1]) 

    print(new_pos)

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
        if last_episode_stop is None or time.time() - last_episode_stop > 5:
            response = requests.post("http://localhost:5000/trigger")
            last_episode_stop = time.time()

    

    rate.sleep()