import numpy as np
from scipy.spatial.transform import Rotation as R
from loop_rate_limiters import RateLimiter
from FrankaClient import FrankaClient
import time
import config

def main():
    # Connect to robot
    robot = FrankaClient(
        server_ip=config.SERVER_IP,
        port=config.SERVER_PORT,
        connection_timeout=config.CONNECTION_TIMEOUT,
        heartbeat_timeout=config.HEARTBEAT_TIMEOUT
    )
    
    # Start cartesian impedance control
    robot.start_cartesian_impedance(Kx=config.CARTESIAN_KX, Kxd=config.CARTESIAN_KXD)
    
    # Predefined poses for testing
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
        [0.30276144, 0.00810913, 0.52167195, -2.25313821, 0.10338608, -1.74913836],
        [0.19047891, 0.02104671, 0.98909324, -0.99779258, 0.14904579, -2.1933166],
        [0.32955277, -0.03149165, 0.61245275, -1.73002143, 0.74150974, -1.58129807],
    ]
    
    # Set up rate limiter for smooth control
    rate = RateLimiter(frequency=config.TEST_CONTROL_FREQUENCY, warn=False)
    
    # Get the initial pose
    current_pose = robot.get_ee_pose()
    print(f"Starting pose: {current_pose}")
    
    # Move through each pose in sequence
    for idx, target_pose in enumerate(poses):
        print(f"Moving to pose {idx+1}/{len(poses)}")
        
        # Get current pose for accurate interpolation
        start_pose = robot.get_ee_pose()
        start_pos = start_pose[:3]
        start_rot = start_pose[3:]
        
        target_pos = np.array(target_pose[:3])
        target_rot = np.array(target_pose[3:])
        
        start_time = time.time()
        
        # Smooth interpolation loop
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # Calculate progress (0.0 to 1.0)
            progress = min(elapsed_time / config.TEST_TRANSITION_TIME, 1.0)
            
            # Linear interpolation for position and rotation
            new_pos = start_pos + progress * (target_pos - start_pos)
            new_pos[0] = config.TEST_FIXED_X_POSITION  # Fix X position
            
            new_rot = start_rot + progress * (target_rot - start_rot)
            
            # Update robot pose
            updated_pose = np.concatenate([new_pos, new_rot])
            robot.update_desired_ee_pose(updated_pose)
            
            # Print progress occasionally
            if int(elapsed_time * 10) % 20 == 0:  # Every 2 seconds
                print(f"Progress: {progress*100:.1f}%")
            
            # Exit when complete
            if progress >= 1.0:
                break
                
            rate.sleep()
        
        print(f"Reached pose {idx+1}")
        time.sleep(0.1)  # Brief pause between poses
    
    print("Motion sequence completed")

if __name__ == "__main__":
    main()
