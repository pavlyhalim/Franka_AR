import requests
from scipy.spatial.transform import Rotation as R
import numpy as np
from loop_rate_limiters import RateLimiter
from mujoco_ar import MujocoARConnector
from FrankaClient import FrankaClient
import time
import config

# Connect to the server
interface = FrankaClient(
    server_ip=config.SERVER_IP,
    port=config.SERVER_PORT,
    connection_timeout=config.CONNECTION_TIMEOUT,
    heartbeat_timeout=config.HEARTBEAT_TIMEOUT
)

# Start the AR connector
connector = MujocoARConnector(port=config.AR_CONNECTOR_PORT, debug=config.AR_DEBUG) 
connector.start()

# Set up rate limiter
rate = RateLimiter(frequency=config.CONTROL_FREQUENCY, warn=False)

# Start the cartesian impedance controller
interface.start_cartesian_impedance(Kx=config.CARTESIAN_KX, Kxd=config.CARTESIAN_KXD)

# Get the initial pose and cache static calculations
start_rot = interface.get_ee_pose()[3:]
start_rot_matrix = R.from_rotvec(start_rot).as_matrix()
start_pos = interface.get_ee_pose()[:3]

# Pre-calculate transformation matrices for efficiency
z_fix_pose = None
if config.INFRONT_OF_ROBOT:
    z_fix_rotation = R.from_euler('z', np.pi).as_matrix()
    z_fix_pose = config.IDENTITY_4x4.copy()  # Use pre-allocated matrix
    z_fix_pose[:3, :3] = z_fix_rotation

# Wait for the AR connector to get the first data
print("Waiting for AR data...")
while connector.get_latest_data()["position"] is None:
    time.sleep(0.01)  # Small sleep to prevent busy waiting

print(f"AR data received, starting control loop at {config.CONTROL_FREQUENCY} Hz...")
last_episode_stop = None
loop_counter = 0
print_interval = int(config.CONTROL_FREQUENCY * config.PRINT_INTERVAL_MULTIPLIER)

# Pre-allocate arrays for hot path (avoid repeated allocation)
phone_pose = config.IDENTITY_4x4.copy()
new_pos = config.ZERO_VECTOR_3.copy()
updated_pose = config.ZERO_VECTOR_6.copy()

while True:
    # Get latest AR data once per loop
    ar_data = connector.get_latest_data()
    
    # Update phone pose transformation matrix (reuse pre-allocated matrix)
    phone_pose[:3, :3] = ar_data["rotation"]
    phone_pose[:3, 3] = ar_data["position"]
    
    # Apply front-of-robot transformation if needed (cached calculation)
    if config.INFRONT_OF_ROBOT and z_fix_pose is not None:
        phone_pose = z_fix_pose @ phone_pose @ z_fix_pose
    
    # Calculate new position with optimized operations (reuse pre-allocated array)
    np.copyto(new_pos, start_pos)
    new_pos += phone_pose[:3, 3] * config.POSITION_SCALE_FACTOR
    
    # Apply safety bounds efficiently
    new_pos[2] = np.clip(new_pos[2], config.MIN_Z_HEIGHT, config.MAX_Z_HEIGHT)
    new_pos[1] = np.clip(new_pos[1], config.MIN_Y_HEIGHT, config.MAX_Y_HEIGHT)
    new_pos[0] = config.FIXED_X_POSITION
    
    # Calculate rotation with cached matrix multiplication
    transformed_matrix = phone_pose[:3, :3] @ start_rot_matrix
    transformed_rot_vec = R.from_matrix(transformed_matrix).as_rotvec()
    
    # Create pose update (reuse pre-allocated array)
    updated_pose[:3] = new_pos
    updated_pose[3:] = transformed_rot_vec
    
    # Update robot pose with improved error handling
    try:
        interface.update_desired_ee_pose(updated_pose)
    except Exception as e:
        if loop_counter % print_interval == 0:
            print(f"Communication error, restarting impedance control: {e}")
        interface.start_cartesian_impedance(Kx=config.CARTESIAN_KX, Kxd=config.CARTESIAN_KXD)
    
    # Handle button press with debouncing
    if ar_data["button"] is True:
        if last_episode_stop is None or time.time() - last_episode_stop > config.BUTTON_DEBOUNCE_TIME:
            try:
                response = requests.post(config.TRIGGER_ENDPOINT, timeout=config.TRIGGER_TIMEOUT)
                last_episode_stop = time.time()
            except requests.RequestException:
                pass  # Ignore network errors for button press
    
    # Optional: Print debug info occasionally (not every loop)
    if loop_counter % print_interval == 0 and loop_counter > 0:
        pos_str = f"[{new_pos[0]:.{config.LOG_POSITION_PRECISION}f}, {new_pos[1]:.{config.LOG_POSITION_PRECISION}f}, {new_pos[2]:.{config.LOG_POSITION_PRECISION}f}]"
        print(f"Control loop active, pos: {pos_str}")
    
    loop_counter += 1
    rate.sleep()