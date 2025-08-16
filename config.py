"""
Configuration file for Franka AR application.
Centralizes all tunable parameters for easy optimization.
"""

import numpy as np

# Network Configuration
SERVER_IP = '127.0.0.1'
SERVER_PORT = 4242
AR_CONNECTOR_PORT = 8888

# Control Loop Configuration
CONTROL_FREQUENCY = 200  # Hz - Reduced from 1000 for better performance
AR_DEBUG = False
POSITION_SCALE_FACTOR = 0.8

# Robot Safety Bounds
MIN_Z_HEIGHT = 0.3
MAX_Z_HEIGHT = 0.9
MIN_Y_HEIGHT = -0.5
MAX_Y_HEIGHT = 0.38
FIXED_X_POSITION = 0.29

# Cartesian Impedance Controller Gains
# Format: [x, y, z, rx, ry, rz]
BASE_KX_GAINS = np.array([750.0, 750.0, 750.0, 15.0, 15.0, 15.0])
BASE_KXD_GAINS = np.array([37.0, 37.0, 37.0, 2.0, 2.0, 2.0])

# Gain scaling factors for tuning
KX_SCALE = 0.4
KXD_SCALE = 0.5

# Pre-computed gains (avoiding repeated calculations)
CARTESIAN_KX = BASE_KX_GAINS * KX_SCALE
CARTESIAN_KXD = BASE_KXD_GAINS * KXD_SCALE

# Pre-allocated matrices for efficiency (avoid repeated np.identity calls)
IDENTITY_4x4 = np.identity(4)
ZERO_VECTOR_3 = np.zeros(3)
ZERO_VECTOR_6 = np.zeros(6)

# Button/Trigger Configuration
BUTTON_DEBOUNCE_TIME = 5.0  # seconds
TRIGGER_ENDPOINT = "http://localhost:5000/trigger"
TRIGGER_TIMEOUT = 0.1  # seconds

# Communication Configuration
CONNECTION_TIMEOUT = 5.0  # seconds
HEARTBEAT_TIMEOUT = 20    # seconds
MAX_RETRIES = 2

# Gripper Configuration
GRIPPER_SPEED = 0.3
GRIPPER_FORCE = 10
GRIPPER_OPEN_WIDTH = 0.085
GRIPPER_CLOSE_WIDTH = 0.055

# Logging Configuration
PRINT_INTERVAL_MULTIPLIER = 0.5  # Print every 0.5 seconds
LOG_POSITION_PRECISION = 3       # Decimal places for position logging

# Robot Front Configuration
INFRONT_OF_ROBOT = False  # Set to True if phone is in front of robot

# C++ Configuration (for random_points.cpp)
# These would be passed as compile-time constants or config file
CPP_MAX_JOINT_VELOCITY = 2.0      # rad/s - Increased from 1.5
CPP_SETTLING_TIME_MS = 100        # ms - Reduced from 300

# Test Configuration
TEST_CONTROL_FREQUENCY = 200      # Hz
TEST_TRANSITION_TIME = 4.0        # seconds per movement
TEST_FIXED_X_POSITION = 0.33