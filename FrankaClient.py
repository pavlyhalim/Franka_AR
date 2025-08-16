import zerorpc
import numpy as np
import time

class FrankaClient:
    def __init__(self, server_ip='localhost', port=4242, connection_timeout=5.0, heartbeat_timeout=20):
        self.server_ip = server_ip
        self.port = port
        self.connection_timeout = connection_timeout
        self.heartbeat_timeout = heartbeat_timeout
        self.server = None
        self._connect()

    def _connect(self):
        """Establish connection to the server with retry logic."""
        try:
            if self.server:
                self.server.close()
            
            self.server = zerorpc.Client(heartbeat=self.heartbeat_timeout, timeout=self.connection_timeout)
            self.server.connect(f"tcp://{self.server_ip}:{self.port}")
        except Exception as e:
            print(f"Failed to connect to server: {e}")
            raise

    def _safe_call(self, method_name, *args, **kwargs):
        """Safely call a server method with automatic reconnection on failure."""
        max_retries = 2
        for attempt in range(max_retries):
            try:
                method = getattr(self.server, method_name)
                return method(*args, **kwargs)
            except Exception as e:
                if attempt < max_retries - 1:
                    print(f"Connection lost, attempting to reconnect... (attempt {attempt + 1})")
                    self._connect()
                    time.sleep(0.1)
                else:
                    raise e

    def get_ee_pose(self):
        flange_pose = np.array(self._safe_call('get_ee_pose'))
        return flange_pose
    
    def get_joint_positions(self):
        return np.array(self._safe_call('get_joint_positions'))
    
    def get_joint_velocities(self):
        return np.array(self._safe_call('get_joint_velocities'))

    def move_to_joint_positions(self, positions: np.ndarray, time_to_go: float):
        self._safe_call('move_to_joint_positions', positions.tolist(), time_to_go)

    def start_cartesian_impedance(self, Kx: np.ndarray, Kxd: np.ndarray):
        self._safe_call('start_cartesian_impedance', Kx.tolist(), Kxd.tolist())
    
    def update_desired_ee_pose(self, pose: np.ndarray):
        self._safe_call('update_desired_ee_pose', pose.tolist())

    def terminate_current_policy(self):
        self._safe_call('terminate_current_policy')

    def close(self):
        if self.server:
            try:
                self.server.close()
            except:
                pass
            self.server = None

    def get_gripper_width(self):
        return self._safe_call('get_gripper_width')
    
    def set_gripper_width(self, width):
        self._safe_call('set_gripper_width', width)
    
    def __del__(self):
        self.close()