import netifaces as ni
import zerorpc
import scipy.spatial.transform as st
import numpy as np
import torch
from polymetis import RobotInterface, GripperInterface

class FrankaInterface:
    def __init__(self):
        self.robot = RobotInterface('localhost')
        self.gripper = GripperInterface('localhost')

    def get_ee_pose(self):
        data = self.robot.get_ee_pose()
        pos = data[0].numpy()
        quat_xyzw = data[1].numpy()
        rot_vec = st.Rotation.from_quat(quat_xyzw).as_rotvec()
        return np.concatenate([pos, rot_vec]).tolist()
    
    def get_joint_positions(self):
        return self.robot.get_joint_positions().numpy().tolist()
    
    def get_joint_velocities(self):
        return self.robot.get_joint_velocities().numpy().tolist()
    
    def move_to_joint_positions(self, positions, time_to_go):
        self.robot.move_to_joint_positions(
            positions=torch.Tensor(positions),
            time_to_go=time_to_go
        )
    
    def start_cartesian_impedance(self, Kx, Kxd):
        self.robot.start_cartesian_impedance(
            Kx=torch.Tensor(Kx),
            Kxd=torch.Tensor(Kxd)
        )

    def update_desired_ee_pose(self, pose):
        pose = np.asarray(pose)
        self.robot.update_desired_ee_pose(
            position=torch.Tensor(pose[:3]),
            orientation=torch.Tensor(st.Rotation.from_rotvec(pose[3:]).as_quat())
        )

    def terminate_current_policy(self):
        self.robot.terminate_current_policy()

    def get_gripper_width(self):
        return self.gripper.get_state().width
    
    def set_gripper_width(self, width):
        # Optimized gripper control - using goto instead of grasp for better performance
        self.gripper.goto(width=width, speed=0.3, force=10, blocking=False)

    def get_joint_angles(self):
        """Alias for get_joint_positions for backward compatibility"""
        return self.get_joint_positions()


# Display network interfaces for debugging
def show_network_interfaces():
    interfaces = ni.interfaces()
    print("Available network interfaces:")
    for iface in interfaces:
        try:
            addr = ni.ifaddresses(iface)[ni.AF_INET][0]['addr']
            print(f"  Interface {iface}: IP Address {addr}")
        except KeyError:
            # Skip interfaces without an IPv4 address
            pass

if __name__ == "__main__":
    show_network_interfaces()
    
    # Start the ZeroRPC server
    print("Starting Franka interface server on port 4242...")
    s = zerorpc.Server(FrankaInterface())
    s.bind("tcp://0.0.0.0:4242")
    s.run()