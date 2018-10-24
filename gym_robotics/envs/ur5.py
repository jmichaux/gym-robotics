import numpy as np

from manipulator import Manipulator

class UR5(Manipulator):
    """
    Interface for controlling UR5 manipulator in 
    simulation and the real world
    """
    def __init__(self, backend='pybullet'):
        self.backend = backend

        if self.backend == 'pybullet':
            import pybullet as p
        elif self.backend == 'mujoco':
            # import mujoco bindings
        elif self.backend == 'real':
            pass
            

    def initial_setup(self, backend):
        """
        Initial setup
        """
        return

    def reset(self):
        """
        Reset
        """
        return

    def create_arm(self):
        """
        Create arm
        """
        return

    def set_joint_positions(self):
        """
        Set joint positions
        """
        return

    def set_joint_velocities(self):
        """
        Set joint velocities
        """
        return

    def set_joint_torques(self):
        """
        Set joint torques
        """
        return

    def move_ee(self, pose):
        """
        Move end effector to desired pose
        """
        return

    def get_ee_pose(self):
        """
        Get current end effector pose
        """
        return

    def get_ee_position(self):
        """
        Get current end effector position
        """
        return

    def get_ee_orientation(self):
        """
        Get current end effector orientation
        """
        return

    def get_joint_positions(self):
        """
        Get current joint positions
        """
        return

    def get_joint_velocities(self):
        """
        Get current joint velocities
        """        
        return

    def get_joint_torques(self):
        """
        Get current joint torques
        """        
        return

    def forward_kinematics(self):
        """
        Calculate forward kinematics
        """
        return

    def inverse_kinematics(self):
        """
        Calculate inverse kinematics
        """
        return

    def sample_pose(self):
        """
        Sample random end end-effector pose
        """
        return

    def sample_joint_positions(self):
        """
        Sample random joint positions
        """
        return