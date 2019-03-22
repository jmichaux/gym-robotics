import os
import numpy as np

class Manipulator():
    def __init__(self, urdf):
        self.urdf = urdf

    def initial_setup(self):
        """
        Initial setup
        """
        raise NotImplementedError()

    def reset(self):
        """
        Reset
        """
        raise NotImplementedError()

    def create_arm(self):
        """
        Create arm
        """
        raise NotImplementedError()

    def set_joint_positions(self):
        """
        Set joint positions
        """
        raise NotImplementedError()

    def set_joint_velocities(self):
        """
        Set joint velocities
        """
        raise NotImplementedError()

    def set_joint_torques(self):
        """
        Set joint torques
        """
        raise NotImplementedError()

    def move_ee(self, pose):
        """
        Move end effector to desired pose
        """
        raise NotImplementedError()

    def get_ee_pose(self):
        """
        Get current end effector pose
        """
        raise NotImplementedError()

    def get_ee_position(self):
        """
        Get current end effector position
        """
        raise NotImplementedError()

    def get_ee_orientation(self):
        """
        Get current end effector orientation
        """
        raise NotImplementedError()

    def get_joint_positions(self):
        """
        Get current joint positions
        """
        raise NotImplementedError()

    def get_joint_velocities(self):
        """
        Get current joint velocities
        """        
        raise NotImplementedError()

    def get_joint_torques(self):
        """
        Get current joint torques
        """        
        raise NotImplementedError()

    def forward_kinematics(self):
        """
        Calculate forward kinematics
        """
        raise NotImplementedError()

    def inverse_kinematics(self):
        """
        Calculate inverse kinematics
        """
        raise NotImplementedError()

    def sample_pose(self):
        """
        Sample random end end-effector pose
        """
        raise NotImplementedError()

    def sample_joint_positions(self):
        """
        Sample random joint positions
        """
        raise NotImplementedError()