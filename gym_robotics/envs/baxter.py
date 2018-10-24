import numpy as np

from manipulator import Manipulator
from utils.urdf import *
from utils.transforms import *
from utils.kinematics import Kinematics

class Baxter(Manipulator):
    """
    Interface for controlling Baxter manipulator in 
    simulation and the real world
    """
    def __init__(self, 
                 backend='pybullet',
                 config=None):
        self.urdf_file = 'assets/baxter/baxter_description/urdf/baxter.urdf'
        self.config = config
        self.backend = backend

        if self.backend == 'pybullet':
            import pybullet as p
        elif self.backend == 'mujoco':
            # import mujoco bindings
        elif self.backend == 'real':
            pass
        self.initial_setup()
            

    def initial_setup(self):
        """
        Initial setup
        """
        if self.backend == 'pybullet':
            objects = [p.loadURDF(self.urdf, useFixedBase=True)]
            self.baxter_id = objects[0]
        elif self.backend == 'mujoco':
            pass
        elif self.backend == 'real':
            pass
        return

    def reset(self):
        """
        Reset
        """
        if self.backend == 'pybullet':
            pass
        elif self.backed == 'mujoco':
            pass
        elif self.backend == 'real':
            pass
        return

    def create_arms(self):
        """
        Create arm
        """
        # create arm objects
        if self.backend == 'pybullet':
            pass
        elif self.backed == 'mujoco':
            pass
        elif self.backend == 'real':
            pass
        # add kinematics
        self.left_arm.kin = Kinematics(self.urdf, 'torso_t0', 'left_endpoint')
        self.right_arm.kin = Kinematics(self.urdf, 'torso_t0', 'right_endpoint')
        return

    def set_joint_positions(self):
        """
        Set joint positions
        """
        if self.backend == 'pybullet':
            pass
        elif self.backed == 'mujoco':
            pass
        elif self.backend == 'real':
            pass        
        return

    def set_joint_velocities(self):
        """
        Set joint velocities
        """
        if self.backend == 'pybullet':
            pass
        elif self.backed == 'mujoco':
            pass
        elif self.backend == 'real':
            pass        
        return

    def set_joint_torques(self):
        """
        Set joint torques
        """
        if self.backend == 'pybullet':
            pass
        elif self.backed == 'mujoco':
            pass
        elif self.backend == 'real':
            pass        
        return

    def move_ee(self, arm, pose):
        """
        Move end effector to desired pose

        Args:
            arm (str): 'left' or 'right'
            pose (list): List specifying end effector pose
        """
        joint_positions = self.inverse_kinematics(arm, pose)
        if self.backend == 'pybullet':
            pass
        elif self.backed == 'mujoco':
            pass
        elif self.backend == 'real':
            pass        
        return

    def get_ee_pose(self):
        """
        Get current end effector pose
        """
        if self.backend == 'pybullet':
            pass
        elif self.backed == 'mujoco':
            pass
        elif self.backend == 'real':
            pass        
        return

    def get_ee_position(self):
        """
        Get current end effector position
        """
        if self.backend == 'pybullet':
            pass
        elif self.backed == 'mujoco':
            pass
        elif self.backend == 'real':
            pass        
        return

    def get_ee_orientation(self):
        """
        Get current end effector orientation
        """
        if self.backend == 'pybullet':
            pass
        elif self.backed == 'mujoco':
            pass
        elif self.backend == 'real':
            pass        
        return

    def get_joint_positions(self):
        """
        Get current joint positions
        """
        if self.backend == 'pybullet':
            pass
        elif self.backed == 'mujoco':
            pass
        elif self.backend == 'real':
            pass        
        return

    def get_joint_velocities(self):
        """
        Get current joint velocities
        """
        if self.backend == 'pybullet':
            pass
        elif self.backed == 'mujoco':
            pass
        elif self.backend == 'real':
            pass                
        return

    def get_joint_torques(self):
        """
        Get current joint torques
        """
        if self.backend == 'pybullet':
            pass
        elif self.backed == 'mujoco':
            pass
        elif self.backend == 'real':
            pass                
        return

    def forward_kinematics(self, arm, joint_positions):
        """
        Calculate forward kinematics

        Args:
            arm (str): 'left' or 'right'
            joint_positions (list or ndarray): Joint positions

        Returns:
            pose (list): Pose of the end effector
        """
        if arm == 'left':
            pose = self.left_arm.kin.forward_kinematics(joint_positions)
        elif arm == 'right':
            pose = self.right_arm.kin.forward_kinematics(joint_positions)
        else:
            raise ValueError("Must specify arm")
        return pose

    def inverse_kinematics(self, arm, pose):
        """
        Calculate inverse kinematics

        Args:
            arm (str): 'left' or 'right'
            pose (list): Pose of the end effector

        Returns:
            joint_positions (list or ndarray): Joint positions
        """
        if arm == 'left':
            joint_positions = self.left_arm.kin.inverse_kinematics(joint_positions)
        elif arm == 'right':
            joint_positions = self.right_arm.kin.inverse_kinematics(joint_positions)
        else:
            raise ValueError("Must specify arm")        
        return joint_positions

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