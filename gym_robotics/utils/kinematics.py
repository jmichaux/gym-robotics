import numpy as np
from transforms import *
from urdf import *

class Kinematics():
    def __init__(self, urdf, first_joint, last_joint, world_transform=None):
        self.chain = URDFChain(urdf, first_joint, last_joint)
        if world_transform is None:
            self.world_transform = np.eye(4)
        else:
            self.world_transform = world_transform
        return

    def get_joint_names(self):
        """
        Return joint names
        """
        joint_names = []
        for joint in self.chain.joints:
            joint_names.append(joint_names)
        return joint_names

    def get_joint_limits(self):
        """
        Return joint limits
        """
        return

    def forward_kinematics(self, joint_positions):
        """
        Return forward kinematics

        Args
            joint_positions (list, tuple, ndarray): List of joint angles

        Returns
            ee_pose (list): pose of end effector with respect to world frame
        """
        joint_positions = list(joint_positions)
        assert (len(joint_positions) == self.chain.n_active)
        H = self.world_transform
        j = 0
        for i in range(len(self.chain._all_joints)):
            if self.chain._all_joints[i].type == "revolute":
                theta = joint_positions[j]
                j += 1
            else:
                theta = 0
            H = H.dot(self.chain._all_joints[i].transform(theta))
        return H

    def inverse_kinematics(self, position, orientation=None):
        """
        Return inverse kinematics
        """
        raise NotImplentedError()