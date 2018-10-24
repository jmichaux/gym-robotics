import xml.etree.ElementTree as ET
from transforms import *

class Joint():
    def __init__(self, joint):
        self.joint = joint
        self.name = joint.attrib["name"]
        self.type = joint.attrib["type"]

    def get_rpy(self):
        """
        Get roll, pitch, yaw angles.
        """
        rpy = self.joint.find("origin").attrib["rpy"].split()
        return float(rpy[0]), float(rpy[1]), float(rpy[2])

    def get_xyz(self):
        """
        Get X, Y, and Z offsets
        """
        xyz = self.joint.find("origin").attrib["xyz"].split()
        return float(xyz[0]), float(xyz[1]), float(xyz[2])

    def get_axis(self):
        """
        Get axis of rotation
        """
        if self.type == "fixed":
            return None
        axis = self.joint.find("axis").attrib["xyz"].split()
        return float(axis[0]), float(axis[1]), float(axis[2])

    def transform(self, theta=0):
        """
        Return Transformation matrix
        """
        roll, pitch, yaw = self.get_rpy()
        x, y, z = self.get_xyz()
        H1 = homog_transform(x, y, z, roll, pitch, yaw)
        if self.type == 'revolute':
            ax = self.get_axis()
            H2 = homog_transform(0., 0., 0., ax[0]*theta, ax[1]*theta, ax[2]*theta)
            return H1.dot(H2)
        else:
            return H1

    def get_dh_params(self):
        raise NotImplentedError()

class URDFChain():
    def __init__(self, urdf, first_joint, last_joint):
        self.root = ET.parse(urdf).getroot()
        self._all_joints = self.create_chain(first_joint, last_joint)
        self.joints = [joint for joint in self._all_joints if joint.type != "fixed"]
        self.n_active = len(self.joints)
        self.n_total = len(self._all_joints)

    def get_all_links(self, first_link_name=None):
        """
        Return all links
        """
        if first_link_name is None:
            return self.root.findall("link")
        else:
            for link in self.root.iter("link"):
                if link.attrib["name"] == first_link_name:
                    return link
        return

    def get_link(self, link_name):
        """
        Return specified link
        """
        for link in self.root.iter("link"):
            if link.attrib["name"] == link_name:
                return link

    def get_joint_names(self, joint_type=None):
        """
        Return list of joint names.
        """
        joints = []
        for joint in self.root.iter("joint"):
            if joint_type is None:
                joints.append(joint.attrib["name"])
            else:
                if joint.attrib["type"] == joint_type:
                    joints.append(joint.attrib["name"])
        return joints

    def get_all_joints(self):
        """
        Return all joints
        """
        return self.root.findall("joint")

    def get_joint(self, joint_name):
        for joint in self.root.iter("joint"):
            if joint.attrib["name"] == joint_name:
                return joint
        return

    def get_previous_joint(self, joint):
        parent_link_name = joint.find("parent").attrib["link"]
        for joint in self.root.iter("joint"):
            try:
                if joint.find("child").attrib["link"] == parent_link_name:
                    return joint
            except:
                pass
        return

    def create_chain(self, first_joint_name, last_joint_name):
        last_joint = self.get_joint(last_joint_name)
        has_prev = True
        chain = [last_joint]
        while has_prev:
            prev_joint = self.get_previous_joint(chain[0])
            if prev_joint is None:
                has_prev = False
            else:
                chain = [prev_joint] + chain
        return [Joint(joint) for joint in chain]

