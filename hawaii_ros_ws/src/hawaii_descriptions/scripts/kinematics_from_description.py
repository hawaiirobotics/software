import numpy as np
from scipy.spatial.transform import Rotation as R
from urdfpy import URDF

class KinematicsFromDescriptionTool(object):
    def __init__(self, configs):
        self.body_frame_name = configs["body_frame"]
        self.space_frame_name = configs["space_frame"]

    def _search_with_index(self, list_, key, attr):
        """
        Helper function to perform a key search through a list of objects
        and return the object along with its index.

        @param list_ - list of objects to search through
        @param key - term to search for
        @param attr - attribute to look for key in for each object
        @return (object, index) tuple for the first matching object that has an attribute matching the key
        @note returns (None, -1) if no object attribute matches the key
        """
        for index, element in enumerate(list_):
            if getattr(element, attr) == key:
                return element, index
        return None, -1

    def _populate(self):
        """
        Populates relevant variables and lists
        """
        self.links = self.robot.links
        self.joints = self.robot.joints

        # init empty lists
        self.link_list = []
        self.tf_list = []
        self.axis_list = []
        self.joint_list = []
        self.rev_joint_list = []
        self.rev_origin_list = []

        # find ee link in urdf tree
        self.body_frame, self.body_frame_index = self._search_with_index(
            self.links, self.body_frame_name, "name"
        )

        self.space_frame = self._search(
            self.links, self.space_frame_name, "name"
        )

        if self.body_frame is None or self.space_frame is None:
            raise ValueError("Body Frame or Space Frame are empty")

    def get_M(self):
        """
        Returns the homogeneous transform matrix

        @return M
        """
        return self.M

    def get_Slist(self, transposed=False):
        """
        Returns the list of joint screw axes in the world frame

        @return Slist
        """
        if transposed:
            return self.Slist
        else:
            return self.Slist.T

    def run(self):
        """
        Builds relevant lists and calculates the M and Slist matrices
        """
        self._populate()
        self._build_link_lists()
        self._calc_M()
        self._calc_Slist()

    def _calc_M(self):
        """
        Calculates the homogeneous matrix describing the pose of the end
            effector in SE(3)
        """
        # fk = forward kinematics
        fk = self.robot.link_fk()
        self.M= fk[self.robot.links[self.body_frame_index]]

    def _calc_Slist(self):
        """
        Calculates the list of screw axes
        """
        num_axes = len(self.axis_list)  # number of revolute axes defines Slist shape
        Slist = np.zeros((6, num_axes))
        for i in range(num_axes):
            w = np.array(self.axis_list[i], dtype="float64")
            q = np.array(self.rev_origin_list[i], dtype="float64")
            v = np.cross(-w, q)
            Slist[:, i] = np.hstack((w, v))
        self.Slist = Slist

    def _build_link_lists(self):
        """
        Builds the transformation and link lists

        Iterates from the specified end effector link to the base frame
        """
        self.link_list.insert(0, self.body_frame)
        self.tf_list.insert(0, self._get_parent_joint(self.body_frame).origin)
        current_link = self._get_parent_link(self.body_frame)

        # iterate backwards through link/joint tree up to base
        while current_link is not None and (current_link.name != self.space_frame.name):

            self.link_list.insert(0, current_link)

            parent_joint = self._get_parent_joint(current_link)
            self.joint_list.insert(0, parent_joint)

            #origin is the pose of child and joint frames relative to the parent link’s frame.
            current_tf_to_parent = parent_joint.origin
            self.tf_list.insert(0, current_tf_to_parent)

            # keep track of revolute joints and their axes
            if parent_joint.joint_type == "revolute":
                self.rev_joint_list.insert(0, parent_joint)
                self.axis_list.insert(0, self._get_parent_axis(current_link))

            current_link = self._get_parent_link(current_link)

        joint_pose = np.eye(4, dtype="float64")
        for i in range(len(self.joint_list)):
            joint_pose = np.dot(joint_pose, self.tf_list[i])
            if self.joint_list[i].joint_type == "revolute":
                self.rev_origin_list.append(joint_pose[:3, 3])

    def _get_parent_link(self, link):
        """
        Returns the parent link given its child

        @param link - child link to find the parent of
        @return parent link to given link
        @note returns None if there is no parent link
        """
        parent_joint = self._get_parent_joint(link)
        if parent_joint is None:
            return None
        return self._search(self.links, parent_joint.parent, "name")

    def _get_parent_joint(self, link):
        """
        Returns the parent joint given its child link

        @param link - child link to find the parent joint of
        @return parent joint to given link
        """
        joint = self._search(self.joints, link.name, "child")
        return joint

    def _get_parent_axis(self, link):
        """
        Returns the parent joint axis given its child

        @param link - child link to the find the parent joint's axis of
        @return axis about which the parent joint rotates
        """
        if self._get_parent_joint(link).joint_type != "revolute":
            return None
        else:
            axis = self._get_parent_joint(link).axis
            return axis

    def _search(self, list_, key, attr):
        """
        Helper function to perform a key search through a list of objects

        @param list_ - list of objects to search through
        @param key - term to search for
        @param attr - attribute to look for key in for each object
        @return the first matching object that has an attribute matching the key
        @note returns None if no object attribute matches the key
        """
        result = [element for element in list_ if getattr(element, attr) == key]
        if len(result) > 0:
            return result[0]
        else:
            return None

    def load_desc_from_file(self, filepath):
        self.robot = URDF.load(filepath)

if __name__ == "__main__":
    # Define your configs here
    configs = {
        "body_frame": "Gripper_Left",
        "space_frame": "base_link",
    }

    # Instantiate the tool and perform operations
    kinematics_tool = KinematicsFromDescriptionTool(configs)
    urdf_file = "../urdf/Student_Arm.urdf"
    kinematics_tool.load_desc_from_file(urdf_file)
    kinematics_tool.run()

    # Access the calculated matrices
    M = kinematics_tool.get_M()
    Slist = kinematics_tool.get_Slist()

    # Print or use the matrices as required
    # Print or use the matrices as required with commas between elements
    print("Homogeneous Transform Matrix (M):\n", '\n'.join(['[' + ', '.join(map(str, row)) + ']' for row in M]))
    print("Screw Axes List (Slist):\n", '\n'.join(['[' + ', '.join(map(str, row)) + ']' for row in Slist]))



