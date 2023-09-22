import os
from math import pi

import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3

from ir_support.robots.DHRobot3D import DHRobot3D


class Gripper(DHRobot3D):
    """Gripper Class holds two finger classes and manages both"""

    def __init__(self, env):
        self.left_finger = GripperFinger()
        self.right_finger = GripperFinger()
        self.left_finger.add_to_env(env)
        self.right_finger.add_to_env(env)

        self.open = [-0.4 * pi, 0.4 * pi]
        self.close = [-0.2 * pi, 0.2 * pi]
        self.finger_offset = SE3(-50e-3, 0, 60e-3)  # Offset of the first finger joint

        # DH links
        links = [rtb.RevoluteDH(d=0, a=0, alpha=0, qlim=[0, 0])]  # Need to have one link, this should never move

        # Names of the robot link files in the directory
        link_names = dict(link0='Base', color0=(0.2, 0.2, 0.2, 1),
                          link1='Base', color1=(0.2, 0.2, 0.2, 1))  # Duplicate

        qtest = [0]
        qtest_transforms = [spb.transl(0, 0, 0), spb.transl(0, 0, 0)]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link_names, name='Gripper', link3d_dir=current_path, qtest=qtest,
                         qtest_transforms=qtest_transforms)

        self.q = qtest
        self.q_last = self.open

    def setq(self, q):
        """Update finger positions and rotation"""
        if q is None:
            q = self.q_last
        self.left_finger.base = self.base * self.finger_offset * SE3.Ry(q[0]) * SE3.Rx(-pi / 2)
        self.left_finger.q = 0  # q[1]  # Poke q values to fix bug

        # Build gripper as radially symmetric
        self.right_finger.base = self.base * SE3.Rz(pi) * self.finger_offset * SE3.Ry(q[0]) * SE3.Rx(-pi / 2)
        self.right_finger.q = 0  # q[1]

        self.q = self.q  # Fix issue described by Quang regarding overloading
        self.q_last = q


class GripperFinger(DHRobot3D):
    """Gripper Finger"""
    def __init__(self):
        # DH links
        links = [rtb.RevoluteDH(d=0, a=5, alpha=0, qlim=[-pi, pi])]

        # Names of the robot link files in the directory
        link_names = dict(link0='Larm', color0=(0.1, 0.1, 0.1, 1),
                          link1='Ltool', color1=(0.1, 0.1, 0.1, 1))

        # A joint config and the 3D object transforms to match that config
        qtest = [0]
        qtest_transforms = [spb.transl(50e-3, 0, 0) @ spb.trotx(pi / 2),
                            spb.transl(30e-3, -35e-3, 0) @ spb.trotx(pi / 2)]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link_names, name='GripperHalf', link3d_dir=current_path, qtest=qtest,
                         qtest_transforms=qtest_transforms)
        self.q = qtest
