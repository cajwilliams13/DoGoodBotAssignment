import os
from math import pi

import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3

from ir_support.robots.DHRobot3D import DHRobot3D


class UR3Lin(DHRobot3D):
    def __init__(self):
        # DH links
        links = self._create_DH()

        # Names of the robot link files in the directory
        link_names = dict(link0='base_rail', color0=(0.2, 0.2, 0.2, 1),  # color option only takes effect for stl file
                          link1='slider_rail', color1=(0.1, 0.1, 0.1, 1),
                          link2='shoulder_ur3',
                          link3='upperarm_ur3',
                          link4='forearm_ur3',
                          link5='wrist1_ur3',
                          link6='wrist2_ur3',
                          link7='wrist3_ur3')

        # A joint config and the 3D object transforms to match that config
        qtest = [0, 0, -pi / 2, 0, 0, 0, 0]
        qtest_transforms = [spb.transl(0, 0, 0),
                            spb.trotx(-pi / 2),
                            spb.transl(0, 0.15712, 0) @ spb.rpy2tr(0, pi, pi / 2, order='xyz'),
                            spb.transl(0, 0.15756, 0.12) @ spb.rpy2tr(0, pi, pi / 2, order='xyz'),
                            spb.transl(0, 0.4, 0.016136) @ spb.rpy2tr(0, pi, pi / 2, order='xyz'),
                            spb.transl(0, 0.61052, 0.015451) @ spb.rpy2tr(0, -pi / 2, pi / 2, order='xyz'),
                            spb.transl(0.001116, 0.61052, 0.09618) @ spb.rpy2tr(0, -pi / 2, -pi / 2, order='xyz'),
                            spb.transl(-0.083429, 0.61052, 0.09618) @ spb.rpy2tr(0, 0, -pi / 2, order='xyz')]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link_names, name='UR3Lin', link3d_dir=current_path, qtest=qtest,
                         qtest_transforms=qtest_transforms)
        self.base = self.base * SE3.Rx(pi / 2) * SE3.Ry(pi / 2)
        self.q = qtest

    @staticmethod
    def _create_DH():
        """
        Create robot's standard DH model
        """
        links = [rtb.PrismaticDH(theta=pi, a=0, alpha=pi / 2, qlim=[-0.8, 0])]  # Prismatic Link
        a = [0, 0.24, 0.21, 0, 0, 0]
        d = [0.1599, 0.1357, 0.1197, 0.08, 0.08, 0]
        alpha = [-pi / 2, -pi, pi, -pi / 2, -pi / 2, 0]
        qlim = [[-2*pi, 2*pi] for _ in range(6)]
        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], qlim=qlim[i])
            links.append(link)
        return links
