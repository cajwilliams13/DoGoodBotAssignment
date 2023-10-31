import os
from math import pi

import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3

from ir_support.robots.DHRobot3D import DHRobot3D


class GantryBot(DHRobot3D):
    def __init__(self):
        # DH links
        links = self._create_DH()

        # Names of the robot link files in the directory
        link_names = dict(link0='gantry_base', color0 = (0.2, 0.2, 0.2, 1),  # color option only takes effect for stl file
                          link1='Gantry_Slider1', color1 = (0.2, 0.2, 0.2, 1),
                          link2='Gantry_Slider2', color2 = (0.2, 0.2, 0.2, 1),
                          link3='Gantry_Slider3', color3 = (0.2, 0.2, 0.2, 1))

        # A joint config and the 3D object transforms to match that config
        qtest = [0, 0, 0]
        qtest_transforms = [spb.transl(0, 0.3, 0) @ spb.rpy2tr(0, 0, -pi/2, order = 'xyz'),
                            spb.transl(0.325, 0.09, 0) @ spb.rpy2tr(0, 0, 0, order = 'xyz'),
                            spb.transl(-0.2, 0, -0.53) @ spb.rpy2tr(0, 0, 0, order='xyz'),
                            spb.transl(-0.2, 0, -1.04) @ spb.rpy2tr(0, 0, 0, order='xyz')]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link_names, name='GantryBot', link3d_dir=current_path, qtest=qtest,
                         qtest_transforms=qtest_transforms)
        self.base = self.base * SE3.Rx(pi/2) * SE3.Ry(pi/2)
        self.q = qtest

        self.q = [0.5, 0.5, 0.5]  # Known safe start values
        self.links[0].qlim = [1, 0]
        self.links[1].qlim = [1, 0]
        self.links[2].qlim = [1, 0]

    @staticmethod
    def _create_DH():
        """
        Create robot's standard DH model
        """
        links = [rtb.PrismaticDH(theta=1, a=0, alpha=0, qlim=[1, 0]),
                 rtb.PrismaticDH(theta = 0, a = 1, alpha = 0, qlim = [1, 0]),
                 rtb.PrismaticDH(theta = 1, a = 0, alpha = pi/2, qlim = [1, 0])]  # Prismatic Link
        return links
