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
        link_names = dict(link0='gantry_base',  # color option only takes effect for stl file
                          link1='Gantry_Slider1',
                          link2='Gantry_Slider2',
                          link3='Gantry_Slider3')

        # A joint config and the 3D object transforms to match that config
        qtest = [0, 0, 0]
        qtest_transforms = [spb.transl(0, 0, 0),
                            spb.trotx(-pi / 2),
                            spb.transl(0, 0, 0) @ spb.rpy2tr(0, 0, 0, order='xyz'),
                            spb.transl(0, 0, 0) @ spb.rpy2tr(0, 0, 0, order='xyz')]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link_names, name='GantryBot', link3d_dir=current_path, qtest=qtest,
                         qtest_transforms=qtest_transforms)
        self.base = self.base * SE3.Rx(pi / 2) * SE3.Ry(pi / 2)
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
        links = [rtb.PrismaticDH(theta=0, a=0, alpha=pi / 2, qlim=[1, 0]),
                 rtb.PrismaticDH(theta = 0, a = 0, alpha = pi/2, qlim = [1, 0]),
                 rtb.PrismaticDH(theta = 0, a = 0, alpha = pi/2, qlim = [1, 0])]  # Prismatic Link
        return links
