import os
from math import pi
import swift

import roboticstoolbox as rtb
import spatialmath.base as spb
from spatialmath import SE3

from ir_support.robots.DHRobot3D import DHRobot3D


class GantryBot(DHRobot3D):
    def __init__(self):
        # DH links
        links = self._create_DH()

        # Names of the robot link files in the directory
        link_names = dict(link0='gantry_base', color0=(0.2, 0.2, 0.2, 1),  # color option only takes effect for stl file
                          link1='Gantry_Slider1_tall_v2', color1=(0.2, 0.2, 0.2, 1),
                          link2='Gantry_Slider2', color2=(0.2, 0.2, 0.2, 1),
                          link3='Gantry_Slider3', color3=(0.2, 0.2, 0.2, 1))

        # A joint config and the 3D object transforms to match that config
        qtest = [0, 0, 0]
        qtest_transforms = [spb.transl(0, 0.3, 0) @ spb.rpy2tr(0, 0, -pi / 2, order='xyz'),
                            spb.transl(0.325, 0.1, -0.01) @ spb.rpy2tr(0, 0, -pi/2, order='xyz'),
                            spb.transl(-0.17, -0.1, -0.04) @ spb.rpy2tr(0, 0, 0, order='xyz'),
                            spb.transl(-0.17, 0, -0.04) @ spb.rpy2tr(0, 0, 0, order='xyz')]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link_names, name='GantryBot', link3d_dir=current_path, qtest=qtest,
                         qtest_transforms=qtest_transforms)
        self.base = self.base * SE3.Rx(pi / 2) * SE3.Ry(pi / 2)
        self.q = qtest

        self.q = [0, 0, 0]  # Known safe start values
        self.links[0].qlim = [0, 0.9]
        self.links[1].qlim = [-0.4, 0.4]
        self.links[2].qlim = [-0.15, 0.1]

        self.gripper_base_offset = SE3(-0.25, 0, 0.65) * SE3.Rz(-90, unit='deg') * SE3(-0.1, -0.07, 0)
        self.tool_offset = SE3(-0.21, 0, 0.65) * SE3.Rx(90, unit='deg') * SE3.Ry(90, unit='deg')

    @staticmethod
    def _create_DH():
        """
        Create robot's standard DH model
        """
        links = [rtb.PrismaticDH(theta=pi / 2, a=0, alpha=pi / 2, qlim=[0, 0.9]),
                 rtb.PrismaticDH(theta=pi / 2, a=0, alpha=pi / 2, qlim=[-0.4, 0.4]),
                 rtb.PrismaticDH(theta=0, a=0, alpha=0, qlim=[-0.15, 0.1])]  # Prismatic Link
        return links

    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime=True)
        self.q = self._qtest
        self.add_to_env(env)
        q_goal = [0.9, -0.4, -0.15]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        # fig = self.plot(self.q, limits= [-1,1,-1,1,-1,1])
        # fig._add_teach_panel(self, self.q)
        for q in qtraj:
            self.q = q
            env.step(0.02)
            # fig.step(0.01)
        # fig.hold()
