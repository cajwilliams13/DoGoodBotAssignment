import os
from math import pi
import swift

import roboticstoolbox as rtb
import spatialmath.base as spb

from ir_support.robots.DHRobot3D import DHRobot3D


class Gripper2(DHRobot3D):
    def __init__(self):
        # DH links
        links = self._create_DH()

        # Names of the robot link files in the directory
        link_names = dict(link0='GripperBase', color0=(0.2, 0.2, 0.2, 1),  # color option only takes effect for stl file
                          link1='GripperPiece2', color1=(0.2, 0.2, 0.2, 1))

        # A joint config and the 3D object transforms to match that config
        qtest = [0]
        qtest_transforms = [spb.transl(0, 0.1, 0) @ spb.rpy2tr(-pi / 2, 0, -pi / 2, order='xyz'),
                            spb.transl(0, 0.1, 0) @ spb.rpy2tr(-pi / 2, 0, -pi / 2, order='xyz')]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link_names, name='GantryBot', link3d_dir=current_path, qtest=qtest,
                         qtest_transforms=qtest_transforms)
        self.q = qtest

        self.q = [0]  # Known safe start values
        # self.links[0].qlim = [0, 0.03]
        self.open = [0.02]
        self.close = [0]
        self.qlast = self.q

    def setq(self, q):
        if q is None:
            q = self.qlast
        self.qlast = self.q
        self.base = self.base
        self.q = q

    @staticmethod
    def _create_DH():
        """
        Create robot's standard DH model
        """
        links = [rtb.PrismaticDH(theta=0, a=0, alpha=pi / 2, qlim=[0, 0.03])]  # Prismatic Link
        return links

    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime=True)
        self.q = self._qtest
        self.add_to_env(env)
        q_goal = [0.03]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        # fig = self.plot(self.q, limits= [-1,1,-1,1,-1,1])
        # fig._add_teach_panel(self, self.q)
        for q in qtraj:
            self.q = q
            env.step(0.02)
            # fig.step(0.01)
        # fig.hold()
