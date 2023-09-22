import roboticstoolbox as rtb
import spatialmath.base
from spatialmath import SE3
from ir_support.robots.DHRobot3D import DHRobot3D
from math import pi
import os


class Prop(DHRobot3D):
    """Oh look, it's a prop.
       It's there, doing its thing. Don't overthink it."""
    # Todo: Known issue where file paths are not formatted properly on linux

    def __init__(self, filename, env, position=None, transform=None, color=None):
        links = [rtb.PrismaticDH(theta=0, a=0, alpha=pi/2, qlim=[0, 0])]  # No links needed, but len 0 is forbidden

        # Default to a brick red
        color = (133, 34, 48) if color is None else color
        norm_color = [c/255 for c in color]

        link_files = dict(link0=filename, color0=norm_color,
                          link1=filename, color1=norm_color)  # Link 1 because 0 link robots are forbidden.

        # A joint config and the 3D object transforms to match that config
        qtest = [0]
        qtest_transforms = [spatialmath.base.transl(0, 0, 0), spatialmath.base.transl(0, 0, 0)]

        current_path = os.path.abspath(os.path.dirname(__file__))
        super().__init__(links, link_files, name=filename, link3d_dir=current_path, qtest=qtest,
                         qtest_transforms=qtest_transforms)
        if position is not None:
            # Smart unit changes, since deg < 2pi is unlikely
            self.base = SE3(position[:3]) * (SE3.Rz(position[3]) if position[3] < 2 * pi else
                                             SE3.Rz(position[3], unit='deg'))

        if transform is not None:
            self.base = transform @ self.base
        self.q = qtest

        self.add_to_env(env)

    def update_transform(self, transform):
        self.base = transform
        self.q = [0]  # Need to poke q, or the transform update doesn't happen
