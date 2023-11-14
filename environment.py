from math import pi

from spatialmath import SE3

from e_stop.e_stop import EStop
from props import Prop


def create_sim_env(env, master_transform=None):
    """
    Create simulation environment.

    :param env: Swift simulation environment
    :param master_transform: Master transform for the environment
    :return: List of props in the simulation
    """

    master_transform = SE3() if master_transform is None else master_transform

    table_offset = SE3(0, 0, 0.65)  # Height of table
    props = [
        Prop('objects\\Table', env, transform=master_transform * table_offset * SE3(-1.5, 1.2, 0), color=(99, 71, 32)),
        Prop('objects\\extinguisher', env, transform=master_transform * SE3(-2, 1.35, 0), color=(102, 15, 13)),
        Prop('objects\\Storage', env, transform=master_transform * SE3(-0.5, -0.5, 0.65) * SE3.Rz(pi / 2),color=(80, 60, 15)),
        Prop('objects\\printer', env, transform=master_transform * table_offset * SE3(0.3, -0.95, 0.02) * SE3.Rz(pi),color=(0, 0, 1)),
        Prop('objects\\Floor', env, transform=master_transform * table_offset * SE3(3, 2.5, -1) * SE3.Rz(pi/2), color = (100, 10, 10)),
        Prop('objects\\Walls', env, transform=master_transform * table_offset * SE3(3, 2.5, -1) * SE3.Rz(pi/2), color = (10, 10, 100)),
        Prop('objects\\OverheadLight', env, transform=master_transform * table_offset * SE3(0.2, -0.1, 1.5), color=(200, 200, 200)),
        Prop('objects\\OverheadLight', env, transform=master_transform * table_offset * SE3(1, -0.1, 1.5), color=(200, 200, 200)),
        Prop('objects\\OverheadLight', env, transform=master_transform * table_offset * SE3(-3, -0.1, 1.5), color=(200, 200, 200)),
        Prop('objects\\WarningSign', env, transform=master_transform * table_offset * SE3(-1.53, 1.5, 0.65) * SE3.Rz(-pi/2) * SE3.Rx(pi/2), color=(200, 50, 50)),
        Prop('objects\\LightCurtain', env, transform=master_transform * table_offset * SE3(-1.53, 1.15, -0.7) * SE3.Rz(-pi / 2), color=(200, 50, 50)),
        Prop('objects\\LightCurtain', env, transform=master_transform * table_offset * SE3(-1.53, -1.15, -0.7) * SE3.Rz(-pi / 2), color=(200, 50, 50)),

    ]

    # Use XYZRz encoded position for gates
    gate_locations = [(-1.5, -2.4, 0, 0), (-1.5, 1.15, 0, 0)]

    extra_gate_locations = [(*g[:2], 0.6, g[3]) for g in gate_locations]  # Add a second layer of gates
    extra_gate_locations += [(*g[:2], 1.2, g[3]) for g in gate_locations]  # Add a third layer of gates
    gate_locations += extra_gate_locations

    for gate in gate_locations:
        props.append(Prop('objects\\w2h4_fence', env, position=gate, color=(50, 50, 50), debug=True))

    return props
