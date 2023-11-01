import json
import time

import swift
from math import pi

from ir_support import UR5
from spatialmath import SE3

from robotController import RobotController
from pathplanner import produce_path_plan, PathPlan
from pathplanner import read_scene
from props import Prop


# Note: PEP8 recommends snake_case as opposed to the CamelCase recommended in the code standard for most variable names.
# Deferring to PEP8 for naming


# Set up second ur5 (temp)
# Prove simultaneous control
#   Pick and place script
#   Reload script
#   8 different movement scripts, 1 for each relocation
# Seperate playback and robot controller
# Migrate ui
# Integrate ui

def create_sim_env(env, master_transform=None):
    """
    Create simulation environment.

    :param env: Swift simulation environment
    :param master_transform: Master transform for the environment
    :return: List of props in the simulation
    """

    master_transform = SE3() if master_transform is None else master_transform

    table_offset = SE3(0, 0, 0.65)  # Height of table
    gate_len = 1.215  # Length of gate

    props = [
        # Prop('objects\\Estop', env, transform=master_transform * table_offset * SE3(-1, 0.65, 2e-3), color=(100, 0, 0)),
        Prop('objects\\Pallet_table', env, transform=master_transform * table_offset * SE3(0, -0.25, 0),
             color=(99, 71, 32)),
        Prop('objects\\extinguisher', env, transform=master_transform * SE3(-0.7, 1.35, 0), color=(102, 15, 13)),
        Prop('objects\\store', env, transform=master_transform * SE3(0, 0, 0.45) * SE3.Rz(pi / 2) * SE3(0, 0.8, 0),
             color=(80, 60, 15)),
        Prop('objects\\printer', env, transform=master_transform * table_offset * SE3(0, -0.95, 0) * SE3.Rz(pi),
             color=(200, 100, 10))
    ]

    # Use XYZRz encoded position
    gate_locations = [(-1.8, 0, 0, 0), (-1.8, -gate_len, 0, 0), (1.8, 0, 0, 0), (1.8, -gate_len, 0, 0),
                      (1.8 - gate_len, -gate_len, 0, 90), (1.8, -gate_len, 0, 90),
                      (1.8 - 2 * gate_len, -gate_len, 0, 90),
                      (1.8, gate_len, 0, 90), (1.8 - 2 * gate_len, gate_len, 0, 90)]

    gate_locations += [(*g[:2], 0.6, g[3]) for g in gate_locations]  # Add a second layer of gates

    # props.append(Prop('objects\\Estop', env, color=(100, 0, 0), transform=master_transform * table_offset *
    #                                                                      SE3(1.82 - gate_len, gate_len - 0.02, 0.561)))

    for gate in gate_locations:
        props.append(Prop('objects\\w2h4_fence', env, transform=master_transform, position=gate, color=(50, 50, 50)))

    return props


def get_reposition_table():
    origin = SE3(-0.5, 0, 0.5)
    start_pos = SE3(-0.56, -0.4, 0.24)
    rot_correct = SE3.Ry(-90, unit="deg") * SE3.Rz(-90, unit="deg")
    move_out_offset = SE3(0.3, 0, 0)
    move_in_offset = SE3(-0.3, 0, 0)
    targets = [SE3(-0.56, -0.15, 0.24), SE3(-0.56, 0.15, 0.24), SE3(-0.56, 0.4, 0.24),
               SE3(-0.56, -0.4, 0.5), SE3(-0.56, -0.15, 0.5), SE3(-0.56, 0.15, 0.5), SE3(-0.56, 0.4, 0.5)]

    paths = []
    for t in targets:
        path = PathPlan()
        pos = SE3()

        pos *= origin
        path.add_path(pos * rot_correct, "rpd")
        pos = start_pos
        path.add_path(pos * rot_correct, "rpd")
        pos *= move_in_offset
        path.add_path(pos * rot_correct, "m")
        path.add_path(action="grb", obj_id=0)
        pos *= move_out_offset
        path.add_path(pos * rot_correct, "m")

        pos = t
        path.add_path(pos * rot_correct, "rpd")
        pos *= move_in_offset
        path.add_path(pos * rot_correct, "m")
        path.add_path(action="rel", obj_id=0)
        pos *= move_out_offset
        path.add_path(pos * rot_correct, "m")
        pos = origin
        path.add_path(pos * rot_correct, "rpd")

        paths.append(path)

    return paths, [None for _ in range(7)]


def get_load_path():
    origin = SE3(-0.5, 0, 0.5)
    start_pos = SE3(0, -0.52, 0.1)
    end_pos = SE3(-0.56, -0.4, 0.24)
    rot_start = SE3.Ry(90, unit="deg") * SE3.Rx(90, unit="deg") * SE3.Rz(90, unit="deg")
    rot_end = SE3.Ry(-90, unit="deg") * SE3.Rz(-90, unit="deg")


    path = PathPlan(SE3(-0.5, 0, 0.5))
    pos = SE3()

    pos *= origin
    path.add_path(pos * rot_start, "rpd")
    pos = start_pos
    path.add_path(pos * rot_start, "rpd")
    pos *= SE3(0, -0.3, 0)
    path.add_path(pos * rot_start, "m")
    path.add_path(action="grb", obj_id=0)
    pos *= SE3(0, 0.3, 0)
    path.add_path(pos * rot_start, "m")

    pos = end_pos
    path.add_path(pos * rot_end, "rpd")
    pos *= SE3(-0.3, 0, 0)
    path.add_path(pos * rot_end, "m")
    path.add_path(action="rel", obj_id=0)
    pos *= SE3(0.3, 0, 0)
    path.add_path(pos * rot_end, "m")
    pos = origin
    path.add_path(pos * rot_end, "rpd")

    return path


def run_robot_prog(robot, env, program, items, tool_offset, item_id):
    held_id = None
    robot.run(program)
    while True:
        running = True
        # running, action = traj_planner.playback_bake('bakes\\true_bake')
        action = robot.simulation_step()  # Perform simulation step
        # traj_planner2.simulation_step()  # Perform simulation step

        if not running:
            break

        if 'grip' in action:
            held_id = item_id

        if held_id is not None:  # Move brick if being held
            end_effector_transform = robot.get_end_effector_transform()
            items[held_id].update_transform(
                end_effector_transform * tool_offset * SE3(0, 0, 0) * SE3.Rx(-90, unit="deg"))

        if 'release' in action:
            held_id = None

        env.step(0.01)
        # env.step(0)

        if action['stop']:
            break


def full_scene_sim(scene_file='altscene.json'):
    # produce_path_plan(scene_file, show_matching=False, show_path=False, save='altplan.json')  # Optionally update path plan before starting

    env = swift.Swift()
    env.launch(realtime=True)

    scene_offset = SE3(0, 0, 0) * SE3.Rz(0, unit='deg')  # Master transform to move the entire robot + room setup
    create_sim_env(env, scene_offset)

    far_far_away = SE3(1000, 0, 0)  # Very far away
    printer_spawn = scene_offset * SE3(0, -0.15, 0.65)

    robot_1_base = scene_offset * SE3(0, 0, 0.65)

    pos_table, fill_table = get_reposition_table()
    load_path = get_load_path()

    null_path = PathPlan(SE3(-0.5, 0, 0.5))
    traj_planner = RobotController(load_path, robot=UR5, swift_env=env, transform=robot_1_base)
    traj_planner_2 = RobotController(null_path, robot=UR5, swift_env=env, transform=scene_offset * SE3(0, 0, 0.65))
    # Tool offset needed for brick manipulation
    tool_offset = traj_planner.tool_offset

    # Place bricks and scene
    items = [Prop('objects\\plate2', env, position, transform=far_far_away, color=(5, 5, 5)) for
              position in
              read_scene(scene_file)[0]]

    # Run call loop to robot controller

    #time.sleep(10)

    item_id = 0
    for item in pos_table:

        run_robot_prog(traj_planner, env, load_path, items, tool_offset, item_id)
        run_robot_prog(traj_planner_2, env, item, items, tool_offset, item_id)
        item_id += 1

    # traj_planner.prove_move(SE3(-0.6, 0, 0.3) * SE3.Rx(180, unit='deg'))  # Rx to flip the end effector facing down

    env.hold()


if __name__ == '__main__':
    full_scene_sim()
    # full_scene_sim('extrabrick.json')
