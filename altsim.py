import json
import time
from math import pi
from threading import Thread

import swift
from ir_support import UR5
from spatialmath import SE3

from e_stop.e_stop import EStop
from GantryBot.GantryBot import GantryBot
from Gripper2.Gripper2 import Gripper2
from pathplanner import PathPlan, read_scene
from props import Prop
from robotController import RobotController
from ui_v1 import run_gui_in_thread


# Arm control from ui -> Override joint values
# Blocks in the cubbies
# Printer plate spawn issue
# Arm uses bake to run

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
        Prop('objects\\TableEdited', env, transform=master_transform * table_offset * SE3(-1.5, 1.2, 0), color=(99, 71, 32)),
        #Prop('objects\\extinguisher', env, transform=master_transform * SE3(-0.7, 1.35, 0), color=(102, 15, 13)),
        Prop('objects\\StorageEdited', env, transform=master_transform * SE3(-0.8, -0.5, 0.65) * SE3.Rz(pi / 2),color=(80, 60, 15)),
        Prop('objects\\printer', env, transform=master_transform * table_offset * SE3(0.3, -0.95, 0.02) * SE3.Rz(pi),color=(0, 0, 1)),
        Prop('objects\\HolderEdited', env, is_stl=False, transform=master_transform * table_offset * SE3(0.8, -0.95, 0) * SE3.Rz(pi)),
        Prop('objects\\FloorEdited', env, is_stl=False, transform=master_transform * table_offset * SE3(3, 2.5, -1) * SE3.Rz(pi/2), color = (100, 10, 10)),
        Prop('objects\\WallsEdited', env, is_stl=False, transform=master_transform * table_offset * SE3(3, 2.5, -1) * SE3.Rz(pi/2), color = (100, 10, 10))

    ]

    # Use XYZRz encoded position
    gate_locations = [(-1.8, 0, 0, 0), (-1.8, -gate_len, 0, 0), (1.8, 0, 0, 0), (1.8, -gate_len, 0, 0),
                      (1.8 - gate_len, -gate_len, 0, 90), (1.8, -gate_len, 0, 90),
                      (1.8 - 2 * gate_len, -gate_len, 0, 90),
                      (1.8, gate_len, 0, 90), (1.8 - 2 * gate_len, gate_len, 0, 90)]

    extra_gate_locations = [(*g[:2], 0.6, g[3]) for g in gate_locations]  # Add a second layer of gates
    extra_gate_locations += [(*g[:2], 1.2, g[3]) for g in gate_locations]  # Add a third layer of gates
    gate_locations += extra_gate_locations
    # props.append(Prop('objects\\Estop', env, color=(100, 0, 0), transform=master_transform * table_offset *
    #                                                                      SE3(1.82 - gate_len, gate_len - 0.02, 0.561)))

    for gate in gate_locations:
        props.append(Prop('objects\\w2h4_fence', env, transform=master_transform, position=gate, color=(50, 50, 50)))

    return props


def get_reposition_table():
    correction = SE3(0.4, 0, 0.24)
    origin = SE3(0.1, 0, 0) * correction
    start_pos = SE3(0, 0.4, 0) * correction
    rot_correct = SE3.Ry(-90, unit="deg") * SE3.Rz(-90, unit="deg")
    move_out_offset = SE3(-0.2, 0, 0)
    move_in_offset = SE3(0.1, 0, 0)
    targets = [SE3(-0.1, 0.1, 0), SE3(-0.1, -0.1, 0), SE3(-0.1, -0.4, 0),
               SE3(-0.1, 0.4, 0.26), SE3(-0.1, 0.1, 0.26), SE3(-0.1, -0.1, 0.26), SE3(-0.1, -0.4, 0.26)]

    targets = [t * correction for t in targets]

    paths = []
    for t in targets:
        path = PathPlan()
        pos = SE3()

        pos *= origin
        path.add_path(pos * rot_correct, "m")
        pos = start_pos
        path.add_path(pos * rot_correct, "m")
        pos *= move_in_offset
        path.add_path(pos * rot_correct, "m")
        path.add_path(action="grb", obj_id=0)
        pos *= move_out_offset
        path.add_path(pos * rot_correct, "m")

        pos = t
        path.add_path(pos * rot_correct, "m")
        pos *= move_in_offset
        pos *= move_in_offset
        path.add_path(pos * rot_correct, "m")
        path.add_path(action="rel", obj_id=0)
        pos *= move_out_offset
        path.add_path(pos * rot_correct, "m")
        pos = origin
        path.add_path(pos * rot_correct, "m")

        paths.append(path)

    return paths


def get_load_path():
    origin = SE3(0, -0.5, 0.5) * SE3.Rz(pi / 2)
    start_pos = SE3(0, -0.52, 0.1)
    end_pos = SE3(-0.56, -0.4, 0.24)
    rot_start = SE3.Ry(90, unit="deg") * SE3.Rx(90, unit="deg") * SE3.Rz(90, unit="deg")
    rot_end = SE3.Ry(-90, unit="deg") * SE3.Rz(-90, unit="deg")

    path = PathPlan(SE3(-0.5, 0, 0.5))

    path.add_path([1.01747430e+00, -4.22917879e-01, -1.71539122e+00, 2.13830883e+00,
                   1.01747401e+00, 1.20284037e-06], "joint")
    path.add_path([1.01747430e+00, -4.22917879e-01, -1.71539122e+00, 2.13830883e+00,
                   1.01747401e+00, 1.20284037e-06], "joint")
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
    pos *= SE3(0.6, 0, 0)
    path.add_path(pos * rot_end, "m")
    path.add_path([1.01747430e+00, -4.22917879e-01, -1.71539122e+00, 2.13830883e+00,
                   1.01747401e+00, 1.20284037e-06], "joint")
    path.add_path([1.01747430e+00, -4.22917879e-01, -1.71539122e+00, 2.13830883e+00,
                   1.01747401e+00, 1.20284037e-06], "joint")

    return path


def full_scene_sim(scene_file='altscene.json'):
    # produce_path_plan(scene_file, show_matching=False, show_path=False, save='altplan.json')  # Optionally update path plan before starting
    env = swift.Swift()
    env.launch(realtime=True)

    scene_offset = SE3(0, 0, 0) * SE3.Rz(0, unit='deg')  # Master transform to move the entire robot + room setup
    create_sim_env(env, scene_offset)

    far_far_away = SE3(1000, 0, 0)  # Very far away
    printer_spawn = scene_offset * SE3(0, -0.15, 0.65)

    robot_1_base = scene_offset * SE3(0, 0, 0.65)

    pos_table = get_reposition_table()
    load_path = get_load_path()

    plates = ["Absent" for _ in range(8)]
    #plates[0] = "Waiting"

    null_path = PathPlan(SE3(-0.5, 0, 0.5))
    pos_table.append(null_path)
    traj_planner = RobotController(null_path, robot=UR5, swift_env=env, transform=robot_1_base)
    traj_planner_2 = RobotController(null_path, robot=GantryBot, swift_env=env,
                                     transform=scene_offset * SE3(-0.7, 0, 0.65) * SE3.Rz(180, unit="deg"),
                                     gripper=Gripper2)
    # traj_planner_2 = RobotController(null_path, robot=UR5, swift_env=env,
    #                                 transform=robot_1_base * SE3(-0.1, 0, 0))
    # Tool offset needed for brick manipulation
    tool_offset = traj_planner.tool_offset
    tool_offset2 = traj_planner_2.tool_offset

    # Place bricks and scene
    items = [Prop('objects\\plate2', env, position, transform=far_far_away, color=(5, 5, 5)) for
             position in
             read_scene(scene_file)[0]]

    # time.sleep(10)

    robot_can_move = [False]
    obstructions = [False for _ in range(8)]
    obstructors = [Prop("objects\\dot", env, transform=far_far_away) for _ in obstructions]
    obs_locations = [[-0.85, -0.15, 0.24], [-0.85, 0.15, 0.24], [-0.85, 0.4, 0.24],
                     [-0.85, -0.40, 0.5], [-0.85, -0.15, 0.5], [-0.85, 0.15, 0.5], [-0.85, 0.40, 0.5],
                     [-0.85, -0.40, 0.24]]
    obs_locations = [scene_offset * SE3(0, 0, 0.7) * SE3(*loc) for loc in obs_locations]

    gui_thread = Thread(target=run_gui_in_thread, kwargs={"r1": traj_planner, "r2": traj_planner_2,
                                                          "plates": plates, "robot_can_move": robot_can_move,
                                                          "obstructions": obstructions})
    gui_thread.start()

    held_id = None
    held_id_2 = None
    plate_in_move = False
    p1_stop = False

    estop_button = EStop(initial_pose=SE3(), use_physical_button=True)
    estop_button.add_to_env(env)

    while True:
        if estop_button.get_state():
            robot_can_move[0] = False
        for i, p in enumerate(plates):
            if p == "Absent":
                items[i].update_transform(far_far_away)
            if p == "Error":
                if robot_can_move[0]:
                    print(f"Remove errored plates: {i + 1}")
                robot_can_move[0] = False
                plate_in_move = False

            if obstructions[i]:
                if p == "Stowed":
                    obstructions[i] = False
                    print("Cannot obstruct cell with stowed plate")
                elif p == "Moving":
                    robot_can_move[0] = False
                    print("Stopping due to obstructed cell")

        for state, obj, loc in zip(obstructions, obstructors, obs_locations):
            if state:
                obj.update_transform(loc)
            else:
                obj.update_transform(far_far_away)

        if not plate_in_move and not any([p == "Moving" for p in plates]):
            for i, p in enumerate(plates):
                if p == "Waiting":
                    if obstructions[i]:
                        plates[i] = "Obstructed"
                        print("Cell is obstructed")
                    else:
                        plate_id = i
                        plate_in_move = True
                        traj_planner.simulation_step(not robot_can_move[0])
                        traj_planner.run(load_path)
                        items[i].update_transform(printer_spawn)
                        p1_stop = False
                elif p == "Obstructed":
                    if not obstructions[i]:
                        plates[i] = "Absent"
            # print(plates)

        action_1 = traj_planner.simulation_step(not robot_can_move[0])
        action_2 = traj_planner_2.simulation_step(not robot_can_move[0])

        if 'grip' in action_1:
            held_id = plate_id

        if held_id is not None:  # Move brick if being held
            end_effector_transform = traj_planner.get_end_effector_transform()
            items[held_id].update_transform(
                end_effector_transform * tool_offset * SE3(0, 0, 0) * SE3.Rx(-90, unit="deg"))

        if 'release' in action_1:
            held_id = None

        # ------------------------------------------------------------------------------------------

        if 'grip' in action_2:
            held_id_2 = plate_id

        if held_id_2 is not None:  # Move brick if being held
            end_effector_transform = traj_planner_2.get_end_effector_transform()
            items[held_id_2].update_transform(
                end_effector_transform * tool_offset2 * SE3(0, 0, 0) * SE3.Rx(-90, unit="deg"))

        if 'release' in action_2:
            held_id_2 = None

        env.step(0.01)
        # env.step(0)

        if action_1['stop'] and plate_in_move:
            if obstructions[plate_id]:
                print("Obstruction detected")
                robot_can_move[0] = False
                plates[plate_id] = "Error"
            else:
                traj_planner_2.run(pos_table[plate_id])
                plates[plate_id] = "Moving"
                p1_stop = True

        if action_2['stop'] and p1_stop:
            plate_in_move = False
            p1_stop = False
            plates[plate_id] = "Stowed"


if __name__ == '__main__':
    full_scene_sim()
    # full_scene_sim('extrabrick.json')
