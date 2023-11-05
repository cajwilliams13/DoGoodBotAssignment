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
from ui.ui_v1 import run_gui_in_thread


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
        Prop('objects\\Table', env, transform=master_transform * table_offset * SE3(-1.5, 1.2, 0), color=(99, 71, 32)),
        Prop('objects\\extinguisher', env, transform=master_transform * SE3(-2, 1.35, 0), color=(102, 15, 13)),
        Prop('objects\\Storage', env, transform=master_transform * SE3(-0.5, -0.5, 0.65) * SE3.Rz(pi / 2),color=(80, 60, 15)),
        Prop('objects\\printer', env, transform=master_transform * table_offset * SE3(0.3, -0.95, 0.02) * SE3.Rz(pi),color=(0, 0, 1)),
        #Prop('objects\\Holder', env, is_stl=False, transform=master_transform * table_offset * SE3(0.8, -0.95, 0) * SE3.Rz(pi)),
        Prop('objects\\Floor', env, transform=master_transform * table_offset * SE3(3, 2.5, -1) * SE3.Rz(pi/2), color = (100, 10, 10)),
        Prop('objects\\Walls', env, transform=master_transform * table_offset * SE3(3, 2.5, -1) * SE3.Rz(pi/2), color = (10, 10, 100)),
        Prop('objects\\OverheadLight', env, transform=master_transform * table_offset * SE3(0.2, -0.1, 1.5), color=(200, 200, 200)),
        Prop('objects\\OverheadLight', env, transform=master_transform * table_offset * SE3(1, -0.1, 1.5), color=(200, 200, 200)),
        Prop('objects\\OverheadLight', env, transform=master_transform * table_offset * SE3(-3, -0.1, 1.5), color=(200, 200, 200)),
        Prop('objects\\WarningSign', env, transform=master_transform * table_offset * SE3(-1.53, 1.5, 0.65) * SE3.Rz(-pi/2) * SE3.Rx(pi/2), color=(200, 50, 50)),
        Prop('objects\\LightCurtain', env, transform=master_transform * table_offset * SE3(-1.53, 1.15, -0.7) * SE3.Rz(-pi / 2), color=(200, 50, 50)),
        Prop('objects\\LightCurtain', env, transform=master_transform * table_offset * SE3(-1.53, -1.15, -0.7) * SE3.Rz(-pi / 2), color=(200, 50, 50)),

    ]

    # Use XYZRz encoded position
    gate_locations = [(-1.5, -2.4, 0, 0), (-1.5, 1.15, 0, 0)]

    extra_gate_locations = [(*g[:2], 0.6, g[3]) for g in gate_locations]  # Add a second layer of gates
    extra_gate_locations += [(*g[:2], 1.2, g[3]) for g in gate_locations]  # Add a third layer of gates
    gate_locations += extra_gate_locations
    # props.append(Prop('objects\\Estop', env, color=(100, 0, 0), transform=master_transform * table_offset *
    #                                                                      SE3(1.82 - gate_len, gate_len - 0.02, 0.561)))

    for gate in gate_locations:
        props.append(Prop('objects\\w2h4_fence', env, transform=master_transform, position=gate, color=(50, 50, 50)))

    estop_button2 = EStop(initial_pose=SE3(0, -1, 0.65))
    estop_button2.add_to_env(env)

    return props


def get_reposition_table() -> list[PathPlan]:
    """List of paths that move a plate from zone 1 to zones 2 - 8"""
    zone_drop_locations = [SE3(-0.1, 0.1, 0), SE3(-0.1, -0.2, 0), SE3(-0.1, -0.5, 0),
                           SE3(-0.1, 0.4, 0.26), SE3(-0.1, 0.1, 0.26), SE3(-0.1, -0.2, 0.26), SE3(-0.1, -0.5, 0.26)]
    # Global correction factors
    robot_correction_factor = SE3(0.1, 0, 0.24)
    rotation_correction = SE3.Ry(-90, unit="deg") * SE3.Rz(-90, unit="deg")

    # Points of interest
    origin = SE3(0.1, 0, 0) * robot_correction_factor
    start_pos = SE3(0, 0.4, 0) * robot_correction_factor

    # Pickup and drop vectors
    move_out_offset = SE3(-0.2, 0, 0)
    move_in_offset = SE3(0.1, 0, 0)

    zone_drop_locations = [t * robot_correction_factor for t in zone_drop_locations]

    paths = []
    for location in zone_drop_locations:
        path = PathPlan()
        pos = SE3()

        pos *= origin
        path.add_path(pos * rotation_correction, "m")

        # Pick up the plate
        pos = start_pos
        path.add_path(pos * rotation_correction, "m")
        pos *= move_in_offset
        path.add_path(pos * rotation_correction, "m")
        path.add_path(action="grb", obj_id=0)
        pos *= move_out_offset
        path.add_path(pos * rotation_correction, "m")

        # Deliver the plate
        pos = location
        path.add_path(pos * rotation_correction, "m")
        pos *= move_in_offset
        pos *= move_in_offset
        path.add_path(pos * rotation_correction, "m")
        path.add_path(action="rel", obj_id=0)
        pos *= move_out_offset
        path.add_path(pos * rotation_correction, "m")

        pos = origin
        path.add_path(pos * rotation_correction, "m")

        paths.append(path)

    return paths


def get_load_path() -> PathPlan:
    """Produce a path from the printer to zone 1"""
    pickup_pos = SE3(0, -0.52, 0.1)
    zone_drop_pos = SE3(-0.56, -0.4, 0.24)
    rot_start = SE3.Ry(90, unit="deg") * SE3.Rx(90, unit="deg") * SE3.Rz(90, unit="deg")
    rot_end = SE3.Ry(-90, unit="deg") * SE3.Rz(-90, unit="deg")

    path = PathPlan(SE3(-0.5, 0, 0.5))

    path.add_path([1.01747430e+00, -4.22917879e-01, -1.71539122e+00, 2.13830883e+00,
                   1.01747401e+00, 1.20284037e-06], "joint")  # Force arm to stable location before each cycle
    path.add_path([1.01747430e+00, -4.22917879e-01, -1.71539122e+00, 2.13830883e+00,
                   1.01747401e+00, 1.20284037e-06], "joint")

    # Pick up at printer
    pos = pickup_pos
    path.add_path(pos * rot_start, "rpd")
    pos *= SE3(0, -0.3, 0)
    path.add_path(pos * rot_start, "m")
    path.add_path(action="grb", obj_id=0)
    pos *= SE3(0, 0.3, 0)
    path.add_path(pos * rot_start, "m")

    # Drop off at zone 1
    pos = zone_drop_pos
    path.add_path(pos * rot_end, "rpd")
    pos *= SE3(-0.3, 0, 0)
    path.add_path(pos * rot_end, "m")
    path.add_path(action="rel", obj_id=0)
    pos *= SE3(0.6, 0, 0)
    path.add_path(pos * rot_end, "m")

    path.add_path([1.01747430e+00, -4.22917879e-01, -1.71539122e+00, 2.13830883e+00,
                   1.01747401e+00, 1.20284037e-06], "joint")  # Force arm to stable location after each cycle
    path.add_path([1.01747430e+00, -4.22917879e-01, -1.71539122e+00, 2.13830883e+00,
                   1.01747401e+00, 1.20284037e-06], "joint")

    return path


def full_scene_sim(scene_file='altscene.json'):
    env = swift.Swift()
    env.launch(realtime=True)

    scene_offset = SE3(0, 0, 0) * SE3.Rz(0, unit='deg')  # Master transform to move the entire robot + room setup

    # Objects not currently visible get sent to the land far, far away
    far_far_away = SE3(1000, 0, 0)  # Very far away

    # Prepare robot paths
    null_path = PathPlan(SE3(-0.5, 0, 0.5))
    reposition_paths = get_reposition_table()
    reposition_paths.append(null_path)  # Allows plate 8 to stay in place
    load_path = get_load_path() 

    # Key object spawn locations
    printer_spawn = scene_offset * SE3(0.3, -0.73, 0.8)
    robot_1_base = scene_offset * SE3(0.3, 0, 0.65)
    robot_2_base = scene_offset * SE3(-0.7, 0, 0.65) * SE3.Rz(180, unit="deg")

    # Spawn robots
    robot_1 = RobotController(null_path, robot=UR5, swift_env=env, transform=robot_1_base)
    robot_2 = RobotController(null_path, robot=GantryBot, swift_env=env, transform=robot_2_base, gripper=Gripper2)

    # Tool offset needed for object manipulation
    tool_offset = robot_1.tool_offset
    tool_offset2 = robot_2.tool_offset

    # Place objects and scene
    create_sim_env(env, scene_offset)
    items = [Prop('objects\\plateWithObj', env, position, transform=far_far_away, color=(0, 100, 0)) for
             position in
             read_scene(scene_file)[0]]

    # Synced variables
    robot_can_move = [False]  # These are wrapped in a single index list to force pass-by reference
    obstructions = [False for _ in range(8)]
    plates_status = ["Absent" for _ in range(8)]

    gui_thread = Thread(target=run_gui_in_thread, kwargs={"r1": robot_1, "r2": robot_2,
                                                          "plates": plates_status, "robot_can_move": robot_can_move,
                                                          "obstructions": obstructions})
    gui_thread.start()

    obstruction_objects = [Prop("objects\\dot", env, transform=far_far_away) for _ in obstructions]
    obs_locations = [[-0.55, -0.15, 0.24], [-0.55, 0.15, 0.24], [-0.55, 0.4, 0.24],
                     [-0.55, -0.40, 0.5], [-0.55, -0.15, 0.5], [-0.55, 0.15, 0.5], [-0.55, 0.40, 0.5],
                     [-0.55, -0.40, 0.24]]
    obs_locations = [scene_offset * SE3(0, 0, 0.7) * SE3(*loc) for loc in obs_locations]

    held_id = None
    held_id_2 = None
    plate_in_move = False
    plate_id = 0
    p1_stop = False

    estop_button = EStop(initial_pose=SE3(-1.3, 0, 0.65), use_physical_button=True)
    estop_button.add_to_env(env)

    while True:
        # Check physical estop
        if estop_button.get_state():
            robot_can_move[0] = False

        # Spawn and manage plates
        for i, p in enumerate(plates_status):
            if p == "Absent":
                items[i].update_transform(far_far_away)
            if p == "Error":
                if robot_can_move[0]:
                    print(f"Remove errored plates: {i + 1}")
                robot_can_move[0] = False
                plate_in_move = False

            if obstructions[i]:
                if p == "Stowed":  # Block user from obstructing a cell with a plate in it
                    obstructions[i] = False
                    print("Cannot obstruct cell with stowed plate")
                elif p == "Moving":
                    robot_can_move[0] = False
                    print("Stopping due to obstructed cell")

        # Spawn and despawn obstruction objects
        for state, obj, loc in zip(obstructions, obstruction_objects, obs_locations):
            if state:
                obj.update_transform(loc)
            else:
                obj.update_transform(far_far_away)

        # Automatically queue up waiting plates (Should never happen in reality)
        if not plate_in_move and not any([p == "Moving" for p in plates_status]):
            for i, p in enumerate(plates_status):
                if p == "Waiting":
                    if obstructions[i]:
                        plates_status[i] = "Obstructed"
                        print("Cell is obstructed")
                    else:
                        plate_id = i
                        plate_in_move = True
                        robot_1.simulation_step(not robot_can_move[0])
                        robot_1.run(load_path)
                        items[i].update_transform(printer_spawn)
                        p1_stop = False
                elif p == "Obstructed":
                    if not obstructions[i]:
                        plates_status[i] = "Absent"

        # Robot 1 handling
        action_1 = robot_1.simulation_step(not robot_can_move[0])

        if 'grip' in action_1:
            held_id = plate_id

        if held_id is not None:  # Move item if being held
            end_effector_transform = robot_1.get_end_effector_transform()
            items[held_id].update_transform(
                end_effector_transform * tool_offset * SE3.Rx(-90, unit="deg"))

        if 'release' in action_1:
            held_id = None

        # Robot 2 handling
        action_2 = robot_2.simulation_step(not robot_can_move[0])

        if 'grip' in action_2:
            held_id_2 = plate_id

        if held_id_2 is not None:  # Move item if being held
            end_effector_transform = robot_2.get_end_effector_transform()
            items[held_id_2].update_transform(
                end_effector_transform * tool_offset2 * SE3.Rx(-90, unit="deg"))

        if 'release' in action_2:
            held_id_2 = None

        env.step(0.01)

        # Sequence control logic
        if action_1['stop'] and plate_in_move:
            if obstructions[plate_id]:
                print("Obstruction detected")
                robot_can_move[0] = False
                plates_status[plate_id] = "Error"
            else:
                robot_2.run(reposition_paths[plate_id])
                plates_status[plate_id] = "Moving"
                p1_stop = True

        if action_2['stop'] and p1_stop:
            plate_in_move = False
            p1_stop = False
            plates_status[plate_id] = "Stowed"


if __name__ == '__main__':
    full_scene_sim()
