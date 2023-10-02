import swift
from spatialmath import SE3

from robotController import RobotController
from pathplanner import produce_path_plan
from pathplanner import read_scene
from props import Prop
from os import path


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
        Prop(path.join('objects', 'Estop'), env, transform=master_transform * table_offset * SE3(-1, 0.65, 2e-3), color=(100, 0, 0)),
        Prop(path.join('objects', 'Pallet_table'), env, transform=master_transform * table_offset, color=(99, 71, 32)),
        Prop(path.join('objects', 'extinguisher'), env, transform=master_transform * SE3(-0.7, 1.35, 0), color=(102, 15, 13))
    ]

    # Use XYZRz encoded position
    gate_locations = [(-1.8, 0, 0, 0), (-1.8, -gate_len, 0, 0), (1.8, 0, 0, 0), (1.8, -gate_len, 0, 0),
                      (1.8 - gate_len, -gate_len, 0, 90), (1.8, -gate_len, 0, 90),
                      (1.8 - 2 * gate_len, -gate_len, 0, 90),
                      (1.8, gate_len, 0, 90), (1.8 - 2 * gate_len, gate_len, 0, 90)]

    gate_locations += [(*g[:2], 0.6, g[3]) for g in gate_locations]  # Add a second layer of gates

    props.append(Prop(path.join('objects', 'Estop'), env, color=(100, 0, 0), transform=master_transform * table_offset *
                                                                          SE3(1.82 - gate_len, gate_len - 0.02, 0.561)))

    for gate in gate_locations:
        props.append(Prop(path.join('objects', 'w2h4_fence'), env, transform=master_transform, position=gate, color=(50, 50, 50)))

    return props


def full_scene_sim(scene_file='scene.json'):
    #produce_path_plan(scene_file, show_matching=True, show_path=True)  # Optionally update path plan before starting

    env = swift.Swift()
    env.launch(realtime=True)

    scene_offset = SE3(0, 0, 0) * SE3.Rz(0, unit='deg')  # Master transform to move the entire robot + room setup
    create_sim_env(env, scene_offset)

    traj_planner = RobotController("path_plan.json", swift_env=env, transform=scene_offset * SE3(0, 0, 0.65),
                                   bake='bake_test')
    # Tool offset needed for brick manipulation
    tool_offset = traj_planner.tool_offset

    # Place bricks and scene
    bricks = [Prop('objects\\brick', env, position, transform=scene_offset * SE3(0, 0, 0.65)) for position in
              read_scene(scene_file)[0]]
    held_brick_id = None

    do_plot_reach = False  # Show reach of robot in scene
    if do_plot_reach:
        traj_planner.plot_reach([8, 8, 20, 20])  # Matplotlib visualising
        traj_planner.plot_reach([0, 0, 20, 20], plot_swift=True, plot_plt=False)  # Swift env reach
        env.step()
        env.hold()
        exit()

    # Run call loop to robot controller
    while True:
        running = True
        # running, action = traj_planner.playback_bake('bakes\\true_bake.bake')
        action = traj_planner.simulation_step()  # Perform simulation step

        if not running:
            break

        if 'grip' in action:
            assert held_brick_id is None or held_brick_id == action['grip']  # Tried to grab brick while brick is held
            held_brick_id = action['grip']

        if held_brick_id is not None:  # Move brick if being held
            end_effector_transform = traj_planner.get_end_effector_transform()
            bricks[held_brick_id].update_transform(end_effector_transform * tool_offset)

        if 'release' in action:
            held_brick_id = None

        env.step(0.02)
        #env.step(0)

        if action['stop']:
            break

    # traj_planner.prove_move(SE3(-0.6, 0, 0.3) * SE3.Rx(180, unit='deg'))  # Rx to flip the end effector facing down

    env.hold()


if __name__ == '__main__':
    full_scene_sim()
    #full_scene_sim('extrabrick.json')
