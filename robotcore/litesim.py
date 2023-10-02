import swift
from spatialmath import SE3
from ir_support import UR3

from robotsim import create_sim_env
from robotController import RobotController


def lite_scene_sim():
    """Scene and UR3 playing back a provided bag file"""
    env = swift.Swift()
    env.launch(realtime=True)

    scene_offset = SE3(0, 0, 0)  # Master transform to move the entire robot + room setup
    traj_planner = RobotController("path_plan.json", env,
                                   transform=scene_offset * SE3(0, 0, 0.65), robot=UR3)  # For in-person demo

    # Place scene
    create_sim_env(env, scene_offset)

    do_plot_reach = False
    if do_plot_reach:
        traj_planner.plot_reach([8, 20, 20])
        #traj_planner.plot_reach([0, 20, 20], plot_swift=True, plot_plt=False)
        env.step()
        env.hold()
        exit()

    # Run call loop to robot controller
    while True:
        running, action = traj_planner.playback_bake("2018-03-20-18-34-46.bag")  # Toggle to play back bag file

        if not running:
            break

        env.step(0)
    env.hold()


if __name__ == '__main__':
    lite_scene_sim()
