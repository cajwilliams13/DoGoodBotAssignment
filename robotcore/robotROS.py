import roslibpy
from robotController import RobotController
from ir_support import UR3

client = roslibpy.Ros(host='192.168.27.1', port=9090)
try:
    traj_planner = RobotController("lite_path.json", ros_client=client, robot=UR3)  # For in-person demo
    running, action = traj_planner.transmit_bake("2018-03-20-18-34-46.bag")

finally:
    client.terminate()
