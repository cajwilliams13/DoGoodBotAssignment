from queue import Queue, Empty
from roboticstoolbox import DHRobot, jtraj
from base_robot_controller import BaseRobotController
from robotController import RobotController


class StandardRobotController(BaseRobotController):
    def __init__(self, robot: DHRobot, command_queue: Queue, response_queue: Queue):
        self.robot_controller = RobotController(robot, command_queue, response_queue)

    def step(self):
        self.robot_controller.simulation_step()

    def add_to_env(self, env):
        self.robot_controller.add_to_env(env)
