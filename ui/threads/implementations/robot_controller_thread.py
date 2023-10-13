import logging
import time
from queue import Empty, Queue

import numpy as np
from roboticstoolbox import DHRobot, jtraj
from spatialmath import SE3

from ..background_thread import BackgroundThread


class RobotControllerThread(BackgroundThread):
    def __init__(self, robot: DHRobot, queue: Queue):
        super().__init__()
        self._robot = robot
        self._queue = queue
        self._robot_trajectory = Queue()

    def startup(self) -> None:
        logging.info('RobotControllerThread started')


    def shutdown(self) -> None:
        logging.info('RobotControllerThread stopped')

    def handle(self) -> None:
        task = None
        try:
            task = self._queue.get(block=False)
            logging.info(f'RobotControllerThread received task: {task}')
        except Empty:
            pass

        if task == "INCREMENT_Q":
            self._robot.q += 0.1
            logging.info(f'Robot q values incremented: {self._robot.q}')
        if task == "AUTO_HOME":
            q_goal = np.zeros(self._robot.n)
            qtraj = jtraj(self._robot.q, q_goal, 50)
            # Add each q value to the robot trajectory queue
            for q in qtraj.q:
                self._robot_trajectory.put(q)
        if isinstance(task, dict):
            x = task['x']
            y = task['y']
            z = task['z']
            desired_pose = SE3(x, y, z)
            q_goal = self._robot.ikine_LM(desired_pose, q0=self._robot.q)
            qtraj = jtraj(self._robot.q, q_goal.q, 50)
            # Add each q value to the robot trajectory queue
            for q in qtraj.q:
                self._robot_trajectory.put(q)
        
        if not self._robot_trajectory.empty():
            self._robot.q = self._robot_trajectory.get()
        time.sleep(0.01)