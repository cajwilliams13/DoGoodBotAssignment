from abc import ABC, abstractmethod
from queue import Queue

from roboticstoolbox import DHRobot

class BaseRobotController(ABC):
  @abstractmethod
  def __init__(self, robot: DHRobot, command_queue: Queue, response_queue: Queue):
    pass

  @abstractmethod
  def step(self):
    pass