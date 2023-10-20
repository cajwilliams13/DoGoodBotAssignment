from queue import Queue, Empty
from roboticstoolbox import DHRobot, jtraj
from base_robot_controller import BaseRobotController

class SimpleRobotController(BaseRobotController):
  def __init__(self, robot: DHRobot, command_queue: Queue, response_queue: Queue):
    self.robot = robot
    self.command_queue = command_queue
    self.response_queue = response_queue
    self.traj_queue = Queue()
    self.processing_request = False
    self.current_request = None

  def step(self):
    try:
      if self.processing_request == False:
        command = self.command_queue.get(block=False)
        if command["command"] == "GO_TO_POSE_REQUEST":
          self.processing_request = True
          self.current_request = command
          self._handle_go_to_pose(command)
    except Empty:
      pass

    if not self.traj_queue.empty():
      self.robot.q = self.traj_queue.get()
    elif self.processing_request == True:
      self.processing_request = False
      self.response_queue.put({
        "id": self.current_request["id"],
        "command": "GO_TO_POSE_RESPONSE",
        "timestamp": 0,
        "successful": True
      })

  def _handle_go_to_pose(self, request):
    q_goal = self.robot.ikine_LM(request["data"], q0=self.robot.q)
    qtraj = jtraj(self.robot.q, q_goal.q, 50)
    for q in qtraj.q:
      self.traj_queue.put(q)
    pass
