from queue import Queue
import time
import uuid
from ir_support import UR3
from simple_robot_controller import SimpleRobotController
from roboticstoolbox import DHRobot
from spatialmath import SE3
from base_robot_controller import BaseRobotController

def main():
  ur3 = UR3()
  command_queue = Queue()
  response_queue = Queue()
  # Replace below with your implementation
  # robot_controller = BaseRobotController(ur3, command_queue, response_queue)
  robot_controller = SimpleRobotController(ur3, command_queue, response_queue)
  test_go_to_pose(ur3, command_queue, response_queue, robot_controller)


def test_go_to_pose(ur3: DHRobot, command_queue: Queue, response_queue: Queue, robot_controller: BaseRobotController):
  go_to_pose_request = {
   "id": uuid.uuid4(),
    "command": "GO_TO_POSE_REQUEST",
    "timestamp": 0,
    "data": SE3(0, 0, 0)
  }

  command_queue.put(go_to_pose_request)
  while response_queue.empty():
    robot_controller.step()
    time.sleep(0.01)
  
  response = response_queue.get()

  expected_response = {
    "id": go_to_pose_request["id"],
    "command": "GO_TO_POSE_RESPONSE",
    "timestamp": 0,
    "successful": True
  }

  if response == expected_response:
    print("Response command good")
  else:
    print("Response command bad")
    print(f"Expected: {expected_response}")
    print(f"Received: {response}")
  
  robot_actual_pose = ur3.fkine(ur3.q)
  robot_desired_pose = go_to_pose_request["data"]

  x1 = robot_actual_pose.A[0,3]
  y1 = robot_actual_pose.A[1,3]
  z1 = robot_actual_pose.A[2,3]

  x2 = robot_desired_pose.A[0,3]
  y2 = robot_desired_pose.A[1,3]
  z2 = robot_desired_pose.A[2,3]

  # Calculate the distance between the two
  distance = ((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)**0.5
  print(distance)
  if distance < 0.01:
    print("Robot at correct pose")
  else:
    print("Robot at incorrect pose")
    print(f"Expected\n: {SE3(0,0,0)}")
    print(f"Received\n: {ur3.fkine(ur3.q)}")



if __name__ == '__main__':
  main()