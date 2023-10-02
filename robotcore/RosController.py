import time
import roslibpy
from json import load

from robotController import read_rosbag


class RosController:
    """ROS controller: Playback shell for controlling a ROS robot from a bakefile"""
    def __init__(self, ros_client):

        print('Setting up ROS connection')
        self.control_topic = None
        self.subscriber = None
        self.last_message = None
        self.ros_client = ros_client
        self._setup_ros_client()

    def _save_message(self, message):
        """Callback function to store ROS message"""
        self.last_message = message

    def _setup_ros_client(self):
        self.ros_client.run()
        self.subscriber = roslibpy.Topic(self.ros_client, '/joint_states',
                                         'sensor_msgs/JointState')  # Joint state subscriber
        self.subscriber.subscribe(lambda l: self._save_message(l))  # Bind callback

        self.control_topic = roslibpy.Topic(self.ros_client, '/scaled_pos_joint_traj_controller/command',
                                            'trajectory_msgs/JointTrajectory')
        self.control_topic.advertise()

    def _send_ros_message(self, joints):
        assert len(joints) == 6
        print(f"Going to j: {joints}")
        joint_trajectory_msg = {
            'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
            'points': [{
                'positions': [float(j) for j in joints],
                'time_from_start': {'secs': 5, 'nsecs': 0}
                # Time for reaching the desired position, which is 5 seconds in this case
            }]
        }
        self.control_topic.publish(roslibpy.Message(joint_trajectory_msg))

    def transmit_bake(self, bakefile):
        """Transmit previously recorded robot actions to ROS
           This should be the ONLY way to communicate with the robot"""
        if bakefile[-3:] == 'bag':
            bake_data = read_rosbag(bakefile)[:]
        else:
            bake_data = load(open(bakefile))

        joint_trajectory_msg = {  # This is the reset move
            'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
            'points': [{
                'positions': [float(j) for j in bake_data[0]['joints']][:6],
                'time_from_start': {'secs': 5, 'nsecs': 0}
            }]
        }
        self.control_topic.publish(roslibpy.Message(joint_trajectory_msg))
        time.sleep(5)

        for entry in bake_data:
            arm_q = entry['joints'][:]
            self._send_ros_message(entry['joints'])
            t = time.time()
            time.sleep(0.1)
            while any(abs(self.last_message['position'] - arm_q) > 5e-3):  # Wait for joints to reach target
                if time.time() > t + 1:
                    t = time.time()
                    print(abs(self.last_message['position'] - arm_q).tolist())  # Print positional error at 1Hz
