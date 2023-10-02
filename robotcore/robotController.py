import json
import time
from math import pi
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as lin
import roboticstoolbox as rtb
import roslibpy

from pathplanner import PathPlan
from props import Prop
from rosbags.highlevel import AnyReader  # For rosbag playback
from spatialmath import SE3
from tqdm import tqdm

from Gripper.Gripper import Gripper
from UR3Lin.UR3Lin import UR3Lin
from myUtils import eng_unit, plural, safe_write_to_file


class RobotController:
    """Controls robot, manages simulation and ROS transmissions"""
    # todo: Separate out a RosRobot class

    def __init__(self, path_json, swift_env=None, ros_client=None, transform=None, bake=None, robot=None, gripper=None):
        if type(bake) != str and bake is not None:
            raise ValueError(f"Bake filename is not a valid type: {type(bake)},  {bake}")

        self.bake_filename = bake
        self.bake_data = []
        self.playbake_data = None
        self.has_played = 0

        self.path_json_file = path_json

        self.path = PathPlan(json_data=open(self.path_json_file))
        self.end_effector_pos = self.path.start_pos

        # Perform sanity checks on the position
        if lin.norm(np.array([a[3] for a in self.end_effector_pos.A[:3]])) < 50e-3:
            # Robot end effector is likely too close to the origin
            raise ValueError("Robot start position is too close to the origin")

        self.instruction_index = 0
        self.current_trajectory = None

        self.base_offset = transform if transform is not None else SE3()

        self.arm = UR3Lin() if robot is None else robot()

        self.gripper = Gripper(swift_env) if gripper is None else gripper(swift_env)

        self.gripper_base_offset = self.gripper.base_offset
        self.tool_offset = self.gripper.tool_offset
        self.inv_tool_offset = SE3(np.linalg.inv(self.tool_offset.A))

        self.arm.base = self.base_offset * self.arm.base

        self.step_count = 5
        self.interp_count = 4

        self.has_env = swift_env is not None
        self.is_transmitting = ros_client is not None and not self.has_env  # Defer to simulation rendering
        self.swift_env = swift_env
        self.ros_client = ros_client

        if not self.has_env and not self.is_transmitting:  # One of these should be present
            raise ValueError("No env or client attached")

        if self.has_env:  # Default to 
            self.arm.add_to_env(self.swift_env)

        elif self.is_transmitting:
            print('Setting up ROS connection')
            self.control_topic = None
            self.subscriber = None
            self.last_message = None
            self._setup_ros_client()

    # ------------ ROS Handling ------------
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
        assert self.is_transmitting
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
        """Transmit previously recorded robot actions to ROS"""
        if self.playbake_data is None:
            self.playbake_data = self._read_bake(bakefile)

        joint_trajectory_msg = {  # This is the reset move
            'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
            'points': [{
                'positions': [float(j) for j in self.playbake_data[0]['joints']][:6],
                'time_from_start': {'secs': 5, 'nsecs': 0}
            }]
        }
        self.control_topic.publish(roslibpy.Message(joint_trajectory_msg))
        time.sleep(5)

        for entry in self.playbake_data:
            self.current_trajectory = {'stop': False, **entry, 'joints': [entry['joints']]}
            self.arm.q = entry['joints'][:]
            self._send_ros_message(entry['joints'])
            t = time.time()
            time.sleep(0.1)
            while any(abs(self.last_message['position'] - self.arm.q) > 5e-3):  # Wait for joints to reach target
                if time.time() > t + 1:
                    t = time.time()
                    print(abs(self.last_message['position'] - self.arm.q).tolist())  # Print positional error at 1Hz

        return True, self.current_trajectory

    @staticmethod
    def read_rosbag(rosbagfile):
        # create reader instance and open for reading
        with AnyReader([Path(rosbagfile)]) as reader:
            connection = [x for x in reader.connections if x.topic == '/joint_states']
            positions = []
            # print(connection.msgdef)
            for connection, timestamp, rawdata in reader.messages(connection):
                msg = reader.deserialize(rawdata, connection.msgtype)
                positions.append(msg.__dict__['position'])  # Couldn't find the intended retrieval function.
        bake = [{'stop': False, 'action': 'm', 'joints': j} for j in positions]
        bake[-1]['stop'] = True
        print(f"Loaded bagfile with {len(bake)} {plural('position', len(bake))}")
        return bake

    # -------- Trajectory Processes --------
    def _get_rapid_trajectory(self, j_start, end):
        """Joint-level interpolation function"""
        ik_result = self._perform_ik_search(end, q0=j_start)

        if ik_result[1] != 1:
            print(f"Warning: Failed rapid IK search: {ik_result}")

        self._check_ik_tolerance(ik_result, end)

        j_end = ik_result[0]
        result = [np.linspace(s, e, self.step_count * self.interp_count * 2) for s, e in zip(j_start, j_end)]
        return np.array(result).T.tolist()

    def _get_precise_trajectory(self, start, end):
        """Position interpolation function"""
        trajectory = rtb.ctraj(start, end, self.step_count)
        joint_states = []
        q = self.arm.q

        for t in trajectory:
            ik_result = self._perform_ik_search(t, q0=q)

            if ik_result[1] != 1:
                print(f"Warning: Failed precise IK search: {ik_result}")

            self._check_ik_tolerance(ik_result, t)
            joint_states.extend(self._interpolate_joints(q, ik_result[0]))

            q = ik_result[0]

        return joint_states

    def get_trajectory(self, end, start=None, rapid=False):
        """Get trajectory to arbitrary point"""
        self.end_effector_pos = self.get_end_effector_transform()
        end = self.base_offset * end * self.inv_tool_offset

        # If rapid, just interpolate joint states
        if rapid:
            start = self.arm.q if start is None else self._perform_ik_search(start)[0]
            return self._get_rapid_trajectory(start, end)

        start = self.end_effector_pos if start is None else self.base_offset * start
        return self._get_precise_trajectory(start, end)

    def get_next_trajectory(self):
        """Get the next trajectory from path"""
        if not self.path.path_points:
            raise ValueError("No points available in the path")

        print(f"Running path: {self.instruction_index + 1}/{len(self.path.path_points)}, "
              f"{round((self.instruction_index + 1)/len(self.path.path_points) * 100):>3}%")

        self.current_trajectory = {'joints': None}
        if self.instruction_index + 1 == len(self.path.path_points):
            self.current_trajectory['joints'] = []  # Empty iterator to force end of program
            return False  # False if end of path

        next_instr = self.path.path_points[self.instruction_index]
        self.instruction_index += 1
        self.current_trajectory = {**self.current_trajectory, **next_instr}
        if next_instr['action'] in ['m', 'rpd']:  # Instruction is a movement command
            end = next_instr['point']
            self.current_trajectory['joints'] = self.get_trajectory(end, rapid=next_instr['action'] == 'rpd')
            print(f"Moving to transform:\n{end}")
            return True

        if next_instr['action'] == 'grb':
            self.current_trajectory['grip'] = next_instr['id']
            self.current_trajectory['gripper'] = np.linspace(self.gripper.open, self.gripper.close,
                                                             self.step_count)[:].tolist()
            print("Picking up")

        elif next_instr['action'] == 'rel':
            self.current_trajectory['release'] = next_instr['id']
            self.current_trajectory['gripper'] = np.linspace(self.gripper.close, self.gripper.open,
                                                             self.step_count)[:].tolist()
            print("Dropping")

        # Arm shouldn't move while gripper is moving. Although it absolutely can if needed
        self.current_trajectory['joints'] = [[*self.arm.q.tolist()] for _ in self.current_trajectory['gripper']]
        return True

    # ------------ Joint and IK ------------
    def _perform_ik_search(self, t, q0=None):
        q0 = self.arm.q if q0 is None else q0

        # Use Newton Raphson since it accepts joint limit constraints
        return self.arm.ik_NR(t, q0=q0, joint_limits=True, slimit=100, mask=np.array([0.01, 1, 1, 1, 1, 1, 1]))

    def _check_ik_tolerance(self, ik_result, target):
        ee_error = self.arm.fkine(ik_result[0]).t - target.t
        if any(5e-3 < ee_error):
            print(f"Warning: IK tolerance above limit: {ee_error}")

    def _interpolate_joints(self, j_start, j_end):
        return [j_start + fraction * (j_end - j_start) for fraction in np.linspace(0, 1, self.interp_count)]

    def get_end_effector_transform(self):
        return self.arm.fkine(self.arm.q)

    # ----------- Sims and plots -----------
    def plot_reach(self, step_sizes: list, z_threshold=0.7, plot_plt=True, plot_swift=False):
        """
            Plot the reach of the robot in matplotlib or swift.

        :param step_sizes: Number of poses to explore per joint
        :param z_threshold: Lower bound on end effector height
        :param plot_plt: Allow matplotlib plotting
        :param plot_swift: Allow swift plotting (slow!)
        :return: None
        """

        # Fill missing step sizes with blanks
        step_sizes.extend([0] * (len(self.arm.links) - len(step_sizes)))

        # Generate joint configuration (lattice?)
        joint_ranges = [np.linspace(link.qlim[0], link.qlim[1], step + 1) for link, step in
                        zip(self.arm.links, step_sizes)]
        grids = np.meshgrid(*joint_ranges, indexing='ij')
        joint_configs = np.stack(grids, axis=-1).reshape(-1, len(self.arm.links))

        # Calculate forward kinematics for each joint configuration
        print(f"Calculating Fkine on {len(joint_configs)} points")
        transforms = np.array([self.arm.fkine(q=r).t.T for r in joint_configs])
        transforms = transforms[transforms[:, 2] >= z_threshold]  # Filter for z height

        # Extract the min and max values for each dimension
        x_min, x_max = transforms[:, 0].min(), transforms[:, 0].max()
        y_min, y_max = transforms[:, 1].min(), transforms[:, 1].max()
        z_min, z_max = transforms[:, 2].min(), transforms[:, 2].max()

        if plot_plt:
            fig = plt.figure(figsize=(8, 8))
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(*transforms.T)

            max_range = np.array([x_max - x_min, y_max - y_min, z_max - z_min]).max(initial=None) / 2.0

            mid_x = (x_max + x_min) * 0.5
            mid_y = (y_max + y_min) * 0.5
            mid_z = (z_max + z_min) * 0.5

            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)

            plt.show()

        if plot_swift:
            for i, t in tqdm(enumerate(transforms), desc='Plotting objects', total=len(transforms)):
                Prop('objects\\dot', self.swift_env, position=[*t, 0], color=(0, 0, 255))

        print(f"Robot reaches dimensions:")
        for dim, mi, ma in zip(['x', 'y', 'z'], [x_min, y_min, z_min], [x_max, y_max, z_max]):
            print(f"  {dim}: {eng_unit(mi, 'm')} : {eng_unit(ma, 'm')}")
        print(f"And explores a volume ~ {eng_unit((x_max - x_min) * (y_max - y_min) * (z_max - z_min), 'm3')}")

    def simulation_step(self):
        """Simulate next action in env"""
        if not self.has_env:
            raise EnvironmentError("Environment must be attached for simulation")

        # If no trajectory/end of trajectory, get the next trajectory
        if self.current_trajectory is None or not len(self.current_trajectory['joints']):
            if not self.get_next_trajectory():
                self._save_bake()
                return {'stop': True}  # End the simulation

        action = self._process_current_step()
        return action

    def _process_current_step(self, do_bake=True):
        """Process and return the current simulation step"""
        current_step = self.current_trajectory['joints'].pop(0)
        self.gripper.base = self.arm.fkine(self.arm.q) * self.gripper_base_offset

        gripper_val = None
        if 'gripper' in self.current_trajectory:
            gripper_val = self.current_trajectory['gripper'].pop(0)
        self.gripper.setq(gripper_val)  # Must run to update base position

        self.arm.q = current_step[:]
        action = {
            'stop': False,
            'joints': current_step,
            **self.current_trajectory
        }
        if do_bake:
            self._record_bake(action, current_step, gripper_val)
        return action
    
    # ---------- Bake management -----------
    def _record_bake(self, action, current_step, gripper_val):
        """Add a bake record"""
        self.bake_data.append({
            'stop': action['stop'],
            'action': action['action'],
            'joints': [float(c) for c in current_step],
            'gripper': [float(g) for g in gripper_val] if gripper_val is not None else None
        })

    def _save_bake(self):
        if self.bake_filename is None:
            return
        safe_write_to_file('bakes\\' + self.bake_filename, json.dumps(self.bake_data), new_file=True, extension='bake')

    def _read_bake(self, bakefile):
        if bakefile[-3:] == 'bag':
            return self.read_rosbag(bakefile)[:]
        self.bake_data = json.load(open(bakefile))
        return self.bake_data[:]  # Slice to copy data, else gets bound to the file unintentionally

    def playback_bake(self, bakefile):
        """Playback recorded robot actions from file"""
        if self.playbake_data is None:
            self.playbake_data = self._read_bake(bakefile)
        if self.has_played == len(self.playbake_data):
            return False, None
        entry = self.playbake_data[self.has_played]
        self.has_played += 1
        self.current_trajectory = {'stop': False, **entry, 'joints': [entry['joints']]}

        self.arm.q = entry['joints'][:]
        self.gripper.base = self.arm.fkine(self.arm.q) * self.gripper_base_offset
        self.gripper.setq(self.current_trajectory['gripper']) if 'gripper' in self.current_trajectory else None
        return True, self.current_trajectory
