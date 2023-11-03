import json
import time
from math import pi
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as lin
import roboticstoolbox as rtb
import roslibpy
import spatialmath

from ir_support import UR5
from pathplanner import PathPlan
from props import Prop
from rosbags.highlevel import AnyReader  # For rosbag playback
from spatialmath import SE3
from tqdm import tqdm

from Gripper.Gripper import Gripper
from myUtils import eng_unit, plural, safe_write_to_file


class RobotController:
    """Controls robot, manages simulation and ROS transmissions"""
    # todo: Separate out a RosRobot class

    def __init__(self, move_list, swift_env=None, ros_client=None, transform=None, bake=None, robot=None, gripper=None):
        assert type(bake) == str or bake is None

        self.bake_filename = bake
        self.bake_data = []
        self.playbake_data = None
        self.has_played = 0

        self.path = move_list
        # self.path.test()  # Just in case
        self.end_effector_pos = self.path.start_pos

        self.instruction_index = 0
        self.current_trajectory = None

        # Perform sanity checks on the position
        if lin.norm(np.array([a[3] for a in self.end_effector_pos.A[:3]])) < 50e-3:
            # Robot end effector is likely too close to the origin
            pass
            #raise ValueError("Robot start position is too close to the origin")

        self.base_offset = transform if transform is not None else SE3()

        self.arm = robot()
        self.gripper = Gripper() if gripper is None else gripper()

        self.debug = False
        if robot == UR5:
            self.gripper_base_offset = SE3(-4e-3, -2e-3, 81e-3)
            self.tool_offset = SE3(0, 0, 210e-3)
        else:
            self.gripper_base_offset = self.arm.gripper_base_offset
            self.tool_offset = self.arm.tool_offset
            #self.debug = True

        self.inv_tool_offset = SE3(np.linalg.inv(self.tool_offset.A))

        self.gripper.add_to_env(swift_env)

        if len(self.arm.q) == 7:
            self.arm.q = [-0.5, 0, -pi / 2, 0, 0, 0, 0]  # Known safe start values
            for i in range(len(self.arm.links)):
                self.arm.links[i].qlim = [-pi, pi]
            self.arm.links[0].qlim = [-0.8, 0]
            self.arm.links[2].qlim = [-pi / 2, 0]
            self.arm.links[3].qlim = [-pi / 2, 0]
            self.arm.links[4].qlim = [-pi, 0]

        elif len(self.arm.q) == 6:
            self.arm.q = [0, -pi / 2, 0, 0, 0, 0]  # Known safe start values
            for i in range(len(self.arm.links)):
                self.arm.links[i].qlim = [-pi, pi]
            self.debug = True


            self.arm.links[1].qlim = [-pi, 0]
            self.arm.links[4].qlim = [0, pi]
            self.arm.links[5].qlim = [0, pi]
            self.gripper_base_offset = SE3()
            self.tool_offset = SE3(0, 0, 210e-3 - 81e-3)

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
            #print('Setting up ROS connection')
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
        #print(f"Going to j: {joints}")
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
                    #print(abs(self.last_message['position'] - self.arm.q).tolist())  # #print positional error at 1Hz

        return True, self.current_trajectory

    @staticmethod
    def read_rosbag(rosbagfile):
        # create reader instance and open for reading
        with AnyReader([Path(rosbagfile)]) as reader:
            connection = [x for x in reader.connections if x.topic == '/joint_states']
            positions = []
            # #print(connection.msgdef)
            for connection, timestamp, rawdata in reader.messages(connection):
                msg = reader.deserialize(rawdata, connection.msgtype)
                positions.append(msg.__dict__['position'])  # Couldn't find the intended retrieval function.
        bake = [{'stop': False, 'action': 'm', 'joints': j} for j in positions]
        bake[-1]['stop'] = True
        #print(f"Loaded bagfile with {len(bake)} {plural('position', len(bake))}")
        return bake

    def run(self, new_path):
        self.path = new_path
        self.instruction_index = 0

    # -------- Trajectory Processes --------
    def _get_rapid_trajectory(self, j_start, end):
        """Joint-level interpolation function"""
        ik_result = self._perform_ik_search(end, q0=j_start)

        if ik_result[1] != 1:
            pass
            #print(f"Warning: Failed rapid IK search: {ik_result}")

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
                pass
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
        return self._get_rmrc(start, end)

    def get_next_trajectory(self):
        """Get the next trajectory from path"""
        if not self.path.path_points:
            return False
            raise ValueError("No points available in the path")

        #print(f"Running path: {self.instruction_index + 1}/{len(self.path.path_points)}, "
        #      f"{round((self.instruction_index + 1)/len(self.path.path_points) * 100):>3}%")

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
            #print(f"Moving to transform:\n{end}")
            return True
        if next_instr['action'] == "joint":
            end = next_instr["point"]
            self.current_trajectory["joints"] = self._interpolate_joints(self.arm.q, end,
                                                                         count=5 * self.step_count * self.interp_count)
            return True

        if next_instr['action'] == 'grb':
            self.current_trajectory['grip'] = next_instr['id']
            self.current_trajectory['gripper'] = np.linspace(self.gripper.open, self.gripper.close,
                                                             self.step_count)[:].tolist()
            #print("Picking up")

        elif next_instr['action'] == 'rel':
            self.current_trajectory['release'] = next_instr['id']
            self.current_trajectory['gripper'] = np.linspace(self.gripper.close, self.gripper.open,
                                                             self.step_count)[:].tolist()
            #print("Dropping")

        # Arm shouldn't move while gripper is moving. Although it absolutely can if needed
        self.current_trajectory['joints'] = [[*self.arm.q.tolist()] for _ in self.current_trajectory['gripper']]
        return True

    # ------------ Joint and IK ------------
    def _perform_ik_search(self, t, q0=None):
        q0 = self.arm.q if q0 is None else q0

        # Use Newton Raphson since it accepts joint limit constraints
        return self.arm.ik_NR(t, q0=q0, joint_limits=True, slimit=100)

    def tr2delta(self,  T1, T2):
        # Calculate translational difference
        trans = T2.t - T1.t

        # Calculate rotational difference
        R_diff = T1.R.T @ T2.R
        angle, axis = spatialmath.SO3(R_diff).angvec()

        # Convert rotational difference to angular velocity
        rot = axis * angle

        return np.hstack((trans, rot))

    def _get_rmrc(self, start, end):
        """RMRC interpolation function"""
        total_time = 500
        step_count = 50
        delta_t = total_time / step_count
        joint_states = []
        q = self.arm.q
        try:
            for i in range(step_count):
                # Compute the desired end-effector pose at the next time step
                T_desired = rtb.ctraj(start, end, (i + 1) / step_count)

                # Compute the Jacobian matrix at the current joint configuration
                J = self.arm.jacob0(q)

                # Compute the error in position and orientation
                T_current = self.arm.fkine(q)
                delta_T = self.tr2delta(T_current, T_desired)

                # Compute the desired end-effector velocity
                v_desired = delta_T / delta_t

                # Solve for joint velocities
                velocity_scaling_factor = 0.1  # Artificially scale down the velocities
                q_dot = velocity_scaling_factor * np.linalg.pinv(J) @ v_desired

                # Integrate joint velocities to get joint positions
                q = q + q_dot * delta_t
                joint_states.append(q.tolist())
        except ValueError:
            print('Math error in rmrc')
            joint_states = self._get_rapid_trajectory(self.arm.q, end)
            
        return joint_states
    
    def _check_ik_tolerance(self, ik_result, target):
        ee_error = self.arm.fkine(ik_result[0]).t - target.t
        if any(5e-3 < ee_error):
            pass
            #print(f"Warning: IK tolerance above limit: {ee_error}")

    def _interpolate_joints(self, j_start, j_end, count=None):
        count = self.interp_count if count is None else count
        return [j_start + fraction * (j_end - j_start) for fraction in np.linspace(0, 1, count)]

    def get_end_effector_transform(self):
        return self.arm.fkine(self.arm.q)

    def prove_move(self, transform):
        self.current_trajectory['joints'] = self.get_trajectory(transform, rapid=True)
        while self.current_trajectory['joints']:
            self._process_current_step(do_bake=False)
            self.swift_env.step(0.01)

        #print(f"Robot is expected to be at: \n{(self.base_offset * transform * self.inv_tool_offset).A}")
        #print(f"Robot is actually at: \n{self.arm.fkine(self.arm.q).A}")
        self.swift_env.hold()

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
        #print(f"Calculating Fkine on {len(joint_configs)} points")
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

        #print(f"Robot reaches dimensions:")
        for dim, mi, ma in zip(['x', 'y', 'z'], [x_min, y_min, z_min], [x_max, y_max, z_max]):
            pass
            #print(f"  {dim}: {eng_unit(mi, 'm')} : {eng_unit(ma, 'm')}")
        #print(f"And explores a volume ~ {eng_unit((x_max - x_min) * (y_max - y_min) * (z_max - z_min), 'm3')}")

    def simulation_step(self, estop):
        """Simulate next action in env"""
        if not self.has_env:
            raise EnvironmentError("Environment must be attached for simulation")

        # If no trajectory/end of trajectory, get the next trajectory
        if self.current_trajectory is None or not len(self.current_trajectory['joints']):
            if not self.get_next_trajectory():
                self._save_bake()
                return {'stop': True}  # End the simulation

        action = self._process_current_step(estop)
        return action

    def _process_current_step(self, estop, do_bake=True):
        """Process and return the current simulation step"""
        if estop:
            action = {
                'stop': False,
                'joints': self.arm.q,
                **self.current_trajectory
            }
            return action
        else:
            current_step = self.current_trajectory['joints'].pop(0)
            self.gripper.base = self.arm.fkine(self.arm.q) * self.gripper_base_offset

            gripper_val = None
            if 'gripper' in self.current_trajectory:
                gripper_val = self.current_trajectory['gripper'].pop(0)
            self.gripper.setq(gripper_val)  # Must run to update base position
            if self.debug:
                pass
                #Prop('objects\\dot', self.swift_env, transform=self.gripper.base)
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

        safe_write_to_file('bakes\\' + self.bake_filename, json.dumps(self.bake_data), new_file=True)

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

    def tweak(self, joint, dist):
        if self.instruction_index != len(self.path.path_points) - 1:
            return
        
        q = self.arm.q
        q[joint] += dist / 180 * pi
        self.arm.q = q
        self.gripper.base = self.arm.fkine(self.arm.q) * self.gripper_base_offset
        self.gripper.setq(None)
