import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as lin
import roboticstoolbox as rtb

from props import Prop
from spatialmath import SE3
from tqdm import tqdm

from queue import Queue, Empty
# from Gripper.Gripper import Gripper
# from UR3Lin.UR3Lin import UR3Lin
from myUtils import eng_unit


class RobotController:
    """Controls robot, manages simulation and ROS transmissions"""

    # todo: Separate out a RosRobot class

    def __init__(self, robot, command_queue, response_queue, transform=None):
        self.end_effector_pos = robot.q

        self.commands = command_queue
        self.response = response_queue

        # Perform sanity checks on the position
        if lin.norm(np.array([a[3] for a in self.end_effector_pos.A[:3]])) < 50e-3:
            # Robot end effector is likely too close to the origin
            raise ValueError("Robot start position is too close to the origin")

        self.current_trajectory = None

        self.base_offset = transform if transform is not None else SE3()

        self.arm = robot  # UR3Lin() if robot is None else robot()

        self.gripper = self.arm.gripper

        self.gripper_base_offset = self.gripper.base_offset
        self.tool_offset = self.gripper.tool_offset
        self.inv_tool_offset = SE3(np.linalg.inv(self.tool_offset.A))

        self.arm.base = self.base_offset * self.arm.base

        self.step_count = 5
        self.interp_count = 4

        self.swift_env = None

        self.current_request = None
        self.has_next_instruction = False

    def add_to_env(self, env):
        self.swift_env = env
        self.gripper.add_to_env(self.swift_env)
        self.arm.add_to_env(self.swift_env)

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
        # raise NotImplementedError() # This code should read the queue
        try:
            self.has_next_instruction = True
            self.current_request = self.commands.get(block=False)
            if self.current_request["command"] == "GO_TO_POSE_REQUEST":
                end = self.current_request["data"]["point"]
                self.current_trajectory["joints"] = self.get_trajectory(end, rapid=self.current_request["data"][
                    "movement_mode"])
                # print(f"Moving to transform:\n{end}")

            elif self.current_request["command"] == "GRAB_REQUEST":
                self.current_trajectory['gripper'] = np.linspace(self.gripper.open, self.gripper.close,
                                                                 self.step_count)[:].tolist()
                self.current_trajectory['joints'] = [[*self.arm.q.tolist()] for _ in self.current_trajectory['gripper']]
                # print("Picking up")

            elif self.current_request["command"] == "RELEASE_REQUEST":
                self.current_trajectory['gripper'] = np.linspace(self.gripper.close, self.gripper.open,
                                                                 self.step_count)[:].tolist()
                self.current_trajectory['joints'] = [[*self.arm.q.tolist()] for _ in self.current_trajectory['gripper']]
                # print("Dropping")
            else:
                self.has_next_instruction = False
                self.response.put({
                    "id": self.current_request["id"],
                    "command": "UNRECOGNISED_COMMAND",
                    "timestamp": 0,
                    "successful": False
                })
        except Empty:
            self.has_next_instruction = False

        # self.current_trajectory = {**self.current_trajectory, **next_instr}

    def finish_movement(self):
        """Post a finished-movement message"""
        response_names = {
            "GO_TO_POSE_REQUEST": "GO_TO_POSE_RESPONSE",
            "GRAB_REQUEST": "GRAB_RESPONSE",
            "RELEASE_REQUEST": "RELEASE_RESPONSE",
        }

        self.has_next_instruction = False
        self.response.put({
            "id": self.current_request["id"],
            "command": response_names.get(self.current_request["command"], "UNKNOWN_RESPONSE"),
            "timestamp": 0,
            "successful": True
        })

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

    def _rmrf_interpolation(self):
        raise NotImplementedError()

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
        # If no trajectory/end of trajectory, get the next trajectory
        if self.current_trajectory is None or not len(self.current_trajectory['joints']):
            if self.current_trajectory is not None:
                self.finish_movement()
            self.get_next_trajectory()
        if self.has_next_instruction:
            action = self._process_current_step()
        # return action

    def _process_current_step(self):
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
        return action
