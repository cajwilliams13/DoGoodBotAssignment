from math import pi

import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as lin
import roboticstoolbox as rtb
import spatialmath

from ir_support import UR5

from myUtils import eng_unit
from props import Prop
from spatialmath import SE3
from tqdm import tqdm

from Gripper.Gripper import Gripper


class RobotController:
    """Controls robot, manages simulation and ROS transmissions"""

    def __init__(self, move_list, swift_env=None, transform=None, robot=None, gripper=None):
        self.path = move_list
        self.end_effector_pos = self.path.start_pos

        self.instruction_index = 0
        self.current_trajectory = None

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

        self.swift_env = swift_env
        self.arm.add_to_env(self.swift_env)

    def run(self, new_path):
        """Instruct robot to run new path"""
        self.path = new_path
        self.instruction_index = 0

    # -------- Trajectory Processes --------
    def _get_rapid_trajectory(self, j_start, end):
        """Joint-level interpolation function"""
        ik_result = self._perform_ik_search(end, q0=j_start)

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
            print(f"Moving to transform:\n{end}")
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
        return self.arm.ik_NR(t, q0=q0, joint_limits=True, slimit=100)

    @staticmethod
    def tr2delta(t1, t2):
        # Calculate translational difference
        trans = t2.t - t1.t

        # Calculate rotational difference
        R_diff = t1.R.T @ t2.R
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
                t_desired = rtb.ctraj(start, end, (i + 1) / step_count)

                # Compute the Jacobian matrix at the current joint configuration
                j = self.arm.jacob0(q)

                # Compute the error in position and orientation
                t_current = self.arm.fkine(q)
                delta_tt = self.tr2delta(t_current, t_desired)

                # Compute the desired end-effector velocity
                v_desired = delta_tt / delta_t

                # Solve for joint velocities
                velocity_scaling_factor = 0.1  # Artificially scale down the velocities
                q_dot = velocity_scaling_factor * np.linalg.pinv(j) @ v_desired

                # Integrate joint velocities to get joint positions
                q = q + q_dot * delta_t
                joint_states.append(q.tolist())

        except ValueError:
            print('Math error in rmrc')
            joint_states = self._get_rapid_trajectory(self.arm.q, end)

        return joint_states

    def _interpolate_joints(self, j_start, j_end, count=None) -> list:
        """Simple joint interpolation"""
        count = self.interp_count if count is None else count
        return [j_start + fraction * (j_end - j_start) for fraction in np.linspace(0, 1, count)]

    def get_end_effector_transform(self):
        return self.arm.fkine(self.arm.q)

    def prove_move(self, transform):
        """Provide verification robot has moved properly"""
        self.current_trajectory['joints'] = self.get_trajectory(transform, rapid=True)
        while self.current_trajectory['joints']:
            self._process_current_step(estop=False)
            self.swift_env.step(0.01)

        print(f"Robot is expected to be at: \n{(self.base_offset * transform * self.inv_tool_offset).A}")
        print(f"Robot is actually at: \n{self.arm.fkine(self.arm.q).A}")
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
        # print(f"Calculating Fkine on {len(joint_configs)} points")
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

    def simulation_step(self, estop):
        """Simulate next action in env"""

        # If no trajectory/end of trajectory, get the next trajectory
        if self.current_trajectory is None or not len(self.current_trajectory['joints']):
            if not self.get_next_trajectory():
                return {'stop': True}  # End the simulation

        action = self._process_current_step(estop)
        return action

    def _process_current_step(self, estop):
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
                # Prop('objects\\dot', self.swift_env, transform=self.gripper.base)
            self.arm.q = current_step[:]
            action = {
                'stop': False,
                'joints': current_step,
                **self.current_trajectory
            }
            return action

    def tweak(self, joint, dist):
        """Nudge one joint by an amount"""
        if self.instruction_index != len(self.path.path_points) - 1:
            return

        q = self.arm.q
        q[joint] += dist / 180 * pi
        self.arm.q = q

        self.gripper.base = self.arm.fkine(self.arm.q) * self.gripper_base_offset
        self.gripper.setq(None)  # Force gripper to follow arm
