import json
import random

import matplotlib.pyplot as plt
import numpy as np
from spatialmath import SE3
from spatialmath.base import trplot

from myUtils import eng_unit, plural, safe_write_to_file

"""
Path Planner
Provides top level target selection and path planning
Evaluation defaults to 'scene.json' and saves to 'path_plan.json'
Optional visualisations available from PathPlan
"""
# todo: Move plotter to PathPlan class


class PathPlan:
    """
    Utility class to handle loading and storing the path plan.

    Methods:
        - get_path: Returns the path.
        - add_path: Adds a point to the path.
        - to_json: Returns the serialised SE3 path in JSON.
        - from_json: Loads the path from a JSON string.
    """

    def __init__(self, start=None, path_points=None, json_data=None):
        self.start_pos = start if start is not None else SE3()
        self.path_points = path_points if path_points is not None else []
        if json_data is not None:
            self.from_json(json_data)

    def get_path(self):
        return self.path_points

    def add_path(self, point=None, action='m', obj_id=None):
        path_point = {'action': action}
        if point is not None:
            path_point['point'] = point
        if obj_id is not None:
            path_point['id'] = obj_id
        self.path_points.append(path_point)

    def _serialise(self):
        # SE3 transforms are not serializable. Convert to 4x4 nested list
        serializable_path = [(point['action'], [row.tolist() for row in point['point'].A] if 'point' in point else None,
                              point['id'] if 'id' in point else None)
                             for point in self.path_points]
        return {"start": [row.tolist() for row in self.start_pos.A],
                "path_points": serializable_path}

    def to_json(self):
        output = json.dumps(self._serialise(), indent=4)
        return output

    def __repr__(self):
        return f"PathPlan({self.start_pos}, {self.path_points})"

    def from_json(self, json_data):
        try:
            data = json.load(json_data)
        except AttributeError:
            data = json.loads(json_data)  # Data is a string already

        self.start_pos = SE3(data['start'])
        self.path_points = []
        for point in data['path_points']:
            action = {'action': point[0]}
            if point[1] is not None:
                action['point'] = SE3(point[1])
            if point[2] is not None:
                action['id'] = point[2]
            self.path_points.append(action)
        return self._serialise()

    def test(self):
        """Test the serializer and json converters. Requires object to already have data stored"""
        serial = self._serialise()
        assert self.from_json(self.to_json()) == serial


class PathPlanner:
    """
    Main path planning utility for the robot.

    Methods:
        - grab, release: Encode grb and rel actions to the path.
        - reset: Returns the path to its starting position.
        - move_to: Encodes a 3 part move to a given position.
    """
    def __init__(self, safe_z):
        self.reset_point = [250e-3, 100e-3, 500e-3]  # Starting and stopping point

        self.robot_location = SE3(self.reset_point) * SE3.Rx(180, unit='deg')  # Current location of end-effector
        self.path = PathPlan(start=self.robot_location)
        self.safe_z = safe_z
        self.gripping = False
        self.brick_id = None

    def grab(self, brick_id):
        brick_id = int(brick_id)  # Flush numpy datatype
        self.path.add_path(action='grb', obj_id=brick_id)
        self.gripping = True
        self.brick_id = brick_id

    def release(self):
        self.path.add_path(action='rel', obj_id=self.brick_id)
        self.brick_id = None
        self.gripping = False

    def reset(self):
        """Go back to starting position"""
        safe_point = SE3(self.robot_location.A.copy())
        safe_point.A[2, 3] = self.safe_z

        rst_point = SE3(self.reset_point) * SE3.Rx(180, unit='deg')
        safe_pos = rst_point.copy()
        safe_pos.A[2, 3] = self.safe_z

        if self.gripping:
            self.release()
        self.path.add_path(safe_point)
        self.path.add_path(safe_pos, 'rpd')  # rpd encodes a rapid movement
        self.path.add_path(rst_point)

    def move_to(self, position):
        # Calculate points above start and end given safe height
        safe_point = SE3(self.robot_location.A.copy())
        safe_point.A[2, 3] = self.safe_z

        safe_pos = SE3(position.A.copy())
        safe_pos.A[2, 3] = self.safe_z

        self.path.add_path(safe_point)  # Up, across, down
        self.path.add_path(safe_pos, 'rpd')  # rpd encodes a rapid movement
        self.path.add_path(position)
        self.robot_location = position

    @staticmethod
    def get_test_cases():
        cases = {}  # Expected to have more test cases. Guess not

        # Randomly distributed points in volume, with random z rotation
        a = [(random.random() * 1e0, random.random() * 1e0,
              random.random() * 0.6, random.random() * 180) for _ in range(20)]

        b = [(random.random() * 1e0, random.random() * 1e0,
              random.random() * 0.6, random.random() * 180) for _ in range(10)]
        cases["random"] = [a, b]
        return cases


def greedy_match(a, b, distance_h=None):
    """Greedy match all points from list 'a' to the nearest available list 'b' point"""
    assert len(a) <= len(b)

    distance_h = distance_h if distance_h is not None else \
        lambda s, e: np.linalg.norm(np.array(s.t) -
                                    np.array(e.t))  # For use with spatialmath transforms
    # lambda s, e: (s[0] - e[0]) ** 2 + (s[1] - e[1]) ** 2 + (s[2] - e[2]) ** 2  # Assumes indexes 0 - 2 are 3d pos
    # lambda s, e: (s[0] - e[0]) ** 2 + (s[1] - e[1]) ** 2  # alt matching for only xy dist

    far_point = SE3(1e12, 1e12, 1e12)  # Somewhere really far out, so it never gets matched
    # [1e6, 1e6, 1e6] # For non-se3

    # copy a, b to prevent modification upstream
    a = a[:]
    b = b[:]

    matches = []
    for i in range(len(a)):
        current_point = a.pop(0)
        distances = [distance_h(current_point, b_cand) for b_cand in b]  # Get dist heuristic to all candidates
        best_index = np.argmin(distances)
        b[best_index] = far_point  # Clear the point but leave it in the list
        matches.append((i, best_index))
    return matches


def _convert_xyzr_to_se3(xyzr):
    # xyzr => xyz + rotation around z axis
    transform = SE3(xyzr[:3]) * SE3.Rz(xyzr[3], unit='deg')
    return transform


def read_scene(filename):
    with open(filename, "r") as json_file:
        data = json.load(json_file)

    return data['bricks'], data['targets']  # Just return the bricks and targets, rest is not needed


def save_scene(filename, bricks, targets):
    data = {
        "bricks": bricks,
        "targets": targets
    }
    filename = safe_write_to_file(filename, json.dumps(data, indent=4))
    print(f"Written path file to {filename}")


def plot_combined(brick_pos, target_pos, match_pairs, paths=None,
                  show_path=False, show_targets=False, show_bricks=False,
                  show_all_bricks=False, show_matches=False):

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    if show_all_bricks:
        for i in [b.A for b in brick_pos]:
            trplot(i, ax=ax, length=0.1)

    if show_bricks and not show_all_bricks:
        for i in [b.A for ind, b in enumerate(brick_pos) if ind in [m[1] for m in match_pairs]]:
            trplot(i, ax=ax, length=0.1)

    if show_targets:
        for i in [b.A for b in target_pos]:
            trplot(i, ax=ax, length=0.1, color='red')

    if show_matches:
        for e, s in match_pairs:
            start = brick_pos[s].t
            end = target_pos[e].t
            ax.plot(*[(a, b) for a, b in zip(start, end)])

    if show_path and paths:
        paths = [point['point'] for point in paths if 'point' in point]
        paths = [point.t for point in paths]
        ax.plot(*zip(*paths))

    plt.show()


def produce_path_plan(scene_file, save=None, show_matching=True, show_path=True, safe_z=0.2, brick_height_offset=40e-3):
    """Read a scene file and arrange brick motions.
    Assumes bricks are reachable by robot and scene is buildable
    Works for any arbitrary arrangement of bricks and targets when n_bricks >= n_targets"""

    # Load in brick and target positions
    # brick_pos, target_pos = PathPlanner.get_test_cases()['random']  # randomly distributed bricks and targets to test
    brick_pos, target_pos = read_scene(scene_file)

    brick_pos = [_convert_xyzr_to_se3(pos) for pos in brick_pos]
    target_pos = [_convert_xyzr_to_se3(pos) for pos in target_pos]

    # Verify enough bricks are provided
    if len(brick_pos) < len(target_pos):
        bricks = len(brick_pos)
        targets = len(target_pos)
        raise AttributeError(f"{targets} {plural('target', targets)} {plural('was', targets, 'were')} provided "
                             f"but only {bricks} {plural('brick', bricks)}")

    # Use a greedy algorithm and match target and destination
    # Allows more bricks to be mapped to fewer targets and picks good options
    # Easier to implement than a more efficient hungarian algo
    matched_pairs = greedy_match(target_pos, brick_pos)

    if show_matching:
        plot_combined(brick_pos, target_pos, matched_pairs, show_targets=True, show_all_bricks=True, show_matches=True)

    # Dynamic safe-z calculation based on brick and target pos, or default to given value
    brick_z = [pos.A[2][3] for pos in brick_pos]
    target_z = [pos.A[2][3] for pos in target_pos]
    safe_z = max(safe_z, max(brick_z) + brick_height_offset, max(target_z) + brick_height_offset)
    print(f"Calculated safe_z of {eng_unit(safe_z, 'm')}")

    # Bricks are placed in the order of the target list
    # Targets are assumed to be buildable in order
    planner = PathPlanner(safe_z)
    for target_index, brick_index in matched_pairs:
        head_offset = SE3.Rx(180, unit='deg')
        t_pos = target_pos[target_index] * head_offset * SE3(0, 0, -brick_height_offset)
        b_pos = brick_pos[brick_index] * head_offset * SE3(0, 0, -brick_height_offset)

        # Straightforward robot path plan
        planner.move_to(b_pos)
        planner.grab(brick_index)
        planner.move_to(t_pos)
        planner.release()
    planner.reset()

    if show_path:
        plot_combined(brick_pos, target_pos, matched_pairs, paths=planner.path.get_path(),
                      show_path=True, show_targets=True, show_bricks=True, show_matches=True)

    # Write this into a file for transmission or playback
    # Add toggle for sequential or overwrite file naming
    save = 'path_plan.json' if save is None else save
    filename = safe_write_to_file(save, planner.path.to_json())
    print(f"Written scene file to {filename}")
    return planner


if __name__ == "__main__":
    produce_path_plan("scene.json", save='path_plan.json')
    # produce_path_plan("extrabrick.json", save='path_plan.json')
