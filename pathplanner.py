import json
from spatialmath import SE3

from myUtils import safe_write_to_file

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
