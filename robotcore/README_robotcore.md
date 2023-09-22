# RobotController

RobotController is a robotic control module designed for general purpose trajectory interpolation, simulation on swift,
and communication with a UR3 via ROS. 

## Features

- **Path Planning**: Automatic selection of bricks and targets for any given scene layout
- **Baking and Playback**: Bake simulated robot movements and play them back or transmit via ROS.
- **ROSbag compatible**: Can read and playback joint positions from a provided ROSbag

## Usage

### Producing a path
```python
from pathplanner import produce_path_plan

produce_path_plan("scene_file.json", save='your_path.json')
```
Format the scene file as a json with fields `bricks` and `targets`. The path will be built in the order of targets provided

```json
{
    "bricks": [
        [ X, Y, Z, Rz ],
          ...
        [ X, Y, Z, Rz ]
    ],
    "targets": [
        [ X, Y, Z, Rz ],
          ...
        [ X, Y, Z, Rz ]
    ]
}
```

### Initialisation for Swift

```python
from robotController import RobotController

controller = RobotController("your_path.json", swift_env=your_env)
```

### Initialisation for Swift

```python
controller = RobotController("your_path.json", ros_client=client)
```
If both a `swift_env` and `ros_client` are specified, the simulated environment will be selected
### Running a Simulation

```python
simulation_actions = controller.simulation_step()
```
`simulation_actions` returns the current trajectory step, as well as information on actions required beyond the robot,
like moving an object or stopping the simulation.
### Baking and Playback

#### To save robot bakes:

```python
controller = RobotController("your_path.json", swift_env=your_env, bake="bake_name")
 ```
#### To play back saved bakes:

```python
controller.playback_bake("your_bake_file")  # Or a ROSbag
```
