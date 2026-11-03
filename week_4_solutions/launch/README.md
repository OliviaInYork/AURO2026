# Launch files for Week 4

Each launch file starts the TurtleBot3 simulation in Gazebo, through `ros_gz_sim`, based on
`turtlebot3_world.launch.py` from the `turtlebot3_gazebo` package. Except for task 10, which
includes task 9, each file starts:

- the Gazebo server, running the TurtleBot3 world (`turtlebot3_world.world`), and the Gazebo GUI;
- the robot state publisher;
- the TurtleBot3, spawned at the position below, with its topics bridged between Gazebo and ROS.

Most also start one or more nodes implementing the week's state machines:

| Launch file | Robot starts at (x, y) | Also starts |
|---|---|---|
| `task3.launch.py` | (−2.0, −0.5) | `task3`, from package `week_4` |
| `task4.launch.py` | (−2.0, −0.5) | `task4` |
| `task5.launch.py` | (−2.0, −0.5) | `task5` |
| `task8.launch.py` | (−2.0, −0.5) | `task8_DetectorFSM` (log level `warn`) and `task8_TurtleBot3FSMRC` |
| `task9.launch.py` | (−5.0, −0.5) | nothing else: the TurtleBot3 world, with the robot in a different position |
| `task10.launch.py` | (−5.0, −0.5) | everything in `task9.launch.py`, and RViz2, with the TurtleBot3 configuration from `turtlebot3_gazebo` |
| `task12.launch.py` | (−2.0, −0.5) | `task5`, with its constants `T` and `F` given as launch arguments |

Nodes are from package `week_4_solutions`, unless stated otherwise.

## Running

Build the packages, and source the workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

Then, for example:

```bash
ros2 launch week_4_solutions task3.launch.py
```

## Launch arguments

All launch files accept (`task10.launch.py` passes them on to `task9.launch.py`):

| Argument | Default | Meaning |
|---|---|---|
| `x_pose` | as in the table above | x coordinate where the robot is spawned |
| `y_pose` | as in the table above | y coordinate where the robot is spawned |
| `use_sim_time` | `true` | whether the robot state publisher uses the simulation's clock. This is a default that must set when using ROS2 with Gazebo in simulation. |

`task12.launch.py` also accepts:

| Argument | Default | Meaning |
|---|---|---|
| `T` | `5` | value of the constant `T` of the state machine |
| `F` | `1` | value of the constant `F` of the state machine |

For example:

```bash
ros2 launch week_4_solutions task12.launch.py T:=3 F:=2
ros2 launch week_4_solutions task3.launch.py x_pose:=0.5 y_pose:=0.5
```

`ros2 launch week_4_solutions <file> --show-args` lists the arguments of a launch file.
