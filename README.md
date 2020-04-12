# HackRover Project

## Dependencies

* ROS Kinetic
* Raspbian Stretch (Probably works on Ubuntu as well but this is untested)

# Usage

**Note:** Before runnening any of the following commands, do the following steps:

1. Open a new terminal and navigate to the root project directory `test_ws`
2. Run the command `roscore`
3. Each individual command below must be run in its own terminal windiow. Each command must also be run from inside `test_ws`

## Running Teleop

Run both drivebase and arm teleop with one command:

`roslaunch py_pkg teleop.launch`

Run both drivebase and arm teleop in debugging mode. This wordks the same as the above command but also prints info to the console:

```
rosrun py_pkg usr_input.py
rosrun py_pkg control_output.py
```

