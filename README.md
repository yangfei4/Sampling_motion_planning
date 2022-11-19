# ECE470_Project
### Note: it's just a starting code now.

This repository contains the code and assets used to simulate our assembly robot.

Instructions for simulating in Gazebo simulators are below.

## Gazebo Instructions
### Setup
Clone the repository and then run:
```
catkin_make
```

This should create a `build` directory.

### Run
To run the gazebo simulation, run the following commands in the workspace directory:
```
source devel/setup.bash
roslaunch ur3_driver ur3_gazebo.launch
```

This should launch Gazebo with the UR3.

Open another terminal, navigate to the workspace directory and run:
```
source devel/setup.bash
rosrun lab2pkg_py lab2_spawn.py
rosrun lab2pkg_py lab2_exec.py --simulator True
```

This will launch the test world which contains three small cubes and perform the Hanoi Lab program in the simulator.
