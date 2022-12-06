# ECE470_Project Smart Assembly Robot

This repository contains the code and assets used to simulate our assembly robot.

Instructions for simulating in Gazebo simulators are below.

## Gazebo Instructions
### Setup
Clone the repository and then run:
```
cd ECE470_Project
catkin_make
```

This should create a `build` and `devel` folder.

### Run
To run the gazebo simulation, run the following commands in the `ECE470_Project` directory:
```
source devel/setup.bash
roslaunch ur3_driver ur3_gazebo.launch
```

This should launch Gazebo with the UR3, which is equiped with a suction gripper.

Open another terminal, navigate to the `ECE470_Project` directory and run:

```
source devel/setup.bash
rosrun lab2pkg_py lab2_spawn.py
```
This will load all the experimental setup, including table with four legs, screw box with M8 screws, human and obstacle. Then run:

```
rosrun lab5pkg_py lab5_exec.py --simulator True
```
This will activate the camera and detect the position of the M8 screw, which is colored in green. Then the gripper will pick up the M8 screws and move them to the holes in the corner of the table.

![Demo figure](https://github.com/D-YF/ECE470_Project/blob/main/figures/exp_setup.png)

[Watch our Robot Promotion Video HERE!](https://www.youtube.com/watch?v=t5e-W_g8SfM)
