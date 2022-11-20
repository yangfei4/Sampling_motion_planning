# ECE470_Project
### Note: it's just a starting code now.

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

This should launch Gazebo with the UR3, which is equiped with a hexagon-prims screwdriver.

Open another terminal, navigate to the `ECE470_Project` directory and run:

```
source devel/setup.bash
rosrun lab2pkg_py lab2_spawn.py
```
This will import our screw holes box and an M8 screw. Then run:

```
rosrun lab5pkg_py lab5_exec.py --simulator True
```
This will activate the camera and detect the position of the M8 screw, which is green in our project update 2. Then the gripper will pick up the M8 screw and move it to another hole.

![Demo figure](https://github.com/D-YF/ECE470_Project/blob/main/figures/Project_Update_2_Demo.png)

[Watch our project update 2 demo video HERE!](https://www.youtube.com/watch?v=HnOLIwM2B2o)
