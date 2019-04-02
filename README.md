# jaco-mpc

This repository basically reimplements the parallel motion planning architecture [1] on a Kinova-jaco arm to guarantee both safety and efficiency. The implementation is done in a simulation platform——vrep, but it can also be easily transferred to a real Kinova arm. 


In the motion planning problem, the end effecter is required to fetch a dynamic target, while collision is required to be avoided between the arm and a dynamic obstacle. Considering no static obstacle, global planning can be simply handled in real time with a trajectory generator; local planner leverages Safety Set Algorithm(SSA)[2] to modify the trajectory dynamically to ensure safety. 

The motion planning problem is formulated in a kinematic form. On a real robot, the embedded controller has taken care of the low-level control, and thus the end effector can track the planner trajectory by giving velocity commands. In vrep, even though we don't have a great low-level controller, we still ignore the effect of dynamics by throwing away the robot hand.

[1] https://arxiv.org/pdf/1808.03983.pdf

[2] http://www.cs.cmu.edu/~cliu6/files/dscc14.pdf

## Dependencies

- ros kinetic
- vrep 3.2.1

## Contents
The repository consists of two folders, kinova-ros-modified and kinova_planning, and one "ttt" file.

### kinova-ros-modified

kinova-ros-modified is modified from official ros packages of Kinova-jaco. The ros package of greatest interest is kinova_driver. The source code files modified are:

- src
  - kinova_fake_joint_trajectory_controller.cpp
  - joint_trajectory_action
    - fake_joint_trajectory_action_server.cpp
  - nodes
    - kinova_arm_driver.cpp
    
### kinova_planning

kinova_planning includes the main program which solves the motion planning problem as well as the launch files to start the whole process. The files mainly come into effect are:

- src
  - base_planning.cpp
- scripts
  - qp_solver.py
- launch
  - kinova_planning.launch
  - trajectory_follower.launch

### kinova.ttt

kinova.ttt can be opened in vrep. It builds the scene and serves as the plant in the planning simulation.

## How to use

1. Open kinova.ttt with vrep and start the simulation.
2. Launch trajectory_follower.launch
3. Launch kinova_planning.launch
