
## Objectives
* To implement autonomous navigation in unknown environment, given the desired goal.

## Path Planning

normal A*      |  A* with collision cost near obstacles
:-------------------------:|:-------------------------:
<img src="https://github.com/Ayush1285/quadrotor_navigation/blob/main/results/normalastar.png" height="400"> | <img src="https://github.com/Ayush1285/quadrotor_navigation/blob/main/results/astarwithcollisioncost.png" height="400">

A* after processing with RDP Algorithm   |  Final Trajectory after B Spline approximation
:-------------------------:|:-------------------------:
<img src="https://github.com/Ayush1285/quadrotor_navigation/blob/main/results/astarafterrdp.png" height="400"> | <img src="https://github.com/Ayush1285/quadrotor_navigation/blob/main/results/finaltraj.png" height="400">

#### A* planner integrated with RVIZ GUI

![](https://github.com/Ayush1285/quadrotor_navigation/blob/main/results/astarrviz.gif)

## Trajectory Tracking using PX4 OFFBOARD Control
**Lemniscate Trajectory Following**

![](https://github.com/Ayush1285/quadrotor_navigation/blob/main/results/lemniscatetraj.gif)


## On-going Work
* Mapping 



