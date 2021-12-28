
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
![](https://github.com/Ayush1285/quadrotor_navigation/blob/main/results/astarrviz.mp4)


## On-going Work
* Developing mapping module.
* Integrating px4 offboard mode + mavros to track the trajectory generated by planners.



