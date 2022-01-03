
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
**Lemniscate Trajectory**

![](https://github.com/Ayush1285/quadrotor_navigation/blob/main/results/lemniscatetraj.gif)

**Circle Trajectory**

![](https://github.com/Ayush1285/quadrotor_navigation/blob/main/results/circletraj.gif)

## Mapping
* Generated Map of cylpillars.world using gmapping package.
* I have used ground truth for odometry information.

<img src="https://github.com/Ayush1285/quadrotor_navigation/blob/main/results/pillarsmap.png" height="400"> 

## Navigation
Autonomous Navigation to given desired goal

**RVIZ**

<img src="https://github.com/Ayush1285/quadrotor_navigation/blob/main/results/navrviz.gif">

**Gazebo**


https://user-images.githubusercontent.com/64409716/147904810-0fa1841c-cde9-49d8-82e0-91144fad8688.mp4



## On-going Work
* Code Cleaning/Optimizing
* Navigation without prior built map

## Future Work
* Implementing different planners
* VIO for Odometry Info

## Acknowledgements
* I have used [B Spline](https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/BSplinePath) module.


