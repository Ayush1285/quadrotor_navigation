
## Objectives
* To implement autonomous navigation in unknown environment, given the desired goal.

## Path Planning

A* <img src="https://github.com/Ayush1285/quadrotor_navigation/blob/main/results/astar.png" height="400"> | A* with collision cost <img src="https://github.com/Ayush1285/quadrotor_navigation/blob/main/results/astarwithcollisioncost.png" height="400">
-------------------|---------------------

#### A* planner integrated with RVIZ GUI
![](https://github.com/Ayush1285/quadrotor_navigation/blob/main/results/astar_rviz.gif)


## On-going Work
* Developing mapping module.
* Integrating px4 offboard mode + mavros to track the trajectory generated by planners.


### Acknowledgements
* I have used [Cubic Spline](https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/CubicSpline) module for generating spline through nodes.

