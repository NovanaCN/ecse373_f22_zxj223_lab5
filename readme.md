
### Final project

The remaining semester will be to fill orders for the ARIAC 2019 challenge. The ROS package will be built upon the previous two laboratories. Accomplishing this task will require all the
elements that have been parts of previous laboratories. It is time to build a robot using this material.


The deliverables for this project will be the typical ROS package and accompanying README.md file as well as a demonstration of the package to the course instructor.


### Run the simulation
```
roslaunch ecse_373_ariac ecse_373_ariac.launch python:=false &
```

Designed to interface with the ARIAC environment to control simulations and locate ordered parts. After installing and building the package, run rosrun cwru_ ecse_ 373_ submission ariac_ Interface Start the node. You may need to cancel the simulation using Gazebo. The arm will respond to the order by loading the required product to AGV. Note: This software package has a startup file, but because the araic startup script has known problems, using the startup file in this software package to start the simulation will prevent the arm from running.