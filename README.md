This package is an example of a drone flight using PX4 SITL and Gazebo.

+ Dependencies
  + Gazebo
  + PX4
  + ROS and MAVROS packages

### Resources

+ [Setup installation](https://clover.coex.tech/en/simulation_native.html)


### Compile

`catkin_make --pkg flight_test`

### Launch

First launch the simulation using the launch file provided:

```
$ source <your_path>/catkin_ws/devel/setup.bash
$ roslaunch clover_simulation simulator.launch
```

In another terminal, launch this node for executing the mission:

```
$ source <your_path>/catkin_ws/devel/setup.bash
rosrun flight_test flight_test
```

In, yet, another terminal, launch this node for landing the drone:

```
$ source <your_path>/catkin_ws/devel/setup.bash
rosrun flight_test cmd_hold
```
