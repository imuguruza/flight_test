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
OR you can use a serial port and send a stop command:

```
rosrun flight_test cmd_stop_serial
# Using a FTDI with RX and TX connected,
# in another terminal launch a stop string to trigger the previous ROS node:
echo "stop" > /dev/ttyUSB0
```

This latest step can be replaced by a Companion Computer
