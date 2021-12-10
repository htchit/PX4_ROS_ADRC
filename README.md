# PX4_ROS_ADRC
 ADRC position controller for PX4 with ROS and MAVROS

Coded by htchit

## Usage

```shell
roslaunch mavros px4.launch
roslaunch adrc adrc_param.launch
```

## Topics

### Subscribe

/mavros/local_position/odom

Get MAV's motion data

/bot_motion_expect

Get expected position

### Publish

/mavros/setpoint_raw/local

MAV control interface



## Tune

There're 5 params in total:

r

h

N

omega

kp

kd
