# How to run

## Master PC
```
roslaunch multi_parking_sys operator_one.launch
```
include dynamixel state, cartographer, teleop

## Rbp
```
ssh ubuntu@192.168.0.100
pw:zoqtmxhs0907
```
```
roslaunch rbp4_sys all_pkg.launch
```
run rplidar_a2, dynamixel

## How to save the map(.pbstream)
```
rosservice call /write_state "{filename: '/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/multi_parking_sysck/map/mymap2.pbstream'}"
```

# Turtlebot3 burger setup
[Document - LeeJaewon](https://github.com/Lee-JaeWon/Turtlebot3-burger-backup)<br>
[Document - ROBOTIS](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
