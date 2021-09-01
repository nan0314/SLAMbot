# Nuturtle Robot
* `<roslaunch nuturtle_robot basic_remote.launch>` to run nodes on the turtlebot
and setup low level control through ROS
# Example Usage
```
roslaunch nuturtle_robot odom_teleop.launch robot:=FranklinTheTurtle.local circle:=false
roslaunch nuturtle_robot odom_teleop.launch robot:=FranklinTheTurtle.local circle:=true
```

