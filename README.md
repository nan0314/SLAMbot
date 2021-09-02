# ME495 Sensing, Navigation and Machine Learning For Robotics

This repository contains packages that enable the odometry and landmark based EKF SLAM for a differential drive robot, several supporting libraries, and testing nodes. The repository has all the resources needed to run either in simulation or on a physical TurtlBot3 burger. Currently, the repository does not support path planning or autonomous driving capabilities.

## Package List

This repository consists of several ROS packages
- nuturtle_description - contains turtle burger urdf and load.launch to launch urdf with optional viewing in rviz
- nurtlesim - Simulates real turtlebot with noise
- nuslam - Extended Kalman Filter SLAM for the turtlebot
- nuturtle_robot - contains code that intercats with with the turtlebot hardware
- rigid2d - A package for handling transformations and other 2D operations for rigid bodies
- trect - test package to make a turtle follow a rectangle trajectory in turtlesim

## Getting Started

### Dependencies

* ROS Noetic (desktop-full)
* [Nuturtlebot package](https://github.com/ME495-Navigation/nuturtlebot.git)

### Installing

These directions assume you have completed installation and setup of ROS Noetic (desktop-full)

* Clone this repository into the src directory of your workspace
* Clone the [Nuturtlebot package](https://github.com/ME495-Navigation/nuturtlebot.git) into the src director of your workspace
* Build and source your workspace

### Executing program

#### Simulation

Run the full SLAM implementation using node groundtruth information to do pose estimation ([Landmark Based EKF SLAM - Simulation Data Known](https://youtu.be/KneLeDTZ0w8))
```
roslaunch nuslam slam.launch
```


This works as a proof of concept for the EKF SLAM as the algorithm is running using the actual ground truth relative locations of the landmarks with respect to the turtlebot with a small amount of noise applied. The red path indicates the odometry which is achieved by dead reckoning alone, the green path is the actual path of the turtlebot, and the blue path is the EKF estimated path of the turtlebot.

Run the full SLAM implementation using actual simulated laser scan data ([Landmark Based EKF SLAM - Simulation Unknown Data Association](https://youtu.be/sqKf19MjhSI))
```
roslaunch nuslam unknown_data_assoc.launch
```

Taking the next step, the input into the EKF SLAM algorithm is generated from simulated laser scan data. The algorithm becomes very sensitive to the perception algorithms to correctly recognize and track landmarks as well as accurately identify the relative distance of the landmark given laser scan data.

#### On Hardware

1. Make sure you are connected via wifi to your Turtlebot3

2. Run the full SLAM implementation on your Turtlebot3 ([Landmark based EKF SLAM - Hardware Unknown Data Association](https://youtu.be/Q8GBbYuN-bo)):

```
roslaunch nuslam unkown_data_assoc.launch robot:=your_turtlebot_hostname.local
```

Finally, the algorithm is deployed on a real turtlebot. With high levels of noise, the landmark inputs are extremely noisy causing poor EKF SLAM performance. More work must be done to address this. See the TODO section.

## TODO

* Further testing for landmark culling to reliably remove false positive landmarks from the state vector
* Implement a more robust method for adding landmarks to the state vector. E.g. require the potential new landmark to be seen three consecutive times before officially adding it to the state vector.
* Implement autonomous planning and driving functionality

## Authors

Nathaniel Nyberg

## Acknowledgments

Inspiration, code snippets, etc.
* This repository was written under the exceptional guidance of Northwestern University Professor Matt Elwin
* [Nuturtlebot package](https://github.com/ME495-Navigation/nuturtlebot.git) provided by [Matt Elwin](https://github.com/m-elwin)



