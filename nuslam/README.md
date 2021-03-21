# Nuslam
Extended Kalman Filter SLAM for the turtlebot
* `<roslaunch nuslam slam.launch>` to visualize controlled robot in rviz with SLAM with data association.
* `<roslaunch nuslam landmark_detect.launch>` to simulate lidar in simulated environment
* `<roslaunch nuslam unknown_data_assoc.launch>` to simulate robot running SLAM without data association


L.003

thresh (nuslam.cpp line 438) = 45
limit (nuslam.cpp line 439) = 0.01

Final odometry error: 0.45006, 0.57283, 0.15998 (x,y,theta)
Final SLAM error: 0.02028, 0.0295, 0.03091 (x,y,theta)

L.004

thresh = 10000000
limit = 10000

Final odometry position: 0.025593, -0.03307, -0.047818 (x,y,theta)
Final SLAM position: 0.001704, -0.002698, -.136767 (x,y,theta)

I believe the SLAM is better at tracking the robot position than the odometry is, however the SLAM is EXTREMELY
noisy. Additionally, there are many false positives in the landmark locations. Even with the threshold value
was set to INT_MAX for adding a new landmark, false new landmarks would still occur. It is likely that the Q and R value needed to be adjusted so that the calculated dk is of a smaller magnitude. Similarly further tuning Q and R may improve the estimated positioning of the tubes, however positioning of the robot was taken as the priority.

![Video](https://drive.google.com/uc?export=view&id=1Gff8mrHfwtLIIsGf9VYb7K4CB4r73sGq)
