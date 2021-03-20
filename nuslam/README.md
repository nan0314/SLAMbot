# Nuslam
Extended Kalman Filter SLAM for the turtlebot
* `<roslaunch nuslam slam.launch>` to visualize controlled robot in rviz with SLAM with data association.
* `<roslaunch nuslam landmark_detect.launch>` to simulate lidar in simulated environment
* `<roslaunch nuslam unknown_data_assoc.launch>` to simulate robot running SLAM without data association

Final odometry error: 0.45006, 0.57283, 0.15998 (x,y,theta)
Final SLAM error: 0.02028, 0.0295, 0.03091 (x,y,theta)
