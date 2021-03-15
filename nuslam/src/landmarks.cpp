/// \file
/// \brief Node provides a kinematic simulation of a differential drive robot
///
/// PARAMETERS:
///     frequency (double): ROS loop frequency
///     wheel_base (double): distance between wheels on diffdrive robot
///     wheel_radius (double): radius of wheels on diffdrive robot
///     left_wheel_joint (string): Name of left wheel joint frame
///     right_wheel_joint (string): Name of right wheel joint frame
/// PUBLISHES:
///     joint_states (sensor_msgs/JointState): messages hold Joint State for each non-fixed joint in the robot
/// SUBSCRIBES:
///     turtle1/cmd_vel (geometry_msgs/Twist): angular/linear velocity commands
/// SERVICES:
///     No services

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "nuslam/nuslam.hpp"
#include <armadillo>
#include <vector>
#include <string>

static ros::Publisher marker_pub;         // Odometry state publisher
static double frequency;                // Ros loop frequency
static double min_range;
static double max_range;
static double angle_inc;
static double samples;
static double tube_radius;
static std::string turtle_frame_id;


/// \brief recieve velocity command and cause fake_turtle to move 
/// to fulfill velocity command
/// \param msg geometry_msg/Twist pointer holding velocity command
void detectCallback(const sensor_msgs::LaserScan& msg){


    ///////////////////////////
    // Find Clusters of Points
    ///////////////////////////

    std::vector<std::vector<geometry_msgs::Point>> clusters;
    std::vector<float> ranges = msg.ranges;
    
    clusters = nuslam::findClusters(ranges,max_range,min_range);

    std::cout << "\r" << clusters.size() << std::endl;


    ///////////////////////////
    // Detrmine Tubes
    ///////////////////////////

    std::vector<std::vector<geometry_msgs::Point>> circles;
    for (auto points : clusters){
        if (nuslam::classifyCircle(points)){
            circles.push_back(points);
        }
    }

    std::cout << "\r" << circles.size() << std::endl << std::endl;


    ///////////////////////////
    // Find Tube Locations
    ///////////////////////////

    visualization_msgs::Marker tube;
    visualization_msgs::MarkerArray tube_array;

    for (auto circle : circles){

        tube = nuslam::fitCircle(circle);
        tube.header.stamp = ros::Time::now();
        tube.header.frame_id = turtle_frame_id;
        tube.scale.x = 2*tube_radius;
        tube.scale.y = 2*tube_radius;
        tube_array.markers.push_back(tube);
    }

    marker_pub.publish(tube_array);
        


    return;
}

int main(int argc, char **argv)
{
    // initialize node/node handles
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle n;

    // read parameters from parameter server

    n.getParam("frequency",frequency);
    n.getParam("tube_radius",tube_radius);
    n.getParam("max_range",max_range);
    n.getParam("min_range",min_range);
    n.getParam("angle_inc",angle_inc);
    n.getParam("samples",samples);
    n.getParam("turtle_frame_id",turtle_frame_id);

    // set up publishers and subscribers
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("real_sensor", frequency);
    ros::Subscriber scan_sub = n.subscribe("scan", 10, detectCallback);

    // set publishing frequency
    ros::Rate loop_rate(frequency);


    int count = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
  }


  return 0;
}