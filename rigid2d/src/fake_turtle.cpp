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
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "rigid2d/diff_drive.hpp"
#include <vector>
#include <string>

static ros::Publisher odom_pub;         // Odometry state publisher
static double frequency;                // Ros loop frequency
static rigid2d::DiffDrive turtle;       // DiffDrive object to track robot configuration  
static std::string odom_frame_id;       // Name of odometry/world frame
static std::string body_frame_id;       // Name of robot/body frame
static std::string left_wheel_joint;    // Name of left wheel joint
static std::string right_wheel_joint;   // Name of right wheel joint


/// \brief recieve velocity command and cause fake_turtle to move 
/// to fulfill velocity command
/// \param msg geometry_msg/Twist pointer holding velocity command
void velCallback(const geometry_msgs::Twist::ConstPtr& msg){

    rigid2d::Twist2D desired_twist;
    std::vector<double> cmd;
    sensor_msgs::JointState js;

    // Update fake turtle based on commanded frequency
    auto dt = 1.0/frequency;
    desired_twist.dth = msg->angular.z*dt;
    desired_twist.dx = msg->linear.x*dt;
    desired_twist.dy = 0;

    cmd = turtle.vel_update(desired_twist);

    // publish to joint state topic (update odometry)
    js.header.stamp = ros::Time::now();
    js.name = {left_wheel_joint, right_wheel_joint};
    js.position = {cmd[0], cmd[1]};

    odom_pub.publish(js);

    return;
}

int main(int argc, char **argv)
{
    // initialize node/node handles
    ros::init(argc, argv, "fake_turtle");
    ros::NodeHandle n;

    // read parameters from parameter server
    double wb,r;

    ros::param::get("~left_wheel_joint",left_wheel_joint);
    ros::param::get("~right_wheel_joint",right_wheel_joint);
    ros::param::get("~wheel_base",wb);
    ros::param::get("~wheel_radius",r);
    n.getParam("frequency",frequency);
    wb/=2;

    // set up publishers and subscribers
    odom_pub = n.advertise<sensor_msgs::JointState>("joint_states", frequency);
    ros::Subscriber vel_sub = n.subscribe("turtle1/cmd_vel", frequency, velCallback);

    // set publishing frequency
    ros::Rate loop_rate(frequency);

    // Initialize differintial drive robot
    turtle = rigid2d::DiffDrive(wb,r);

    int count = 0;
    while (ros::ok())
    {
                
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
  }


  return 0;
}