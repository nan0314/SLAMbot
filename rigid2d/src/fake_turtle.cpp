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

ros::Publisher odom_pub;
static rigid2d::DiffDrive turtle;
static std::string odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint;
double frequency = 1000;


void velCallback(const geometry_msgs::Twist::ConstPtr& msg){

    rigid2d::Twist2D desired_twist;
    std::vector<double> cmd;
    sensor_msgs::JointState js;

    auto dt = 1.0/frequency;

    desired_twist.dth = msg->angular.z*dt;
    desired_twist.dx = msg->linear.x*dt;
    desired_twist.dy = 0;

    cmd = turtle.vel_update(desired_twist);

    js.header.stamp = ros::Time::now();
    js.name = {left_wheel_joint, right_wheel_joint};
    js.position = {cmd[0], cmd[1]};

    odom_pub.publish(js);



    
    return;
}

int main(int argc, char **argv)
{
    // initialize node/node handles
    ros::init(argc, argv, "odometer");
    ros::NodeHandle n;

    // read parameters from parameter server
    double wb,r;

    ros::param::get("~left_wheel_joint",left_wheel_joint);
    ros::param::get("~right_wheel_joint",right_wheel_joint);
    ros::param::get("~wheel_base",wb);
    ros::param::get("~wheel_radius",r);
    wb/=2;

    // set up publishers and subscribers
    odom_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
    ros::Subscriber vel_sub = n.subscribe("turtle1/cmd_vel", 1000, velCallback);

    // set publishing frequency
    ros::Rate loop_rate(1000);

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