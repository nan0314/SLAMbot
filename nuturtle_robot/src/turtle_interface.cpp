/// \file
/// \brief TODO
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
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>
#include "rigid2d/diff_drive.hpp"
#include <vector>
#include <string>

///////////////////////////////
// Constants
///////////////////////////////

const float MAX_ANG_VEL = 6.67;         // Maximum angular velocity in rad/s

///////////////////////////////
// Global Varibles
///////////////////////////////

static ros::Publisher vel_pub;         // Odometry state publisher
static ros::Publisher joint_pub;
static int frequency = 100;          // Ros loop frequency
static rigid2d::DiffDrive turtle;       // DiffDrive object to track robot configuration  
static std::string left_wheel_joint;    // Name of left wheel joint
static std::string right_wheel_joint;   // Name of right wheel joint



/// \brief reads in a twist and converts it to specific wheel commands that make the turtlebot move
/// \param msg geometry_msg/Twist pointer holding velocity command
void velCallback(const geometry_msgs::Twist::ConstPtr& msg){

    using std::vector;
    using namespace rigid2d;

    // Build Twist2D from geometry_msgs/Twist
    Twist2D velocity_twist;
    velocity_twist.dth = msg->angular.z;
    velocity_twist.dx = msg->linear.x;
    velocity_twist.dy = 0;

    // Convert twist to wheel commands in format [left_wheel_cmd, right_wheel_cmd]
    vector<double> wheel_cmds = turtle.twist2control(velocity_twist);

    // Convert wheel commands to integer values between -256 and 256
    wheel_cmds[0] = wheel_cmds[0]/MAX_ANG_VEL*255;
    wheel_cmds[1] = wheel_cmds[1]/MAX_ANG_VEL*255;

    // Check for saturation
    if (wheel_cmds[0] > 255){
        wheel_cmds[0] = 255;
    } else if (wheel_cmds[1] > 255){
        wheel_cmds[1] = 255;
    } else if (wheel_cmds[0] < -255){
        wheel_cmds[0] = -255;
    } else if (wheel_cmds[1] < -255){
        wheel_cmds[1] = -255;
    } 

    // Publish wheel command
    nuturtlebot::WheelCommands cmd;
    cmd.left_velocity = wheel_cmds[0];
    cmd.right_velocity = wheel_cmds[1];
    vel_pub.publish(cmd);

    return;
}

/// \brief reads in raw encoder data and outputs it as joint angles and velocities
/// \param msg nuturtlebot/SensorData message holding encoder data
void sensorCallback(const nuturtlebot::SensorData msg){
    
    using std::vector;

    // Convert encoder data to radians (12 bit absolute encoder-- 4096increments/2pi)
    vector<double> encoder_rad;

    double left_rad = double(msg.left_encoder)/4096.0*2*rigid2d::PI/10;
    double right_rad = double(msg.right_encoder)/4096.0*2*rigid2d::PI/10;
    encoder_rad.push_back(left_rad);
    encoder_rad.push_back(right_rad);

    // Initialize joint state message
    sensor_msgs::JointState js;
    js.header.stamp = ros::Time::now();



    // publish to joint state topic (update odometry)
    js.name = {left_wheel_joint, right_wheel_joint};
    js.position = {left_rad, right_rad};
    vector<double> prev = turtle.getEncoders();
    js.velocity = {left_rad-prev[0],right_rad-prev[1]};
    joint_pub.publish(js);

    // Update diffdrive object (keep previous angle up to date)
    turtle.update(encoder_rad);

    return;
}

int main(int argc, char **argv)
{
    // initialize node/node handles
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle n;

    ros::param::get("~left_wheel_joint",left_wheel_joint);
    ros::param::get("~right_wheel_joint",right_wheel_joint);

    // set up publishers and subscribers
    vel_pub = n.advertise<nuturtlebot::WheelCommands>("wheel_cmd", frequency);
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 500);
    ros::Subscriber vel_sub = n.subscribe("cmd_vel", 10, velCallback);
    ros::Subscriber sensor_sub = n.subscribe("sensor_data", 10, sensorCallback);

    // set publishing frequency
    ros::Rate loop_rate(frequency);

    // Initialize differintial drive robot
    double wb = 0.16/2;
    double r = 0.033;
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