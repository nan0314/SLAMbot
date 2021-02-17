/// \file
/// \brief publishes commands that let the robot drive in a circle of a specified radius at a specified speed
///
/// PARAMETERS:
///     speed (double): specified speed
///     radius (double): specified radius
/// PUBLISHES:
///     cmd_vel (geometry_msgs/Twist): angular/linear velocity commands
/// SUBSCRIBES:
///     No subscribers
/// SERVICES:
///     control (nuturtle_robot/control): causes the robot to travel either clockwise, counter clockwise, or stop

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nuturtle_robot/control.h>
#include "rigid2d/diff_drive.hpp"
#include <vector>

// turtle state machine states
enum Control{
    STOP,
    COUNTER,
    CLOCK,
};

///////////////////////////////
// Global Varibles
///////////////////////////////

static ros::Publisher vel_pub;         // Odometry state publisher
static double frequency = 100;          // Ros loop frequency
static Control state = CLOCK;           // Direction for robot to move in


bool control(nuturtle_robot::control::Request  &req,
            nuturtle_robot::control::Response &res )
{
    if (req.command == "cw"){
        state = CLOCK;
    } else if (req.command == "ccw"){
        state = COUNTER;
    } else if (req.command == "stop"){
        state = STOP;
    } else {
        ROS_INFO_STREAM("\rInvalid command-- please call control service again with valid command (cw, ccw, stop)");
    }

    return true;
}

int main(int argc, char **argv)
{
    // initialize node/node handles
    ros::init(argc, argv, "follow_circle");
    ros::NodeHandle n;

    // get private parameters from parameter server
    double v,r;
    ros::param::get("~speed",v);        // (m/s)
    ros::param::get("~radius",r);       // (m)

    // set up services
    ros::ServiceServer service = n.advertiseService("control", control);

    // set up publishers and subscribers
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", frequency);

    // initialize message
    geometry_msgs::Twist out;

    // set publishing frequency
    ros::Rate loop_rate(frequency);

    int count = 0;
    while (ros::ok())
    {
                
        ros::spinOnce();

        if (state == CLOCK){
            out.linear.x = v;
            out.angular.z = v/r;
        } else if (state == COUNTER){
            out.linear.x = -v;
            out.angular.z = -v/r;
        } else if (state == STOP) {
            out.linear.x = 0;
            out.angular.z = 0;
        }

        vel_pub.publish(out);

        loop_rate.sleep();
        ++count;
  }

  return 0;
}