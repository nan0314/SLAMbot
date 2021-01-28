/// \file turtle_rect.cpp
/// \brief This file is a ROS node used to make a turtle in turtlesim teleport and travel
///         in a user specified rectangle.
///
/// PARAMETERS:
///     max_xdot (double): maximum xvelocity of turtle
///     max_wdot (double): maximum angular velocity
///     frequency (double): control loop frequency
/// PUBLISHES:
///     turtle1/cmd_vel (geometry_msgs/Twist): controls linear and angular velocity of turtle 1
/// SUBSCRIBES:
///     turtle1/pose (turtlesim/Pose): The x, y, theta, linear velocity, and angular velocity of turtle1
/// SERVICES:
///     start (trect/start): Teleports the turtle, draws a trajectory, and follows that trajectory. Takes
///     four integer inputs x, y, w, and h

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <std_srvs/Empty.h>
#include <trect/start.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/TeleportRelative.h>


// Constants
constexpr double PI=3.14159265358979323846; // The value of pi

// turtle state machine states
enum State{
    IDLE,
    BOTTOM,
    RIGHT,
    TOP,
    LEFT,
    ROTATE,
};


// Global Variables
static double max_xdot, max_wdot;       // maximum turtle speeds
static double target;                   // current pose target
static int frequency;                   // frequency of control loop/publishing frequency
static double w;                        // width of rectangle
static double h;                        // height of rectangle
static State state = IDLE;              // current state of turtle initialized to idle
static State prev;                      // previous state of turtle
static turtlesim::PoseConstPtr pose;    // holds pose information of turtle (x, y, theta, linear velocity, and angular velocity)
static ros::ServiceClient abs_tel;      // service client for turtle1/TeleportAbsolute
static ros::ServiceClient rel_tel;      // service client for turtle1/TeleportRelative
static ros::ServiceClient erase;        // service client for clear
static ros::ServiceClient pen_color;    // service client for turtle1/SetPen


/// \brief stores callback from turtlesim::Pose topic
/// \param msg - turtlesim::PoseConstPtr recieved from Pose topic
void poseCallback(const turtlesim::Pose::ConstPtr& msg){
    pose = msg;
    return;
}

/// \brief causes the node to clear the background of the simulator, 
///        draw the desired trajectory, and causes the robot to start following trajectory
/// \param req - request recieved from service call
/// \param response - service response
/// \return true if succesfully completed
bool start(trect::start::Request  &req,
            trect::start::Response &res )
    {
        // Store rectangle size
        w = req.w;
        h = req.h;

        // initialize service classes
        turtlesim::TeleportAbsolute init_pos;
        turtlesim::TeleportRelative pos;
        turtlesim::SetPen color;
        std_srvs::Empty empty;

        // set pen to white
        color.request.r = 255;
        color.request.g = 255;
        color.request.b = 255;
        color.request.width = 3;
        color.request.off = 0;
        pen_color.call(color);

        // place turtle at user specified position
        init_pos.request.x = req.x;
        init_pos.request.y = req.y;
        init_pos.request.theta = 0;
        abs_tel.call(init_pos);

        // clear the background
        erase.call(empty);


        // draw desired trajectory
        pos.request.linear = req.w;     
        rel_tel.call(pos);              // Draws bottom side of rectangle

        pos.request.linear = req.h;     
        pos.request.angular = PI/2;
        rel_tel.call(pos);              // Rotates turtle 90 degrees cc, draws right side

        pos.request.linear = req.w;     
        rel_tel.call(pos);              // Rotates turtle 90 degrees, draws top side

        pos.request.linear = req.h;
        rel_tel.call(pos);              // Rotates turtle 90 degrees, draws left side

        pos.request.linear = 0;     
        rel_tel.call(pos);              // Rotates turtle 90 degrees (returns to start position)

        // change pen color to black
        color.request.r = 0;
        color.request.g = 0;
        color.request.b = 0;
        color.request.width = 3;
        color.request.off = 0;
        pen_color.call(color);

        // cause robot to start following trajectory
        state = BOTTOM;
        target = req.x + w;     // create first target, end of bottom line
        ros::spinOnce();
        return true;
    }

int main(int argc, char **argv)
{
    // initialize node/node handles
    ros::init(argc, argv, "turtle_rect");
    ros::NodeHandle n;

    // read parameters from parameter server
    n.getParam("max_xdot",max_xdot);
    n.getParam("max_wdot",max_wdot);
    n.getParam("frequency",frequency);

    // set up publishers and subscribers
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 1000, poseCallback);

    // set up services
    ros::ServiceServer service = n.advertiseService("start", start);
    abs_tel = n.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
    rel_tel = n.serviceClient<turtlesim::TeleportRelative>("turtle1/teleport_relative");
    pen_color = n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
    erase = n.serviceClient<std_srvs::Empty>("clear");

    // set publishing frequency
    ros::Rate loop_rate(frequency);

    // log parameters from Parameter Server 
    ROS_INFO("max_xdot: %f", max_xdot);
    ROS_INFO("max_wdot: %f", max_wdot);
    ROS_INFO("frequency: %d", frequency);

    int count = 0;
    while (ros::ok())
    {

        // initialize velocity command service class
        geometry_msgs::Twist msg;

        // state machine to move turtle in rectangle
        switch(state){

            // idle case turtle does not move
            case IDLE:

                msg.linear.x = 0;
                msg.angular.z = 0;                
                vel_pub.publish(msg);
                break;

            // Bottom case turtle moves forward until reaching end of bottom line
            case BOTTOM:

                // if target has not been reached move forward else switch to Rotate state
                if(pose->x<target){
                    msg.linear.x = max_xdot;
                    vel_pub.publish(msg);
                } else{
                    state = ROTATE;
                    prev = BOTTOM;
                    target = PI/2;  // sets target to pi/2 radians for Rotate case
                }
                break;

            // Right case turtle moves forward until reaching end of right line
            case RIGHT:
                
                // if target has not been reached move forward else switch to Rotate case
                if(pose->y<target){
                    msg.linear.x = max_xdot;
                    vel_pub.publish(msg);
                } else{
                    state = ROTATE;
                    prev = RIGHT;
                    target = PI;    // sets target to pi radians for Rotate case
                }
                break;

            // Top case turtle moves forward until reaching end of top line
            case TOP:

                // if target has not been reached move forward else switch to Rotate case
                if(pose->x>target){
                    msg.linear.x = max_xdot;
                    vel_pub.publish(msg);
                } else{
                    state = ROTATE;
                    prev = TOP;
                    target = -PI/2; // Sets target to -pi/2 radians for rotate case
                }
                break;
            
            // Left case turtle moves forward until reaching end of left line
            case LEFT:

                // if target has not been reached move forward else switch to Rotate case
                if (pose->y>target){
                    msg.linear.x = max_xdot;
                    vel_pub.publish(msg);
                } else{
                    state = ROTATE;
                    prev = LEFT;
                    target = 0;
                }
                break;

            // Rotate case turtle rotates until target is reached then switches to next case based
            // on the previous case
            case ROTATE:
                if (fabs(pose->theta - target) > 0.01){
                    msg.linear.x = 0;
                    msg.angular.z = max_wdot;
                    vel_pub.publish(msg);
                } else{
                    msg.angular.z = 0;
                    switch(prev){
                        case BOTTOM:
                            state = RIGHT;
                            target = pose->y + h;
                            break;
                        case RIGHT:
                            state = TOP;
                            target = pose->x - w;
                            break;
                        case TOP:
                            state = LEFT;
                            target = pose->y - h;
                            break;
                        case LEFT:
                            state = IDLE;
                            break; 
                    }
                }
                break;
        }
                
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
  }


  return 0;
}