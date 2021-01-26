/// \file turtle_rect.cpp
/// \brief This file is a ROS node used to make a turtle in turtlesim teleport and travel
///         in a user specified rectangle.
///
/// PARAMETERS:
///     parameter_name (parameter_type): description of the parameter
/// PUBLISHES:
///     turtle1/cmd_vel (geometry_msgs/Twist): controls linear and angular velocity of turtle 1
/// SUBSCRIBES:
///     turtle1/pose (turtlesim/Pose): The x, y, theta, linear velocity, and angular velocity of turtle1
/// SERVICES:
///     start (trect/start): Teleports the turtle, draws a trajectory, and follows that trajectory

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
        target = req.x + w;
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


    int count = 0;
    while (ros::ok())
    {
            
        // log parameters from Parameter Server 
        ROS_INFO_STREAM(max_xdot);
        ROS_INFO_STREAM(max_wdot);
        ROS_INFO_STREAM(frequency);

        // initialize velocity command service class
        geometry_msgs::Twist msg;

        // state machine to move turtle in rectangle
        switch(state){
            case IDLE:

                msg.linear.x = 0;
                msg.angular.z = 0;                
                vel_pub.publish(msg);
                break;
            case BOTTOM:

                if(pose->x<target){
                    msg.linear.x = max_xdot;
                    vel_pub.publish(msg);
                } else{
                    state = ROTATE;
                    prev = BOTTOM;
                    target = PI/2;
                }
                break;
            case RIGHT:
                
                if(pose->y<target){
                    msg.linear.x = max_xdot;
                    vel_pub.publish(msg);
                } else{
                    state = ROTATE;
                    prev = RIGHT;
                    target = PI;
                }
                break;
            case TOP:

                if(pose->x>target){
                    msg.linear.x = max_xdot;
                    vel_pub.publish(msg);
                } else{
                    state = ROTATE;
                    prev = TOP;
                    target = -PI/2;
                }
                break;
            case LEFT:

                if (pose->y>target){
                    msg.linear.x = max_xdot;
                    vel_pub.publish(msg);
                } else{
                    state = ROTATE;
                    prev = LEFT;
                    target = 0;
                }
                break;
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