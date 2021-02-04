#include <ros/ros.h>
#include <rigid2d/set_pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "rigid2d/diff_drive.hpp"
#include <vector>
#include <string>

static ros::Publisher odom_pub;
static sensor_msgs::JointState::ConstPtr phi;
static rigid2d::DiffDrive turtle;
static std::string odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint;


void stateCallback(const sensor_msgs::JointState::ConstPtr& msg){

    using std::vector;
    vector<double> phi{0,0};

    // Update internal odometry state

    for (int i = 0; i<msg->name.size(); i++){    // Get current wheel angles

        if (msg->name[i] == left_wheel_joint){
            phi[0] = msg->position[i];
        } else if (msg->name[i] == right_wheel_joint) {
            phi[1] = msg->position[i];
        }
    }

    rigid2d::Twist2D dq = turtle.update(phi);


    ///////////////////////////////
    // Publish odometry message
    ///////////////////////////////


    //since all odometry is 6DOF we'll need a quaternion created from yaw
    tf2::Quaternion odom_q;
    odom_q.setRPY(0, 0, turtle.getTh());


    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_id;

    geometry_msgs::Quaternion odom_quat = tf2::toMsg(odom_q);


    //set the position
    odom.pose.pose.position.x = turtle.getX();
    odom.pose.pose.position.y = turtle.getY();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = body_frame_id;
    odom.twist.twist.linear.x = dq.dx;
    odom.twist.twist.linear.y = dq.dy;
    odom.twist.twist.angular.z = dq.dth;

    // publish the message
    odom_pub.publish(odom);


    ////////////////////////////////
    // Broadcast transform
    ////////////////////////////////

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    ROS_INFO_STREAM(dq.dy);
    

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = odom_frame_id;
    transformStamped.child_frame_id = body_frame_id;
    transformStamped.transform.translation.x = turtle.getX();
    transformStamped.transform.translation.y = turtle.getY();
    transformStamped.transform.translation.z = 0.0;
    
    transformStamped.transform.rotation.x = odom_q.x();
    transformStamped.transform.rotation.y = odom_q.y();
    transformStamped.transform.rotation.z = odom_q.z();
    transformStamped.transform.rotation.w = odom_q.w();

     br.sendTransform(transformStamped);
    
    return;
}

bool set_pose(rigid2d::set_pose::Request  &req,
            rigid2d::set_pose::Response &res )
{
    // Set
    turtle.set_pose(req.th,req.x,req.y);

    ///////////////////////////////
    // Publish odometry message
    ///////////////////////////////

    nav_msgs::Odometry odom;
    odom.header.frame_id = odom_frame_id;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    tf2::Quaternion odom_q;
    odom_q.setRPY(0, 0, turtle.getTh());
    geometry_msgs::Quaternion odom_quat = tf2::toMsg(odom_q);


    //set the position
    odom.pose.pose.position.x = turtle.getX();
    odom.pose.pose.position.y = turtle.getY();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = body_frame_id;
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;

    // publish the message
    odom_pub.publish(odom);

    return true;
}

int main(int argc, char **argv)
{
    // initialize node/node handles
    ros::init(argc, argv, "odometer");
    ros::NodeHandle n;

    // read parameters from parameter server
    double wb,r;

    ros::param::get("~odom_frame_id",odom_frame_id);
    ros::param::get("~body_frame_id",body_frame_id);
    ros::param::get("~left_wheel_joint",left_wheel_joint);
    ros::param::get("~right_wheel_joint",right_wheel_joint);
    n.getParam("wheel_base",wb);
    n.getParam("wheel_radius",r);
    wb/=2;

    // set up publishers and subscribers
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Subscriber joint_sub = n.subscribe("joint_states", 1000, stateCallback);

    // set up services
    ros::ServiceServer service = n.advertiseService("set_pose", set_pose);

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