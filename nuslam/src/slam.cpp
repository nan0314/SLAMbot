/// \file
/// \brief Publishes Odometry messages in a standard ROS way-- updates internal odometry state,
///        publishes a nav_msgs/Odometry message on the odom topic, and broadcasts the transform 
///        between the odometry frame and the body frame on /tf using a tf2 broadcaster

///
/// PARAMETERS:
///     frequency (double): ROS loop frequency
///     odom_frame_id (string): Name of odometry/world frame
///     body_frame_id (string): Name of body/robot frame
///     left_wheel_joint (string): Name of left wheel joint frame
///     right_wheel_joint (string): Name of right wheel joint frame
/// PUBLISHES:
///     odom (nav_msgs/Odometry): messages hold robot state information (3D linear/angular positions)
/// SUBSCRIBES:
///     joint_states (sensor_msgs/JointState): messages hold Joint State for each non-fixed joint in the robot
/// SERVICES:
///     set_pose (rigid2d/set_pose): reset the location of the odometry so that the robot thinks it is at the 
///     requested configuration


#include <ros/ros.h>
#include <rigid2d/set_pose.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "rigid2d/diff_drive.hpp"
#include "nuslam/nuslam.hpp"
#include <vector>
#include <string>

static ros::Publisher odom_pub;         // Odometry state publisher
static ros::Publisher odom_path_pub;
static ros::Publisher slam_path_pub;
static nav_msgs::Path odom_path;
static nav_msgs::Path estimated_path;
static double frequency;                // Ros loop frequency
static int n = 10;                      // Maximum number of landmarks
static nuslam::Filter filter;
static rigid2d::DiffDrive turtle;       // DiffDrive object to track robot configuration  
static rigid2d::Twist2D u_t;
static std::string odom_frame_id;       // Name of odometry/world frame
static std::string body_frame_id;       // Name of robot/body frame
static std::string left_wheel_joint;    // Name of left wheel joint frame
static std::string right_wheel_joint;   // Name of right wheel joint frame
static arma::vec estimation(3+2*n,arma::fill::ones);



/// \brief Publish a nav_msgs/Odometry message on the odom topic and 
/// broadcast the transform between odom_frame_id and the body_frame_id on /tf
/// when subscriber recieves message.
/// \param msg sensor_msgs/JointState pointer holding joint configurations
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
    // Get the current control
    vector<double> prev = turtle.getEncoders();
    vector<double> dphi = {phi[0]-prev[0],phi[1]-prev[1]}; 
    u_t = turtle.control2twist(dphi);

    rigid2d::Twist2D dq = turtle.update(phi);

    ///////////////////////////////
    // Odometry
    ///////////////////////////////


    //since all odometry is 6DOF we'll need a quaternion created from yaw
    tf2::Quaternion odom_q;
    odom_q.setRPY(0, 0, turtle.getTh());

    // initialize odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_id;
    geometry_msgs::Quaternion odom_quat = tf2::toMsg(odom_q); // convert tf2 quaternion to geometry_msg


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

    // Add pose to path
    geometry_msgs::PoseStamped ps;
    ps.pose = odom.pose.pose;
    odom_path.poses.push_back(ps);

    // publish the messages
    odom_pub.publish(odom);
    odom_path_pub.publish(odom_path);

    ////////////////////////////////
    // Transforms
    ////////////////////////////////

    // initialize tf2 transform broadcaster
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = odom_frame_id;
    transformStamped.child_frame_id = body_frame_id;

    // set translational information
    transformStamped.transform.translation.x = turtle.getX();
    transformStamped.transform.translation.y = turtle.getY();
    transformStamped.transform.translation.z = 0.0;
    
    // set rotational information
    transformStamped.transform.rotation.x = odom_q.x();
    transformStamped.transform.rotation.y = odom_q.y();
    transformStamped.transform.rotation.z = odom_q.z();
    transformStamped.transform.rotation.w = odom_q.w();

    // broadcast transform to tf2
    br.sendTransform(transformStamped);
    
    return;
}


void slamCallback(const visualization_msgs::MarkerArray msg){

    ///////////////////////////////
    // SLAM
    ///////////////////////////////

    // Format prediction from odometry
    estimation(0) = turtle.getTh();
    estimation(1) = turtle.getX();
    estimation(2) = turtle.getY();


    // Propogate the uncertainty/Feed the prediction to the filter
    ekf.predict(estimation);

    // Perform SLAM to refine estimation
    arma::vec z_i(2);
    int j;

    for (auto marker : msg){

        z_i(0) = marker.pose.position.x;
        z_i(1) = marker.pose.position.y;   
        j = marker.id;

        estimation = update(u_t,z_i,j);
    }


    // Format estimated state into pose object
    geometry_msgs::Pose estimated_pose;
    estimated_pose.orientation.z = estimation(0);
    estimated_pose.position.x = estimation(1);
    estimated_pose.position.y = estimation(2);

    // Add pose to path
    geometry_msgs::PoseStamped ps;
    ps.pose = estimated_pose;
    odom_path.poses.push_back(ps);

    estimated_path.poses.push_back(ps);
    slam_path_pub.publish(estimated_path);
}



/// \brief reset the location of the odometry so that the robot thinks it is at the 
/// requested configuration
/// \param req - set_pose request (user specified location)
/// \param res - service response/output object (unused)
bool set_pose(rigid2d::set_pose::Request  &req,
            rigid2d::set_pose::Response &res )
{
    // Update DiffDrive object
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
    n.getParam("frequency",frequency);
    wb/=2;

    // set up publishers and subscribers
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", frequency);
    odom_path_pub = n.advertise<nav_msgs::Path>("odom_path",frequency);
    slam_path_pub = n.advertise<nav_msgs::Path>("slam_path",frequency);
    ros::Subscriber joint_sub = n.subscribe("joint_states", frequency, stateCallback);
    ros::Subscriber sensor_sub = n.subscribe("fake_sensor",10,slamCallback);

    // set up services
    ros::ServiceServer service = n.advertiseService("set_pose", set_pose);

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