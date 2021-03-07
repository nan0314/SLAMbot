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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "rigid2d/diff_drive.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <armadillo>
#include <nav_msgs/Path.h>
#include <vector>
#include <string>
#include <random>
#include <cmath>

static ros::Publisher odom_pub;         // Odometry state publisher
static ros::Publisher tube_pub;         // Publishes tube locations
static ros::Publisher path_pub;
static double frequency;                // Ros loop frequency
static nav_msgs::Path path;
static rigid2d::DiffDrive turtle;       // DiffDrive object to track robot configuration  
static std::string world_frame_id;       // Name of odometry/world frame
static std::string turtle_frame_id;       // Name of robot/body frame
static std::string left_wheel_joint;    // Name of left wheel joint
static std::string right_wheel_joint;   // Name of right wheel joint
static std::normal_distribution<double> wheel_slip;
static std::normal_distribution<double> trans_noise;
static std::normal_distribution<double> rot_noise;
static std::normal_distribution<double> u;
static arma::mat L(2,2);
static double max_range;
static double trans_var;
static std::vector<double> tube_var;
static double rot_var;
static double slip_min;
static double slip_max;
static double tube_radius;
static std::vector<double> tube_x, tube_y;
static std::vector<double> recorded_angles = {0,0};


std::mt19937 & get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }


bool inRange(geometry_msgs::Pose turtle_pose, geometry_msgs::Pose tube_pose){
    double dist_squared = pow((turtle_pose.position.x-tube_pose.position.x),2) + pow(turtle_pose.position.y-tube_pose.position.y,2);
    double dist = pow(dist_squared,0.5);
    if (dist < max_range){
        return true;
    } else {
        return false;
    }
}

bool collided(){

    for (int i = 0; i<tube_x.size(); i++){
        double dist_squared = pow((turtle.getX()-tube_x[i]),2) + pow(turtle.getY()-tube_y[i],2);
        double dist = pow(dist_squared,0.5);
        if (dist<=tube_radius+.08){
            return true;
        }
    }

    return false;
}


/// \brief recieve velocity command and cause fake_turtle to move 
/// to fulfill velocity command
/// \param msg geometry_msg/Twist pointer holding velocity command
void velCallback(const geometry_msgs::Twist::ConstPtr& msg){

    rigid2d::Twist2D desired_twist;
    rigid2d::Twist2D noisy_twist;
    std::vector<double> desired_dphi;

    sensor_msgs::JointState js;
    
    // Add Gaussian noise to the velocity command
    double ang_vel = msg->angular.z + rot_noise(get_random());
    double trans_vel = msg->linear.x + trans_noise(get_random());

    // Update fake turtle based on commanded frequency
    auto dt = 1.0/frequency;
    desired_twist.dth = ang_vel*dt;
    desired_twist.dx = trans_vel*dt;
    desired_twist.dy = 0;

    desired_dphi = turtle.twist2control(desired_twist);
    recorded_angles[0] += desired_dphi[0];
    recorded_angles[1] += desired_dphi[1];

    // Calculate wheel_slip
    std::vector<double> omega = turtle.twist2control(desired_twist); // Get wheel speed
    double etaleft = wheel_slip(get_random());
    double etaright = wheel_slip(get_random());
    
    omega[0] *= etaleft;
    omega[1] *= etaright;
    noisy_twist = turtle.control2twist(omega);

    if (!collided()){
        turtle.vel_update(noisy_twist);
    }

    // publish to joint state topic (update odometry)
    js.header.stamp = ros::Time::now();
    js.name = {left_wheel_joint, right_wheel_joint};
    js.position = {recorded_angles[0], recorded_angles[1]};

    odom_pub.publish(js);

    ////////////////////////////////
    // Publish to navigation path
    ////////////////////////////////

    // Get robot pose
    geometry_msgs::PoseStamped pose_stamp;
    geometry_msgs::Pose current_pose;
    current_pose.position.x = turtle.getX();
    current_pose.position.y = turtle.getY();
    current_pose.orientation.z = turtle.getTh();
    pose_stamp.pose = current_pose;

    // Append pose to path
    path.poses.push_back(pose_stamp);
    path_pub.publish(path);


    ////////////////////////////////
    // Publish Tube Locations
    ////////////////////////////////

    // Set up MarkerArray
    visualization_msgs::Marker tube;
    geometry_msgs::Pose tube_pose;
    visualization_msgs::MarkerArray tube_array;

    for (int i = 0; i<tube_x.size(); i++){
        
        // Get the tube location
        arma::vec u_vec(2);
        u_vec(0) = u(get_random());
        u_vec(1) = u(get_random());
        arma::vec tube_noise = L*u_vec;
        tube_pose.position.x = tube_x[i] + tube_noise(0) - turtle.getX();
        tube_pose.position.y = tube_y[i] + tube_noise(1) - turtle.getY();

        // Set up the marker msg for the tube;
        tube.header.stamp = ros::Time::now();
        tube.header.frame_id = turtle_frame_id;
        tube.ns = "real";
        tube.id = i;
        tube.type = visualization_msgs::Marker::CYLINDER;
        tube.pose = tube_pose;
        tube.scale.x = 2*tube_radius;
        tube.scale.y = 2*tube_radius;
        tube.scale.z = 1;
        tube.color.r = 80./255.;
        tube.color.g = 220./255.;
        tube.color.b = 100./255.;       
        tube.color.a = 1;

        if (inRange(current_pose,tube_pose)){
            tube.action = 0;
        } else {
            tube.action = 2;
        }

        // Append it to Markers message
        tube_array.markers.push_back(tube);

    }

    tube_pub.publish(tube_array);


    ////////////////////////////////
    // Broadcast transform
    ////////////////////////////////

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    tf2::Quaternion odom_q;
    odom_q.setRPY(0, 0, turtle.getTh());

    // initialize tf2 transform broadcaster
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = world_frame_id;
    transformStamped.child_frame_id = turtle_frame_id;

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

int main(int argc, char **argv)
{
    // initialize node/node handles
    ros::init(argc, argv, "tube_world");
    ros::NodeHandle n;

    // read parameters from parameter server
    double wb, r;
    
    ros::param::get("~left_wheel_joint",left_wheel_joint);
    ros::param::get("~right_wheel_joint",right_wheel_joint);
    ros::param::get("~wheel_base",wb);
    ros::param::get("~wheel_radius",r);
    n.getParam("world_frame_id",world_frame_id);
    n.getParam("turtle_frame_id",turtle_frame_id);
    n.getParam("frequency",frequency);
    n.getParam("tube_var",tube_var);
    n.getParam("trans_var",trans_var);
    n.getParam("rot_var",rot_var);
    n.getParam("slip_min",slip_min);
    n.getParam("slip_max",slip_max);
    n.getParam("tube_x",tube_x);
    n.getParam("tube_y",tube_y);
    n.getParam("tube_radius",tube_radius);
    n.getParam("max_range",max_range);

    // Set up normal distributions
    std::normal_distribution<double> d((slip_max+slip_min)/2,(slip_max - (slip_max+slip_min)/2));
    wheel_slip = d;
    std::normal_distribution<double> d1(0.0,trans_var);
    trans_noise = d1;
    std::normal_distribution<double> d2(0.0,rot_var);
    rot_noise = d2;
    std::normal_distribution<double> d3(0.0,1);
    u = d3;

    arma::mat Q(2,2);
    Q(0,0) = tube_var[0];
    Q(0,1) = tube_var[1];
    Q(1,0) = tube_var[2];
    Q(1,1) = tube_var[3];

    L = chol(Q);
    
    wb/=2;

    // set up publishers and subscribers
    odom_pub = n.advertise<sensor_msgs::JointState>("joint_states", frequency);
    tube_pub = n.advertise<visualization_msgs::MarkerArray>("fake_sensor",frequency,true);
    ros::Publisher truth_pub = n.advertise<visualization_msgs::MarkerArray>("ground_truth",frequency,true);
    path_pub = n.advertise<nav_msgs::Path>("real_path",frequency);
    ros::Subscriber vel_sub = n.subscribe("turtle1/cmd_vel", 10, velCallback);

    // set publishing frequency
    ros::Rate loop_rate(frequency);

    // Initialize differintial drive robot
    turtle = rigid2d::DiffDrive(wb,r);

    // Set Path header
    path.header.stamp = ros::Time::now();
    path.header.frame_id = world_frame_id;

    // Set up MarkerArray
    visualization_msgs::Marker tube;
    geometry_msgs::Pose tube_pose;
    visualization_msgs::MarkerArray tube_array;

    for (int i = 0; i<tube_x.size(); i++){

        // Get the tube location
        tube_pose.position.x = tube_x[i];
        tube_pose.position.y = tube_y[i];

        // Set up the marker msg for the tube;
        tube.header.stamp = ros::Time::now();
        tube.header.frame_id = world_frame_id;
        tube.ns = "real";
        tube.id = i;
        tube.type = 3;
        tube.action = 0;
        tube.pose = tube_pose;
        tube.scale.x = 2*tube_radius;
        tube.scale.y = 2*tube_radius;
        tube.scale.z = 1;
        tube.color.r = 1;
        tube.color.g = 1;
        tube.color.b = 1;
        tube.color.a = 1;

        // Append it to Markers message
        tube_array.markers.push_back(tube);

    }
    truth_pub.publish(tube_array);

    int count = 0;
    while (ros::ok())
    {
                
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
  }


  return 0;
}