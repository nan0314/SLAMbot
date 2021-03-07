/// \file slam.cpp
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
#include <unordered_map>
#include <vector>
#include <string>

static ros::Publisher odom_pub;         // Odometry state publisher
static ros::Publisher odom_path_pub;
static ros::Publisher slam_path_pub;
static ros::Publisher tube_pub;
static nav_msgs::Path odom_path;
static nav_msgs::Path estimated_path;
static double frequency;                // Ros loop frequency
static double tube_radius;
static int max_landmarks = 10;                      // Maximum number of landmarks
static nuslam::Filter filter;
static rigid2d::DiffDrive turtle;       // DiffDrive object to track robot configuration 
static rigid2d::DiffDrive slambot; 
static std::string odom_frame_id;       // Name of odometry/world frame
static std::string body_frame_id;       // Name of robot/body frame
static std::string left_wheel_joint;    // Name of left wheel joint frame
static std::string right_wheel_joint;   // Name of right wheel joint frame
static std::string map_frame_id;
static std::string world_frame_id;
static std::vector<double> cmd = {0,0};
static std::unordered_map<int,int> init;
static arma::vec estimation(3+2*max_landmarks,arma::fill::ones);



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
    cmd = phi;

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

    std::vector<double> prev = slambot.getEncoders();
    std::vector<double> dphi;
    dphi.push_back(cmd[0]-prev[0]);
    dphi.push_back(cmd[1]-prev[1]);
    rigid2d::Twist2D u_t = slambot.control2twist(dphi);
    rigid2d::Twist2D dq = slambot.update(cmd);

    // Perform SLAM to refine estimation
    arma::vec z_i(2);
    int j;

    // Propogate the uncertainty/Feed the prediction to the filter
    estimation = filter.predict(u_t);


    for (auto marker : msg.markers){

        if (marker.action == 2){
            continue;
        }
            
        // Extract the cartesian distances between the landmark and turtle
        std::vector<double> cartesian;
        cartesian.push_back(marker.pose.position.x);
        cartesian.push_back(marker.pose.position.y);

        // Calculate z_i
        std::vector<double> polar = nuslam::cartesian2polar(cartesian);
        z_i(0) = polar[0];
        z_i(1) = rigid2d::normalize_angle(polar[1] - estimation(0));
        j = marker.id;

        if (init.find(j) == init.end()){
            init[j] = 1;
            filter.initialize_landmark(z_i,j);
            estimation = filter.getEstimate();
        }


        // update our estimated state
        estimation = filter.update(u_t,z_i,j);
        estimation(0) = rigid2d::normalize_angle(estimation(0));
    }

    // Set up MarkerArray
    visualization_msgs::Marker tube;
    geometry_msgs::Pose tube_pose;
    visualization_msgs::MarkerArray tube_array;

    // Handle estimated tubes

    for (int j = 0; j<max_landmarks; j++){

         if (init.find(j) == init.end()){
            continue;
        }

        tube_pose.position.x = estimation(3+2*j);
        tube_pose.position.y = estimation(4+2*j);

        // Set up the marker msg for the tube;
        tube.header.stamp = ros::Time::now();
        tube.header.frame_id = map_frame_id;
        tube.ns = "estimated";
        tube.id = j;
        tube.type = 3;
        tube.action = 0;
        tube.pose = tube_pose;
        tube.scale.x = 2*tube_radius;
        tube.scale.y = 2*tube_radius;
        tube.scale.z = 1;
        tube.color.r = 117.0/255.0;
        tube.color.g = 255.0/255.0;
        tube.color.b = 255.0/255.0;
        tube.color.a = 1;

        // Append it to Markers message
        tube_array.markers.push_back(tube);
    }

    tube_pub.publish(tube_array);


    // Format estimated state into pose object  
    tf2::Quaternion slam_q;
    slam_q.setRPY(0, 0, estimation(0));
    geometry_msgs::Quaternion slam_quat = tf2::toMsg(slam_q); // convert tf2 quaternion to geometry_msg

    geometry_msgs::Pose estimated_pose;
    estimated_pose.orientation = slam_quat;
    estimated_pose.position.x = estimation(1);
    estimated_pose.position.y = estimation(2);

    // Add pose to path
    geometry_msgs::PoseStamped ps;
    ps.pose = estimated_pose;

    estimated_path.poses.push_back(ps);
    slam_path_pub.publish(estimated_path);


    ////////////////////////////////
    // Transforms
    ////////////////////////////////

    rigid2d::Vector2D v;
    v.x = turtle.getX();
    v.y = turtle.getY();
    rigid2d::Transform2D T_ob(v,0);
    v.x = estimation(1);
    v.y = estimation(2);
    rigid2d::Transform2D T_mb(v,0);

    rigid2d::Transform2D T_mo = T_mb*T_ob.inv();

    // Calculate odom to turtle rotational transform
    tf2::Quaternion slam2odom_q;
    slam2odom_q.setRPY(0, 0, rigid2d::normalize_angle(asin(T_mo.getStheta())));
    geometry_msgs::Quaternion slam2odom_quat = tf2::toMsg(slam2odom_q); // convert tf2 quaternion to geometry_msg

    // initialize tf2 transform broadcaster
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = map_frame_id;
    transformStamped.child_frame_id = odom_frame_id;

    // set translational information
    transformStamped.transform.translation.x = T_mo.getX();
    transformStamped.transform.translation.y = T_mo.getY();
    transformStamped.transform.translation.z = 0.0;
    
    // set rotational information
    transformStamped.transform.rotation.x = slam2odom_q.x();
    transformStamped.transform.rotation.y = slam2odom_q.y();
    transformStamped.transform.rotation.z = slam2odom_q.z();
    transformStamped.transform.rotation.w = slam2odom_q.w();

    // broadcast transform to tf2
    br.sendTransform(transformStamped);


    tf2_ros::TransformBroadcaster wbr;
    geometry_msgs::TransformStamped wtransformStamped; 
    // Calculate odom to turtle rotational transform
    tf2::Quaternion world2map_q;
    world2map_q.setRPY(0, 0, 0);
    geometry_msgs::Quaternion world2map_quat = tf2::toMsg(world2map_q); // convert tf2 quaternion to geometry_msg

    // initialize tf2 transform broadcaster 
    wtransformStamped.header.stamp = ros::Time::now();
    wtransformStamped.header.frame_id = world_frame_id;
    wtransformStamped.child_frame_id = map_frame_id;

    // set translational information
    wtransformStamped.transform.translation.x = 0;
    wtransformStamped.transform.translation.y = 0;
    wtransformStamped.transform.translation.z = 0.0;
    
    // set rotational information
    wtransformStamped.transform.rotation.x = world2map_q.x();
    wtransformStamped.transform.rotation.y = world2map_q.y();
    wtransformStamped.transform.rotation.z = world2map_q.z();
    wtransformStamped.transform.rotation.w = world2map_q.w();

    // broadcast transform to tf2
    wbr.sendTransform(wtransformStamped);

    return;
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
    ros::init(argc, argv, "slam");
    ros::NodeHandle n;

    // read parameters from parameter server
    double wb,r;
    std::vector<double> Qrow;
    std::vector<double> Rrow;

    ros::param::get("~odom_frame_id",odom_frame_id);
    ros::param::get("~body_frame_id",body_frame_id);
    ros::param::get("~left_wheel_joint",left_wheel_joint);
    ros::param::get("~right_wheel_joint",right_wheel_joint);
    n.getParam("tube_radius",tube_radius);
    n.getParam("Q",Qrow);
    n.getParam("R",Rrow);
    n.getParam("world_frame_id",world_frame_id);
    n.getParam("map_frame_id",map_frame_id);
    n.getParam("wheel_base",wb);
    n.getParam("wheel_radius",r);
    n.getParam("frequency",frequency);
    n.getParam("max_landmarks", max_landmarks);
    wb/=2;

    // Reformat Q and R
    arma::mat Q(3,3);
    Q(0,0) = Qrow[0];
    Q(0,1) = Qrow[1];
    Q(0,2) = Qrow[2];
    Q(1,0) = Qrow[3];
    Q(1,1) = Qrow[4];
    Q(1,2) = Qrow[5];
    Q(2,0) = Qrow[6];
    Q(2,1) = Qrow[7];
    Q(2,2) = Qrow[8];

    arma::mat R(2,2);
    R(0,0) = Rrow[0];
    R(0,1) = Rrow[1];
    R(1,0) = Rrow[2];
    R(1,1) = Rrow[3];

    filter = nuslam::Filter(max_landmarks,Q,R);

    // set up publishers and subscribers
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", frequency);
    odom_path_pub = n.advertise<nav_msgs::Path>("odom_path",frequency);
    slam_path_pub = n.advertise<nav_msgs::Path>("slam_path",frequency);
    tube_pub = n.advertise<visualization_msgs::MarkerArray>("estimated_tubes",10);
    ros::Subscriber joint_sub = n.subscribe("joint_states", frequency, stateCallback);
    ros::Subscriber sensor_sub = n.subscribe("fake_sensor",10,slamCallback);

    // set up services
    ros::ServiceServer service = n.advertiseService("set_pose", set_pose);


    // Set Path header
    odom_path.header.stamp = ros::Time::now();
    odom_path.header.frame_id = world_frame_id;
    estimated_path.header.stamp = ros::Time::now();
    estimated_path.header.frame_id = world_frame_id;
    // set publishing frequency
    ros::Rate loop_rate(10);

    // Initialize differintial drive robot
    turtle = rigid2d::DiffDrive(wb,r);
    slambot = rigid2d::DiffDrive(wb,r);
    estimation(0) = 0;
    estimation(1) = 0;
    estimation(2) = 0;

    int count = 0;
    while (ros::ok())
    {
                
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
  }


  return 0;
}