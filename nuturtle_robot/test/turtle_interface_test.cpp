#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>
#include "rigid2d/diff_drive.hpp"
#include <vector>
#include <string>

void trans_callback(const nuturtlebot::WheelCommands msg){

    CHECK(msg.left_velocity == 38);
    CHECK(msg.right_velocity == 38);

    return;
}


TEST_CASE("trans_vel","[trans_vel]") {
    
    // this test case subscribes to a topic and checks the results
    ros::NodeHandle n;
    const auto sub = n.subscribe("wheel_cmd",1000,trans_callback);

    // a latched publisher is used to account for the case that we publish
    // this message before the node in turtle_interface can subscribe to the
    // FranklinTheTurtle/cmd_vel topic
    const auto pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000,true);
    geometry_msgs::Twist out;
    out.linear.x = 0.033;
    pub.publish(out);
    ros::Rate r(100.0);

    for(int i = 0; ros::ok() && i!=200; i++){
        ros::spinOnce();
        r.sleep();
    }

}


void rot_callback(const nuturtlebot::WheelCommands msg){

    CHECK(msg.left_velocity == -38);
    CHECK(msg.right_velocity == 38);

    return;
}


TEST_CASE("rot_vel","[rot_vel]") {
    
    // this test case subscribes to a topic and checks the results
    ros::NodeHandle n;
    const auto sub = n.subscribe("wheel_cmd",1000,rot_callback);

    // a latched publisher is used to account for the case that we publish
    // this message before the node in turtle_interface can subscribe to the
    // FranklinTheTurtle/cmd_vel topic
    const auto pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000,true);
    geometry_msgs::Twist out;
    out.angular.z = 0.4125;
    pub.publish(out);
    ros::Rate r(100.0);

    for(int i = 0; ros::ok() && i!=200; i++){
        ros::spinOnce();
        r.sleep();
    }

}



void encoder_callback(const sensor_msgs::JointState::ConstPtr& msg){
    double PI = 3.141592653589793116;
    CHECK(msg->position[0] - PI < 0.001);
    CHECK(msg->position[1] - PI < 0.001);

    return;
}


TEST_CASE("encoder","[encoder]") {
    
    // this test case subscribes to a topic and checks the results
    ros::NodeHandle n;
    const auto sub = n.subscribe("joint_states",1000,encoder_callback);

    // a latched publisher is used to account for the case that we publish
    // this message before the node in turtle_interface can subscribe to the
    // FranklinTheTurtle/cmd_vel topic
    const auto pub = n.advertise<nuturtlebot::SensorData>("sensor_data",1000,true);
    nuturtlebot::SensorData out;
    out.left_encoder = 2048;
    out.right_encoder = 2048;
    pub.publish(out);
    ros::Rate r(100.0);

    for(int i = 0; ros::ok() && i!=200; i++){
        ros::spinOnce();
        r.sleep();
    }

}