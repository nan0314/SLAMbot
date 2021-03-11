/// \file tube_world.cpp
/// \brief Simulates real turtlebot with noise
///
/// PARAMETERS:
///     tube_x vector<double>: x positions of tubes 
///     tube_y vector<double>: y positions of tubes
///     tube_radius (double): tube radius
///     max_range (double): maximum range of laser scanner
///     tube_var (arma::mat): covariance matrix for x-y sensor
///     trans_var (double): variance for dx in the control twist
///     rot_var (double): variance for dth in the control twist
///     slip_min (double): minimum wheel slip coefficient
///     slip_max (double): maximum wheel slip coefficient
///     frequency (double): ROS loop frequency
///     wheel_base (double): distance between wheels on diffdrive robot
///     wheel_radius (double): radius of wheels on diffdrive robot
///     left_wheel_joint (string): Name of left wheel joint frame
///     right_wheel_joint (string): Name of right wheel joint frame
///     world_frame_id (string): Name of world frame
///     turtle_frame_id (string): Name of turtle frame
///     
/// PUBLISHES:
///     joint_states (sensor_msgs/JointState): messages hold Joint State for each non-fixed joint in the robot
///     fake_sensor (visualization_msgs/MarkerArray): messages hold landmark positions with noise
///     ground_truth (visualization_msgs/MarkerArray): messages hold landmark positions without noise
///     real_path (nav_msgs): holds pose states for the simulated turtle for each time step
/// SUBSCRIBES:
///     turtle1/cmd_vel (geometry_msgs/Twist): angular/linear velocity commands
/// SERVICES:
///     No services

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
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

static ros::Publisher odom_pub;                         // Odometry state publisher
static ros::Publisher tube_pub;                         // Publishes tube locations
static ros::Publisher path_pub;                         // Publishes path of actual robot
static ros::Publisher sensor_pub;                       // Publishes Laser scanner methods
static double frequency;                                // Ros loop frequency
static nav_msgs::Path path;                             // Path of actual robot
static rigid2d::DiffDrive turtle;                       // DiffDrive object to track robot configuration  
static std::string world_frame_id;                      // Name of odometry/world frame
static std::string turtle_frame_id;                     // Name of robot/body frame
static std::string left_wheel_joint;                    // Name of left wheel joint
static std::string right_wheel_joint;                   // Name of right wheel joint
static std::normal_distribution<double> wheel_slip;     // Wheel slip normal distribution
static std::normal_distribution<double> trans_noise;    // Translational velocity normal distribution
static std::normal_distribution<double> rot_noise;      // Rotational velocity normal distribution
static std::normal_distribution<double> u;              // x-y sensor state normal distribution
static std::normal_distribution<double> laser_dist;         
static arma::mat L(2,2);                                // x-y sensor state covariance matrix
static double trans_var;                                // Translational velocity variance
static double rot_var;                                  // Rotational velocity variance
static double slip_min;                                 // Minimum wheel slip coefficient
static double slip_max;                                 // Maximum wheel slip coefficient
static double tube_radius;                              // Tube radius
static double max_range;                                // Maximum range of laser scanner
static double min_range;                                // Minimum range of laser scanner
static double resolution;                               // Laser distance resolution
static double angle_inc;                                   // Angle increment 
static int samples;                                     // Number of laser scanner samples
static double laser_noise;                              // Laser scanner noise variance
static double border_width;                             // Width of border wall
static double border_height;                            // Height of border wall
static std::vector<double> tube_var;                    // Tube location covariance
static std::vector<double> tube_x, tube_y;              // Tube positions
static std::vector<double> recorded_angles = {0,0};     // Wheel angles without noise
visualization_msgs::Marker wall;
geometry_msgs::Point point1;
geometry_msgs::Point point2;
geometry_msgs::Point point3;
geometry_msgs::Point point4;




/// \brief Returns a random number
std::mt19937 & get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }


/// \brief Returns true if tube is within range of laser scanner
/// \param turtle_pose - position of turtle
/// \param tube_pose - tube position
/// \returns true if tube is less than maxrange, else false
bool inRange(geometry_msgs::Pose turtle_pose, geometry_msgs::Pose tube_pose){
    double dist_squared = pow((turtle_pose.position.x-tube_pose.position.x),2) + pow(turtle_pose.position.y-tube_pose.position.y,2);
    double dist = pow(dist_squared,0.5);
    if (dist < max_range){
        return true;
    } else {
        return false;
    }
}

/// \brief Returns true if robot is collided with tube
/// \return true if robot is collided with tube, else false
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

std::vector<double> findIntersect(double x1, double y1, double x2, double y2, geometry_msgs::Point w1, geometry_msgs::Point w2){

    std::vector<double> out;
    double x3 = w1.x;
    double y3 = w1.y;
    double x4 = w2.x;
    double y4 = w2.y;

    double d1 = x1*y2 - x2*y1;
    double d2 = x3*y4 - x4*y3;
    double bottom = (x1-x2)*(y3-y4) - (x3-x4)*(y1-y2);
    double xtop = d1*(x3-x4) - d2*(x1-x2);
    double ytop = d1*(y3-y4) - d2*(y1-y2);

    if (fabs(bottom) > 0.0000001){
        out.push_back(xtop/bottom);
        out.push_back(ytop/bottom);
    } else {
        out.push_back(max_range + 1);
        out.push_back(max_range + 1);
    }

    return out;

}


/// \brief recieve velocity command and cause fake_turtle to move 
/// to fulfill velocity command with applied noise
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



    ////////////////////////////////
    // Publish Lidar Data
    ////////////////////////////////

    std::vector<float> ranges(360);
    std::vector<float> intensities(360);
    std::fill(intensities.begin(),intensities.end(),4000);
    std::fill(ranges.begin(),ranges.end(),max_range + 1);

    double x = turtle.getX();
    double y = turtle.getY();

    // Check for circles with lidar
    for (int i = 0; i<tube_x.size(); i++){

        double x1 = x - tube_x[i];
        double y1 = y - tube_y[i];

        if (sqrt(pow(x1,2) + pow(y1,2)) - tube_radius > max_range){
            continue;
        }
        
        int heading = round((atan2(y1,x1))/angle_inc);

        for (int degree = heading - 45; degree < heading + 45; degree++){

            double x2 = x1 + max_range*cos(double(degree)*angle_inc);
            double y2 = y1 + max_range*sin(double(degree)*angle_inc);

            double dx = x2 - x1;
            double dy = y2 - y1;
            double dr = sqrt(pow(dx,2) + pow(dy,2));
            double D = x1*y2 - x2*y1;

            double discriminant = pow(tube_radius,2)*pow(dr,2) - pow(D,2);

            if (fabs(discriminant) < 0.000001){
                double intercept_x = D*dy/pow(dr,2);
                double intercept_y = -D*dx/pow(dr,2);

                double r = sqrt(pow(intercept_x - x1,2) + pow(intercept_y - y1,2)) + laser_dist(get_random());
                int angle = 180 + round(degree - turtle.getTh()/angle_inc);
                if (angle < 0){
                    angle += 360;
                }

                int r_int = r/resolution;
                r = double(r_int)*resolution;
                if (r < ranges[angle]){
                    ranges[angle] = r;
                }

            } else if (discriminant > 0) {
                double intercept_x1 = (D*dy + dy/fabs(dy)*dx*sqrt(discriminant))/pow(dr,2);
                double intercept_y1 = (-D*dx + fabs(dy)*sqrt(discriminant))/pow(dr,2);
                
                double intercept_x2 = (D*dy - dy/fabs(dy)*dx*sqrt(discriminant))/pow(dr,2);
                double intercept_y2 = (-D*dx - fabs(dy)*sqrt(discriminant))/pow(dr,2);

                double r = sqrt(pow(intercept_x1 - x1,2) + pow(intercept_y1 - y1,2)) + laser_dist(get_random());
                double r2 = sqrt(pow(intercept_x2 - x1,2) + pow(intercept_y2 - y1,2)) + laser_dist(get_random());

                if (r2 < r){
                    r = r2;
                }

                int angle = 180 + round(degree - turtle.getTh()/angle_inc);
                if (angle < 0){
                    angle += 360;
                } else if (angle > 359){
                    angle -= 360;
                }

                int r_int = r/resolution;
                r = double(r_int)*resolution;
                if (r < ranges[angle]){
                    ranges[angle] = r;

                }
            }
        }

        
    }

    // Check for walls

    for (int degree = 0; degree < 360; degree++){
        double x2 = x + max_range*cos(double(degree)*angle_inc);
        double y2 = y + max_range*sin(double(degree)*angle_inc);

        int angle = round(degree - turtle.getTh()/angle_inc);
        if (angle < 0){
            angle += 360;
        } else if (angle > 359){
            angle -= 360;
        }

        std::vector<double> intersection = findIntersect(x,y,x2,y2,point1,point2);
        double r = sqrt(pow(x-intersection[0],2) + pow(y-intersection[1],2)) + laser_dist(get_random());
        int r_int = r/resolution;
        r = double(r_int)*resolution;
        if ((degree < 90 & degree > 269) & r<ranges[angle]){
            ranges[angle] = r;
        }

        intersection = findIntersect(x,y,x2,y2,point2,point3);
        r = sqrt(pow(x-intersection[0],2) + pow(y-intersection[1],2)) + laser_dist(get_random());
        r_int = r/resolution;
        r = double(r_int)*resolution;
        if (degree > 180 & r<ranges[angle]){
            ranges[angle] = r;
        }

        intersection = findIntersect(x,y,x2,y2,point3,point4);
        r = sqrt(pow(x-intersection[0],2) + pow(y-intersection[1],2)) + laser_dist(get_random());
        r_int = r/resolution;
        r = double(r_int)*resolution;
        if ((degree > 90 & degree < 270) & r<ranges[angle]){
            ranges[angle] = r;
        }

        intersection = findIntersect(x,y,x2,y2,point4,point1);
        r = sqrt(pow(x-intersection[0],2) + pow(y-intersection[1],2)) + laser_dist(get_random());
        r_int = r/resolution;
        r = double(r_int)*resolution;
        if (degree < 180 & r<ranges[angle]){
            ranges[angle] = r;
        }

    }

    // Set up LaserScan object
    sensor_msgs::LaserScan scan;

    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = turtle_frame_id;

    scan.range_min = min_range;
    scan.range_max = max_range;
    scan.angle_min = 0;
    scan.angle_max = 6.28319;
    scan.angle_increment = angle_inc;
    scan.ranges = ranges;
    scan.intensities = intensities;

    sensor_pub.publish(scan);  

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
    n.getParam("min_range",min_range);
    n.getParam("angle_inc",angle_inc);
    n.getParam("samples",samples);
    n.getParam("resolution",resolution);
    n.getParam("laser_noise",laser_noise);
    n.getParam("border_width",border_width);
    n.getParam("border_height",border_height);

    // Set up normal distributions
    std::normal_distribution<double> d((slip_max+slip_min)/2,(slip_max - (slip_max+slip_min)/2));
    wheel_slip = d;
    std::normal_distribution<double> d1(0.0,trans_var);
    trans_noise = d1;
    std::normal_distribution<double> d2(0.0,rot_var);
    rot_noise = d2;
    std::normal_distribution<double> d3(0.0,1);
    u = d3;
    std::normal_distribution<double> d4(0.0,laser_noise);
    laser_dist = d4;

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
    sensor_pub = n.advertise<sensor_msgs::LaserScan>("scan", 5);
    path_pub = n.advertise<nav_msgs::Path>("real_path",frequency);
    ros::Publisher wall_pub = n.advertise<visualization_msgs::Marker>("wall",frequency,true);
    ros::Publisher truth_pub = n.advertise<visualization_msgs::MarkerArray>("ground_truth",frequency,true);
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

    // Set up border wall

    wall.header.stamp = ros::Time::now();
    wall.header.frame_id = world_frame_id;
    wall.ns = "real";
    wall.id = 0;
    wall.type = 4;
    wall.action = 0;
    wall.scale.x = .1;
    wall.color.a = 1;
    wall.color.r = 1;
    wall.color.g = 1;
    wall.color.b = 1;
    wall.pose.position.x = 0;
    wall.pose.position.y = 0;

    point1.x = border_height/2;
    point1.y = border_width/2;
    point1.z = 0;
    wall.points.push_back(point1);

    point2.x = border_height/2;
    point2.y = -border_width/2;
    point2.z = 0;
    wall.points.push_back(point2);

    point3.x = -border_height/2;
    point3.y = -border_width/2;
    point3.z = 0;
    wall.points.push_back(point3);

    point4.x = -border_height/2;
    point4.y = border_width/2;
    point4.z = 0;
    wall.points.push_back(point4);
    wall.points.push_back(point1);

    wall_pub.publish(wall);
    

    int count = 0;
    while (ros::ok())
    {
                
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


  return 0;
}