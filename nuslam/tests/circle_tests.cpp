#include <catch_ros/catch.hpp>
#include "nuslam/nuslam.hpp"
#include <vector>
#include <cmath>


TEST_CASE("Tests circle fitting algorithm","[fitCircle]"){ // Nathaniel, Nyberg
    using namespace nuslam;
    using std::vector;

    geometry_msgs::Point p;
    visualization_msgs::Marker result;

    vector<geometry_msgs::Point> cluster;
    vector<geometry_msgs::Point> cluster2;

    vector<double> x_pos1 = {1,2,5,7,9,3};
    vector<double> y_pos1 = {7,6,8,7,5,7};
    vector<double> x_pos2 = {-1,-.3,.3,1};
    vector<double> y_pos2 = {0,-.06,.1,0};

    for (int i = 0; i<x_pos1.size(); i++){
        p.x = x_pos1[i];
        p.y = y_pos1[i];

        cluster.push_back(p);
    }

    result = fitCircle(cluster);

    REQUIRE(fabs(result.pose.position.x - 4.615482) < 1e-4);
    REQUIRE(fabs(result.pose.position.y - 2.807354) < 1e-4);
    REQUIRE(fabs(result.scale.x/2 - 4.8275) < 1e-4);

    for (int i = 0; i<x_pos2.size(); i++){
        p.x = x_pos2[i];
        p.y = y_pos2[i];

        cluster2.push_back(p);
    }

    result = fitCircle(cluster2);

    REQUIRE(fabs(result.pose.position.x - 0.4908357) < 1e-4);
    REQUIRE(fabs(result.pose.position.y - -22.15212) < 1e-4);
    REQUIRE(fabs(result.scale.x/2 - 22.17979) < 1e-4);
    
}