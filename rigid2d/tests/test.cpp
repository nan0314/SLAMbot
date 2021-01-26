#define CATCH_CONFIG_MAIN
#include "rigid2d/catch.hpp"
#include "rigid2d/rigid2d.hpp"
#include <cmath>
#include <iostream>
#include <sstream>

/* !!!
Compile using command: g++ -Wall -Wextra -g -std=c++17 -o test test.cpp rigid2d.cpp (takes a really long time)
!!! */ 

/// \brief testing transform2D default constructor
TEST_CASE("Default constructor creates identity transform","[def_constructor]"){ // Nathaniel, Nyberg
    using namespace rigid2d;

    Transform2D trans = Transform2D();
        
    REQUIRE(almost_equal(trans.getCtheta(),1));
    REQUIRE(almost_equal(trans.getStheta(),0));
    REQUIRE(almost_equal(trans.getX(),0));
    REQUIRE(almost_equal(trans.getY(),0));
}

/// \brief testing transform2D double radians constructor
TEST_CASE("Constructor using only radians component","[rad_constructor]"){ // Nathaniel, Nyberg
    using namespace rigid2d;

    Transform2D trans = Transform2D(PI);
        
    REQUIRE(almost_equal(trans.getCtheta(),cos(PI)));
    REQUIRE(almost_equal(trans.getStheta(),sin(PI)));
    REQUIRE(almost_equal(trans.getX(),0));
    REQUIRE(almost_equal(trans.getY(),0));
}

/// \brief testing transform2D vector v constructor
TEST_CASE("Constructor using only vector component","[vector_constructor]"){ // Nathaniel, Nyberg
    using namespace rigid2d;

    Vector2D v;
    v.x = 2;
    v.y = 3;
    Transform2D trans = Transform2D(v);
        
    REQUIRE(almost_equal(trans.getCtheta(),1));
    REQUIRE(almost_equal(trans.getStheta(),0));
    REQUIRE(almost_equal(trans.getX(),2));
    REQUIRE(almost_equal(trans.getY(),3));
}

/// \brief testing transform2D full constructor
TEST_CASE("Constructor using both components","[full_constructor]"){ // Nathaniel, Nyberg
    using namespace rigid2d;

    Vector2D v;
    v.x = 2;
    v.y = 3;
    Transform2D trans = Transform2D(v,PI);
        
    REQUIRE(almost_equal(trans.getCtheta(),cos(PI)));
    REQUIRE(almost_equal(trans.getStheta(),sin(PI)));
    REQUIRE(almost_equal(trans.getX(),2));
    REQUIRE(almost_equal(trans.getY(),3));
}

/// \brief testing transform2D >> operator overload
TEST_CASE("Input a Transform from istream","[input]"){ // Nathaniel, Nyberg
    using namespace rigid2d;

    Transform2D trans;

    std::unique_ptr<std::istream> is;
    is = std::make_unique<std::istringstream>(std::istringstream{"-30 2 3"});
    *is >> trans;

    REQUIRE(almost_equal(trans.getCtheta(),cos(deg2rad(-30))));
    REQUIRE(almost_equal(trans.getStheta(),sin(deg2rad(-30))));
    REQUIRE(almost_equal(trans.getX(),2));
    REQUIRE(almost_equal(trans.getY(),3));
}

/// \brief testing transform2D << operator overload
TEST_CASE("Output a Transform to ostream","[output]"){ // Nathaniel, Nyberg
    using namespace rigid2d;

    Transform2D trans = Transform2D();
    double test;
    std::stringstream ss;
    ss << trans;

    ss >> test;
    while(ss.fail()){
        ss.clear();
        ss.ignore(1);
        ss >> test;
    }

    REQUIRE(almost_equal(test,0));

    ss >> test;
    while(ss.fail()){
        ss.clear();
        ss.ignore(1);
        ss >> test;
    }

    REQUIRE(almost_equal(test,0));

    ss >> test;
    while(ss.fail()){
        ss.clear();
        ss.ignore(1);
        ss >> test;
    }

    REQUIRE(almost_equal(test,0));

}

/// \brief testing Vector2D operator()(Vector2D v) const
TEST_CASE("Transformation applied to Vector2D", "[transform]") // Lin, Liu 
{
    using namespace rigid2d; 
    Vector2D v; 
    v.x = -1;
    v.y = 2;

    Transform2D trans21 = Transform2D(v,PI); 
    Vector2D v1; 
    v1.x = -3; 
    v1.y = 1; 

    Vector2D v2; 
    v2 = trans21(v1);

    REQUIRE(almost_equal(v2.x, 2)); 
    REQUIRE(almost_equal(v2.y, 1));
}

/// \brief testing Transform Composition Operator
TEST_CASE("Transform Composition Operator (*=)", "[transform]"){ // Arun, Kumar

    using namespace rigid2d;

    Vector2D v;
    v.x = 3;
    v.y = 9;

    Transform2D transMat = Transform2D(v, PI/9);
    Transform2D invTransMat = transMat.inv();

    transMat*=invTransMat;

    REQUIRE(almost_equal(transMat.getCtheta(), 1));
    REQUIRE(almost_equal(transMat.getStheta(), 0));
    REQUIRE(almost_equal(transMat.getX(), 0));
    REQUIRE(almost_equal(transMat.getY(), 0));
}
