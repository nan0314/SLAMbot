#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"


std::vector<double> diff_drive::twist2control(const rigid2d::Twist2D V){
    using std::vector;

    vector<double> out;
    out.push_back(V.dx/r - V.dth*wb/r);
    out.push_back(V.dx/r+V.dth*wb/r);

    return out;
}

void diff_drive::update(const std::vector<double> dphi){
    
    using namespace rigid2d;

    // Calculate dq_b
    double dth = -dphi[0]*r/2/wb + dphi[1]*r/2/wb;
    double dx = dphi[0]*r/2 + dphi[1]*r/2;
    Twist2D dphi_twist = Twist2D(dth,dx,0);

    // Convert to world frame dq 
    auto dq = Transform2D(th)(dphi_twist);

    // Update q
    th+=dq.dth;
    x+=dq.dx;
    y+=dq.dy;
}