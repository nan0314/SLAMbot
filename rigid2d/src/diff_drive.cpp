#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"

namespace rigid2d{
    DiffDrive::DiffDrive(){
        wb = 0.16/2;
        r = 0.033;
        th = 0;
        x = 0;
        y = 0;
    }


    DiffDrive::DiffDrive(double wb, double r){
        this->wb = wb;
        this->r = r;
        th = 0;
        x = 0;
        y = 0;
    }

    DiffDrive::DiffDrive(double wb, double r, double th, double x, double y){
        this->wb = wb;
        this->r = r;
        this->th = th;
        this->x = x;
        this->y = y;
    }

    std::vector<double> DiffDrive::twist2control(const rigid2d::Twist2D V){
        using std::vector;

        vector<double> out;
        out.push_back(V.dx/r - V.dth*wb/r);
        out.push_back(V.dx/r+V.dth*wb/r);

        return out;
    }

    rigid2d::Twist2D DiffDrive::control2twist(const std::vector<double> dphi){
        double dth = -dphi[0]*r/2.0/wb + dphi[1]*r/2.0/wb;
        double dx = dphi[0]*r/2.0 + dphi[1]*r/2.0;
        Twist2D dphi_twist = Twist2D(dth,dx,0);
    }


    rigid2d::Twist2D DiffDrive::update(const std::vector<double> phi){
        
        using namespace rigid2d;

        // Use current configuration to calculate delta
        std::vector<double> dphi;
        dphi.push_back(phi[0]-prev_angles[0]);
        dphi.push_back(phi[1]-prev_angles[1]);

        // Calculate dq_b
        double dth = -dphi[0]*r/2.0/wb + dphi[1]*r/2.0/wb;
        double dx = dphi[0]*r/2.0 + dphi[1]*r/2.0;
        Twist2D dphi_twist = Twist2D(dth,dx,0);

        Transform2D Tbb = integrateTwist(dphi_twist);
        Twist2D d_qb = Twist2D(asin(Tbb.getStheta()),Tbb.getX(),Tbb.getY());

        // Convert to world frame dq 
        auto dq = Transform2D(th)(d_qb);

        // Update q
        th+=dq.dth;
        x+=dq.dx;
        y+=dq.dy;
        th = normalize_angle(th);
        prev_angles = phi;

        return dq;
    }

    std::vector<double> DiffDrive::vel_update(rigid2d::Twist2D desired_twist){

        // Find dq_b
        Transform2D Tbb = integrateTwist(desired_twist);
        Twist2D d_qb = Twist2D(asin(Tbb.getStheta()),Tbb.getX(),Tbb.getY());

        // Convert to world frame dq 
        auto dq = Transform2D(th)(d_qb);

        // update wheel angles
        std::vector<double> wheel_change = twist2control(desired_twist);
        prev_angles[0] += wheel_change[0];
        prev_angles[1] += wheel_change[1];

        // Update q
        th+=dq.dth;
        x+=dq.dx;
        y+=dq.dy;
        th = normalize_angle(th);

        return prev_angles;
    }

    void DiffDrive::set_pose(double th = 0, double x = 0, double y = 0){
        this->th = th;
        this->x = x;
        this->y = y;
    }

    double DiffDrive::getTh(){
        return th;
    }

    double DiffDrive::getX(){
        return x;
    }

    double DiffDrive::getY(){
        return y;
    }

    std::vector<double> DiffDrive::getEncoders(){
        return prev_angles;
    }

}
