#include <iostream>
#include "rigid2d/rigid2d.hpp"

namespace rigid2d {

    // Vector Functions
    Vector2D Vector2D::norm(){
        
        Vector2D out;
        double mag = sqrt(pow(x,2)+pow(y,2));

        out.x = x/mag;
        out.y = y/mag;

        return out;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        
        const double first = v.x;
        const double second = v.y;
        
        return os << "[" << first << " " << second  << "]" <<std::endl;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v){

        using namespace std;

        is >> v.x;
        while(is.fail()){
            is.clear();
            is.ignore(1);
            is >> v.x;
        }
        is >> v.y;
        while(is.fail()){
            is.clear();
            is.ignore(1);
            is >> v.y;
        }

        return is;
    }
    

    // Transform2D Functions 
    Transform2D::Transform2D(){
        ctheta = cos(0);
        stheta = sin(0);
        x = 0;
        y = 0;
    }

    Transform2D::Transform2D(const Vector2D & trans){
        stheta = sin(0);
        ctheta = cos(0);
        x = trans.x;
        y = trans.y;
    }

    Transform2D::Transform2D(double radians){
        ctheta = cos(radians);
        stheta = sin(radians);
        x = 0;
        y = 0;
    }

    Transform2D::Transform2D(const Vector2D & trans, double radians){
        ctheta = cos(radians);
        stheta = sin(radians);
        x = trans.x;
        y = trans.y;
    }

    double Transform2D::getCtheta() const{
        return ctheta;
    }

    double Transform2D::getStheta() const{
        return stheta;
    }

    double Transform2D::getX() const{
        return x;
    }

    double Transform2D::getY()const{
        return y;
    }
    
    Vector2D Transform2D::operator()(Vector2D v) const{
        Vector2D v_prime;

        v_prime.x = v.x*ctheta - v.y*stheta + x;
        v_prime.y = v.x*stheta + v.y*ctheta + y;

        return v_prime;
    }

    Transform2D Transform2D::inv() const{
        Vector2D v;
        double new_stheta = -stheta;
        v.x = -(x*ctheta - y*-stheta);
        v.y = -(x*-stheta + y*ctheta);

        return Transform2D(v,asin(new_stheta));
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        
        double newcos = ctheta*rhs.ctheta - stheta*rhs.stheta;
        double newsin = stheta*rhs.ctheta + ctheta*rhs.stheta;
        x = ctheta*rhs.x - stheta*rhs.y + x;
        y = rhs.x*stheta + rhs.y*ctheta + y;
        ctheta = newcos;
        stheta = newsin;

        return *this;
    }

    Twist2D Transform2D::operator()(Twist2D & V) const{
        double v_xprime = V.v_x*ctheta - V.v_y*stheta;
        double v_yprime = V.v_x*stheta + V.v_y*ctheta;

        return Twist2D(V.w,v_xprime,v_yprime);
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        using namespace std;

        return os << "dtheta (degrees): " << rad2deg(asin(tf.stheta)) << " dx: " << tf.x << " dy: " << tf.y << endl;

    }
    
    std::istream & operator>>(std::istream & is, Transform2D & tf){

        double degrees;
        Vector2D v;
        
        is >> degrees;
        while(is.fail()){
            is.clear();
            is.ignore(1);
            is >> degrees;
        }
        is >> v.x;
        while(is.fail()){
            is.clear();
            is.ignore(1);
            is >> v.x;
        }
        is >> v.y;
        while(is.fail()){
            is.clear();
            is.ignore(1);
            is >> v.y;
        }

        tf = Transform2D(v,deg2rad(degrees));

        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        
        lhs*=rhs;

        return lhs;
    }


    // Twist Functions
    Twist2D::Twist2D(){
        v_x = 0;
        v_y = 0;
        w = 0;
    }

    Twist2D::Twist2D(double v_x,double v_y, double w){
        this->v_x = v_x;
        this->v_y = v_y;
        this->w = w;
    }

    std::ostream & operator<<(std::ostream & os, const Twist2D & V){
        using namespace std;

        return os << "w (rad/s): " << V.w << " v_x: " << V.v_x << " v_y: " << V.v_y << endl;
    }

    std::istream & operator>>(std::istream & is, Twist2D & V){

        double w;
        double x;
        double y;
        
        is >> w;
        while(is.fail()){
            is.clear();
            is.ignore(1);
            is >> w;
        }
        is >> x;
        while(is.fail()){
            is.clear();
            is.ignore(1);
            is >> x;
        }
        is >> y;
        while(is.fail()){
            is.clear();
            is.ignore(1);
            is >> y;
        }

        V = Twist2D(w,x,y);

        return is;
    }


}