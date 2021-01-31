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

    Vector2D& Vector2D::operator+=(const Vector2D &rhs){
        x+=rhs.x;
        y+=rhs.y;
        return *this;
    }

    Vector2D& Vector2D::operator-=(const Vector2D &rhs){
        x-=rhs.x;
        y-=rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator*=(double scalar){
        x*=scalar;
        y*=scalar;
        return *this;
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
    
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs){
        return lhs+=rhs;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D & rhs){
        return lhs-=rhs;
    }

    Vector2D operator*(Vector2D lhs, const double & rhs){
        return lhs*=rhs;
    }

    Vector2D operator*(const double & rhs, Vector2D lhs){
        return lhs*=rhs;
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
        Transform2D Tinv;
        Tinv.ctheta = ctheta;
        Tinv.stheta = -stheta;
        Tinv.x = -(x*ctheta + y*stheta);
        Tinv.y = -(x*-stheta + y*ctheta);

        return Tinv;
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
        double v_xprime = y*V.dth + V.dx*ctheta - V.dy*stheta;
        double v_yprime = V.dx*stheta + V.dy*ctheta - x*V.dth;

        return Twist2D(v_xprime,v_yprime,V.dth);
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
        dx = 0;
        dy = 0;
        dth = 0;
    }

    Twist2D::Twist2D(double v_x,double v_y, double w){
        this->dx = v_x;
        this->dy = v_y;
        this->dth = w;
    }

    std::ostream & operator<<(std::ostream & os, const Twist2D & V){
        using namespace std;

        return os << "w (rad/s): " << V.dth << " v_x: " << V.dx << " v_y: " << V.dy << endl;
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

    Transform2D integrateTwist(const Twist2D &V){

        Transform2D out;

        
        if (V.dth == 0){        // Case pure translation-- add deltas
            Vector2D v;
            v.x = V.dx;
            v.y = V.dth;
            out = Transform2D(v);
        } else
        {
            Vector2D bs;
            bs.x = -V.dx/V.dth;
            bs.y = V.dy/V.dth;
            auto T_bs = Transform2D(bs);        // Translate to/from center of rotations
            auto T_ssp = Transform2D(V.dth);    // Rotate about center of rotation

            out = T_bs*T_ssp*(T_bs.inv());
        }

        return out;
        
    }


}
