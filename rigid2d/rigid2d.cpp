#include <iostream>
#include "rigid2d.hpp"

namespace rigid2d {

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

    Transform2D::Transform2D(const Vector2D & trans){
        x = trans.x;
        y = trans.y;
        stheta = sin(0);
        ctheta = cos(0);
    }

    Transform2D::Transform2D(double radians){
        ctheta = cos(radians);
        stheta = sin(radians);
        x = 0;
        y = 0;
    }

    Transform2D::Transform2D(const Vector2D & trans, double radians){
        x = trans.x;
        y = trans.y;
        ctheta = cos(radians);
        stheta = sin(radians);
    }

    Vector2D Transform2D::operator()(Vector2D v) const{
        Vector2D v_prime;

        v_prime.x = v.x*ctheta - v.y*stheta + x;
        v_prime.y = v.x*stheta + v.y*ctheta + y;

        return v_prime;
    }

    Transform2D Transform2D::inv() const{
        Transform2D T_inv;
        T_inv.ctheta = ctheta;
        T_inv.stheta = -stheta;
        T_inv.x = -(x*T_inv.ctheta - y*T_inv.stheta);
        T_inv.y = -(x*T_inv.stheta + y*T_inv.ctheta);

        return T_inv;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        Transform2D out;
        out.x += out.ctheta*rhs.x - out.stheta*rhs.y;
        out.y += rhs.x*out.stheta + rhs.y*out.ctheta;
        out.ctheta = out.ctheta*rhs.ctheta - out.stheta*rhs.stheta;
        out.stheta = out.stheta*rhs.ctheta + out.ctheta*rhs.stheta;

        return out;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        using namespace std;

        return os << "dtheta (degrees): " << rad2deg(acos(tf.ctheta)) << " dx: " << tf.x << " dy: " << tf.y << endl;

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
}
int main(){

    using namespace std;

    rigid2d::Vector2D v;
    cin>>v;
    cout<<v << endl;

    return 0;
}