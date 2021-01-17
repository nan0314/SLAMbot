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
    
    Transform2D::Transform2D(){
        ctheta = cos(0);
        stheta = sin(0);
        x = 0;
        y = 0;
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
        Vector2D v;
        double new_stheta = -stheta;
        v.x = -(x*ctheta - y*-stheta);
        v.y = -(x*-stheta + y*ctheta);

        return Transform2D(v,asin(new_stheta));
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        
        x = ctheta*rhs.x - stheta*rhs.y + x;
        y = rhs.x*stheta + rhs.y*ctheta + y;
        ctheta = ctheta*rhs.ctheta - stheta*rhs.stheta;
        stheta = stheta*rhs.ctheta + ctheta*rhs.stheta;

        return *this;
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

    using namespace rigid2d;

    Transform2D T_ab;
    Transform2D T_bc;

    std::cout << "Enter a transformation as 3 numbers (degrees, dx, dy) separated by spaces or newlines:" << std::endl;
    std::cin >> T_ab;

    std::cout << "Enter a second transformation as 3 numbers (degrees, dx, dy) separated by spaces or newlines:" << std::endl;
    std::cin >> T_bc;
    std::cout << std::endl;

    Transform2D T_ba = T_ab.inv();
    Transform2D T_cb = T_bc.inv();

    Transform2D T_ac = T_ab*T_bc;
    Transform2D T_ca = T_ac.inv();

    std::cout << "T_ab:\n" << T_ab << std::endl;
    std::cout << "T_ba:\n" << T_ba << std::endl;
    std::cout << "T_bc:\n" << T_bc << std::endl;
    std::cout << "T_cb:\n" << T_cb << std::endl;
    std::cout << "T_ac:\n" << T_ac << std::endl;
    std::cout << "T_ca:\n" << T_ca << std::endl;

    Vector2D in;
    char frame;
    std::cout << "Enter a 2d vector: " << std::endl;
    std::cin >> in;

    std::cout << "What frame is the vector in (a, b, or c)?" << std::endl;
    std:: cin >> frame;

    if (frame == 'a'){
        std::cout << "In frame a: " << in << std::endl;
        std::cout << "In frame b: " << T_ab(in) << std::endl;
        std::cout << "In frame c: " << T_ac(in) << std::endl;
    } else if (frame == 'b'){
        std::cout << "In frame a: " << T_ba(in) << std::endl;
        std::cout << "In frame b: " << (in) << std::endl;
        std::cout << "In frame c: " << T_bc(in) << std::endl;
    } else if (frame == 'c'){
        std::cout << "In frame a: " << T_ca(in) << std::endl;
        std::cout << "In frame b: " << T_cb(in) << std::endl;
        std::cout << "In frame c: " << (in) << std::endl;
    }else{
        std::cout << "Invalid Frame" << std::endl;
    }


    return 0;
}