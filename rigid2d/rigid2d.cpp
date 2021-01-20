#include <iostream>
#include "rigid2d.hpp"

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
    Twist2D V;
    char frame;

    std::cout << "Enter a 2d vector: " << std::endl;
    std::cin >> in;
    std::cout << std::endl;

    std::cout << "Enter a twist as 3 numbers (w, v_x, v_y) separated by spaces or newlines: " << std::endl;
    std::cin >> V;
    std::cout << std::endl;

    std::cout << "What frame is the vector and twist in (a, b, or c)?" << std::endl;
    std:: cin >> frame;
    std::cout << std::endl;

    if (frame == 'a'){
        std::cout << "In frame a: \n" << std::endl;
        std::cout << "Vector: " << in;
        std::cout << "Twist: " << V << std::endl;


        std::cout << "In frame b: \n" << std::endl;
        std::cout << "Vector: " << T_ba(in);
        std::cout << "Twist: " << T_ba(V) << std::endl;

        std::cout << "In frame c: \n" << std::endl;
        std::cout << "Vector: " << T_ca(in);
        std::cout << "Twist: " << T_ca(V) << std::endl;


    } else if (frame == 'b'){
        std::cout << "In frame a: \n" << std::endl;
        std::cout << "Vector: " << T_ab(in);
        std::cout << "Twist: " << T_ab(V) << std::endl;


        std::cout << "In frame b: \n" << std::endl;
        std::cout << "Vector: " << (in);
        std::cout << "Twist: " << (V) << std::endl;

        std::cout << "In frame c: \n" << std::endl;
        std::cout << "Vector: " << T_cb(in);
        std::cout << "Twist: " << T_cb(V) << std::endl;
    } else if (frame == 'c'){
        std::cout << "In frame a: \n" << std::endl;
        std::cout << "Vector: " << T_ac(in);
        std::cout << "Twist: " << T_ac(V) << std::endl;


        std::cout << "In frame b: \n" << std::endl;
        std::cout << "Vector: " << T_cb(in);
        std::cout << "Twist: " << T_cb(V) << std::endl;

        std::cout << "In frame c: \n" << std::endl;
        std::cout << "Vector: " << (in);
        std::cout << "Twist: " << (V) << std::endl;
    }else{
        std::cout << "Invalid Frame" << std::endl;
    }


    return 0;
}
    // Transform2D Functions 
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

    Twist2D Transform2D::operator()(Twist2D & V) const{
        double v_xprime = V.v_x*ctheta - V.v_y*stheta;
        double v_yprime = V.v_x*stheta + V.v_y*ctheta;

        return Twist2D(V.w,v_xprime,v_yprime);
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
