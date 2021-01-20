#include <iostream>
#include "rigid2d.hpp"


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