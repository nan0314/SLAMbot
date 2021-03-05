#include <catch_ros/catch.hpp>
#include "nuslam/nuslam.hpp"
#include <vector>
#include <cmath>


TEST_CASE("Tests conversion from cartesian to polar coordinates","[cartesion2polar]"){ // Nathaniel, Nyberg
    using namespace nuslam;
    using rigid2d::almost_equal;
    using std::vector;

    vector<double> polar = cartesian2polar({3,4});

    REQUIRE(almost_equal(polar[0],5));
    REQUIRE(almost_equal(polar[1],atan(4./3.)));
}

TEST_CASE("Tests Filter constructor","[constructor]"){ // Nathaniel, Nyberg
    using namespace nuslam;
    using std::vector;
    using rigid2d::almost_equal;

    arma::mat Q(3,3,arma::fill::eye);
    arma::mat R(2,2,arma::fill::eye);

    Filter filter(2, Q, R);

    arma::mat checkQ = filter.getQ();
    arma::mat checkR = filter.getR();
    arma::mat checkUncertainty = filter.getUncertainty();
    arma::vec checkEstimate = filter.getEstimate();
        
    // Check Q
    for (int i = 0; i<checkQ.n_rows; i++){
        for (int j = 0; j<checkQ.n_cols; j++){
            if (i == j & i < 3){
                REQUIRE(almost_equal(checkQ(i,j),1));
            } else {
                REQUIRE(almost_equal(checkQ(i,j),0));
            }
        }
    }


    // Check R
    for (int i = 0; i<checkR.n_rows; i++){
        for (int j = 0; j<checkR.n_cols; j++){
            if (i == j){
                REQUIRE(almost_equal(checkR(i,j),1));
            } else {
                REQUIRE(almost_equal(checkR(i,j),0));
            }
        }
    }


    // Check initial uncertainty matrix
    for (int i = 0; i<checkUncertainty.n_rows; i++){
        for (int j = 0; j<checkUncertainty.n_cols; j++){
            if (i == j & i < 3){
                REQUIRE(almost_equal(checkUncertainty(i,j),0));
            } else if (i == j & i >= 3){
                REQUIRE(almost_equal(checkUncertainty(i,j),INT_MAX));
            } else {
                REQUIRE(almost_equal(checkUncertainty(i,j),0));
            }
        }
    }

    // Check initial estimate
    for (int i = 0; i<checkEstimate.n_elem; i++){
        if (i<3){
            REQUIRE(almost_equal(checkEstimate(i),0));
        } else {
            REQUIRE(almost_equal(checkEstimate(i),1));
        }
    }

}

TEST_CASE("Tests A Matrix","[A]"){ // Nathaniel, Nyberg
    using namespace nuslam;
    using std::vector;
    using rigid2d::almost_equal;

    arma::mat Q(3,3,arma::fill::eye);
    arma::mat R(2,2,arma::fill::eye);

    Filter filter(2, Q, R);
    rigid2d::Twist2D u_t1(0,1,0);
    arma::mat A_t1 = filter.A(u_t1);

    for (int i = 0; i<A_t1.n_rows; i++){
        for (int j = 0; j<A_t1.n_cols; j++){
            if (i == 1 & j == 0){
                REQUIRE(almost_equal(A_t1(i,j),0));
            } else if (i == 2 & j == 0){
                REQUIRE(almost_equal(A_t1(i,j),1));
            } else if (i == j){
                REQUIRE(almost_equal(A_t1(i,j),1));
            } else {
                REQUIRE(almost_equal(A_t1(i,j),0));
            }
        }
    }


    rigid2d::Twist2D u_t2(rigid2d::PI,rigid2d::PI,0);
    arma::mat A_t2 = filter.A(u_t2);

    for (int i = 0; i<A_t2.n_rows; i++){
        for (int j = 0; j<A_t2.n_cols; j++){
            if (i == 1 & j == 0){
                REQUIRE(almost_equal(A_t2(i,j),-2));
            } else if (i == 2 & j == 0){
                REQUIRE(almost_equal(A_t2(i,j),0));
            } else if (i == j){
                REQUIRE(almost_equal(A_t2(i,j),1));
            } else {
                REQUIRE(almost_equal(A_t2(i,j),0));
            }
        }
    }


}


TEST_CASE("Tests H matrix","[H]"){ // Nathaniel, Nyberg
    using namespace nuslam;
    using rigid2d::almost_equal;
    using std::vector;

    arma::mat Q(3,3,arma::fill::eye);
    arma::mat R(2,2,arma::fill::eye);

    Filter filter(2, Q, R);

    arma::mat H_i = filter.H(1);

    for (int i = 0; i<H_i.n_rows; i++){
        for (int j = 0; j<H_i.n_cols; j++){
            if (i == 0 & j == 1){
                REQUIRE(almost_equal(H_i(i,j),-1/pow(2,0.5)));
            } else if (i == 0 & j == 2){
                REQUIRE(almost_equal(H_i(i,j),-1/pow(2,0.5)));
            } else if (i == 0 & j == 3+2){
                REQUIRE(almost_equal(H_i(i,j),1/pow(2,0.5)));
            } else if (i == 0 & j == 4+2){
                REQUIRE(almost_equal(H_i(i,j),1/pow(2,0.5)));
            } else if (i == 1 & j == 0){
                REQUIRE(almost_equal(H_i(i,j),-1));
            } else if (i == 1 & j == 1){
                REQUIRE(almost_equal(H_i(i,j),1./2.));
            } else if (i == 1 & j == 2){
                REQUIRE(almost_equal(H_i(i,j),-1./2.));
            } else if (i == 1 & j == 3+2){
                REQUIRE(almost_equal(H_i(i,j),-1./2.));
            } else if (i == 1 & j == 4+2){
                REQUIRE(almost_equal(H_i(i,j),1./2.));
            } else {
                REQUIRE(almost_equal(H_i(i,j),0));
            }
        }
    }
}

TEST_CASE("Tests h vector","[h]"){ // Nathaniel, Nyberg
    using namespace nuslam;
    using std::vector;
    using rigid2d::almost_equal;

    arma::mat Q(3,3,arma::fill::eye);
    arma::mat R(2,2,arma::fill::eye);

    Filter filter(2, Q, R);

    arma::vec z_i = filter.h(1);

    REQUIRE(almost_equal(z_i(0),pow(2,0.5)));
    REQUIRE(almost_equal(z_i(1),rigid2d::PI/4));

}