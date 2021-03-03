#include "nuslam/nuslam.hpp"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"


namespace nuslam{

    std::vector<double> cartesian2polar(std::vector<double> cartesian){


        std::vector<double> polar;
        polar.push_back(pow(pow(cartesian[0],2) + pow(cartesian[1],2),0.5))
        polar.push_back()
    }

    std::vector<double> polar2cartesion(std::vector<double> polar){

    }

    void Filter::initialize_uncertainty(){
        uncertainty = arma::mat(2*n + 3, 2*n + 3, arma::fill::eye);
        uncertainty*=INT_MAX;
        uncertainty(0,0) = 0;
        uncertainty(1,1) = 0;
        uncertainty(2,2) = 0;
    }

    Filter::Filter() : n(10) {
        initialize_uncertainty();
        Q = arma::mat(2*n+3, 2*n + 3, arma::fill::zeros);
    }

    Filter::Filter(double n, arma::mat Q) : n(n) {
        initialize_uncertainty();
        this->Q = arma::mat(2*n+3, 2*n + 3, arma::fill::zeros);
        for (int i = 0; i<Q.n_rows; i++){
            for (int j = 0; j<Q.n_cols; i++){
                this->Q(i,j) = Q(i,j);
            }
        }
    }


    void Filter::predict(const arma::vec prediction, const rigid2d::Twist2D& u_t){

        // Calculate the A matrix
        arma::mat A_t = A(u_t);

        // Set the estimated_xi to the prediction
        estimated_xi = prediction;

        // Propogate the uncertainty
        uncertainty = A_t*uncertainty*A_t.t() + Q;

        return;
    }



    arma::mat Filter::A(const rigid2d::Twist2D& u_t){

        arma::mat out(2*n+3,2*n+3,arma::fill::eye);
        double th_tm = estimated_xi(0);

        if (rigid2d::almost_equal(u_t.dth,0)){
            out(0,1) += -u_t.dx*sin(th_tm);
            out(0,2) += u_t.dx*cos(th_tm);
        } else {
            out(0,1) += -u_t.dx/u_t.dth*cos(th_tm) + u_t.dx/u_t.dth*cos(th_tm+u_t.dth);
            out(0,2) += -u_t.dx/u_t.dth*sin(th_tm) + u_t.dx/u_t.dth*sin(th_tm+u_t.dth);
        }

        return out;
    }

    arma::mat Filter::H(int j){

        double delx = estimated_xi(3+j) - estimated_xi(1);
        double dely = estimated_xi(4+j) - estimated_xi(2);
        double d = pow(pow(delx,2)+pow(dely,2),0.5);

        arma::mat out(3+2*n,3+2*n,arma::fill::zeros);

        out(0,1) = -delx/pow(d,0.5);
        out(0,2) = -dely/pow(d,0.5);
        out(0,2+2*j) = delx/pow(d,0.5);
        out(0,3+2*j) = dely/pow(d,0.5);

        out(1,0) = -1;
        out(1,1) = dely/d;
        out(1,2) = -delx/d;
        out(0,2+2*j) = -dely/d;
        out(0,3+2*j) = delx/d;

        return out;
    }


    arma::vec Filter::h(int j){
        
    }


    arma::vec Filter::update(rigid2d::Twist2D u_t, arma::vec z_i, int j){

        arma::vec prev_estimate = estimated_xi;
        estimated_xi = predicted_xi;

        // Propogate Uncertainty

        // iterate through z to create improve estimate
        for (int i = 0; i<(z))



        return estimated_xi;
    }


}