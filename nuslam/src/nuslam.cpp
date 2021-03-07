#include "nuslam/nuslam.hpp"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <cmath>


namespace nuslam{

    std::vector<double> cartesian2polar(std::vector<double> cartesian){

        std::vector<double> polar;

        polar.push_back(pow(pow(cartesian[0],2) + pow(cartesian[1],2),0.5));
        polar.push_back(atan2(cartesian[1],cartesian[0]));

        return polar;
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
        estimated_xi  = arma::vec(3+2*n,arma::fill::zeros);
        estimated_xi(0) = 0;
        estimated_xi(1) = 0;
        estimated_xi(2) = 0;
        Q = arma::mat(2*n+3, 2*n + 3, arma::fill::zeros);
    }

    Filter::Filter(double n, arma::mat Q_in, arma::mat R) : n(n), R(R) {
        initialize_uncertainty();
        estimated_xi  = arma::vec(3+2*n,arma::fill::ones);
        estimated_xi(0) = 0;
        estimated_xi(1) = 0;
        estimated_xi(2) = 0;
        this->Q = arma::mat(2*n+3, 2*n + 3, arma::fill::zeros);
        for (int i = 0; i<Q_in.n_rows; i++){
            for (int j = 0; j<Q_in.n_cols; j++){
                this->Q(i,j) = Q_in(i,j);
            }
        }
        
    }

    arma::mat Filter::getUncertainty(){
        return uncertainty;
    }

    arma::mat Filter::getQ(){
        return Q;
    }

    arma::mat Filter::getR(){
        return R;
    }

    arma::vec Filter::getEstimate(){
        return estimated_xi;
    }


    arma::vec Filter::predict(const rigid2d::Twist2D& u_t){

        // Calculate the A matrix
        // arma::mat A_t = A(u_t);
        arma::mat A_t = arma::mat(3+2*n,3+2*n,arma::fill::eye);
        double th = estimated_xi(0);

        if (fabs(u_t.dth) < .000001){
            A_t(1,0) += -u_t.dx*sin(th);
            A_t(2,0) += u_t.dx*cos(th);

            estimated_xi(0) = estimated_xi(0) + 0;
            estimated_xi(1) = estimated_xi(1) + u_t.dx * cos(th);
            estimated_xi(2) = estimated_xi(2) + u_t.dx * sin(th);

        } else {
            A_t(1,0) += -u_t.dx/u_t.dth*cos(th) + u_t.dx/u_t.dth*cos(th + u_t.dth);
            A_t(2,0) += -u_t.dx/u_t.dth*sin(th) + u_t.dx/u_t.dth*sin(th + u_t.dth);

            estimated_xi(0) = estimated_xi(0) + u_t.dth;
            estimated_xi(1) = estimated_xi(1) + -u_t.dx/u_t.dth*sin(th) + u_t.dx/u_t.dth*sin(th + u_t.dth);
            estimated_xi(2) = estimated_xi(2) + u_t.dx/u_t.dth*cos(th) - u_t.dx/u_t.dth*cos(th + u_t.dth);
        }

        // Set the estimated_xi to the prediction
        estimated_xi(0) = rigid2d::normalize_angle(estimated_xi(0));

        // Propogate the uncertainty
        uncertainty = A_t*uncertainty*A_t.t() + Q;

        return estimated_xi;
    }

    void Filter::initialize_landmark(arma::vec z_i,int j){
        estimated_xi(3+2*j) = estimated_xi(1) + z_i(0)*cos(z_i(1) + estimated_xi(0));
        estimated_xi(4+2*j) = estimated_xi(2) + z_i(0)*sin(z_i(1) + estimated_xi(0));

        return;
    }

    arma::mat Filter::A(const rigid2d::Twist2D& u_t){

        arma::mat out(2*n+3,2*n+3,arma::fill::eye);
        double th_tm = rigid2d::normalize_angle(estimated_xi(0));

        if (rigid2d::almost_equal(u_t.dth,0)){
            out(1,0) += -u_t.dx*sin(th_tm);
            out(2,0) += u_t.dx*cos(th_tm);
        } else {
            out(1,0) += -u_t.dx/u_t.dth*cos(th_tm) + u_t.dx/u_t.dth*cos(th_tm+u_t.dth);
            out(2,0) += -u_t.dx/u_t.dth*sin(th_tm) + u_t.dx/u_t.dth*sin(th_tm+u_t.dth);
        }

        return out;
    }

    arma::mat Filter::H(int j){

        double delx = estimated_xi(3+2*j) - estimated_xi(1);
        double dely = estimated_xi(4+2*j) - estimated_xi(2);
        double d = pow(delx,2)+pow(dely,2);

        arma::mat out(2,3+2*n,arma::fill::zeros);

        out(0,1) = -delx/pow(d,0.5);
        out(0,2) = -dely/pow(d,0.5);
        out(0,3+2*j) = delx/pow(d,0.5);
        out(0,4+2*j) = dely/pow(d,0.5);

        out(1,0) = -1;
        out(1,1) = dely/d;
        out(1,2) = -delx/d;
        out(1,3+2*j) = -dely/d;
        out(1,4+2*j) = delx/d;

        return out;
    }


    arma::vec Filter::h(int j){

        arma::vec z(2);
        std::vector<double> cartesian;
        std::vector<double> polar;

        cartesian.push_back(estimated_xi(3+2*j) - estimated_xi(1));
        cartesian.push_back(estimated_xi(4+2*j) - estimated_xi(2));

        // Calculate z_i
        polar = nuslam::cartesian2polar(cartesian);

        z(0) = polar[0];
        z(1) = rigid2d::normalize_angle(polar[1] - estimated_xi(0));
        
        return z;
    }


    arma::vec Filter::update(rigid2d::Twist2D u_t, arma::vec z_i, int j){

        // Compute estimate measurement for jth landmark
        arma::vec z_est = h(j);

        // Compute Kalman gain
        arma::mat H_i = H(j);
        arma::mat K_i = uncertainty*arma::trans(H_i) * arma::inv(H_i*uncertainty*arma::trans(H_i) + R);

        // Refine the estimated state
        arma::vec dz = z_i - z_est;
        dz[1] = rigid2d::normalize_angle(dz[1]);
        estimated_xi = estimated_xi + K_i*dz;
        estimated_xi(0) = rigid2d::normalize_angle(estimated_xi(0));

        // Update the uncertainty matrix
        uncertainty = (arma::mat(3+2*n, 3+2*n,arma::fill::eye) - K_i*H_i) * uncertainty;

        return estimated_xi;
    }


}