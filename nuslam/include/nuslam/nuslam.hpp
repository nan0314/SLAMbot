#ifndef NUSLAM_INCLUDE_GUARD_HPP
#define NUSLAM_INCLUDE_GUARD_HPP

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <vector>
#include <armadillo>

namespace nuslam{

    std::vector<double> cartesian2polar(std::vector<double> cartesian);

    std::vector<double> polar2cartesion(std::vector<double> polar);

    class Filter{

        private:

        int n;
        arma::mat uncertainty;
        arma::mat Q;
        arma::mat R;
        arma::vec estimated_xi;

        /// \brief sets uncertainty to the initial value assuming we know
        /// initial position of robot and don't know positions of landmarks
        void initialize_uncertainty();

        public:

        /// \brief default constructor for Filter class-- sets landmarks
        /// to ten and initializes Q,R, and uncertainty assuming certainty in x-y sensor
        Filter();

        /// \brief sets number of landmarks to n and initializes Q,R, and
        /// uncertainty
        /// \param n - number of landmarks
        Filter(double n, arma::mat Q, arma::mat R);

        /// \brief updates the predicted state in the filter and propogates
        /// the uncertainty matrix
        /// \param prediction - updated prediction from odometry
        /// \param u_t - current control signal
        void predict(const arma::vec prediction, const rigid2d::Twist2D& u_t);

        /// \brief Outputs linear state estimation matrix given twist u_t
        /// \param u_t - given control twist u_t [dtheta, dx, 0]
        /// \returns linear state estimation matrix A_t
        arma::mat A(const rigid2d::Twist2D& u_t);

        /// \brief Outputs linear map estimation matrix
        /// \param j - landmark number/ID
        /// \param mx - x position of landmark
        /// \param my - y position of landmark
        /// \param turtle - current position of turtle
        arma::mat H(int j);

        arma::vec h(int j);


        ///
        arma::vec update(rigid2d::Twist2D u_t, arma::vec z_i,int j);
    };

    

}




#endif
