#ifndef NUSLAM_INCLUDE_GUARD_HPP
#define NUSLAM_INCLUDE_GUARD_HPP

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <armadillo>

namespace nuslam{

    /// \brief takes a vector of lidar range data, lidar maximum range, and minimum range
    /// and returns a vector of clusters of points
    /// \param ranges - 360 element vector<float> of lidar range data
    /// \param max_range - maximum range of the lidar
    /// \param min_range - minimum range of the lidar
    /// \returns vector containing clusters of points vectors (vector of geometry_msgs::Point);
    std::vector<std::vector<geometry_msgs::Point>> findClusters(std::vector<float> ranges, double max_range, double min_range);

    /// \brief takes a cluster (vector of geometry_msgs::Point) and returns true if circle
    /// else returns false
    /// \param cluster - geometry_msgs::Point vector of points containing to same landmark
    /// \returns true if object is circle else false
    bool classifyCircle(std::vector<geometry_msgs::Point> cluster);

    /// \brief Takes vector of points belonging to a circular object and returns a 
    /// visualization_msgs::Marker cylinder centered at the center of the detected circle
    /// \param circle - cluster of points belonging to a circular object
    /// \returns cylinder centered at the center of the detected circle
    visualization_msgs::Marker fitCircle(std::vector<geometry_msgs::Point> circle);


    /// \brief takes a 2d vector in cartesian [x,y] coordinates and converts it
    ///  into a 2d vector in polar coordinates [r,phi] 
    ///  \param cartesian - 2d vector<double> [x,y]
    ///  \returns 2d vector<double> [r,phi] in polar coordinates
    std::vector<double> cartesian2polar(std::vector<double> cartesian);

    /// \brief an Extended Kalman Filter object
    class Filter{

        private:

        int n;                      // Maximum number of landmarks
        int N = 0;                  // Number of landmarks already observed.
        arma::mat uncertainty;      // Covariance matrix
        arma::mat Q;                // State covariance
        arma::mat R;                // Sensor covariance
        arma::vec estimated_xi;     // Estimation vector

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
        /// \param Q_in - 3x3 state covariance matrix
        /// \param R - 2x2 sensor covariance matrix
        Filter(double n, arma::mat Q_in, arma::mat R);

        /// \brief uncertainty accessor function
        arma::mat getUncertainty();

        /// \brief Q accessor function
        arma::mat getQ();

        /// \brief R accessor function
        arma::mat getR();

        /// \brief Estimated state accessor function
        arma::vec getEstimate();

        /// \brief Initializes the position of a landmark seen for the first time
        /// \param z_i - Sensor reading of individual landmark
        /// \param j - inidviual landmark ID
        void initialize_landmark(arma::vec z_i, int j);

        /// \brief updates the estimated state according to the control and propogates
        /// the uncertainty matrix
        /// \param u_t - current control signal
        /// \returns new estimation matrix
        arma::vec predict(const rigid2d::Twist2D& u_t);

        /// \brief Outputs linear state estimation matrix given twist u_t
        /// \param u_t - given control twist u_t [dtheta, dx, 0]
        /// \returns linear state estimation matrix A_t
        arma::mat A(const rigid2d::Twist2D& u_t);           // DEPRECATED

        /// \brief Outputs linear map estimation matrix
        /// \param j - landmark number/ID
        /// \param temp - state estimation vector
        /// \return linear map estimation matrix
        arma::mat H2(int j,arma::vec temp);

        /// \brief Outputs linear map estimation matrix
        /// \param j - landmark number/ID
        /// \return linear map estimation matrix
        arma::mat H(int j);


        /// \brief Calculates theoretical sensor measurement
        /// \param j - landmark number/ID
        arma::vec h(int j);

        /// \brief Refines estimated state using kalman filter
        /// \param u_t - given control twist u_t [dtheta, dx, 0]
        /// \param z_i - sensor reading of individual landmark
        /// \param j - landmark number/ID
        /// \returns refined estimate state
        arma::vec update(rigid2d::Twist2D u_t, arma::vec z_i,int j);

        /// \brief Performs data association on observed objects
        /// \param z_i - observed object relative position in range-bearing coordinate
        /// [r theta]
        /// \returns Returns an integer ID representing the object
        int associate_landmark(arma::vec z_i);
    };    

}




#endif
