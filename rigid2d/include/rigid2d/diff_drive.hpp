#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP

#include "rigid2d/rigid2d.hpp"
#include <vector>

namespace rigid2d{

    /// \file
    /// \brief Library for modeling kinematics of a differential drive robot

    /// \brief a differential drive robot
    class DiffDrive{

        private:
        double wb, r, th, x, y;
        std::vector<double> prev_angles{0,0};

        public:

        /// \brief default constructor-- sets paramaters as follows:
        /// wb: 0.16
        /// r: 0.033
        /// th: 0
        /// x: 0 
        /// y: 0
        DiffDrive();

        /// \brief sets wb and r and initializes all other parameters
        /// to 0
        /// \param wb - wheel base parameter (distance between wheels)
        /// \param r - wheel radius parameter
        DiffDrive(double wb, double r);

        /// \brief initializes all parameters
        /// \param wb - wheel base parameter (distance between wheels)
        /// \param r - wheel radius
        /// \param th - angular component of pose
        /// \param x - x position of pose
        /// \param y - y position of pose
        DiffDrive(double wb, double r, double th, double x, double y);

        /// \brief Convert a desired twist to the equivalent wheel velocities
        /// required to achieve that twist
        /// \param V desired twist
        /// \return vector [phi_left phi_right]
        std::vector<double> twist2control(const rigid2d::Twist2D V);

        /// \brief Convert wheel velocities to the equivalent desired twist
        /// \param dphi wheel velocities
        /// \return desired twist
        rigid2d::Twist2D control2twist(const std::vector<double> dphi);

        /// \brief Update the configuration of the robot, given updated wheel angles
        /// \param angles updated wheel angles
        /// \return robot body twist
        rigid2d::Twist2D update(const std::vector<double> phi);

        /// \brief Update the configuration of the robot, given velocities [dtheta,dx,dy]
        /// \param desired_twist robot velocities [dtheta,dx,dy]
        /// \return resulting wheel angles of robot
        std::vector<double> vel_update(rigid2d::Twist2D desired_twist);


        /// \brief reset robot configuration to user specified configuration [th,x,y]
        /// \param th value to reset private member th to
        /// \param x value to reset private member x to
        /// \param y value to reset private member y to 
        void set_pose(double th, double x, double y);

        /// \brief accessor function to get private member th
        /// \return value of private member th
        double getTh();

        /// \brief accessor function for private member x
        /// \return value of private member x
        double getX();

        /// \brief accessor function for pivate member y
        /// \return value of private member y
        double getY();

        /// \brief accessor function for private member prev_angles
        /// \return value of private member prev_angles
        std::vector<double> getEncoders();

    };

}



#endif
