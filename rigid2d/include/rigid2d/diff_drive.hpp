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

        /// \brief Update the configuration of the robot, given updated wheel angles
        /// \param angles updated wheel angles
        /// \return robot body twist
        rigid2d::Twist2D update(const std::vector<double> phi);

        std::vector<double> vel_update(rigid2d::Twist2D dphi_twist);



        /// \brief reset configuration to [th,x,y] = [0,0,0]
        void set_pose(double th, double x, double y);

        double getTh();

        double getX();

        double getY();

    };

}



#endif
