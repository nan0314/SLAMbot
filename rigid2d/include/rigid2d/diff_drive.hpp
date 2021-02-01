#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP

#include "rigid2d/rigid2d.hpp"
#include <vector>

/// \file
/// \brief Library for modeling kinematics of a differential drive robot

/// \brief a differential drive robot
class diff_drive{

    private:
    double wb, r, th, x, y;

    public:

    /// \brief Convert a desired twist to the equivalent wheel velocities
    /// required to achieve that twist
    /// \param V desired twist
    /// \return vector [phi_left phi_right]
    std::vector<double> twist2control(const rigid2d::Twist2D V);

    /// \brief Update the configuration of the robot, given updated wheel angles
    /// \param angles updated wheel angles
    void update(const std::vector<double> angles);


};

#endif
