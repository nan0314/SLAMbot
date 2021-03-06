#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>

namespace rigid2d
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        if (fabs(d1 - d2) < epsilon) {
            return true;
        } else {
            return false;
        }
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr double deg2rad(double deg)
    {
        return deg*PI/180;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return rad*180/PI;
    }

    /// \brief turns any angle into the equivalent one between -pi and pi
    /// \param rad any angle in radians
    /// \return equivalent angle in radians between -pi and pi
    double normalize_angle(double rad);

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(almost_equal(0,1,2), "is zero failed");
    static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    static_assert(almost_equal(deg2rad(180),PI), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg failed");
    static_assert(almost_equal(rad2deg(PI),180), "rad2deg failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");


    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        double x = 0.0;
        double y = 0.0;

        /// \brief outputs a normalized vector2D
        /// \return unit vector in same direction as input
        Vector2D norm();

        /// \brief perform basic vector addition and store result 
        /// in this object
        /// \param rhs vector to be summed
        /// \return sum of vector and rhs
        Vector2D &operator+=(const Vector2D &rhs);

        /// \brief perform basic vector subtraction and store result 
        /// in this object
        /// \param rhs vector to be subtracted
        /// \return vector minus rhs
        Vector2D &operator-=(const Vector2D &rhs);

        /// \brief scale this vector and store the result 
        /// in this object
        /// \param scalar - value to scale vector by
        /// \return scaled vector2D
        Vector2D & operator*=(double scalar);

    };

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [xcomponent, ycomponent]
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief sum two vectors together, returning their sum
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the sum of the two vectors
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

    /// \brief subtract one vector from another, returning their difference
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the result of lhs minus rhs
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

    /// \brief scale a vector, returning the scaled vector
    /// \param lhs - vector to be scaled
    /// \param rhs - scalar factor
    /// \return the scaled vector
    Vector2D operator*(Vector2D lhs, const double & rhs);

    /// \brief scale a vector, returning the scaled vector
    /// \param lhs - scalar factor
    /// \param rhs - vector to be scaled
    /// \return the scaled vector
    Vector2D operator*(const double &lhs, Vector2D rhs);



    /// \brief a twist in 2 dimensions
    struct Twist2D{

        double dth,dx,dy;

        /// \brief Create a twist with no velocity
        Twist2D();

        /// \brief Create a twist with a linear and angular component
        /// \param w - the angular velocity
        /// \param v_x - the linear velocity in the x direction
        /// \param v)y - the linear velocity in the y direction
        Twist2D(double w, double v_x, double v_y);

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Twist2D & V);

    };

    /// \brief should print a human readable version of the twist:
    /// An example output:
    /// w (rad/s): 90 v_x: 3 v_y: 5
    /// \param os - an output stream
    /// \param V - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & V);

    /// \brief Read a twist from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (w, v_x, v_y) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Twist2D & V);

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    private:
        double ctheta, stheta, x, y;

    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(const Vector2D & trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(const Vector2D & trans, double radians);

        /// \brief accessor function to access ctheta private member
        /// \return cos of dtheta
        double getCtheta() const;

        /// \brief accessor function to access stheta private member
        /// \return sin of dtheta 
        double getStheta() const;

        /// \brief accessor function to access x private member
        /// \return dx
        double getX() const;

        /// \brief accessor function to access y private member
        /// \return dy
        double getY()const;

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \returns a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

        /// \brief convert a twist to a different reference frame using the adjoint
        /// \param V - the twist to be converted
        /// \return - the transformed twist in a different reference frame.
        Twist2D operator()(Twist2D & V) const;

    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief compute the transformation corresponding to a rigid body 
    /// following a constant twist (in its original body frame) for one time unit
    /// \param V - twist to be integrated
    /// \return transformation of integration
    Transform2D integrateTwist(const Twist2D &V);
    
}

#endif
