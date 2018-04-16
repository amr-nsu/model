#ifndef DEF_MODEL_H
#define DEF_MODEL_H

#include "def.math.h"

///note файл стандартных определений модели

namespace def
{
    inline double get_sideslip_angle(const arma::vec& velocity)
    {
        double beta = 0;
        if (velocity(0) != 0)
            beta = asin(velocity(1) / arma::norm(velocity,2));

        return beta;
    }

    inline double get_angle_of_attack(const arma::vec& velocity)
    {
        double alpha = 0; //MathDef.PI * 0.5;

        if (velocity(0) != 0)
            alpha = atan(velocity(2) / velocity(0));

        return alpha;
    }

    inline arma::vec get_liftforce_direction(const arma::vec& velocity)
    {
        arma::colvec tmp = arma::normalise(velocity);
        tmp(0) = -tmp(0);
        std::swap(tmp(0), tmp(2));

        return std::move(tmp);
    }

    inline arma::vec get_dragforce_direction(const arma::vec& velocity)
    {
        return -arma::normalise(velocity); //norm and inverse direction
    }

    inline arma::vec get_linear_velocity(const arma::vec& velocity, const arma::vec& angular_velocity,
                                         const arma::vec& displacement, const arma::vec& euler_angles)
    {
        return def::coord_rotational_matrix(euler_angles)*velocity+skew_matrix(angular_velocity)*displacement;
    }

    inline double get_tilded_phi(const arma::vec& euler_angles)
    {
        double phi = euler_angles(0);
        double theta = euler_angles(1);
        double psi = euler_angles(2);

        return std::atan((std::sin(phi)*std::sin(theta)*std::cos(psi)-std::cos(phi)*std::sin(psi))/(std::cos(theta)*std::cos(psi)));
    }

    inline double get_tilded_theta(const arma::vec& euler_angles)
    {
        double phi = euler_angles(0);
        double theta = euler_angles(1);
        double psi = euler_angles(2);

        return std::atan((std::cos(phi)*std::sin(theta)*std::cos(psi)-std::sin(phi)*std::sin(psi)*std::sin(get_tilded_phi(euler_angles)))/(std::cos(theta)*std::cos(psi)));
    }

    inline double get_dotted_tilded_phi(const arma::vec& euler_angles, const arma::vec& angular_velocity)
    {
        arma::rowvec tmp = arma::ones(1,3);

        tmp(0) = -std::cos(get_tilded_phi(euler_angles))*std::tan(get_tilded_phi(euler_angles));
        tmp(1) = std::sin(get_tilded_phi(euler_angles))*std::tan(get_tilded_phi(euler_angles));

        arma::mat ret = tmp*angular_velocity;

        return ret(0,0);
    }

    inline arma::mat get_rectangle_inertia(double x, double y, double z,
                                           double density, arma::vec displacment)
    {
        arma::mat inertia = arma::zeros(3,3);

        double Jx = density*( x*y*y*y*z + x*y*z*z*z)/12;
        double Jy = density*( x*x*x*y*z + x*y*z*z*z)/12;
        double Jz = density*( x*x*x*y*z + x*y*y*y*z)/12;

        double dx = displacment(0);
        double dy = displacment(1);
        double dz = displacment(2);

        double mass = x*y*z*density;

        inertia(0,0) = Jx + mass*( dy*dy + dz*dz);
        inertia(1,1) = Jy + mass*( dx*dx + dz*dz);
        inertia(2,2) = Jz + mass*( dx*dx + dy*dy);
        inertia(0,1) = inertia(1,0) = 0 + mass*dx*dy;
        inertia(1,2) = inertia(2,1) = 0 + mass*dy*dz;
        inertia(0,2) = inertia(2,0) = 0 + mass*dx*dz;

        return std::move(inertia);
    }

}
#endif // DEF_MODEL_H
