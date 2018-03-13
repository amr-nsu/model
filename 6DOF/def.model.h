#ifndef DEF_MODEL_H
#define DEF_MODEL_H

#include "def.math.h"

///note файл стандартных определений модели

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

inline arma::vec get_linear_velocity(const arma::vec& velocity, const arma::vec& angular_velocity, const arma::vec& displacement)
{
    return velocity+skew_matrix(angular_velocity)*displacement;
}


#endif // DEF_MODEL_H
