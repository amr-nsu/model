#ifndef AILERON_H
#define AILERON_H

#include "def.h"

class Aileron
{
    const double COEF_LIFT_ASYMMETRIC = 0.0001; //denoted by C_L_delta_a
    const double COEF_DRAG_ASYMMETRIC = 0.0001; //denoted by C_D_delta_a
    const double C_l_delta_a = 0.0021; //denoted by C_l_delta_a
    const double C_n_delta_a = 0.004; //denoted by C_n_delta_a

    const double COEF_LIFT_SYMMETRIC = 0.21/1;//1.5;//denoted by C_L_delta_s
    const double COEF_DRAG_SYMMETRIC = 0.3/1;//15;//denoted by C_D_delta_s

    arma::vec brake_angles = arma::colvec({0, 0});

    arma::vec get_control() const
    {
        arma::vec tmp(2);

        tmp(0) = brake_angles(0) - brake_angles(1);
        tmp(1) = std::min(brake_angles(0), brake_angles(1));

        return std::move(tmp);
    }

public:
    Aileron();

    arma::vec operator() (void) const {return brake_angles;}
    void operator ()(arma::vec angles) {brake_angles = angles;}

    arma::vec get_force(const arma::vec& velocity, double air_pressure) const
    {
        arma::vec delta = get_control();

        double dsign = sign(delta(0));

        arma::vec lift_direction = get_liftforce_direction(velocity);
        arma::vec drag_direction = get_dragforce_direction(velocity);

        arma::mat tmp(3,2);

        tmp.col(0) = (COEF_LIFT_ASYMMETRIC * lift_direction + COEF_DRAG_ASYMMETRIC * drag_direction) * dsign;
        tmp.col(1) = (COEF_LIFT_SYMMETRIC * lift_direction + COEF_DRAG_SYMMETRIC * drag_direction);

        return tmp * air_pressure * delta;
    }

    arma::vec get_force_momentum(double air_pressure, double span_to_flap_ratio) const
    {
        arma::vec delta = get_control();

        arma::vec tmp = arma::zeros(3,1);
        tmp(0) = C_l_delta_a * span_to_flap_ratio;
        tmp(2) = C_n_delta_a * span_to_flap_ratio;
        return tmp * air_pressure * delta(0);
    }
};

#endif // AILERON_H
