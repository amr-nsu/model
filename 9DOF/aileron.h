#ifndef AILERON_H
#define AILERON_H

#include "def/def.h"

class Aileron
{
    const double COEF_LIFT_ASYMMETRIC = 0.0001; //denoted by C_L_delta_a
    const double COEF_DRAG_ASYMMETRIC = 0.0001; //denoted by C_D_delta_a
    const double C_l_delta_a = 0.0021; //denoted by C_l_delta_a
    const double C_n_delta_a = 0.004; //denoted by C_n_delta_a

    const double COEF_LIFT_SYMMETRIC = 0.21/1;//denoted by C_L_delta_s
    const double COEF_DRAG_SYMMETRIC = 0.3/1;//denoted by C_D_delta_s

    arma::vec brake_angles = arma::colvec({0, 0});

public:
    Aileron() { }

    arma::vec operator() (void) const {return brake_angles;}
    void operator ()(arma::vec angles) {brake_angles = angles;}
    arma::vec get_control() const;

    arma::vec get_liftforce(const arma::vec& velocity, double air_pressure) const;
    arma::vec get_dragforce(const arma::vec& velocity, double air_pressure) const;

    arma::vec get_force(const arma::vec& velocity, double air_pressure) const;
    arma::vec get_force_momentum(double air_pressure, double span_to_flap_ratio) const;

};

#endif // AILERON_H
