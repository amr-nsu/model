#ifndef PARAFOIL_H
#define PARAFOIL_H

#include "def/def.h"
#include "aileron.h"

struct PState
{
    double air_pressure = 0;
    double coef_lift = 0.4;
    double coef_drag = 0.15;
    double angle_of_attack = 0;
 };

std::ostream& operator << (std::ostream& os, const PState& state);


class Parafoil
{
    const double COEF_LIFT_BASE = 0.4; //0.28; //denoted by C_P_L0
    const double COEF_LIFT_ALPHA = 2; //0.83;//1.98; //2; //denoted by C_P_LAlpha
    const double COEF_DRAG_BASE = 0.15; //0.12; //0.25; //denoted by C_P_D0 changed from 0.15; ?(0.25)
    const double COEF_DRAG_ALPHA = 0; //-0.2;//-0.28; //denoted by C_P_DAlpha
    const double COEF_DRAG_ALPHA_2 = 1; //1.;//1.44; //denoted by C_P_DAlpha

    const double COEF_l_p = -0.1;  //denoted by C_l_p
    const double COEF_l_phi = -0.05; //denoted by C_l_phi
    const double COEF_m_q = -2;//-2; //denoted by C_m_q
    const double COEF_m_base = 0.018; //denoted by C_m_theta
    const double COEF_m_alpha = -0.2; //denoted by C_m_alpha
    const double COEF_n_r = -0.07;//-0.07; //denoted by C_n_r

    const double AREA = 1.2; //(1.4)
    const double SPAN = 2.7; //denoted by b (2.49)
    const double CHORD = 0.51;//0.62;//denoted by c (0.62)
    const double FLAP_WIDTH = 0.1;//0.4; //denoted by d (0.4)

    const arma::vec TRANS_PARA_BODY_VECTOR = arma::colvec ({-0.266, 0,-1.066});	// denoted by S_PB {-0.266},{0},{-1.066}

    const double WIDTH = 0.1;
    const double MASS = def::AIR_DENSITY * WIDTH * AREA;
    const arma::mat INERTIA = def::get_rectangle_inertia(
                CHORD, SPAN, WIDTH, def::AIR_DENSITY, TRANS_PARA_BODY_VECTOR
                );

    void update(const arma::vec& velocity, const arma::vec& ang_velocity);

    PState state;

    Aileron aileron;

public:

    Parafoil() { }

    void set_brake_angles(const arma::vec& angles);
    arma::vec get_brake_angles() const;

    PState get_state() const;

    arma::mat get_inertia() const;
    arma::mat get_mass() const;
    arma::mat get_apperant_mass_inertia() const;
    arma::mat get_apperant_mass() const;

    arma::vec get_aileron_liftforce(const arma::vec& velocity) const;
    arma::vec get_aileron_dragforce(const arma::vec& velocity) const;

    arma::vec get_aeroliftforce(const arma::vec& velocity, const arma::vec& ang_velocity) const;
    arma::vec get_aerodragforce(const arma::vec& velocity, const arma::vec& ang_velocity) const;

    arma::vec get_aeroforce(const arma::vec& velocity, const arma::vec& ang_velocity) const;
    arma::vec get_apperent_mass_force() const;
    arma::vec get_force(const arma::vec& velocity, const arma::vec& ang_velocity);

    arma::vec get_aeroforce_momentum(const arma::vec& velocity, const arma::vec& ang_velocity) const;
    arma::vec get_apperent_mass_momentum() const;
    arma::vec get_apperent_mass_additional_momentum() const;
    arma::vec get_aerodynamic_momentum(const arma::vec& velocity, const arma::vec& ang_velocity, const arma::vec& euler_angles) const;
    arma::vec get_force_momentum(const arma::vec& velocity, const arma::vec& ang_velocity, const arma::vec& euler_angles);

};



#endif // PARAFOIL_H
