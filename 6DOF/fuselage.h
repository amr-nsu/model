#ifndef FUSELAGE_H
#define FUSELAGE_H

#include "def/def.h"

struct FState
{
    double thrust = 0;
};

std::ostream& operator << (std::ostream& os, const FState& state);


class Fuselage
{
    const double AREA = 0.4 * 0.15;
    const double MASS = 2133 * 1e-3;

    const double COEF_DRAG_BASE = 0.125; //denoted by C_F_DO
    const double COEF_DRAG_ALPHA = 0.95; //denoted by C_F_DAlpha

    const arma::vec TRANS_FUSE_BODY_VECTOR = arma::colvec({0.037, 0, 0.149}); // denoted by S_FB
    const arma::vec TRANS_MOTOR_BODY_VECTOR = arma::colvec({0.037, 0, 0.139});

    double thrust = 0;

    ////public static final	double [][] inertiaArr = {{0.336, 0, -0.059}, {0, 0.292, 0}, {-0.059, 0, 0.109}}; // 0,0.292,0
    //public static final	double [][] inertiaArr = {{0.423, 0, -0.03}, {0, 0.401, 0}, {-0.03, 0, 0.053}}; // 0,0.292,0

    const arma::mat INERTIA = def::get_rectangle_inertia(
                0.17, 0.15, 0.4, MASS/(0.17* 0.15 *0.4), TRANS_FUSE_BODY_VECTOR
                );
    //= arma::mat ({{0.423, 0, -0.03}, {0, 0.401, 0}, {-0.03, 0, 0.053}});



    FState state;

    void update(void);

public:
    Fuselage() { }

    FState get_state() const;


    void set_thrust(double thrust);

    arma::mat get_inertia() const;
    arma::mat get_mass() const;

    arma::vec get_thrust() const;
    arma::vec get_weight_force(const arma::vec& eulerAngles) const;
    arma::vec get_aeroforce(const arma::vec& velocity, const arma::vec& ang_velocity) const;
    arma::vec get_force(const arma::vec& velocity, const arma::vec& ang_velocity, const arma::vec& euler_angles);

    arma::vec get_aeroforce_momentum(const arma::vec& velocity, const arma::vec& ang_velocity) const;
    arma::vec get_thrust_momentum() const;
    arma::vec get_force_momentum(const arma::vec& velocity, const arma::vec& ang_velocity);


};

#endif // FUSELAGE_H
