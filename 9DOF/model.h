#ifndef MODEL_H
#define MODEL_H

#include "def/def.h"
#include "parafoil.h"
#include "fuselage.h"

struct State
{
    double fuel = 15;
    double time_step = 0.01;
    double timestamp = 0.0;
    arma::vec angular_vel_parafoil = arma::zeros(3,1);
    arma::vec angular_vel_fuselage = arma::zeros(3,1);
    arma::vec euler_angles_parafoil = arma::zeros(3,1);
    arma::vec euler_angles_fuselage = arma::zeros(3,1);
    arma::vec linear_velocity = arma::zeros(3,1);
    arma::vec react_force = arma::zeros(3,1);
    arma::vec coordinates = arma::zeros(3,1);
    arma::vec wind_force_geog = arma::zeros(3,1);
};

std::ostream& operator << (std::ostream& os, const State& state);


class Model
{
    State state;

    Parafoil parafoil;
    Fuselage fuselage;

    void update();

    arma::vec eulerMethod(const arma::vec& n);
    arma::vec rungeKuttaMethod(const arma::vec& n);


    //see diploma
    arma::vec calculate_F(const arma::vec& n);

    typedef void (*Algorithm)(Fuselage& fuselage, Parafoil& parafoil, State& state);

    Algorithm algorithm = nullptr;
    Algorithm consumption = nullptr;

public:

    void refill(double fuel) {state.fuel += fuel;}
    void set_coordinates(const arma::vec& coordinates) {state.coordinates = coordinates;}
    void set_linear_velocity(const arma::vec& linear_velocity) {state.linear_velocity = linear_velocity;}


    void operator[] (Algorithm alg) {algorithm = alg;}
    void operator() (Algorithm con) {consumption = con;}


    void operator() (std::ostream& os, double time);

};


#endif // MODEL_H
