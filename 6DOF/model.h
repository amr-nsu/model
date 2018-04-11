#ifndef MODEL_H
#define MODEL_H

#include "def/def.h"
#include "parafoil.h"
#include "fuselage.h"

struct State
{
    double fuel = 10;
    double time_step = 0.01;
    double timestamp = 0.0;
    arma::vec angular_vel = arma::zeros(3,1);
    arma::vec euler_angles = arma::zeros(3,1);
    arma::vec linear_velocity = arma::zeros(3,1);
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

    arma::vec consecutive_angular_velocity();
    arma::vec consecutive_euler_angles() const;
    arma::vec consecutive_linear_velocity();
    arma::vec consecutive_coordinates() const;

    //see diploma
    arma::vec calculate_F(const arma::vec& velocity, const arma::vec& ang_velocity, const arma::vec& euler_angles);

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
