#ifndef MODEL_H
#define MODEL_H

#include "def.h"
#include "parafoil.h"
#include "fuselage.h"

struct State
{
    double fuel = 15;
    double time_step = 0.01;
    double timestamp = 0.0;
    arma::vec angular_vel = arma::zeros(3,1);
    arma::vec euler_angles = arma::zeros(3,1);
    arma::vec linear_velocity = arma::zeros(3,1);
    arma::vec coordinates = arma::zeros(3,1);
    arma::vec wind_force_geog = arma::zeros(3,1);
};

class Model
{

    State state;

    Parafoil parafoil;
    Fuselage fuselage;

    void update()
    {
        if(state.fuel <= 0)
            fuselage.set_thrust(0);
        double alpha = 0.15;
        state.fuel -= state.time_step * 0.1 * (1 + alpha*(norm(fuselage.get_thrust()) - 8.4));

        State tmp(state);

        tmp.angular_vel = consecutive_angular_velocity();
        tmp.euler_angles = consecutive_euler_angles();
        tmp.linear_velocity = consecutive_linear_velocity();
        tmp.coordinates = consecutive_coordinates();
        tmp.timestamp = state.timestamp + state.time_step;

        state = tmp;
    }

    arma::vec consecutive_angular_velocity()
    {
        arma::vec body_linear_velocity = inv(coord_rotational_matrix(state.euler_angles)) * state.linear_velocity;

        arma::vec momentum = fuselage.get_force_momentum(body_linear_velocity, state.angular_vel)
                + parafoil.get_force_momentum(body_linear_velocity, state.angular_vel, state.euler_angles);

        return state.angular_vel
                + inv(fuselage.get_inertia()) * momentum * state.time_step
                - inv(fuselage.get_inertia()) * skew_matrix(state.angular_vel) * fuselage.get_inertia() * state.angular_vel* state.time_step;
    }

    arma::vec consecutive_euler_angles() const
    {
        return angular_velocity_rotational_matrix(state.euler_angles) * state.angular_vel * state.time_step + state.euler_angles;
    }

    arma::vec consecutive_linear_velocity()
    {
        arma::vec body_linear_velocity = inv(coord_rotational_matrix(state.euler_angles)) * state.linear_velocity;

        arma::vec force = fuselage.get_force(body_linear_velocity, state.angular_vel, state.euler_angles)
                + parafoil.get_force(body_linear_velocity, state.angular_vel);

        return coord_rotational_matrix(state.euler_angles) * (body_linear_velocity
                + arma::inv(fuselage.get_mass()) * force * state.time_step
                - skew_matrix(state.angular_vel) * body_linear_velocity * state.time_step);
    }

    arma::vec consecutive_coordinates() const
    {
        return state.coordinates + state.linear_velocity * state.time_step;
    }

    typedef void (*func)(Fuselage& fuselage, Parafoil& parafoil);

    func algorithm = nullptr;

public:
    void refill(double fuel) {state.fuel += fuel;}
    void set_coordinates(const arma::vec& coordinates) {state.coordinates = coordinates;}
    void set_linear_velocity(const arma::vec& linear_velocity) {state.linear_velocity = linear_velocity;}


    void operator[] (func alg) {algorithm = alg;}

    void operator() (std::ostream& os, double time);

};

#endif // MODEL_H
