#include "model.h"

using namespace arma;

std::ostream& operator << (std::ostream& os, const State& state)
{
    os << state.timestamp << " "
       << state.angular_vel(0) << " "
       << state.angular_vel(1) << " "
       << state.angular_vel(2) << " "
       << state.euler_angles(0) << " "
       << state.euler_angles(1) << " "
       << state.euler_angles(2) << " "
       << state.linear_velocity(0) << " "
       << state.linear_velocity(1) << " "
       << state.linear_velocity(2) << " "
       << state.coordinates(0) << " "
       << state.coordinates(1) << " "
       << -state.coordinates(2) << " "
       << state.wind_force_geog(0) << " "
       << state.wind_force_geog(1) << " "
       << state.wind_force_geog(2) << " "
       << state.fuel;
    return os;
}

void Model::operator() (std::ostream& os, double time)
{
    os << "time" << " " << "w_x" << " " << "w_y" << " " << "w_z" << " " << "phi" << " "
       << "theta" << " " << "psi" << " " << "v_x" << " " << "v_y" << " " << "v_z" << " "
       << "x" << " " << "y" << " " << "z" << " " << "wind_x" << " " << "wind_y" << " "
       << "wind_z" << " " << "fuel" << " " << "air_pressure" << " " << "coef_lift" << " "
       << "coef_drag" << " " << "angle_of_attack" << " " << "thrust" << std::endl;

    for(double t = 0; (t < time) & (state.coordinates(2) < 0); t+=state.time_step)
    {
        algorithm(fuselage, parafoil, state);
        update();
        os << state << " " << parafoil.get_state() << " " << fuselage.get_state() << std::endl;
    }
}

void Model::update()
{
    if(state.fuel >= 0)
        consumption(fuselage, parafoil, state);
    else
    {
        fuselage.set_thrust(0);
        parafoil.set_brake_angles(vec({0,0}));
    }

    State tmp(state);

    tmp.angular_vel = consecutive_angular_velocity();
    tmp.euler_angles = consecutive_euler_angles();
    tmp.linear_velocity = consecutive_linear_velocity();
    tmp.coordinates = consecutive_coordinates();
    tmp.timestamp = state.timestamp + state.time_step;

    state = tmp;
}

vec Model::consecutive_angular_velocity()
{
    vec body_linear_velocity = inv(def::coord_rotational_matrix(state.euler_angles)) * state.linear_velocity;

    vec momentum = fuselage.get_force_momentum(body_linear_velocity, state.angular_vel)
            + parafoil.get_force_momentum(body_linear_velocity, state.angular_vel, state.euler_angles);

    mat inertia = fuselage.get_inertia() + parafoil.get_inertia();

    return state.angular_vel
            + inv(inertia) * momentum * state.time_step
            - inv(inertia) * def::skew_matrix(state.angular_vel) * (inertia) * state.angular_vel* state.time_step;
}

vec Model::consecutive_euler_angles() const
{
    return def::angular_velocity_rotational_matrix(state.euler_angles) * state.angular_vel * state.time_step + state.euler_angles;
}

vec Model::consecutive_linear_velocity()
{
    vec body_linear_velocity = inv(def::coord_rotational_matrix(state.euler_angles)) * state.linear_velocity;

    vec force = fuselage.get_force(body_linear_velocity, state.angular_vel, state.euler_angles)
            + parafoil.get_force(body_linear_velocity, state.angular_vel);

    return def::coord_rotational_matrix(state.euler_angles) * (body_linear_velocity
            + inv(fuselage.get_mass() + parafoil.get_mass()) * force * state.time_step
            - def::skew_matrix(state.angular_vel) * body_linear_velocity * state.time_step);
}

vec Model::consecutive_coordinates() const
{
    return state.coordinates + state.linear_velocity * state.time_step;
}
