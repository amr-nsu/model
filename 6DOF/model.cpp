#include "model.h"

using namespace arma;

std::ostream& operator << (std::ostream& os, const State& state)
{
    vec velocity = def::coord_rotational_matrix(state.euler_angles)*state.linear_velocity;

    os << state.timestamp << " "
       << state.angular_vel(0) << " "
       << state.angular_vel(1) << " "
       << state.angular_vel(2) << " "
       << state.euler_angles(0) << " "
       << state.euler_angles(1) << " "
       << state.euler_angles(2) << " "
       << velocity(0) << " "
       << velocity(1) << " "
       << velocity(2) << " "
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
        update();
        algorithm(fuselage, parafoil, state);
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

    vec a = zeros(12,1);
    a.rows(0,2) = state.linear_velocity;
    a.rows(3,5) = state.angular_vel;
    a.rows(6,8) = state.coordinates;
    a.rows(9,11) = state.euler_angles;

    vec n = a+ state.time_step * calculate_F(state.linear_velocity, state.angular_vel, state.euler_angles);

    tmp.angular_vel = n.rows(3,5);//consecutive_angular_velocity();
    tmp.euler_angles = n.rows(9,11);//consecutive_euler_angles();
    tmp.linear_velocity = n.rows(0,2);//consecutive_linear_velocity();
    tmp.coordinates = n.rows(6,8);//consecutive_coordinates();
    tmp.timestamp = state.timestamp + state.time_step;

    state = tmp;
}

vec Model::consecutive_euler_angles() const
{
    return def::angular_velocity_rotational_matrix(state.euler_angles) * state.angular_vel * state.time_step + state.euler_angles;
}

vec Model::consecutive_coordinates() const
{
    return state.coordinates + def::coord_rotational_matrix(state.euler_angles)*state.linear_velocity * state.time_step;
}

arma::vec Model::calculate_F(const arma::vec& velocity, const arma::vec& ang_velocity, const arma::vec& euler_angles)
{
    arma::mat A = arma::zeros(6,6);
    arma::vec M = arma::zeros(6,1);
    arma::vec H;
    arma::vec L;

    arma::mat mass = fuselage.get_mass()+parafoil.get_mass();

    arma::mat inertia = fuselage.get_inertia()+parafoil.get_inertia();

    H = fuselage.get_force(velocity, ang_velocity, euler_angles)
        + parafoil.get_force(velocity, ang_velocity)
        - mass*def::skew_matrix(ang_velocity)*velocity;

    L = fuselage.get_force_momentum(velocity, ang_velocity)
        + parafoil.get_force_momentum(velocity, ang_velocity, euler_angles)
        - def::skew_matrix(ang_velocity)*inertia*ang_velocity;

    M.rows(0, 2) = H;
    M.rows(3, 5) = L;

    A.rows(0, 2).cols(0,2) = mass;
    A.rows(3, 5).cols(3,5) = inertia;

    arma::vec F = arma::zeros(12,1);

    F.rows(0, 5) = inv(A)*M;
    F.rows(6,8) = def::coord_rotational_matrix(euler_angles) * velocity;
    F.rows(9,11) = def::angular_velocity_rotational_matrix(euler_angles) * ang_velocity;

    return std::move(F);
}
