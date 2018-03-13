#include "model.h"

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
       << state.fuel << std::endl;
    return os;
}

void Model::operator() (std::ostream& os, double time)
{
    for(double t = 0; t < time; t+=state.time_step)
    {
        algorithm(fuselage, parafoil);
        update();
        os << state;
    }
}


