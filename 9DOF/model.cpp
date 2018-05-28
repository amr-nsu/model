#include "model.h"

using namespace arma;

std::ostream& operator << (std::ostream& os, const State& state)
{
    os << state.timestamp << " "
       << state.angular_vel_parafoil(0) << " "
       << state.angular_vel_parafoil(1) << " "
       << state.angular_vel_parafoil(2) << " "
       << state.euler_angles_parafoil(0) << " "
       << state.euler_angles_parafoil(1) << " "
       << state.euler_angles_parafoil(2) << " "
       << state.angular_vel_fuselage(0) << " "
       << state.angular_vel_fuselage(1) << " "
       << state.angular_vel_fuselage(2) << " "
       << state.euler_angles_fuselage(0) << " "
       << state.euler_angles_fuselage(1) << " "
       << state.euler_angles_fuselage(2) << " "
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
//    os << "1. time" << " " << "2. wp_x" << " " << "3. wp_y" << " " << "4. wp_z"
//       << "5. ap_x" << " " << "6. ap_y" << " " << "7. ap_z"
//       << "8. wb_x" << " " << "9. wb_y" << " " << "10. wb_z"
//       << "11. ab_x" << " " << "12. ab_y" << " " << "13. ab_z"
//       << "14. v_x" << " " << "15. v_y" << " " << "16. v_z" << " "
//       << "17. x" << " " << "18. y" << " " << "19. z" << " " << "20. wind_x" << " " << "21. wind_y" << " "
//       << "22. wind_z" << " " << "23. fuel" << " " << "24. air_pressure" << " " << "25.coef_lift" << " "
//       << "26. coef_drag" << " " << "27. angle_of_attack" << " " << "28. delta_s" << " " << "29. delta_a" << " " << "thrust" << std::endl;

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

    vec a = zeros(18,1);
    a.rows(0,2) = state.linear_velocity;
    a.rows(3,5) = state.angular_vel_parafoil;
    a.rows(6,8) = state.angular_vel_fuselage;
    a.rows(9,11) = state.euler_angles_fuselage;
    a.rows(12,14) = state.euler_angles_parafoil;
    a.rows(15,17) = state.coordinates;

    vec n = rungeKuttaMethod(a);
    //vec n = eulerMethod(a);

    tmp.linear_velocity = n.rows(0,2);
    tmp.angular_vel_parafoil = n.rows(3,5);
    tmp.angular_vel_fuselage = n.rows(6,8);
    tmp.euler_angles_fuselage = n.rows(9,11);
    tmp.euler_angles_parafoil = n.rows(12,14);
    tmp.coordinates = n.rows(15,17);
    tmp.timestamp = state.timestamp + state.time_step;

    state = tmp;
}

vec Model::eulerMethod(const vec& n)
{
    return n + state.time_step * calculate_F(n);
}

vec Model::rungeKuttaMethod(const vec& n)
{
    double h = state.time_step;
    vec k1 = h * calculate_F(n);
    vec k2 = h * calculate_F(n+k1*0.5);
    vec k3 = h * calculate_F(n+k2*0.5);
    vec k4 = h * calculate_F(n+k3);

    return n + (k1 + 2*k2 + 2*k3 + k4)/6;
}

vec Model::calculate_F(const vec& n)
{
    vec velocity = n.rows(0,2);
    vec angular_vel_parafoil = n.rows(3,5);
    vec angular_vel_fuselage = n.rows(6,8);
    vec euler_angles_fuselage = n.rows(9,11);
    vec euler_angles_parafoil = n.rows(12,14);
    vec coordinates = n.rows(15,17);

    mat A = zeros(12,12);
    vec M = zeros(12,1);
    vec M1 = zeros(3,1);
    vec M2 = zeros(3,1);
    vec M3 = zeros(3,1);
    vec M4 = zeros(3,1);

    mat Mc = zeros(3,1);
    Mc(2) = 0.47*(def::get_tilded_phi(euler_angles_parafoil)-def::get_tilded_phi(euler_angles_fuselage))
            +0,035*(def::get_dotted_tilded_phi(euler_angles_parafoil, angular_vel_parafoil)-def::get_dotted_tilded_phi(euler_angles_fuselage, angular_vel_fuselage));

    M1 = parafoil.get_force(velocity, angular_vel_parafoil, euler_angles_parafoil)
            - parafoil.get_mass()*def::skew_matrix(angular_vel_parafoil)
            *def::skew_matrix(angular_vel_parafoil)*parafoil.get_displacement();

    M2 = parafoil.get_force_momentum(velocity, angular_vel_parafoil, euler_angles_parafoil)
            - def::coord_rotational_matrix(euler_angles_parafoil)
            *def::coord_rotational_matrix(euler_angles_fuselage).t()*Mc
            -def::skew_matrix(angular_vel_parafoil)*parafoil.get_inertia()
            *angular_vel_parafoil;

    M3 = fuselage.get_force(velocity, angular_vel_fuselage, euler_angles_fuselage)
            - fuselage.get_mass()*def::skew_matrix(angular_vel_fuselage)
            *def::skew_matrix(angular_vel_fuselage)*fuselage.get_displacement();

    M4 = Mc - def::skew_matrix(angular_vel_fuselage)
            *fuselage.get_inertia()*angular_vel_fuselage;

    M.rows(0, 2) = M1;
    M.rows(3, 5) = M2;
    M.rows(6, 8) = M3;
    M.rows(9, 11) = M4;

    A.rows(0, 2).cols(0,2) = (parafoil.get_mass() + parafoil.get_apperant_mass())*def::coord_rotational_matrix(euler_angles_parafoil);
    A.rows(0, 2).cols(3,5) = -(parafoil.get_mass() + parafoil.get_apperant_mass()) * def::skew_matrix(parafoil.get_displacement());
    A.rows(0, 2).cols(9,11)= -def::coord_rotational_matrix(euler_angles_parafoil);

    A.rows(3, 5).cols(3,5) = parafoil.get_inertia() + parafoil.get_apperant_mass_inertia();
    A.rows(3, 5).cols(9,11) = def::skew_matrix(parafoil.get_displacement()) * def::coord_rotational_matrix(euler_angles_parafoil);

    A.rows(6,8).cols(0,2) = fuselage.get_mass() * def::coord_rotational_matrix(euler_angles_fuselage);
    A.rows(6,8).cols(6,8) = -fuselage.get_mass() * def::skew_matrix(fuselage.get_displacement());
    A.rows(6,8).cols(9,11) = def::coord_rotational_matrix(euler_angles_fuselage);

    A.rows(9,11).cols(6,8) = fuselage.get_inertia();
    A.rows(9,11).cols(9,11) = - def::skew_matrix(fuselage.get_displacement()) * def::coord_rotational_matrix(euler_angles_fuselage);

    vec F = zeros(18,1);

    F.rows(0, 8) = ((vec)(inv(A)*M)).rows(0, 8);
    F.rows(9,11) = def::angular_velocity_rotational_matrix(euler_angles_fuselage) * angular_vel_fuselage;
    F.rows(12,14) = def::angular_velocity_rotational_matrix(euler_angles_parafoil) * angular_vel_parafoil;
    F.rows(15,17) = velocity;

    return std::move(F);
}
