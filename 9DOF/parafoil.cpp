#include "parafoil.h"

using namespace arma;
using namespace def;

std::ostream& operator << (std::ostream& os, const PState& state)
{
    os << state.air_pressure << " "
       << state.coef_lift << " "
       << state.coef_drag << " "
       << state.angle_of_attack  << " "
       << state.delta_a << " "
       << state.delta_s;

    return os;
}

PState Parafoil::get_state() const
{
    return state;
}

void Parafoil::update(const vec& velocity, const vec& ang_velocity, const vec& euler_angles)
{
    vec para_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_PARA_BODY_VECTOR, euler_angles);
    double para_velocity_norm = norm(para_velocity);

    state.angle_of_attack = get_angle_of_attack(para_velocity);
    state.coef_lift = COEF_LIFT_BASE + COEF_LIFT_ALPHA * state.angle_of_attack;
    state.coef_drag = COEF_DRAG_BASE + COEF_DRAG_ALPHA * state.angle_of_attack
            + COEF_DRAG_ALPHA_2 * state.angle_of_attack * state.angle_of_attack;

    state.air_pressure = 0.5 * AIR_DENSITY * AREA * para_velocity_norm * para_velocity_norm;

    state.delta_a = aileron.get_control()(0);
    state.delta_s = aileron.get_control()(1);

}

vec Parafoil::get_aileron_dragforce(const vec& velocity, const vec& ang_velocity, const vec& euler_angles) const
{
    vec para_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_PARA_BODY_VECTOR, euler_angles);

    return aileron.get_dragforce(para_velocity, state.air_pressure);
}

vec Parafoil::get_aileron_liftforce(const vec& velocity, const vec& ang_velocity, const vec& euler_angles) const
{
    vec para_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_PARA_BODY_VECTOR, euler_angles);

    return aileron.get_liftforce(velocity, state.air_pressure);
}

void Parafoil::set_brake_angles(const vec& angles)
{
    aileron(angles);
}

vec Parafoil::get_brake_angles() const
{
    return aileron();
}

vec Parafoil::get_aerodragforce(const vec& velocity, const vec& ang_velocity, const vec& euler_angles) const
{
    vec para_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_PARA_BODY_VECTOR, euler_angles);

    return get_dragforce_direction(para_velocity)* state.coef_drag * state.air_pressure;
}

vec Parafoil::get_aeroliftforce(const vec& velocity, const vec& ang_velocity, const vec& euler_angles) const
{
    vec para_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_PARA_BODY_VECTOR, euler_angles);

    return get_liftforce_direction(para_velocity)* state.coef_lift * state.air_pressure;
}

vec Parafoil::get_apperent_mass_force(const vec& velocity, const vec& ang_velocity, const vec& euler_angles) const
{
    vec para_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_PARA_BODY_VECTOR, euler_angles);

    mat ang_mat = def::skew_matrix(ang_velocity);

    return get_apperant_mass() * ang_mat * ang_mat * TRANS_PARA_BODY_VECTOR - ang_mat * get_apperant_mass() * para_velocity;
}

vec Parafoil::get_apperent_mass_momentum(const vec& velocity, const vec& ang_velocity, const vec& euler_angles) const
{
    vec para_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_PARA_BODY_VECTOR, euler_angles);

    def::skew_matrix(ang_velocity);

    return - def::skew_matrix(ang_velocity)*get_apperant_mass_inertia()* ang_velocity
           - def::skew_matrix(para_velocity)*get_apperant_mass()* para_velocity;
}

vec Parafoil::get_aerodynamic_momentum(const vec& velocity, const vec& ang_velocity, const vec& euler_angles) const
{
    vec para_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_PARA_BODY_VECTOR, euler_angles);
    double para_velocity_norm = norm(para_velocity);

    if (para_velocity_norm == 0)
       return zeros(3,1);

    vec tmp = zeros(3);

    double p = ang_velocity(0);
    double q = ang_velocity(1);
    double r = ang_velocity(2);
    double phi = euler_angles(0);

    tmp(0) = (COEF_l_p * SPAN * SPAN * p) / (2 * para_velocity_norm) + COEF_l_phi * SPAN * phi;
    tmp(1) = (COEF_m_q * CHORD * CHORD * q) / (2 * para_velocity_norm)
            + COEF_m_base * CHORD + COEF_m_alpha * CHORD * state.angle_of_attack;
    tmp(2) = (COEF_n_r * SPAN * SPAN * r) / (2 * para_velocity_norm);

    return tmp * state.air_pressure;
}

vec Parafoil::get_force_momentum(const vec& velocity, const vec& ang_velocity, const vec& euler_angles)
{
    update(velocity, ang_velocity, euler_angles);

    return get_aerodynamic_momentum(velocity, ang_velocity, euler_angles)
            + aileron.get_force_momentum(state.air_pressure,SPAN/FLAP_WIDTH)
            + get_apperent_mass_momentum(velocity, ang_velocity, euler_angles);
}

vec Parafoil::get_force(const vec& velocity, const vec& ang_velocity, const vec& euler_angles)
{
    update(velocity, ang_velocity, euler_angles);

    vec para_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_PARA_BODY_VECTOR, euler_angles);

    return get_aeroliftforce(velocity, ang_velocity, euler_angles) + get_aerodragforce(velocity, ang_velocity, euler_angles)
            + aileron.get_force(para_velocity, state.air_pressure) + get_apperent_mass_force(velocity, ang_velocity, euler_angles)
            + get_weight_force(euler_angles);
}

vec Parafoil::get_weight_force(const vec& euler_angles) const
{
    mat tmp = coord_rotational_matrix(euler_angles);

    return tmp * MASS * GRAVITY_ACCELERATION;
}

mat Parafoil::get_apperant_mass() const
{
    mat tmp =  zeros(3,3);

    double ka = 0.848*math::pi()/4;
    double kb = 0.339*math::pi()/4;
    double kc = SPAN/(SPAN+CHORD)*math::pi()/4;

    double A = ka*def::AIR_DENSITY*std::pow(FLAP_WIDTH,2)*SPAN*(1+8/3*std::pow(ARC_HEIGHT,3));
    double B = kb*def::AIR_DENSITY*CHORD*(std::pow(FLAP_WIDTH,2)+2*std::pow(ARC_HEIGHT,2)*(1-std::pow(FLAP_WIDTH,2)));
    double C = kc*def::AIR_DENSITY*std::pow(CHORD,2)*SPAN*std::sqrt(1+2*std::pow(ARC_HEIGHT,2)*(1-std::pow(FLAP_WIDTH,2)));

    tmp(0, 0) = A;
    tmp(1, 1) = B;
    tmp(2, 2) = C;

    return std::move(tmp);
}

mat Parafoil::get_apperant_mass_inertia() const
{
    mat tmp =  zeros(3,3);

    double ka = 0.055*SPAN/(SPAN+CHORD);
    double kb = 0.0308*SPAN/(SPAN+CHORD);
    double kc = 0.0555;

    double A = ka*def::AIR_DENSITY*std::pow(CHORD,2)*std::pow(SPAN,3);
    double B = kb*def::AIR_DENSITY*std::pow(CHORD,4)*SPAN*(1+math::pi()/6*(1+SPAN/CHORD)*SPAN/CHORD*std::pow(ARC_HEIGHT*FLAP_WIDTH,2));
    double C = kc*def::AIR_DENSITY*std::pow(FLAP_WIDTH,2)*std::pow(SPAN,3)*(1+8*std::pow(ARC_HEIGHT,2));

    tmp(0, 0) = A;
    tmp(1, 1) = B;
    tmp(2, 2) = C;

    return std::move(tmp);
}

mat Parafoil::get_mass() const
{
   return eye(3,3) * MASS;
}

mat Parafoil::get_inertia() const
{
   return INERTIA;
}

vec Parafoil::get_displacement()const
{
    return TRANS_PARA_BODY_VECTOR;
}
