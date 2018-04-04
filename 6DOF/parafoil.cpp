#include "parafoil.h"

using namespace arma;
using namespace def;

std::ostream& operator << (std::ostream& os, const PState& state)
{
    os << state.air_pressure << " "
       << state.coef_lift << " "
       << state.coef_drag << " "
       << state.angle_of_attack;

    return os;
}

PState Parafoil::get_state() const
{
    return state;
}


void Parafoil::update(const vec& velocity, const vec& ang_velocity)
{
    vec para_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_PARA_BODY_VECTOR);
    double para_velocity_norm = norm(para_velocity);

    state.angle_of_attack = get_angle_of_attack(para_velocity);
    state.coef_lift = COEF_LIFT_BASE + COEF_LIFT_ALPHA * state.angle_of_attack;
    state.coef_drag = COEF_DRAG_BASE + COEF_DRAG_ALPHA * state.angle_of_attack
            + COEF_DRAG_ALPHA_2 * state.angle_of_attack * state.angle_of_attack;

    state.air_pressure = 0.5 * AIR_DENSITY * AREA * para_velocity_norm * para_velocity_norm;
}

vec Parafoil::get_aileron_liftforce(const vec& velocity) const
{
  return aileron.get_liftforce(velocity, state.air_pressure);
}
vec Parafoil::get_aileron_dragforce(const vec& velocity) const
{
    return aileron.get_dragforce(velocity, state.air_pressure);
}

void Parafoil::set_brake_angles(const vec& angles)
{
    aileron(angles);
}

vec Parafoil::get_brake_angles() const
{
    return aileron();
}

vec Parafoil::get_aerodragforce(const vec& velocity, const vec& ang_velocity) const
{
    vec para_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_PARA_BODY_VECTOR);

    return get_dragforce_direction(para_velocity)* state.coef_drag * state.air_pressure;
}

vec Parafoil::get_aeroliftforce(const vec& velocity, const vec& ang_velocity) const
{
    vec para_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_PARA_BODY_VECTOR);

    return get_liftforce_direction(para_velocity)* state.coef_lift * state.air_pressure;
}

vec Parafoil::get_aeroforce(const vec& velocity, const vec& ang_velocity) const
{
    vec para_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_PARA_BODY_VECTOR);

    vec aeroliftforce = get_liftforce_direction(para_velocity)* state.coef_lift * state.air_pressure;
    vec aerodragforce = get_dragforce_direction(para_velocity)* state.coef_drag * state.air_pressure;

    return aeroliftforce + aerodragforce;
}

vec Parafoil::get_aeroforce_momentum(const vec& velocity, const vec& ang_velocity) const
{
    return skew_matrix(TRANS_PARA_BODY_VECTOR) * get_aeroforce(velocity, ang_velocity);
}

vec Parafoil::get_apperent_mass_force() const
{
    return zeros(3,1);
}

vec Parafoil::get_apperent_mass_momentum() const
{
    return get_apperent_mass_additional_momentum()
            + skew_matrix(TRANS_PARA_BODY_VECTOR) * get_apperent_mass_force();
}

vec Parafoil::get_apperent_mass_additional_momentum() const
{
    return zeros(3,1);
}

vec Parafoil::get_aerodynamic_momentum(const vec& velocity, const vec& ang_velocity, const vec& euler_angles) const
{
    vec para_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_PARA_BODY_VECTOR);
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
    update(velocity, ang_velocity);

    return get_aeroforce_momentum(velocity, ang_velocity) + get_aerodynamic_momentum(velocity, ang_velocity, euler_angles) +
            aileron.get_force_momentum(state.air_pressure,SPAN/FLAP_WIDTH) + get_apperent_mass_momentum();
}

vec Parafoil::get_force(const vec& velocity, const vec& ang_velocity)
{
    update(velocity, ang_velocity);

    return get_aeroforce(velocity, ang_velocity) + aileron.get_force(velocity, state.air_pressure) + get_apperent_mass_force();
}

mat Parafoil::get_mass() const
{
   return eye(3,3) * MASS;
}

mat Parafoil::get_inertia() const
{
   return INERTIA;
}
