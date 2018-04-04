#include "aileron.h"

using namespace arma;
using namespace def;

vec Aileron::get_control() const
{
    vec tmp(2);

    tmp(0) = brake_angles(0) - brake_angles(1);
    tmp(1) = std::min(brake_angles(0), brake_angles(1));

    return std::move(tmp);
}

vec Aileron::get_liftforce(const vec& velocity, double air_pressure) const
{
    vec delta = get_control();

    double dsign = sign(delta(0));

    vec lift_direction = get_liftforce_direction(velocity);

    mat tmp(3,2);

    tmp.col(0) = COEF_LIFT_ASYMMETRIC * lift_direction * dsign;
    tmp.col(1) = COEF_LIFT_SYMMETRIC * lift_direction;

    return tmp * air_pressure * delta;
}

vec Aileron::get_dragforce(const vec& velocity, double air_pressure) const
{
    vec delta = get_control();

    double dsign = sign(delta(0));

    vec drag_direction = get_dragforce_direction(velocity);

    mat tmp(3,2);

    tmp.col(0) = COEF_DRAG_ASYMMETRIC * drag_direction * dsign;
    tmp.col(1) = COEF_DRAG_SYMMETRIC * drag_direction;

    return tmp * air_pressure * delta;
}

vec Aileron::get_force(const vec& velocity, double air_pressure) const
{
    vec delta = get_control();

    double dsign = sign(delta(0));

    vec lift_direction = get_liftforce_direction(velocity);
    vec drag_direction = get_dragforce_direction(velocity);

    mat tmp(3,2);

    tmp.col(0) = (COEF_LIFT_ASYMMETRIC * lift_direction + COEF_DRAG_ASYMMETRIC * drag_direction) * dsign;
    tmp.col(1) = (COEF_LIFT_SYMMETRIC * lift_direction + COEF_DRAG_SYMMETRIC * drag_direction);

    return tmp * air_pressure * delta;
}

vec Aileron::get_force_momentum(double air_pressure, double span_to_flap_ratio) const
{
    vec delta = get_control();

    vec tmp = zeros(3,1);
    tmp(0) = C_l_delta_a * span_to_flap_ratio;
    tmp(2) = C_n_delta_a * span_to_flap_ratio;
    return tmp * air_pressure * delta(0);
}
