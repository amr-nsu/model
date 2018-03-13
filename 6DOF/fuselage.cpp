#include "fuselage.h"

using namespace arma;

void Fuselage::set_thrust(double thrust)
{
    this->thrust = thrust;
}

vec Fuselage::get_thrust() const
{
    return colvec({thrust, 0, 0});
}

vec Fuselage::get_thrust_momentum() const
{
    return skew_matrix(TRANS_MOTOR_BODY_VECTOR) * get_thrust();
}

vec Fuselage::get_weight_force(const vec& eulerAngles) const
{
    mat tmp = coord_rotational_matrix(eulerAngles);

    return inv(tmp) * MASS * GRAVITY_ACCELERATION;
}

vec Fuselage::get_aeroforce(const vec& velocity, const vec& ang_velocity) const
{
    vec fuse_velocity = get_linear_velocity(velocity, ang_velocity, TRANS_FUSE_BODY_VECTOR);

    double angle_of_attack = get_angle_of_attack(fuse_velocity);
    double coef_drag = COEF_DRAG_BASE + COEF_DRAG_ALPHA * angle_of_attack * angle_of_attack;

    double fuse_velocity_norm = norm(fuse_velocity);

    return  -0.5 * AIR_DENSITY * AREA * fuse_velocity_norm * coef_drag * fuse_velocity;
}

vec Fuselage::get_aeroforce_momentum(const vec& velocity, const vec& ang_velocity) const
{
    return skew_matrix(TRANS_FUSE_BODY_VECTOR) * get_aeroforce(velocity, ang_velocity);
}

vec Fuselage::get_force_momentum(const vec& velocity, const vec& ang_velocity) const
{
    return  get_aeroforce_momentum(velocity, ang_velocity) + get_thrust_momentum();
}

vec Fuselage::get_force(const vec& velocity, const vec& ang_velocity, const vec& euler_angles) const
{
    return get_weight_force(euler_angles) + get_aeroforce(velocity, ang_velocity) + get_thrust();
}

mat Fuselage::get_mass() const
{
   return eye(3,3) * MASS;
}

mat Fuselage::get_inertia() const
{
   return INERTIA;
}
