#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "def/def.h"
#include "model.h"

//typedef void (*func)(Fuselage& fuselage, Parafoil& parafoil);


///note алгоритмы управления

namespace  alg
{

double base_thrust = 4;//5.2; // базовая тяга
double height = 50; // высота для удержания

/**
 * @brief Алгоритм свободного полета. Нулевая тяга и нулевой затяжение элеронов.
 *
 */
void algorithm_gliding(Fuselage& fuselage, Parafoil& parafoil, State& state)
{
    fuselage.set_thrust(0);
    parafoil.set_brake_angles(arma::colvec({0,0}));
}

/**
 * @brief Алгоритм поддержания высоты. Измеряемые выходы: высота полета и y-компонента угловой скорости.
 *        Управляющие выходы: сила тяги. Задающие воздействия: высота удержания и базовая тяга
 *        горизонтального полета.
 */
void algorithm_keeping_heigth(Fuselage& fuselage, Parafoil& parafoil, State& state)
{
    double thrust = 0.9 * (alg::height + state.coordinates(2));
    thrust -= 20 * (state.angular_vel_fuselage(1) + state.angular_vel_parafoil(1));
    thrust += base_thrust;

    if(thrust < 0) thrust = 0;
    if(thrust > 15) thrust = 15;

    fuselage.set_thrust(thrust);
}

double base = 0;
double mean = 0.;
double test_mean = 0;

std::ofstream file("graph");

void algorithm_minization_fuel_consumption(Fuselage& fuselage, Parafoil& parafoil, State& state)
{
    const double amplitude = 0.005;
    const double freqency = 20;

    double signal = amplitude * sin(M_PI * freqency * state.timestamp);

    algorithm_keeping_heigth(fuselage, parafoil, state);

    double coef = 2;

    double maxAngle = M_PI/3;

    if(base > maxAngle/coef-amplitude) base = maxAngle/coef-amplitude;
    if(amplitude > base) base = amplitude;

    double ailerons = coef*(base + signal);

    if(ailerons>maxAngle) ailerons = maxAngle;

    parafoil.set_brake_angles(arma::colvec({ailerons, ailerons}));

    double lift = parafoil.get_state().coef_lift;
    double drag = parafoil.get_state().coef_drag;


//    double lift = norm(
//                parafoil.get_aeroliftforce(state.linear_velocity,state.angular_vel_parafoil, state.euler_angles_parafoil)
//                + parafoil.get_aileron_liftforce(state.linear_velocity,state.angular_vel_parafoil, state.euler_angles_parafoil)
//                );
//    double drag = norm(
//                parafoil.get_aerodragforce(state.linear_velocity,state.angular_vel_parafoil, state.euler_angles_parafoil)
//                + parafoil.get_aileron_dragforce(state.linear_velocity,state.angular_vel_parafoil, state.euler_angles_parafoil)
//                //+ fuselage.get_aeroforce(state.linear_velocity,state.angular_vel_fuselage, state.euler_angles_fuselage)
//                );

    double n_output = 0;
    if(drag != 0)
        n_output = lift/drag;

    double signal_shifted = amplitude * sin(M_PI * freqency * (state.timestamp-state.time_step));

    double T_f = 0.9;
    mean += state.time_step/T_f * (n_output * signal - mean);

    base += 10000. * mean  * state.time_step;

   // std::cout << ailerons << std::endl;


}

double mean_ = 0;

void algorithm_minization_fuel_consumption_modified(Fuselage& fuselage, Parafoil& parafoil, State& state)
{
    algorithm_keeping_heigth(fuselage, parafoil, state);

    const double amplitude = 0.005;
    const double freqency =  20;
    double test_signal = amplitude * sin(M_PI * freqency * state.timestamp);

    double ailerons = base + test_signal;

    if(0 > ailerons) ailerons = 0;
    if(ailerons > M_PI/4) ailerons = M_PI/4;

    parafoil.set_brake_angles(arma::colvec({ailerons, ailerons}));

    double lift = norm(
                parafoil.get_aeroliftforce(state.linear_velocity,state.angular_vel_parafoil, state.euler_angles_parafoil)
                + parafoil.get_aileron_liftforce(state.linear_velocity,state.angular_vel_parafoil, state.euler_angles_parafoil)
                );
    double drag = norm(
                parafoil.get_aerodragforce(state.linear_velocity,state.angular_vel_parafoil, state.euler_angles_parafoil)
                + parafoil.get_aileron_dragforce(state.linear_velocity,state.angular_vel_parafoil, state.euler_angles_parafoil)
                //+ fuselage.get_aeroforce(state.linear_velocity,state.angular_vel_fuselage, state.euler_angles_fuselage)
                );

    double n_output = 0;
    if(drag != 0)
        n_output = lift/drag;

    file << base << " " << mean_ << " " << test_mean << std::endl;

    double tmp = n_output * test_signal;

    double nu = 50;

    mean_ = tmp/nu + (nu-1)/nu*mean_;

    double coef = 200;

    base += coef * mean_  * state.time_step;
}

}

#endif // ALGORITHM_H
