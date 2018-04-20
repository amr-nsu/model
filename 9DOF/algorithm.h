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

    double lift = parafoil.get_state().coef_lift - 0.21 * parafoil.get_state().delta_s;
    double drag = parafoil.get_state().coef_drag  - 0.3 * parafoil.get_state().delta_s;


    double n_output = 0;
    if(drag != 0)
        n_output = lift/drag;

    double T_f = 0.9;
    mean += state.time_step/T_f * (n_output * signal - mean);

    base += 10000. * mean  * state.time_step;

   // std::cout << ailerons << std::endl;


}


}

#endif // ALGORITHM_H
