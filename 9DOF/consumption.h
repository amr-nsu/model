#ifndef CONSUMPTION_H
#define CONSUMPTION_H

#include "def/def.h"
#include "model.h"

namespace con
{
    void no_consuption(Fuselage& fuselage, Parafoil& parafoil, State& state) { }

    void linear_consuption(Fuselage& fuselage, Parafoil& parafoil, State& state)
    {
        const double alpha = 0.0739668225475;
        state.fuel -= state.time_step * alpha;
    }

    void linear_thrust_depended_consuption(Fuselage& fuselage, Parafoil& parafoil, State& state)
    {
        const double alpha = 0.0192482091823;
        state.fuel -= state.time_step * alpha * fuselage.get_state().thrust;
    }

    void theoretical_consuption(Fuselage& fuselage, Parafoil& parafoil, State& state)
    {
        double coef = 0.01;
        double weight = arma::norm(fuselage.get_weight_force(state.euler_angles_fuselage)
                                   + parafoil.get_weight_force(state.euler_angles_parafoil));

        double lift = parafoil.get_state().coef_lift;
        double drag = parafoil.get_state().coef_drag;

        double func = lift/drag;

        state.fuel -= state.time_step * coef * weight *norm(state.linear_velocity) / func;
    }

}

#endif // CONSUMPTION_H
