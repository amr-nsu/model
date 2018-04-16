#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "def/def.h"
#include "model.h"

//typedef void (*func)(Fuselage& fuselage, Parafoil& parafoil);


///note алгоритмы управления

namespace  alg
{

double base_thrust = 8.4; // базовая тяга
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
    thrust -= 20 * state.angular_vel(1);
    thrust += base_thrust;

    if(thrust < 0) thrust = 0;
    if(thrust > 15) thrust = 15;

    fuselage.set_thrust(thrust);
}

double base = 0;
double mean = 0.;

void algorithm_minization_fuel_consumption(Fuselage& fuselage, Parafoil& parafoil, State& state)
{
    const double amplitude = 0.00075;
    const double freqency = 20;

    double T_f = 1.5;

    double test_signal = amplitude * sin(M_PI * freqency * state.timestamp);

    algorithm_keeping_heigth(fuselage, parafoil, state);

    double coef = 4;

    double maxAngle = M_PI/3;

    if(base > maxAngle/coef-amplitude) base = maxAngle/coef-amplitude;
    if(amplitude > base) base = amplitude;

    double ailerons = coef*(base + test_signal);

    if(ailerons>maxAngle) ailerons = maxAngle;

    parafoil.set_brake_angles(arma::colvec({ailerons, ailerons}));

    //double lift = parafoil.get_state().coef_lift;
    //double drag = parafoil.get_state().coef_drag;

    double lift = norm(
                parafoil.get_aeroliftforce(state.linear_velocity, state.angular_vel)
                + parafoil.get_aileron_liftforce(state.linear_velocity, state.angular_vel)
                );
    double drag = norm(
                parafoil.get_aerodragforce(state.linear_velocity, state.angular_vel)
                + parafoil.get_aileron_dragforce(state.linear_velocity, state.angular_vel)
                );

    double n_output = 0;// = lift/drag ? drag != 0 : 0;
    if(drag != 0)
        n_output = lift/drag;

    mean += 1./T_f * (n_output * test_signal - mean) * 0.01;

    base += 200. * mean  * 0.01;

   // std::cout << ailerons << std::endl;


}

//    public void evaluateWithSearch(double _time, double height)
//    {
//            double amplitude = 0.1;
//            double OMEGA = 2.*PI*10;
//            double T_f = 1.5;

//            double T_a = 2;

//            double base = 0;
//            double mean = 0.;

//            double alpha = 0;

//            double eler = 0;
//            int i = 0;

//            for(double time = 0; time <  _time ; time+=0.01)
//            {
//                double test_signal = amplitude * sin(OMEGA * time);

//                double coeft = 25;
//                double coefb = 1.;

//                double thrust = 0.9 * (height+state.coordinates.get(2,0));
//                thrust-= 20*state.angVel.get(1,0);
//                thrust += coeft*(coefb *base + test_signal);
//                thrust+=4.6;
//                if(thrust < 0) thrust = 0;
//                if(thrust > 15) thrust = 15;
//                setThrustBody(thrust);

//                state.update(parafoil, fuselage);

//                alpha = parafoil.getState().attackAngle;//thrust;

//                //double lift = parafoil.getAeroLiftForce(state).normF();
//                //double drag = parafoil.getAeroDragForce(state).normF();

//                double lift = parafoil.getAeroLiftForce(state).normF() + parafoil.getControlBrakeLiftForce(state).normF();
//                double drag = (parafoil.getAeroDragForce(state).normF()+ parafoil.getControlBrakeDragForce(state).normF());

//                //double drag = (parafoil.getAeroDragForce(state).normF()+ parafoil.getControlBrakeDragForce(state).normF()+fuselage.getAeroForce(state).normF());

//                //double lift = parafoil.getAeroLiftForce(state).plus(parafoil.getControlBrakeLiftForce(state)).normF();
//                //double drag = (parafoil.getAeroDragForce(state).plus(parafoil.getControlBrakeDragForce(state)).plus(fuselage.getAeroForce(state)).normF());

//                double n_output = lift/drag;

//                //double n_output = (0.83*alpha + 0.28) / (1.*alpha * alpha - 0.2* alpha + 0.12);

//                //mean += 1/(PI*amplitude*amplitude)*((n_output*test_signal)*0.01*OMEGA);

//                mean += 1./T_f * (n_output * amplitude * sin(OMEGA * time) - mean) * 0.01;

//                base += 200. * mean  * 0.01;

//            }

//        }
//    }


//    public void evaluateWithSearchEngine(double _time, double height)
//    {
//            double amplitude = 0.007;
//            double OMEGA = 2.*PI*10;
//            double T_f = 1.5;

//            double T_a = 2;

//            double base = 0;
//            double mean = 0.;

//            double alpha = 0;

//            double intb = 0;

//            for(double time = 0; time <  _time ; time+=0.01)
//            {
//                double test_signal = amplitude * sin(OMEGA * time);

//                double coeft = 5;

//                double thrust = 0.9 * (height+state.coordinates.get(2,0));
//                thrust-= 20*state.angVel.get(1,0);
//                thrust+= coeft*base;
//                thrust+=1.*intb;
//                if(thrust < 0) thrust = 0;
//                if(thrust > 15) thrust = 15;
//                setThrustBody(thrust);

//                double coef = 2.1;

//                intb += ((height+state.coordinates.get(2,0)) - intb)*0.75;

//                double maxAngle = PI/2;

//                if(base > maxAngle/coef-amplitude) base = maxAngle/coef-amplitude;
//                if(amplitude > base) base = amplitude;

//                double eilerons = coef*(base + test_signal);

//                if(eilerons>maxAngle) eilerons = maxAngle;

//                setLeftBrakeAngle(eilerons);
//                setRightBrakeAngle(eilerons);

//                state.update(parafoil, fuselage);

//                alpha = parafoil.getState().attackAngle;//thrust;
//                //alpha += 1/T_a*(thrust*0.01-alpha)*0.01;

//                //double lift = parafoil.getAeroLiftForce(state).normF();
//                //double drag = parafoil.getAeroDragForce(state).normF();

//                double lift = parafoil.getAeroLiftForce(state).normF() + parafoil.getControlBrakeLiftForce(state).normF();
//                double drag = (parafoil.getAeroDragForce(state).normF()+ parafoil.getControlBrakeDragForce(state).normF());

//                //double drag = (parafoil.getAeroDragForce(state).normF()+ parafoil.getControlBrakeDragForce(state).normF()+fuselage.getAeroForce(state).normF());

//                //double lift = parafoil.getAeroLiftForce(state).plus(parafoil.getControlBrakeLiftForce(state)).normF();
//                //double drag = (parafoil.getAeroDragForce(state).plus(parafoil.getControlBrakeDragForce(state)).plus(fuselage.getAeroForce(state)).normF());

//                double n_output = lift/drag;

//                //double n_output = (0.83*alpha + 0.28) / (1.*alpha * alpha - 0.2* alpha + 0.12);

//                //mean += 1/(PI*amplitude*amplitude)*((n_output*test_signal)*0.01*OMEGA);

//                mean += 1./T_f * (n_output * amplitude * sin(OMEGA * time) - mean) * 0.01;

//                base += 200. * mean  * 0.01;
//        }
//    }

}

#endif // ALGORITHM_H
