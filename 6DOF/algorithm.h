#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "def.h"
#include "model.h"

//typedef void (*func)(Fuselage& fuselage, Parafoil& parafoil);


///note алгоритмы управления

namespace  alg {


void algorithm_gliding(Fuselage& fuselage, Parafoil& parafoil)
{
    fuselage.set_thrust(0);
    parafoil.set_brake_angles(arma::colvec({0,0}));
}

//    public void evaluateWithSearchEngineSignal(double _time, double height)
//    {
//        try
//        {

//            File logger = new File("log" + version);
//            PrintWriter ilogger = new PrintWriter(logger);

//            double amplitude = 0.00075;
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

//                double thrust = 0.9 * (height+state.coordinates.get(2,0));
//                thrust-= 20*state.angVel.get(1,0);
//                thrust+=7.6;
//                if(thrust < 0) thrust = 0;
//                if(thrust > 15) thrust = 15;
//                setThrustBody(thrust);

//                double coef = 2.7;

//                double maxAngle = PI/6;

//                if(base > maxAngle/coef-amplitude) base = maxAngle/coef-amplitude;
//                if(amplitude > base) base = amplitude;

//                double eilerons = coef*(base + test_signal);

//                if(eilerons>maxAngle) eilerons = maxAngle;

//                setLeftBrakeAngle(eilerons);
//                setRightBrakeAngle(eilerons);

//                //double elerAp = 0.005;
//                //double elerB = 0.01;
//                //eler = elerAp*sin(OMEGA * time) + elerB;


//                //if(i % 100 == 0) eler += 0.001;

//                //setLeftBrakeAngle(eler);
//                //setRightBrakeAngle(eler);
//                //++i;


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

//                //System.out.println(mean);

//                //if(abs(o_output - n_output) < 1e-1) amplitude = amplitude;
//                //if(o_output < n_output) amplitude -= 0.0001;
//                //else amplitude += 0.001;

//                log();
//                ilogger.println(state.timeStamp + " " + base);
//                ilogger.flush();
//                if(getCoords().get(2,0) >=0 )
//                    return;
//            }

//        }
//        catch (Exception e)
//        {
//            e.printStackTrace();
//        }
//    }

//    public void evaluate(double _time, double height)
//    {

//        for (double t=0; t<=_time; t+=state.timeStep)
//        {
//            double thrust = 0.9 * (height+state.coordinates.get(2,0));
//            thrust-= 20*state.angVel.get(1,0);
//            thrust+=8.4;
//            if(thrust < 0) thrust = 0;
//            if(thrust > 15) thrust = 15;
//            setThrustBody(thrust);


//            state.update(parafoil, fuselage);

//            log();
//            if(getCoords().get(2,0) >=0 )
//                return;
//        }
//    }

//    public void evaluateWithSearch(double _time, double height)
//    {
//        try
//        {

//            File logger = new File("log" + version);
//            PrintWriter ilogger = new PrintWriter(logger);



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


//                log();
//                ilogger.println(state.timeStamp + " " + base);
//                ilogger.flush();
//                if(getCoords().get(2,0) >=0 )
//                    return;
//            }

//        }
//        catch (Exception e)
//        {
//            e.printStackTrace();
//        }
//    }


//    public void evaluateWithSearchEngine(double _time, double height)
//    {
//        try
//        {

//            File logger = new File("log" + version);
//            PrintWriter ilogger = new PrintWriter(logger);

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


//                log();
//                ilogger.println(state.timeStamp + " " + base);
//                ilogger.flush();
//                if(getCoords().get(2,0) >=0 )
//                    return;
//            }

//        }
//        catch (Exception e)
//        {
//            e.printStackTrace();
//        }
//    }

//    public void evaluatem(double _time, double height, double weight)
//    {

//        for (double t=0; t<=_time; t+=state.timeStep)
//        {
//            double thrust = weight * (height+state.coordinates.get(2,0));
//            thrust-= 20*state.angVel.get(1,0);
//            thrust+=8.4;
//            if(thrust < 0) thrust = 0;
//            if(thrust > 15) thrust = 15;
//            setThrustBody(thrust);


//            state.update(parafoil, fuselage);

//            log();
//            if(getCoords().get(2,0) >=0 )
//                return;
//        }
//    }};

};

#endif // ALGORITHM_H
