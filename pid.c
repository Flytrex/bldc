#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include "pid.h"

PID pid_init( double dt, double max, double min, double Kp, double Kd, double Ki ){
	PID pid;
	
	pid.dt = dt;
	pid.max = max;
	pid.min = min;
	pid.Kp = Kp;
	pid.Kd = Kd;
	pid.Ki = Ki;
	pid.pre_error = 0;
	pid.integral = 0;
	
	return pid;
}


double pid_calc(PID *pid, double setpoint, double pv){
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = pid->Kp * error;

    // Integral term
    pid->integral += error * pid->dt;
    double Iout = pid->Ki * pid->integral;

    // Derivative term
    double derivative = (error - pid->pre_error) / pid->dt;
    double Dout = pid->Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > pid->max )
        output = pid->max;
    else if( output < pid->min )
        output = pid->min;

    // Save error to previous error
    pid->pre_error = error;

    return output;
}


#endif
