#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include "pid.h"

int utils_truncate_number_d(double *number, double min, double max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < min) {
		*number = min;
		did_trunc = 1;
	}

	return did_trunc;
}



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


double pid_calc(PID *pid, double error){
    // Calculate error
    //double error = setpoint - pv;

    // Proportional term
    double Pout = pid->Kp * error;

    // Integral term
    pid->integral += error * pid->dt;
    double Iout = pid->Ki * pid->integral;

    // I-term  windup protection
    utils_truncate_number_d(&pid->integral, -pid->max*2, pid->max*2);

    // Derivative term
    double derivative = (error - pid->pre_error) / pid->dt;
    double Dout = pid->Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    utils_truncate_number_d(&output, pid->min, pid->max);

    // Save error to previous error
    pid->pre_error = error;

    return output;
}


#endif
