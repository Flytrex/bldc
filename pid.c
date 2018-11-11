#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include "pid.h"

int utils_truncate_number_d(float *number, float min, float max) {
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



PID pid_init( float dt, float max, float min, float Kp, float Kd, float Ki ){
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


float pid_calc(PID *pid, float error){
    // Calculate error
    //float error = setpoint - pv;

    // Proportional term
    float Pout = pid->Kp * error;

    // Integral term
    pid->integral += error * pid->dt * pid->Ki;
    float Iout =  pid->integral;

    // I-term  windup protection
    utils_truncate_number_d(&pid->integral, -pid->max, pid->max);

    // Derivative term
    float derivative = (error - pid->pre_error) / pid->dt;
    float Dout = pid->Kd * derivative;

    // Calculate total output
    float output = Pout + Iout + Dout;

    // Restrict to max/min
    utils_truncate_number_d(&output, pid->min, pid->max);

    // Save error to previous error
    pid->pre_error = error;

    return output;
}


#endif
