#ifndef _PID_H_
#define _PID_H_

typedef struct PID{
	double dt; // loop interval time
	double max; // maximum value of manipulated variable
	double min; // minimum value of manipulated variable
	double Kp; // proportional gain
	double Kd; // derivative gain
	double Ki; // Integral gain
	double pre_error;
	double integral;
} PID;


// Returns an initialized PID
PID pid_init(double dt, double max, double min, double Kp, double Kd, double Ki);


// Returns the manipulated variable given an error between setpoint and current process value
double pid_calc(PID *pid, double error );


#endif
