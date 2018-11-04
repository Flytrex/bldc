#ifndef _PID_H_
#define _PID_H_

typedef struct PID{
	float dt; // loop interval time
	float max; // maximum value of manipulated variable
	float min; // minimum value of manipulated variable
	float Kp; // proportional gain
	float Kd; // derivative gain
	float Ki; // Integral gain
	float pre_error;
	float integral;
} PID;


// Returns an initialized PID
PID pid_init(float dt, float max, float min, float Kp, float Kd, float Ki);


// Returns the manipulated variable given an error between setpoint and current process value
float pid_calc(PID *pid, float error );


#endif
