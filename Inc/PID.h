#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


//throttle parameters
#define MIN_THROTTLE 0
#define MAX_THROTTLE 4095
#define MIN_RPM      0
#define MAX_RPM      250

#define MIN_PWM_WIDTH   1
#define MAX_PWM_WIDTH   450

/* Controller parameters */
#define PID_KP  0.5f
#define PID_KI  0.5f
#define PID_KD  0.004f

#define PID_TAU 0.02f//lower value avoids the higher amplitude overshoot

#define PID_LIM_MIN    1.0f
#define PID_LIM_MAX  450.0f//50% of allowable pwmwidth

#define PID_LIM_MIN_INT -500.0f
#define PID_LIM_MAX_INT  500.0f

#define SAMPLE_TIME_S 0.01f

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 100.0f

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;

	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

} PIDController;

int mapFunction(int throttle);
void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif
