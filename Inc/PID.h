#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


//throttle parameters
#define MIN_THROTTLE 0
#define MAX_THROTTLE 4095
#define MIN_RPM      0
#define MAX_RPM      220

#define MIN_PWM_WIDTH   5
#define MAX_PWM_WIDTH   240

/* Controller parameters */
//#define PID_KP  1.0f//50.0f
//#define PID_KI  15.85f//0.5f
//#define PID_KD  0.44f//0.25f
#define PID_KP  50.0f
#define PID_KI  0.5f
#define PID_KD  0.25f

#define PID_TAU 0.002f//lower value avoids the higher amplitude overshoot

#define PID_LIM_MIN    5.0f
#define PID_LIM_MAX  240.0f//50% of allowable pwmwidth

#define PID_LIM_MIN_INT -100.0f
#define PID_LIM_MAX_INT  100.0f

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
