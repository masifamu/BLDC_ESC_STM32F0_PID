#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#define DIRECT 0
#define REVERSE 1


#define MANUAL 0
#define AUTOMATIC 1

//throttle parameters
#define MIN_THROTTLE 0
#define MAX_THROTTLE 4095
#define MIN_RPM      0
#define MAX_RPM      250


int mapFunction(int throttle);
double getPIDOutput(void);
void setPIDOutput(double out);
void setPIDInput(double measuredValue, double setPoint);
void Compute(void);
void SetTunings(double Kp, double Ki, double Kd);
void SetSampleTime(int NewSampleTime);
void SetOutputLimits(double Min, double Max);
void SetMode(int Mode);
int GetMode(void);
void Initialize(void);
void resetPID(void);
void SetControllerDirection(int Direction);

#endif

