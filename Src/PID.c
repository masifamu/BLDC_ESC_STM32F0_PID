#include "PID.h"
#include "stdint.h"
#include "bldc.h"

extern uint32_t time;

/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
double outMin, outMax;

int inAuto = 0;
int controllerDirection = DIRECT;
 
void Compute(void)
{
   if(!inAuto) return;
//   unsigned long now = (unsigned long)time;
//   int timeChange = (now - lastTime);
//   if(timeChange>=SampleTime)
//   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ITerm- kd * dInput;
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
 
      /*Remember some variables for next time*/
      lastInput = Input;
//      lastTime = now;
//   }
}
 
void setPIDOutput(double out){
	Output = out;
}

double getPIDOutput(void){
	return Output;
}
void setPIDInput(double measuredValue, double setPoint){
	Input = measuredValue;
	Setpoint = setPoint;
}

int mapFunction(int throttle){
    int y=0;
	
		if (throttle < BLDC_ADC_STOP) { return 0; }

		if (throttle > BLDC_ADC_MAX) {	return 2500; }
		
    y=((throttle-MIN_THROTTLE)*(MAX_RPM-MIN_RPM)/(MAX_THROTTLE-MIN_THROTTLE))+MIN_RPM;
    return y;
}

//to tune the PID on the fly
void SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0|| Kd<0) return;
 
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
 
//to call the PID at regular interval
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 

//To avoid the windup problem
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
 
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
 
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}
 
//This function is needed when we want to set the ouput variable(manual) of PID manually
//and when do not want the PID to set the ouput variable
//	This is something that we need whenver we want to turn the PID function OFF and want 
//	its ouput to be at 0 or some value we can do this but before doing so we need to call 
//	setMode function with MANUAL as its parameter
void SetMode(int Mode)
{
    uint8_t newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}
 
int GetMode(void)
{
    return inAuto;
}
//This function is to track the transition of mode from MANUAL to AUTOMATIC
//
//will be call from setMode function
void Initialize(void)
{
   lastInput = Input;
   ITerm = Output;
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}

//This function is needed when the pid out is supposed to increase as the reference point increase (forward or direct)
//or when the pid output is supposed to increase as the reference point decrease (reverse direction)
//it motoring application we are most often going to use forward direction, because as the throttle increase the pid output 
//supposed to increase

//should be called before the PID runs.
void SetControllerDirection(int Direction)
{
   controllerDirection = Direction;
}

void resetPID(void){
	Input = 0;
	Output = 0;
	Setpoint = 0;
	ITerm = 0;
	lastInput = 0;
}

