#ifndef _BLDC_LIB_H_
#define _BLDC_LIB_H_

#include "stdint.h"

#define BLDC_CHOPPER_PERIOD 2880
#define BLDC_SPEEDING_FACTOR 0.8

#define BLDC_PWMTOPKEYS

/* when working with variable resistor
#define BLDC_ADC_START 15
#define BLDC_ADC_STOP 5
#define BLDC_ADC_MAX 4000
*/
//when working with throtle
#define BLDC_ADC_START 1150
#define BLDC_ADC_STOP 1090
#define BLDC_ADC_MAX 4000
/////////////////////////////////////////////////////////////////////////
#define BLDC_STOP	0
#define BLDC_CW		1
#define BLDC_CCW	2


#define wheelDia                  0.65
#define HSCutsInOneCycle          266


typedef enum{ STOPPED, RUNNING, ACCELERATING, DEACCELERATING} MotorState;
typedef enum{ BREAK_PRESSED, BREAK_RELEASED} BreakState;

#define UH	0
#define UL	1
#define VH	2
#define VL	3
#define WH	4
#define WL	5

void BLDC_Init(void);
void BLDC_HallSensorsInit(void);
void BLDC_PWMTimerInit(void);
uint8_t BLDC_HallSensorsGetPosition(void);
void BLDC_MotorSetSpin(uint8_t spin);
uint8_t BLDC_MotorGetSpin(void);
void BLDC_MotorStop(void);
void BLDC_MotorCommutation(uint16_t hallpos);
uint16_t BLDC_ADCToPWM(uint16_t ADC_VALUE);
void BLDC_SetPWM(uint16_t PWM);
void usart_init(void);
void USARTSend(char *);

void toggleGreenLED(void);
uint8_t isThrotleProperlyConnected(uint32_t timeValue, uint16_t throtleValue);
void BLDC_UpdatePWMWidth(uint8_t update);
void BLDC_MotorResetInverter(void);
void BLDC_MotorSetStopDirection(uint8_t stoppingSpin);
#endif
