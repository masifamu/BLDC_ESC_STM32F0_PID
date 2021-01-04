#ifndef _BLDC_LIB_H_
#define _BLDC_LIB_H_

#include "stdint.h"

#define BLDC_CHOPPER_PERIOD 480
#define BLDC_SPEEDING_FACTOR 0.8

#define BLDC_PWMTOPKEYS

#define UART_THROTTLE_DEBUG
//#define UART_COMM_DEBUG
//#define UART_HALL_DEBUG
//#define MEASURE_POWER

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

#ifdef MEASURE_POWER
//motor parameter macros
#define minBattThreVolt           36
#define maxBattThreVolt           56
#define waitAftLowVoltDet         5000//in msec
#endif

#define wheelDia                  0.65
#define HSCutsInOneCycle          266

#ifdef UART_COMM_DEBUG
//thermistor parameter
#define R1										100710//100k
#define coeffA                0.003354016f
#define coeffB								0.0002569850f
#define coeffC								0.000002620131f
#define coeffD								0.00000006383091f
#define rTherm25C							100000

//current sensor parameter
#define Rlow										3.217f
#define Rup											2.176f
#define sensitivity							66			//66mv/A
//#define DO_CURRENT_AVGING
#endif

//hall sensor pin macros
#define HS_PINS							  GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7

//fast port macros
#define FIO_SET(port, pins)                port->BSRR = (pins)        //set pins
#define FIO_CLR(port, pins)                port->BRR = (pins)        //reset pins
#define FIO_FLP(port, pins)                port->ODR ^= (pins)        //flip pins
#define FIO_GET(port, pins)                ((port->IDR) & pins)        //read pins

//led pins
#define YELLOW_LED 							GPIO_PIN_3		//port B
#define GREEN_LED 							GPIO_PIN_4		//port B
#define BLUE_LED 								GPIO_PIN_12		//port A

//Inverter Pins
#define MOS_YL 								GPIO_PIN_1
#define MOS_GL 								GPIO_PIN_0
#define MOS_BL 								GPIO_PIN_7

//F/R pin
#define MOTOR_ROTATION_SELECT_PIN			GPIO_PIN_15		//on port A

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
#ifdef UART_COMM_DEBUG
uint16_t getHeatSinkTemp(uint16_t adcBuffer3);
uint16_t getProcVoltage(uint16_t adcBuffer5);
uint16_t getProcTemp(uint16_t adcBuffer4);
#endif
#ifdef MEASURE_POWER
uint16_t getCurrentDrawn(uint16_t adcBuffer2);
#endif
void toggleGreenLED(void);
uint8_t isThrotleProperlyConnected(uint32_t timeValue, uint16_t throtleValue);
void BLDC_UpdatePWMWidth(uint8_t update);
void BLDC_MotorResetInverter(void);
void BLDC_MotorSetStopDirection(uint8_t stoppingSpin);
#endif
