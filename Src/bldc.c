#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "string.h"
#include "stdio.h"
#include "bldc.h"
#include "math.h"


#define TIM1CH1(x) TIM1->CCR1=x
#define TIM1CH2(x) TIM1->CCR2=x
#define TIM1CH3(x) TIM1->CCR3=x

#define CH1 1
#define CH2 2
#define CH3 3

uint8_t BLDC_MotorSpin = 0,toUpdate=0,stoppingDirection=0;
uint8_t BLDC_STATE[6] = {0,0,0,0,0,0};
uint16_t PWMWIDTH=0;
extern uint32_t time;
extern uint32_t localTime;

uint16_t noOfHSCuts=0;

#ifndef BLDC_PWMCOMPLEMENTARYMODE
uint8_t BLDC_STATE_PREV[6] = {0,0,0,0,0,0};
#endif

// BLDC motor steps tables
static const uint8_t BLDC_BRIDGE_STATE_FORWARD[8][6] =   // Motor steps
{
//	UH,UL		VH,VL	WH,WL
   { 0,0	,	0,0	,	0,0 },  // 0 //000
   { 0,1	,	0,0	,	1,0 },
   { 0,0	,	1,0	,	0,1 },
   { 0,1	,	1,0	,	0,0 },
   { 1,0	,	0,1	,	0,0 },
   { 0,0	,	0,1	,	1,0 },
   { 1,0	,	0,0	,	0,1 },
   { 0,0	,	0,0	,	0,0 },  // 0 //111
};

// BLDC motor backwared steps tables
static const uint8_t BLDC_BRIDGE_STATE_BACKWARD[8][6] =   // Motor steps
{
//	UH,UL		VH,VL	WH,WL
   { 0,0	,	0,0	,	0,0 },  // 0 //000
   { 1,0	,	0,0	,	0,1 },
   { 0,0	,	0,1	,	1,0 },
   { 1,0	,	0,1	,	0,0 },
   { 0,1	,	1,0	,	0,0 },
   { 0,0	,	1,0	,	0,1 },
   { 0,1	,	0,0	,	1,0 },
   { 0,0	,	0,0	,	0,0 },  // 0 //111
};


uint8_t isThrotleProperlyConnected(uint32_t timeValue, uint16_t throtleValue){
	//monitor the delta in throtle here so that if it is disconnected somehow, engine can be shut-down
	
	//checking that after power on withing 1sec if the throtle is more that 3000, meaning that the throtle
	//is not properly connected
	if(timeValue <= 1000 && throtleValue >= 3000){
		//throtle is not properly connected
		return 0;//return false here
	}else{
		//throtle is properly connected.
		return 1;//return true here
	}
}

void toggleGreenLED(void){
	static uint16_t counter=0;
	if(++counter > 128){
		counter=0;
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	}
}

void BLDC_Init(void) {
	BLDC_MotorResetInverter();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin) {
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
	BLDC_MotorCommutation(BLDC_HallSensorsGetPosition());
	
	noOfHSCuts++;
}

uint8_t BLDC_HallSensorsGetPosition(void) {
	uint8_t temp=(uint8_t)((GPIOB->IDR) & (GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7))>>5;
	return temp;
}

uint8_t BLDC_MotorGetSpin(void) {
	return BLDC_MotorSpin;
}

void BLDC_MotorSetSpin(uint8_t spin) {
	BLDC_MotorSpin = spin;
}

void BLDC_MotorSetStopDirection(uint8_t stoppingSpin){
	stoppingDirection = stoppingSpin;//forward
}


void BLDC_MotorStop(void)
{
	BLDC_SetPWM(1);
}

void BLDC_MotorResetInverter(void)
{
		BLDC_SetPWM(0);
	
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); 
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); 

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

		BLDC_MotorSpin = 0;
		memset(BLDC_STATE_PREV, 0, sizeof(BLDC_STATE_PREV));
}


#ifdef BLDC_PWMTOPKEYS
void BLDC_MotorCommutation(uint16_t hallpos){
	localTime=time;
	
	if ((BLDC_MotorSpin == BLDC_CW || BLDC_MotorSpin == BLDC_STOP) && stoppingDirection == BLDC_CW) {
		memcpy(BLDC_STATE, BLDC_BRIDGE_STATE_FORWARD[hallpos], sizeof(BLDC_STATE));
	}
	else if((BLDC_MotorSpin == BLDC_CCW || BLDC_MotorSpin == BLDC_STOP) && stoppingDirection == BLDC_CCW){
		memcpy(BLDC_STATE, BLDC_BRIDGE_STATE_BACKWARD[hallpos], sizeof(BLDC_STATE));
	}

	// Disable if need
	if (!BLDC_STATE[UH]) TIM1CH3(0);
	if (!BLDC_STATE[UL]) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	if (!BLDC_STATE[VH]) TIM1CH2(0);
	if (!BLDC_STATE[VL]) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	if (!BLDC_STATE[WH]) TIM1CH1(0);
	if (!BLDC_STATE[WL]) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

	// Enable if need. If previous state is Enabled then not enable again. Else output do flip-flop.
	if (BLDC_STATE[UH] & !BLDC_STATE[UL] & !BLDC_STATE_PREV[UH]) {
		toUpdate=CH3; 
		BLDC_UpdatePWMWidth(CH3);
	}
	if (BLDC_STATE[UL] & !BLDC_STATE[UH] & !BLDC_STATE_PREV[UL]) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	}
	if (BLDC_STATE[VH] & !BLDC_STATE[VL] & !BLDC_STATE_PREV[VH]) {
		toUpdate=CH2;
		BLDC_UpdatePWMWidth(CH2);
	}
	if (BLDC_STATE[VL] & !BLDC_STATE[VH] & !BLDC_STATE_PREV[VL]) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	}
	if (BLDC_STATE[WH] & !BLDC_STATE[WL] & !BLDC_STATE_PREV[WH]) {
		toUpdate=CH1;
		BLDC_UpdatePWMWidth(CH1);
	}
	if (BLDC_STATE[WL] & !BLDC_STATE[WH] & !BLDC_STATE_PREV[WL]) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	}
	memcpy(BLDC_STATE_PREV, BLDC_STATE, sizeof(BLDC_STATE));
}
#endif

uint16_t BLDC_ADCToPWM(uint16_t ADC_VALUE) {
	uint32_t tmp;

	if (ADC_VALUE < BLDC_ADC_STOP) { return 0; }

	if (ADC_VALUE > BLDC_ADC_MAX) {	return (BLDC_CHOPPER_PERIOD * BLDC_SPEEDING_FACTOR)+1; }

	tmp = (uint32_t)(ADC_VALUE-BLDC_ADC_STOP) * (uint32_t)(BLDC_CHOPPER_PERIOD * BLDC_SPEEDING_FACTOR) / (uint32_t)(BLDC_ADC_MAX - BLDC_ADC_START);

	//to maintain the lower PWM width
	return (uint16_t) tmp-3;
}

void BLDC_SetPWM(uint16_t PWM)
{
	PWMWIDTH=PWM;
}

void BLDC_UpdatePWMWidth(uint8_t update){
	if(update == CH1){
		TIM1CH1(PWMWIDTH);
	}else if(update == CH2){
		TIM1CH2(PWMWIDTH);
	}else if(update == CH3){
		TIM1CH3(PWMWIDTH);
	}
}

