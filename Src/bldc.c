#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "string.h"
#include "stdio.h"
#include "bldc.h"
#include "math.h"
#include "PID.h"


#define TIM1CH1(x) TIM1->CCR1=x
#define TIM1CH2(x) TIM1->CCR2=x
#define TIM1CH3(x) TIM1->CCR3=x

#define CH1 1
#define CH2 2
#define CH3 3

#ifdef UART_HALL_DEBUG
char printDataString1[50] = "buffer here\r\n";
#endif

uint8_t BLDC_MotorSpin = 0,toUpdate=0,stoppingDirection=0;
uint8_t BLDC_STATE[6] = {0,0,0,0,0,0};
uint16_t PWMWIDTH=0;
extern uint32_t time;
extern uint32_t localTime;


extern PIDController pid;
extern uint16_t PID_targetSpeed;
extern uint16_t PID_measuredSpeed;

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

#ifdef MEASURE_POWER
uint16_t getCurrentDrawn(uint16_t adcBuffer2){
	#ifdef DO_CURRENT_AVGING
	static uint32_t avg2=0;
	static uint8_t count2=0;
	static uint16_t current=0;
	if(++count2 <= 4){
		avg2 += adcBuffer2;
	}else{
		count2=0;
		avg2 = avg2>>2;//5 right shift = div by 32
		current = (uint16_t)(((uint32_t)((float)avg2*((Rlow+Rup)/Rlow)*3300.0f))>>12);
		current = (uint16_t)((float)(current - 2550)/sensitivity*1000);
		avg2=0;
	}
	#endif
	#ifndef DO_CURRENT_AVGING
	uint16_t current=0;
	current = (uint16_t)(((uint32_t)((float)adcBuffer2*((Rlow+Rup)/Rlow)*3300.0f))>>12);
	current = (uint16_t)((float)(current - 2550)/sensitivity*1000);
	#endif
	return current;
}
#endif

#ifdef UART_COMM_DEBUG
uint16_t getHeatSinkTemp(uint16_t adcBuffer3){
	static uint32_t sum=0;
	static uint8_t count=0;
	static uint16_t temperature=0;
	if(++count <= 200){
		sum += adcBuffer3;
	}else{
		count=0;
		sum /=200;
		float HSVoltK=0, Vth=0.0,Rth=0.0;
		Vth = 3300 - ((sum*3300)>>12);
		Rth = (R1 * Vth)/(3300 - Vth);
		Vth = log(Rth/rTherm25C);//using Vth agian to save memory
		HSVoltK = 1.0/(coeffA+coeffB*Vth+coeffC*pow(Vth,2)+coeffD*pow(Vth,3));
		
		sum=0;
		temperature = ((uint16_t)(HSVoltK-273.15));
	}
	return temperature;
}

uint16_t getProcVoltage(uint16_t adcBuffer5){
	// 1530 is corresponding to the 1.2326v
	return ((uint16_t)((uint32_t)(3300*1530)/adcBuffer5));
}
uint16_t getProcTemp(uint16_t adcBuffer4){
	static uint32_t avg4=0;
	static uint8_t count4=0;
	static uint16_t pTemp=0;
	if(++count4 <= 16){
		avg4 += adcBuffer4;
	}else{
		count4 = 0;
		avg4 = avg4 >> 4;
		//sensitivity=4.3mV/C=5steps, 1770 is default value at 25C, 25 is addedd to get the actual value
		pTemp = ((uint16_t)((1770-(uint16_t)avg4)/5+25));
		avg4 = 0;
	}
	return pTemp;
}
#endif

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
	if(++counter > 1024){
		counter=0;
		FIO_FLP(GPIOB,GREEN_LED);
	}
}

void BLDC_Init(void) {
	BLDC_MotorResetInverter();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		//PIDController_Update(&pid, PID_targetSpeed, PID_measuredSpeed);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin) {
  //interrupt on pins are being reset from the EXTI interrupt handler in stm32f0xx_it.c  
	FIO_FLP(GPIOB,YELLOW_LED);
	BLDC_MotorCommutation(BLDC_HallSensorsGetPosition());
	
	noOfHSCuts++;
}

uint8_t BLDC_HallSensorsGetPosition(void) {	return ((uint8_t)((GPIOB->IDR) & (HS_PINS))>>5); }

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

	FIO_SET(GPIOB,GPIO_PIN_13);
	FIO_SET(GPIOB,GPIO_PIN_14);
	FIO_SET(GPIOB,GPIO_PIN_15);

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
	if (!BLDC_STATE[UL]) FIO_SET(GPIOB, MOS_YL);
	if (!BLDC_STATE[VH]) TIM1CH2(0);
	if (!BLDC_STATE[VL]) FIO_SET(GPIOB, MOS_GL);
	if (!BLDC_STATE[WH]) TIM1CH1(0);
	if (!BLDC_STATE[WL]) FIO_SET(GPIOA, MOS_BL);

	// Enable if need. If previous state is Enabled then not enable again. Else output do flip-flop.
	if (BLDC_STATE[UH] & !BLDC_STATE[UL] & !BLDC_STATE_PREV[UH]) {
		toUpdate=CH3; 
		BLDC_UpdatePWMWidth(CH3);
	}
	if (BLDC_STATE[UL] & !BLDC_STATE[UH] & !BLDC_STATE_PREV[UL]) { FIO_CLR(GPIOB, MOS_YL); }
	if (BLDC_STATE[VH] & !BLDC_STATE[VL] & !BLDC_STATE_PREV[VH]) {
		toUpdate=CH2;
		BLDC_UpdatePWMWidth(CH2);
	}
	if (BLDC_STATE[VL] & !BLDC_STATE[VH] & !BLDC_STATE_PREV[VL]) { FIO_CLR(GPIOB, MOS_GL); }
	if (BLDC_STATE[WH] & !BLDC_STATE[WL] & !BLDC_STATE_PREV[WH]) {
		toUpdate=CH1;
		BLDC_UpdatePWMWidth(CH1);
	}
	if (BLDC_STATE[WL] & !BLDC_STATE[WH] & !BLDC_STATE_PREV[WL]) { FIO_CLR(GPIOA, MOS_BL); }
	
	memcpy(BLDC_STATE_PREV, BLDC_STATE, sizeof(BLDC_STATE));
}
#endif

uint16_t BLDC_ADCToPWM(uint16_t ADC_VALUE) {
	uint32_t tmp;

	if (ADC_VALUE < BLDC_ADC_STOP) { return 0;	}

	if (ADC_VALUE > BLDC_ADC_MAX) {	return (BLDC_CHOPPER_PERIOD * BLDC_SPEEDING_FACTOR)+1;	}

	tmp = (uint32_t)(ADC_VALUE-BLDC_ADC_STOP) * (uint32_t)(BLDC_CHOPPER_PERIOD * BLDC_SPEEDING_FACTOR) / (uint32_t)(BLDC_ADC_MAX - BLDC_ADC_START);

	//to maintain the lower PWM width
	return (uint16_t) tmp-3;
}

void BLDC_SetPWM(uint16_t PWM){	PWMWIDTH=PWM; }

void BLDC_UpdatePWMWidth(uint8_t update){
	if(update == CH1){ TIM1CH1(PWMWIDTH);	}
	else if(update == CH2){ TIM1CH2(PWMWIDTH); }
	else if(update == CH3){	TIM1CH3(PWMWIDTH); }
}

