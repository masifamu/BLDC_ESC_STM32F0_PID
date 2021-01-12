/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bldc.h"
#include "stdio.h"
#include "string.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char printDataString[100] = "buffer here\r\n";//{'\0',};

uint16_t ADCBuffer[6]={0,};
extern uint32_t time;
extern uint8_t toUpdate;

uint16_t throtle=0;
uint16_t rpm=0;
uint16_t targetRPM=0;

static uint16_t t;

MotorState motorState;
BreakState breakState;

extern uint16_t noOfHSCuts;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		
		static uint32_t counter;
		t++;
		
		rpm=((rpm<<5)-rpm+(uint16_t)((noOfHSCuts*1200)/HSCutsInOneCycle))>>5;
		noOfHSCuts=0;
		
		if(rpm == 0) counter++; else counter = 0;
		if(counter >= 10) motorState = STOPPED; else motorState = RUNNING;
		
		Compute();
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t pwmWidth=0;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_ADCEx_Calibration_Start(&hadc);
	HAL_ADC_Start_DMA(&hadc,(uint32_t*)&ADCBuffer,6);
	
	BLDC_Init();
  
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
	
	//PID setting
	SetOutputLimits(1,2500);
	SetMode(MANUAL);
	SetControllerDirection(DIRECT);
	SetTunings(20,20,0.2);
	SetSampleTime(50);
	__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);
	__HAL_TIM_ENABLE(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		if( __HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_BREAK) == 1){//break pressed
			breakState = BREAK_PRESSED;
			SetMode(MANUAL);
			__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
		}else{//released
			breakState = BREAK_RELEASED;
		}
		
		throtle = ((throtle<<3)-throtle+ADCBuffer[0])>>3;
		targetRPM = ((targetRPM<<5)-targetRPM+mapFunction(throtle))>>5;
		setPIDInput(rpm,targetRPM);
		
//		//Checking for throttle accident management
//		if(!isThrotleProperlyConnected(time,throtle)){
//			while(1){
//				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
//				HAL_Delay(200);
//			}
//		}
		
		snprintf(printDataString,100, "%d,%d,%0.2f,%d,%d\n",t,rpm,getPIDOutput(),targetRPM,pwmWidth);//heavy code
		HAL_UART_Transmit(&huart1,(uint8_t*)printDataString,strlen(printDataString),HAL_MAX_DELAY);
		
		//motor control block
    if (throtle > BLDC_ADC_START) {
			if (BLDC_MotorGetSpin() == BLDC_STOP) {
				// Check Reverse pin
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) != 0) {
					// Forward
					BLDC_MotorSetSpin(BLDC_CW);
					BLDC_MotorSetStopDirection(BLDC_CW);
				}else{
					// Backward
					BLDC_MotorSetSpin(BLDC_CCW);
					BLDC_MotorSetStopDirection(BLDC_CCW);
				}
				pwmWidth=BLDC_ADCToPWM(throtle);
				BLDC_SetPWM(pwmWidth);
				BLDC_MotorCommutation(BLDC_HallSensorsGetPosition());
				BLDC_UpdatePWMWidth(toUpdate);
			}
			
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
			//check here if the break is pressed or not, if pressed it this two should not execute else execute
			if(breakState == BREAK_RELEASED){
				SetMode(AUTOMATIC);
				pwmWidth=1+(((pwmWidth<<5)-pwmWidth+(uint16_t)getPIDOutput())>>5);
			} else { pwmWidth =30;}
			BLDC_SetPWM(pwmWidth);
    }else{
			if (BLDC_MotorGetSpin() != BLDC_STOP) {
				//meaning motor is still running
				if (throtle < BLDC_ADC_STOP) {
					BLDC_MotorStop();
					BLDC_MotorSetSpin(BLDC_STOP);
						SetMode(MANUAL);
				}
			}
			toggleGreenLED();
    }
		if(motorState == STOPPED){
			resetPID();
			if(throtle < BLDC_ADC_STOP){
				BLDC_SetPWM(1);
				BLDC_UpdatePWMWidth(toUpdate);
			}
		}
   }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
