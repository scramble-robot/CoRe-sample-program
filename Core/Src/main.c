/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_uart.h"
#include <stdbool.h>
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
/* Private variables ---------------------------------------------------------*/
extern rc_info_t rc;
uint8_t buf[200];
uint8_t tmp[8];

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;

uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

int16_t motorPower[8]={};

enum OperatingMode {
	stop=1,
	move=3,
	launch=2
};

enum MagazineChangeMode {
	left=1,
	LRstop=3,
	right=2
};

uint8_t mode;

static const uint16_t PwmPeriod = 57600;
static const uint16_t ServoAngle0 = PwmPeriod * 0.5 / 20.0;
static const uint16_t ServoAngle180 = PwmPeriod * 2.4 / 20.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
  	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
    		Error_Handler();
  	}
//  		xprintf(" id=%d [0]=%d [1]=%[2]=%d\r\n",RxHeader.StdId,RxData[0],RxData[1],RxData[2]);
}

int8_t SetMotorPower(uint8_t motorID,int16_t power){
	uint8_t MaxmotorID=7;
	uint8_t MinmotorID=0;

	if((motorID<MinmotorID)||(MaxmotorID<motorID)){
		return -1;
	}
	motorPower[motorID]=power;
}

void CANOutPut(){
	if(0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)){
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.DLC = 8;
		TxHeader.TransmitGlobalTime = DISABLE;

		TxHeader.StdId = 0x200;
		for(uint8_t i=0;i<4;i++){
			TxData[i*2]=motorPower[i]>>8;
			TxData[i*2+1]=motorPower[i];
			tmp[i*2]=motorPower[i]>>8;
			tmp[i*2+1]=motorPower[i];
		}

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
		{
			Error_Handler();
		}

		TxHeader.StdId = 0x1FF;
		for(uint8_t i=0;i<4;i++){
			TxData[i*2]=motorPower[i+4]>>8;
			TxData[i*2+1]=motorPower[i+4];
			tmp[i*2]=motorPower[i+4]>>8;
			tmp[i*2+1]=motorPower[i+4];
		}

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
		{
			Error_Handler();
		}

	}
}

float pwm_power=0;
uint32_t ReloadState=0;
uint32_t underReloadState=0;
int16_t v[8]={0};

uint8_t launcherFlag=0;
uint8_t launcherLR=LRstop;//
uint16_t countLR=0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //GPIO割込みピン入力割込み
{
	if(GPIO_Pin == GPIO_PIN_11){
		if(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11)){
			if(underReloadState==0){
				v[6]=0;
			}
			underReloadState=1;//waitting
			ReloadState=0;
			v[6]=0;
		}
	}
	if(GPIO_Pin == GPIO_PIN_14){
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14)){
			ReloadState=1;
		}
	}
	if (GPIO_Pin == GPIO_PIN_0){
		if(HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_0)){
			if(launcherLR==right){
				countLR=0;
				launcherFlag=0;
				v[7]=0;
			}
		}
	}
	if (GPIO_Pin == GPIO_PIN_12){
		if(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_12)){
			if(launcherLR==left){
				countLR=0;
				launcherFlag=0;
				v[7]=0;
			}
		}
	}
	if (GPIO_Pin == GPIO_PIN_10){
			if(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10)){
					v[0]=0;
					v[1]=0;
					v[2]=0;
					v[3]=0;
					v[4]=0;
					v[5]=0;
					v[6]=0;
					v[7]=0;
					mode=stop;
			}else{
				mode=move;
			}
	}
}

uint8_t CANState=1;
uint8_t launchFlag=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim10){
    	if((mode==move)||(mode==launch)){
			if (launchFlag==0){
				if (ReloadState==0){
					v[5]=0;
				}
				if(underReloadState==0){
					v[6]=500;
				}else if (underReloadState==1){
					v[6]=0;
				}
			}else if(launcherFlag==0){
				if(underReloadState==1){
					if(ReloadState==0){
						if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14)){
							v[5]=-3000;
							ReloadState=1;
						}else{
							v[5]=3000;
						}
					}else if(ReloadState>=700){
						v[5]=0;
						underReloadState=2;
					}else if(ReloadState>=1){
						v[5]=-4500;//behind the load
						ReloadState++;
					}
				}else{
					v[5]=0;
				}
			}


			if(underReloadState>=2900*3){
				v[6]=1800;
			}else if(underReloadState>=4800){
				underReloadState++;
				v[6]=-2000;
			}else if(underReloadState>=2){
				underReloadState++;
				v[6]=0;
			}
    	}

    	if(v[7]!=0){
    		countLR++;
    		if(countLR>6000){
    			countLR=0;
    			v[7]=0;
    			launcherFlag=0;
    		}
    	}

		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.DLC = 8;
		TxHeader.TransmitGlobalTime = DISABLE;
    	if(CANState){
			TxHeader.StdId = 0x200;
			TxData[0] = v[0]>>8;
			TxData[1] = v[0];
			TxData[2] = v[1]>>8;
			TxData[3] = v[1];
			TxData[4] = v[2]>>8;
			TxData[5] = v[2];
			TxData[6] = v[3]>>8;
			TxData[7] = v[3];
			CANState=0;
    	}else{
    		TxHeader.StdId = 0x1FF;
			TxData[0] = v[4]>>8;
			TxData[1] = v[4];
			TxData[2] = v[5]>>8;
			TxData[3] = v[5];
			TxData[4] = v[6]>>8;
			TxData[5] = v[6];
			TxData[6] = v[7]>>8;
			TxData[7] = v[7];
			CANState=1;
    	}


		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
		{
			Error_Handler();
		}

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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
	/* open dbus uart receive it */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	pwm_power=71*2.0/10.0;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_power);
	HAL_Delay(2000);
	pwm_power=71*1/10.0;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_power);
	HAL_Delay(2000);
  dbus_uart_init();

	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin,GPIO_PIN_RESET);
	sprintf(buf,"Start up\n");
	//	HAL_UART_Transmit( &huart2, buf, strlen(buf), 0xFFFF );

	HAL_CAN_Start(&hcan1);
	uint16_t val;

	int16_t theta=0,x_dot=0,y_dot=0;
	HAL_TIM_Base_Start_IT(&htim10);

	if(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11)){
			underReloadState=1;//waitting
	}
	countLR=0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		uart_receive_handler(&huart1); //コントローラ値 受信

//		sprintf(buf, "CH1: %4d  CH2: %4d  CH3: %4d  CH4: %4d  SW1: %1d  SW2: %1d %x\r\n", rc.ch1, rc.ch2, rc.ch3, rc.ch4, rc.sw1, rc.sw2,val);

		if(mode!=stop){

			x_dot=rc.ch3*2;
			y_dot=rc.ch4;
			theta=rc.ch1*2;

	    	if((v[6]==0)&&(mode==move)&&(v[5]==0)){
	    			if(launcherLR!=rc.sw2){
						launcherLR=rc.sw2;
						launcherFlag=1;
						if(launcherLR==left){
							v[7]=1200;
						}else{
							v[7]=-1100;//To Right
						}
					}
	    		}


			if(launcherFlag==0){
				if(mode==launch){
					pwm_power=71*1.95/10.0;
					launchFlag=1;
				}else{
					pwm_power=71*1./10.0;
					launchFlag=0;
				}
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_power);
			}

			//matrix calculation
			v[0]=x_dot-y_dot+theta;
			v[1]=-x_dot-y_dot+theta;
			v[2]=-x_dot+y_dot+theta;
			v[3]=x_dot+y_dot+theta;

			for(uint8_t i=0;i<4;i++){
				v[i]=v[i]*9/2;//velocity coefficient
			}
			mode=rc.sw1;
			if(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10)){
				v[0]=0;
				v[1]=0;
				v[2]=0;
				v[3]=0;
				v[4]=0;
				v[5]=0;
				v[6]=0;
				v[7]=0;
				mode=stop;
			}
//		HAL_UART_Transmit( &huart2, buf, strlen(buf), 0xFFFF );
		}else{
			if(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10)){
				v[0]=0;
				v[1]=0;
				v[2]=0;
				v[3]=0;
				v[4]=0;
				v[5]=0;
				v[6]=0;
				v[7]=0;
				mode=stop;
			}else{
				mode=rc.sw1;
			}
			pwm_power=71*1./10.0;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_power);
		}

//
////		sprintf(buf,"%x%x  %x%x ",StateSW[0],tmp[1],tmp[2],tmp[3]);
////		HAL_UART_Transmit( &huart2, buf, strlen(buf), 0xFFFF );
//
//
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
	while(1)
	{
	}
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
