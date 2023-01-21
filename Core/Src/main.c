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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_uart.h"
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
//コントローラ関?��?
extern rc_info_t rc;
uint8_t buf[200];
uint8_t tmp[8];

//CAN関?��?
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;

uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

int16_t motorPower[8]={0x0000};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
			//HAL_Delay(1);
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
  MX_CAN1_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	/* open dbus uart receive it */
	dbus_uart_init();

	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin,GPIO_PIN_RESET);
	sprintf(buf,"Start up\n");
//	HAL_UART_Transmit( &huart2, buf, strlen(buf), 0xFFFF );

	HAL_CAN_Start(&hcan1);
	uint16_t val;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		uart_receive_handler(&huart1); //コントローラ値 受信

		val=(int16_t)rc.ch1*0x01FF/660;
		sprintf(buf, "CH1: %4d  CH2: %4d  CH3: %4d  CH4: %4d  SW1: %1d  SW2: %1d %x\r\n", rc.ch1, rc.ch2, rc.ch3, rc.ch4, rc.sw1, rc.sw2,val);
		HAL_UART_Transmit( &huart2, buf, strlen(buf), 0xFFFF );

		sprintf(buf,"%x%x  %x%x ",tmp[0],tmp[1],tmp[2],tmp[3]);
		HAL_UART_Transmit( &huart2, buf, strlen(buf), 0xFFFF );

		SetMotorPower(0,-val);
		SetMotorPower(1,val);
		SetMotorPower(2,val);
		SetMotorPower(3,val);

		CANOutPut();

		//	SetMotorPower(2,rc.ch1*0x2FFF/660);

//		if(0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)){
//		    TxHeader.StdId = 0x200;                 // CAN ID
//		    TxHeader.RTR = CAN_RTR_DATA;            // フレームタイプはデータフレーム
//		    TxHeader.IDE = CAN_ID_STD;              // 標準ID(11ﾋﾞｯﾄ)
//		    TxHeader.DLC = 8;                       // データ長は8バイトに
//		    TxHeader.TransmitGlobalTime = DISABLE;  // ???
//		    TxData[0] = 0x11;
//		    TxData[1] = 0x22;
//		    TxData[2] = 0x33;
//		    TxData[3] = 0x44;
//		    TxData[4] = 0x55;
//		    TxData[5] = 0x66;
//		    TxData[6] = 0x77;
//		    TxData[7] = 0x88;
//		    if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//		    			{
//		    				Error_Handler();
//		    			}
//		}

//		HAL_Delay(10);
		HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin); //Lチカ
//		HAL_Delay(30);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

