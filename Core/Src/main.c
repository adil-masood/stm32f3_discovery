/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	static const uint8_t mpu6050 = (0x68 << 1);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void gyro_init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	uint8_t lineend[2] = {0x0D,0x0A};
	
	uint8_t whoamiadd = 0x75;
	uint8_t tempadd = 0x41;
	uint8_t tempval8[2];
	static uint16_t tempval;
	uint8_t whoval=0x00;
	uint8_t gyro_add = 0x43;
	uint8_t gyroval[6]={0};
	int16_t gyrox=0x0000;
	int16_t gyroy=0x0000;
	int16_t gyroz=0x0000;
	int sum_gyrox=0;
	int sum_gyroy=0;
	int sum_gyroz=0;
	int avg_gyrox=0;
	int avg_gyroy=0;
	int avg_gyroz=0;
	int count=0;
	char gyroxstr[6];
	char gyroystr[6];
	char gyrozstr[6];
	char avgx_str[20];
	char avgy_str[20];
	char avgz_str[20];
	uint8_t smpl_div = 0x19;
	volatile uint8_t checker;
	
	uint8_t ch_buff[3]={1,1,1};
	char chbuff_str1[6];
	char chbuff_str2[6];
	char chbuff_str3[6];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_UART_Transmit(&huart1,(uint8_t *)"BEFORE",strlen("BEFORE"),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,lineend,2,HAL_MAX_DELAY);
	if(HAL_I2C_Master_Transmit(&hi2c1,mpu6050,&tempadd,1,HAL_MAX_DELAY) != HAL_OK){
		HAL_UART_Transmit(&huart1,(uint8_t *)"I2C_ERROR (1)",strlen("I2C_ERROR (1)"),HAL_MAX_DELAY);
		}
	if(HAL_I2C_Master_Receive(&hi2c1,mpu6050,ch_buff,3,HAL_MAX_DELAY) != HAL_OK){
		HAL_UART_Transmit(&huart1,(uint8_t *)"I2C_ERROR (2)",strlen("I2C_ERROR (2)"),HAL_MAX_DELAY);
	}
	sprintf(chbuff_str1,"%d",ch_buff[0]);
	sprintf(chbuff_str2,"%d",ch_buff[1]);
	sprintf(chbuff_str3,"%d",ch_buff[2]);
	HAL_UART_Transmit(&huart1,(uint8_t *)chbuff_str1,strlen(chbuff_str1),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,lineend,2,HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,(uint8_t *)chbuff_str2,strlen(chbuff_str2),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,lineend,2,HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,(uint8_t *)chbuff_str3,strlen(chbuff_str3),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,lineend,2,HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,(uint8_t *)"AFTER",strlen("AFTER"),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,lineend,2,HAL_MAX_DELAY);
	gyro_init();
	if(HAL_I2C_Master_Transmit(&hi2c1,mpu6050,&smpl_div,1,HAL_MAX_DELAY) != HAL_OK){
		HAL_UART_Transmit(&huart1,(uint8_t *)"I2C_ERROR (1)",strlen("I2C_ERROR (1)"),HAL_MAX_DELAY);
		}
	if(HAL_I2C_Master_Receive(&hi2c1,mpu6050,ch_buff,3,HAL_MAX_DELAY) != HAL_OK){
		HAL_UART_Transmit(&huart1,(uint8_t *)"I2C_ERROR (2)",strlen("I2C_ERROR (2)"),HAL_MAX_DELAY);
	}
	sprintf(chbuff_str1,"%d",ch_buff[0]);
	sprintf(chbuff_str2,"%d",ch_buff[1]);
	sprintf(chbuff_str3,"%d",ch_buff[2]);
	HAL_UART_Transmit(&huart1,(uint8_t *)chbuff_str1,strlen(chbuff_str1),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,lineend,2,HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,(uint8_t *)chbuff_str2,strlen(chbuff_str2),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,lineend,2,HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,(uint8_t *)chbuff_str3,strlen(chbuff_str3),HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1,lineend,2,HAL_MAX_DELAY);
	
	if(HAL_I2C_Master_Transmit(&hi2c1,mpu6050,&whoamiadd,1,HAL_MAX_DELAY) != HAL_OK){
			checker = 0x01;
		}
	if(HAL_I2C_Master_Receive(&hi2c1,mpu6050,&whoval,1,HAL_MAX_DELAY) != HAL_OK){
		checker = 0x02;
	}
	
  while (1)
  {
		
		if(HAL_I2C_Master_Transmit(&hi2c1,mpu6050,&gyro_add,1,HAL_MAX_DELAY) != HAL_OK){
			checker = 0x03;
		}
		if(HAL_I2C_Master_Receive(&hi2c1,mpu6050,gyroval,6,HAL_MAX_DELAY) != HAL_OK){
			checker = 0x04;
		}
		count+=1;
		gyrox = (int16_t)((gyroval[0]<<8) | gyroval[1]);
		gyroy = (int16_t)((gyroval[2]<<8) | gyroval[3]);
		gyroz = (int16_t)((gyroval[4]<<8) | gyroval[5]);
		sum_gyrox+=gyrox;
		sum_gyroy+=gyroy;
		sum_gyroz+=gyroz;
		avg_gyrox = sum_gyrox/count;
		avg_gyroy = sum_gyroy/count;
		avg_gyroz = sum_gyroz/count;
		
		//sprintf(gyroxstr,"%d",gyrox);
	//	sprintf(avgx_str,"%d",avg_gyrox);
	//	sprintf(avgy_str,"%d",avg_gyroy);
	//	sprintf(avgz_str,"%d",avg_gyroz);
		//HAL_UART_Transmit(&huart1,(uint8_t *)gyroxstr,strlen(gyroxstr),HAL_MAX_DELAY);
		//HAL_UART_Transmit(&huart1,(uint8_t *)gyroystr,strlen(gyroystr),HAL_MAX_DELAY);
		//HAL_UART_Transmit(&huart1,(uint8_t *)gyrozstr,strlen(gyrozstr),HAL_MAX_DELAY);
		//HAL_UART_Transmit(&huart1,(uint8_t *)avgx_str,strlen(avgx_str),HAL_MAX_DELAY);
		//HAL_UART_Transmit(&huart1,lineend,2,HAL_MAX_DELAY);
		HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void gyro_init(){
	uint8_t buffer[4] = {0x19,0x00,0x06,0x10};
  // 1st pointer to register at address 0x19
  // 2nd addr=0x19 <SMPLRT_DIV registr> || "SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV)"
  // 3rd 00000110 addr=0x1A <CONFIG registr> || "DLPF <Fs=1KHz, mode=6>, fifo mode replace"
  // 4th 1000dps addr=0x1B <GYRO CONFIG registr> || "gyro full scale +-250dps, enable DLPF by fchoice_b = 2b'00"
	if(HAL_I2C_Master_Transmit(&hi2c1,mpu6050,buffer,4,HAL_MAX_DELAY) != HAL_OK){
		HAL_UART_Transmit(&huart1,(uint8_t *)"i2c_error: gyro_init() (1)",strlen("i2c_error: gyro_init() (1)"),HAL_MAX_DELAY);
	}
	
	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
