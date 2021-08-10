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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "uart_print.h"
#include "math.h"

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
uint32_t adc_buff[16];
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
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	//char buff2_str[10];
	//uint8_t status_add;
	//uint8_t status_val;
	
	uint8_t accel_add = 0x3B | (1<<7);
	uint8_t accelval[6];
	int16_t accelx;
	int16_t accely;
	int16_t accelz;
	float accelx_scale;
	float accely_scale;
	float accelz_scale;
	float gfilterx=0;
	float gfiltery=0;
	float gfilterz=0;
	float Roll;
	float Pitch;
	gyro_init();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
		
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2,&accel_add,1,HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi2,accelval,6,HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		accelx = (int16_t)((accelval[0]<<8) | accelval[1]);
		accely = (int16_t)((accelval[2]<<8) | accelval[3]);
		accelz = (int16_t)((accelval[4]<<8) | accelval[5]);
		accelx_scale = (2.0 * 9.81 / 32768.0)*((float)accelx);
		accely_scale = (2.0 * 9.81 / 32768.0)*((float)accely);
		accelz_scale = (2.0 * 9.81 / 32768.0)*((float)accelz);/*
		gfilterx = 0.9 * gfilterx + 0.1 * accelx_scale;
		gfiltery = 0.9 * gfiltery + 0.1 * accely_scale;
		gfilterz = 0.9 * gfilterz + 0.1 * accelz_scale;
		//Roll = atan2(gfiltery, gfilterz) * 180/(3.1415);
		//Pitch = atan2(-gfilterx, sqrt(gfiltery*gfiltery + gfilterz*gfilterz)) * 180/(3.1415);
		Roll = atan2(-gfiltery, gfilterz) * 180/(3.1415);
		Pitch = atan2(gfilterx, sqrt(gfiltery*gfiltery + gfilterz*gfilterz)) * 180/(3.1415);
		print_float(&huart1,&Roll);
		print_str(&huart1,",");
		print_float(&huart1,&Pitch);
		lineend(&huart1,WINDOWS);*/
		/*
		print_int(&huart1,&accelx,sizeof(accelx));
		print_str(&huart1,",");
		print_int(&huart1,&accely,sizeof(accely));
		print_str(&huart1,",");
		print_int(&huart1,&accelz,sizeof(accelz));
		lineend(&huart1,WINDOWS);*/
		/*
		print_float(&huart1,&gfilterx);
		print_str(&huart1,",");
		print_float(&huart1,&gfiltery);
		print_str(&huart1,",");
		print_float(&huart1,&gfilterz);
		lineend(&huart1,WINDOWS);
		*/
		
		print_float(&huart1,&accelx_scale);
		print_str(&huart1,",");
		print_float(&huart1,&accely_scale);
		print_str(&huart1,",");
		print_float(&huart1,&accelz_scale);
		lineend(&huart1,WINDOWS);
		/*if(Roll > 80 && (Pitch < 15 && Pitch > -15)){
			print_str(&huart1,"Left Side!");
			lineend(&huart1,WINDOWS);
		}
		else if(Roll > 158 && Pitch < -45){
			print_str(&huart1,"right Side!");
			lineend(&huart1,WINDOWS);
		}
		else if(Roll > 90 && Roll < 120 && Pitch < -42){
			print_str(&huart1,"Back Side!");
			lineend(&huart1,WINDOWS);
		}
		else if(Roll > 80 && Pitch > 28){
			print_str(&huart1,"Front Side!");
			lineend(&huart1,WINDOWS);
		}
		//else{
		//	print_str(&huart1,"Front Side!");
		//	lineend(&huart1,WINDOWS);
		//}
		*/
		HAL_Delay(100);
		//HAL_Delay(50);
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
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
	uint8_t int_conf[3] = {0x37,0x10,0x40};
	//uint8_t int_conf[3] = {0x37,0x10,0x01};
	uint8_t wom_th[2] = {0x1F, 0x22};
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,buffer,4,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(100);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(&hspi2,int_conf,3,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,wom_th,2,HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	//uint8_t pow_mgnt[3] = {0x6B,0x00,0x07};
	uint8_t accel_config_2[3] = {0x1D,0x05,0x09};
	//uint8_t MOT_DETECT_CTRL[2] = {0x69,0xC0};
	//uint8_t LP_ACCEL_ODR_CTRL[2] = {0x1E,0x0B};
	//uint8_t pow_mgnt1[3] = {0x6B,0x20};
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(&hspi2,pow_mgnt,2,HAL_MAX_DELAY);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	//HAL_Delay(100);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(&hspi2,accel_config_2,2,HAL_MAX_DELAY);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	//HAL_Delay(100);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(&hspi2,MOT_DETECT_CTRL,2,HAL_MAX_DELAY);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	//HAL_Delay(100);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(&hspi2,LP_ACCEL_ODR_CTRL,2,HAL_MAX_DELAY);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	//HAL_Delay(100);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(&hspi2,pow_mgnt1,2,HAL_MAX_DELAY);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	//HAL_Delay(100);
	
/*	if(HAL_I2C_Master_Transmit(&hi2c1,mpu6050,buffer,4,HAL_MAX_DELAY) != HAL_OK){
		HAL_UART_Transmit(&huart1,(uint8_t *)"i2c_error: gyro_init() (1)",strlen("i2c_error: gyro_init() (1)"),HAL_MAX_DELAY);
	}
	if(HAL_I2C_Master_Transmit(&hi2c1,mpu6050,buff2,3,HAL_MAX_DELAY) != HAL_OK){
		HAL_UART_Transmit(&huart1,(uint8_t *)"i2c_error: gyro_init() (2)",strlen("i2c_error: gyro_init() (1)"),HAL_MAX_DELAY);
	}
	*/
}

void personal(void){
	static uint8_t lineend[2] = {0x0D,0x0A};
	static char raw_str[30]; 
	for(int i=0;i<16;i++){
		sprintf(raw_str,"%d",(adc_buff[i]));
		HAL_UART_Transmit(&huart1,(uint8_t *)raw_str,strlen(raw_str),HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1,lineend,2,HAL_MAX_DELAY);
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
