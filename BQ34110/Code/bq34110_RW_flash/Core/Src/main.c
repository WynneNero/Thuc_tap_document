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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define BQ34110 0xAA

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//COMMAND
typedef enum
{
	COMMAND_Current						= 0x0C,
	COMMAND_Voltage 					= 0x08,
	COMMAND_RemainingCapacity 			= 0x10,
	COMMAND_FullChargeCapacity 			= 0x12,
	COMMAND_DesignCapacity				= 0x3C,
	COMMAND_ManufacturerAccessControl	= 0x3E,
	COMMAND_MACData						= 0x40,		//it returns first MSB value,
	COMMAND_MACDataSum					= 0x60,
	COMMAND_MACDataLen					= 0x61

}Command_typedef;

//SUBCOMMAND
typedef enum
{
	SUB_Addr_DesignCapacity			= 0x41F5,
	SUB_Addr_FlashUpdateOKVoltage	= 0x4157,
	SUB_Addr_IntCoeff3				= 0x41C5

}Sub_typedef;




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


//#define CURRENT
#define VOLTAGE
//#define REMAINING_CAPACITY
//#define FULL_CHARGE_CAPACITY
#define DESIGN_CAPACITY

#define MANUFACTURER_ACCESS_CONTROL
#define WRITE_DATA_FLASH
#define READ_DATA_FLASH
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Read_BQ34110(I2C_HandleTypeDef *hi2c, Command_typedef command, uint8_t *rcv_data, uint16_t size);
void Transmit_SubCommand(I2C_HandleTypeDef *hi2c,Command_typedef command , Sub_typedef subCommand);
void Write_Data(I2C_HandleTypeDef *hi2c,Command_typedef command , uint8_t* data, uint8_t size);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t command = 0;
uint8_t rcv_data[32] = {0};
uint8_t trans_data[32] = {0};
uint8_t checkSum = 0;

int16_t	 value_Current						= 0;
uint16_t value_Voltage	 					= 0;
uint16_t value_RemainingCapacity 			= 0;
uint16_t value_FullChargeCapacity 			= 0;
uint16_t value_DesignCapacity				= 0;
uint16_t value_ManufacturerAccessControl 	= 0;
uint16_t value_MACData						= 0;
uint16_t value_MACDataSum					= 0;
uint16_t value_MACDataLen					= 0;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#ifdef VOLTAGE
	 HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	 Read_BQ34110(&hi2c1, COMMAND_Voltage, rcv_data, 2);
	 value_Voltage = rcv_data[0] + (rcv_data[1]<<8);
	 HAL_Delay(500);
#endif


#ifdef REMAINING_CAPACITY
	 HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	 Read_BQ34110(&hi2c1, COMMAND_RemainingCapacity, rcv_data, 2);
	 value_RemainingCapacity = rcv_data[0] + (rcv_data[1]<<8);
	 HAL_Delay(500);
#endif

#ifdef FULL_CHARGE_CAPACITY
	 HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	 Read_BQ34110(&hi2c1, COMMAND_FullChargeCapacity, rcv_data, 2);
	 value_FullChargeCapacity = rcv_data[0] + (rcv_data[1]<<8);
	 HAL_Delay(500);
#endif




#ifdef WRITE_DATA_FLASH
	 HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	 	 //write subCommand
	 Transmit_SubCommand(&hi2c1, COMMAND_ManufacturerAccessControl, SUB_Addr_DesignCapacity);
	 	 //write data
	 trans_data[0] = 0x17;
	 trans_data[1] = 0x70;
	 Write_Data(&hi2c1, COMMAND_MACData, trans_data, 2);
	 	 //write checkSum
	 checkSum = (0xFF - (0x41 + 0xF5 + trans_data[0] + trans_data[1])) & 0xFF;
	 trans_data[0] = checkSum;
	 Write_Data(&hi2c1, COMMAND_MACDataSum, trans_data, 1);
	 	 //write Length
	 trans_data[0] = 0x06;  //= (4 + length of MACData() bytes)
	 Write_Data(&hi2c1, COMMAND_MACDataLen, trans_data, 1);

	 HAL_Delay(500);
#endif

#ifdef READ_DATA_FLASH
	 HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

	 Transmit_SubCommand(&hi2c1, COMMAND_ManufacturerAccessControl, SUB_Addr_DesignCapacity);
	 Read_BQ34110(&hi2c1, COMMAND_ManufacturerAccessControl, rcv_data, 2);
	 Read_BQ34110(&hi2c1, COMMAND_MACData, rcv_data, 2);
	 Read_BQ34110(&hi2c1, COMMAND_MACDataSum, rcv_data, 1);
	 Read_BQ34110(&hi2c1, COMMAND_MACDataLen, rcv_data, 1);

	 HAL_Delay(500);
#endif

#ifdef DESIGN_CAPACITY
	 HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	 Read_BQ34110(&hi2c1, COMMAND_DesignCapacity, rcv_data, 2);
	 value_DesignCapacity = rcv_data[0] + (rcv_data[1]<<8);
	 HAL_Delay(500);
#endif


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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Read_BQ34110(I2C_HandleTypeDef *hi2c, Command_typedef command, uint8_t *rcv_data, uint16_t size)
{
	uint8_t temp = command;
	 if( HAL_I2C_Master_Transmit(hi2c, BQ34110, (uint8_t*)&temp, 1, HAL_MAX_DELAY) != HAL_OK)
	 {
		 Error_Handler();
	 }

	 if(  HAL_I2C_Master_Receive(hi2c, BQ34110, (uint8_t*)rcv_data, size, HAL_MAX_DELAY) != HAL_OK)
	 {
		 Error_Handler();
	 }
}
void Transmit_SubCommand(I2C_HandleTypeDef *hi2c,Command_typedef command , Sub_typedef subCommand)
{
	uint8_t trans_data[3] = {0};
	trans_data[0] = command;
	trans_data[1] = subCommand & 0x00FF;			//LSB
	trans_data[2] = subCommand >> 8;				//MSB
	 if( HAL_I2C_Master_Transmit(hi2c, BQ34110, (uint8_t*)trans_data, 3, HAL_MAX_DELAY) != HAL_OK)
	 {
		 Error_Handler();
	 }
}

void Write_Data(I2C_HandleTypeDef *hi2c,Command_typedef command , uint8_t* data, uint8_t size)
{
	uint8_t trans_data[33] = {0};
	trans_data[0] = command;
	for(int i = 1; i <= size; i++)
	{
		trans_data[i] = data[i - 1];
	}

	 if( HAL_I2C_Master_Transmit(hi2c, BQ34110, (uint8_t*)trans_data, (uint16_t)(size + 1), HAL_MAX_DELAY) != HAL_OK)
	 {
		 Error_Handler();
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
