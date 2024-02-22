/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "Math.h"

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
I2C_HandleTypeDef hi2c1;



/* USER CODE BEGIN PV */
#define EEPROM_ADDRESS 0xA8

#define PAGE_SIZE 32     // in Bytes
#define PAGE_NUM  128    // number of pages


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void writeToEEPROM(uint8_t *, uint16_t, uint8_t);

void readFromEEPROM(uint8_t *, uint16_t, uint8_t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static  uint16_t memPointer;



int st = 0;

typedef struct {
    int member1;
    char member2;
    char member3[2];
} StructureA;


typedef struct {
    uint8_t member1;
    char member2;
} StructureB;

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

  StructureA a = {1, 'A', "Hi"};
  StructureB b = {2, 'B'};
  StructureB c = {3, 'C'};
  unsigned char byteArrayofStructureA[sizeof(StructureA)] = {0};
  unsigned char byteArrayofStructureB[sizeof(StructureB)] = {0};
  unsigned char byteArrayofStructureC[sizeof(StructureB)] = {0};

  memcpy(byteArrayofStructureA, &a, sizeof(StructureA));
  memcpy(byteArrayofStructureB, &b, sizeof(StructureB));
  memcpy(byteArrayofStructureC, &c, sizeof(StructureB));





  uint8_t PAGE=0;
  uint8_t offset=0;

  uint8_t paddrposition = log(PAGE_SIZE)/log(2);



  memPointer = PAGE<<(paddrposition)|offset;


  uint8_t information[20];
  uint8_t info[40];

  StructureA resultA;

  StructureB resultB;

  StructureB resultC;

  uint8_t size1 = sizeof(byteArrayofStructureA);
  uint8_t size2 = sizeof(byteArrayofStructureB);
  uint8_t size3 = sizeof(byteArrayofStructureC);

  uint16_t startingMemoryAddress1 = memPointer;
  uint16_t startingMemoryAddress2 = memPointer + size1;
  uint16_t startingMemoryAddress3 = memPointer + size1 + size2;

  uint8_t informationA[size1];
  uint8_t informationB[size2];
  uint8_t informationC[size3];


  HAL_StatusTypeDef eeproRet = HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_ADDRESS, 2, 1000);
  if(eeproRet==HAL_OK)
  {
	writeToEEPROM(&byteArrayofStructureA, startingMemoryAddress1, size1);

//  	startingMemoryAddress2 = memPointer;

  	writeToEEPROM(&byteArrayofStructureB, startingMemoryAddress2, size2);

//  	startingMemoryAddress3 = memPointer;

 	writeToEEPROM(&byteArrayofStructureC, startingMemoryAddress3, size3);



  	readFromEEPROM(informationA, startingMemoryAddress1, size1);

  	readFromEEPROM(informationB, startingMemoryAddress2, size2);

  	readFromEEPROM(informationC, startingMemoryAddress3, size3);

  	readFromEEPROM(info, startingMemoryAddress1,40);





  	memcpy(&resultA, informationA, sizeof(StructureA));
  	memcpy(&resultB, informationB, sizeof(StructureB));
  	memcpy(&resultC, informationC, sizeof(StructureB));


  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void readFromEEPROM(uint8_t *info, uint16_t startingMemoryAddress,uint8_t size)
{
	uint16_t pos=0;

	HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, startingMemoryAddress, 2, &info[pos], size, 1000);

}

void writeToEEPROM(uint8_t *dataPointer, uint16_t startingMemoryAddress, uint8_t sizeOfData)
{
	uint16_t currentMemoryAddress = startingMemoryAddress;
	uint16_t bytesRemaining = sizeOfData;


	while(bytesRemaining > 0)
	{

		// Calculate the amount of data that can be written on current Page
		uint16_t remainingBytesOnPage = PAGE_SIZE - (currentMemoryAddress % PAGE_SIZE);

		//Calculate the amount of bytes to write
		uint16_t sizeOfDataToWrite = (remainingBytesOnPage > bytesRemaining)? bytesRemaining:remainingBytesOnPage;


		HAL_StatusTypeDef writeRet = HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, currentMemoryAddress, 2, (uint8_t*)(dataPointer), sizeOfDataToWrite, 1000);

		HAL_Delay(5);

		if (writeRet != HAL_OK)
		{

			st=1;
			break;
		}

		bytesRemaining -= sizeOfDataToWrite;

		dataPointer += sizeOfDataToWrite;

		currentMemoryAddress += sizeOfDataToWrite;

		if((currentMemoryAddress % PAGE_SIZE) == 0)
		{
			currentMemoryAddress = (((currentMemoryAddress/PAGE_SIZE)+1)*PAGE_SIZE);
		}


	}


//	memPointer += sizeOfData;
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
