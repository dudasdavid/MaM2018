
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart6;

osThreadId defaultTaskHandle;
osThreadId sensorTaskHandle;
osThreadId controlTaskHandle;
osThreadId commTaskHandle;
osThreadId idleTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static uint32_t mot1Cntr = 0; //L1
static uint32_t mot2Cntr = 0; //L2
static uint32_t mot3Cntr = 0; //L3
static uint32_t mot4Cntr = 0; //R1
static uint32_t mot5Cntr = 0; //R2
static uint32_t mot6Cntr = 0; //R3

static const float inputAxis[10] = {0, 0.2, 0.7, 1.5, 2.5, 5.0, 6.0, 7.0, 8.5, 10.0};
static const float outCurve[10]  = {0, 0.1, 0.5, 1.0, 1.5, 3.0, 4.0, 5.0, 7.0, 10.0};

static float toMeterFactor = 0.00294;
static float speedMagicConstant = 200;
static float invalidSpeedThreshold = 1.0;
static volatile float antiWindup = 2;
static volatile float ctrlP = 15;
static volatile float ctrlI = 1;
static volatile float ctrlD = 1;

static volatile float posWheelL1_prev = 0.0;
static volatile float posWheelL2_prev = 0.0;
static volatile float posWheelL3_prev = 0.0;
static volatile float posWheelR1_prev = 0.0;
static volatile float posWheelR2_prev = 0.0;
static volatile float posWheelR3_prev = 0.0;
        
static volatile float posWheelL1 = 0.0;
static volatile float posWheelL2 = 0.0;
static volatile float posWheelL3 = 0.0;
static volatile float posWheelR1 = 0.0;
static volatile float posWheelR2 = 0.0;
static volatile float posWheelR3 = 0.0;
        
static volatile float spdWheelL1_prev = 0.0;
static volatile float spdWheelL2_prev = 0.0;
static volatile float spdWheelL3_prev = 0.0;
static volatile float spdWheelR1_prev = 0.0;
static volatile float spdWheelR2_prev = 0.0;
static volatile float spdWheelR3_prev = 0.0;
        
static volatile float spdWheelL1 = 0.0;
static volatile float spdWheelL2 = 0.0;
static volatile float spdWheelL3 = 0.0;
static volatile float spdWheelR1 = 0.0;
static volatile float spdWheelR2 = 0.0;
static volatile float spdWheelR3 = 0.0;
        
static volatile float spdWheelL1Filtered = 0.0;
static volatile float spdWheelL2Filtered = 0.0;
static volatile float spdWheelL3Filtered = 0.0;
static volatile float spdWheelR1Filtered = 0.0;
static volatile float spdWheelR2Filtered = 0.0;
static volatile float spdWheelR3Filtered = 0.0;
        
static volatile float spdWheelL1Norm = 0.0;
static volatile float spdWheelL2Norm = 0.0;
static volatile float spdWheelL3Norm = 0.0;
static volatile float spdWheelR1Norm = 0.0;
static volatile float spdWheelR2Norm = 0.0;
static volatile float spdWheelR3Norm = 0.0;
    
static volatile float referenceSpeedLeft = 0;
static volatile float referenceSpeedRight = 0;
static volatile float refL = 0;
static volatile float refR = 0;

static volatile float referenceSpeedWheelL1 = 0;
static volatile float referenceSpeedWheelL2 = 0;
static volatile float referenceSpeedWheelL3 = 0;
static volatile float referenceSpeedWheelR1 = 0;
static volatile float referenceSpeedWheelR2 = 0;
static volatile float referenceSpeedWheelR3 = 0;

static volatile float spdErrorWheelL1 = 0;
static volatile float spdErrorWheelL2 = 0;
static volatile float spdErrorWheelL3 = 0;
static volatile float spdErrorWheelR1 = 0;
static volatile float spdErrorWheelR2 = 0;
static volatile float spdErrorWheelR3 = 0;
static volatile float spdErrorWheelL1Before = 0;
static volatile float spdErrorWheelL2Before = 0;
static volatile float spdErrorWheelL3Before = 0;
static volatile float spdErrorWheelR1Before = 0;
static volatile float spdErrorWheelR2Before = 0;
static volatile float spdErrorWheelR3Before = 0;
static volatile float spdIntErrorWheelL1 = 0;
static volatile float spdIntErrorWheelL2 = 0;
static volatile float spdIntErrorWheelL3 = 0;
static volatile float spdIntErrorWheelR1 = 0;
static volatile float spdIntErrorWheelR2 = 0;
static volatile float spdIntErrorWheelR3 = 0;
static int16_t forceWheelL1 = 0;
static int16_t forceWheelL2 = 0;
static int16_t forceWheelL3 = 0;
static int16_t forceWheelR1 = 0;
static int16_t forceWheelR2 = 0;
static int16_t forceWheelR3 = 0;
static volatile int16_t forceWheelL1BeforeSaturation = 0;
static volatile int16_t forceWheelL2BeforeSaturation = 0;
static volatile int16_t forceWheelL3BeforeSaturation = 0;
static volatile int16_t forceWheelR1BeforeSaturation = 0;
static volatile int16_t forceWheelR2BeforeSaturation = 0;
static volatile int16_t forceWheelR3BeforeSaturation = 0;
static volatile float spdDerivatedErrorWheelL1 = 0;
static volatile float spdDerivatedErrorWheelL2 = 0;
static volatile float spdDerivatedErrorWheelL3 = 0;
static volatile float spdDerivatedErrorWheelR1 = 0;
static volatile float spdDerivatedErrorWheelR2 = 0;
static volatile float spdDerivatedErrorWheelR3 = 0;
static volatile int16_t absForceWheelL1 = 0;
static volatile int16_t absForceWheelL2 = 0;
static volatile int16_t absForceWheelL3 = 0;
static volatile int16_t absForceWheelR1 = 0;
static volatile int16_t absForceWheelR2 = 0;
static volatile int16_t absForceWheelR3 = 0;

static volatile uint32_t timeStamp =0;
static volatile uint32_t timeOutGuard =0;
static uint8_t receiveBuffer[3]={0};
static volatile float referenceSpeed = 0;
static volatile float referenceSpeedOrig = 0;
static volatile float referenceAngle = 0;


static volatile float gyroX,gyroY,gyroZ = 0;
static volatile float compassX,compassY,compassZ = 0;
static volatile float compassX_Max,compassY_Max,compassZ_Max = 0;
static volatile float compassX_Min,compassY_Min,compassZ_Min = 0;
static volatile uint8_t initialCompassCalib = 1;

static volatile float compassX_Range,compassY_Range,compassZ_Range = 0;
static volatile float compassX_Norm,compassY_Norm,compassZ_Norm = 0;
static volatile float compassX_NormFiltered,compassY_NormFiltered,compassZ_NormFiltered = 0;
static volatile float compassX_NormFilteredTilted,compassY_NormFilteredTilted = 0.0f;
static volatile float roll,pitch,heading = 0;
static volatile float temperature,temperatureGyro = 0;
static volatile float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f;
static volatile float accX,accY,accZ = 0;
static volatile float accXFiltered,accYFiltered,accZFiltered = 0;
static volatile float accXFilteredKalman,accYFilteredKalman,accZFilteredKalman = 0;
static volatile float accXFilteredLPF,accYFilteredLPF,accZFilteredLPF = 0;

static volatile bool isRunning = 0;
static volatile bool isTurning = 0;
static volatile bool isHighSpeed = 0;
static volatile bool isServoDown = 0;
static volatile bool isReverse = 0;
static volatile int8_t potValue = 0;

static volatile uint8_t byte0 = 0;
static volatile uint8_t byte1 = 0;
static volatile uint8_t byte2 = 0;
static volatile uint8_t errorByte = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
void StartDefaultTask(void const * argument);
void StartSensorTask(void const * argument);
void StartControlTask(void const * argument);
void StartCommTask(void const * argument);
void StartIdleTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim2, TIM_CHANNEL_3);
  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim3, TIM_CHANNEL_4);
  
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); //RIGHT RW
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); //RIGHT FW
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); //LEFT FW
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); //LEFT RW
  
  TIM1->CCR1 = 0*4800/100; //RIGHT3
  TIM1->CCR2 = 0*4800/100; //RIGHT2
  TIM1->CCR3 = 0*4800/100; //RIGHT1
  TIM1->CCR4 = 0*4800/100; //LEFT3
  //TIM2->CCR1 = 0*4800/100; //LEFT2
  TIM2->CCR2 = 0*4800/100; //LEFT1
  TIM2->CCR3 = 0*4800/100; //LEFT2
  
  TIM3->CCR1 = 0*20000/100;
  TIM3->CCR2 = 0*20000/100;
  TIM3->CCR3 = 0*20000/100;
  TIM3->CCR4 = 0*20000/100;

  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI); 
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED6);
  
  BSP_GYRO_Init();
  BSP_ACCELERO_Init();
  MAGNET_Init();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of sensorTask */
  osThreadDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 128);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  /* definition and creation of controlTask */
  osThreadDef(controlTask, StartControlTask, osPriorityHigh, 0, 128);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* definition and creation of commTask */
  osThreadDef(commTask, StartCommTask, osPriorityAboveNormal, 0, 128);
  commTaskHandle = osThreadCreate(osThread(commTask), NULL);

  /* definition and creation of idleTask */
  osThreadDef(idleTask, StartIdleTask, osPriorityLow, 0, 128);
  idleTaskHandle = osThreadCreate(osThread(idleTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4800;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 95;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 0;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim9);

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11|LD4_Pin 
                          |LD3_Pin|LD5_Pin|LD6_Pin|LEFT_FW_Pin 
                          |LEFT_RW_Pin|RIGHT_FW_Pin|RIGHT_RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT2_EXTI1_Pin LEFT3_EXTI2_Pin RIGHT1_EXTI3_Pin RIGHT2_EXTI4_Pin 
                           RIGHT3_EXTI5_Pin PC8 */
  GPIO_InitStruct.Pin = LEFT2_EXTI1_Pin|LEFT3_EXTI2_Pin|RIGHT1_EXTI3_Pin|RIGHT2_EXTI4_Pin 
                          |RIGHT3_EXTI5_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD11 LD4_Pin 
                           LD3_Pin LD5_Pin LD6_Pin LEFT_FW_Pin 
                           LEFT_RW_Pin RIGHT_FW_Pin RIGHT_RW_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11|LD4_Pin 
                          |LD3_Pin|LD5_Pin|LD6_Pin|LEFT_FW_Pin 
                          |LEFT_RW_Pin|RIGHT_FW_Pin|RIGHT_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  switch ( GPIO_Pin ) {
  case GPIO_PIN_0:
    mot1Cntr++;
    break;
  case GPIO_PIN_1:
    mot2Cntr++;
    break;
  case GPIO_PIN_2:
    mot3Cntr++;
    break;
  case GPIO_PIN_3:
    mot4Cntr++;
    break;
  case GPIO_PIN_4:
    mot5Cntr++;
    break;
  case GPIO_PIN_5:
    mot6Cntr++;
    break;
  case GPIO_PIN_8:
    mot1Cntr++;
    break; 
  default:
    break;
  }
}
/*
float calcSpdRef_LUT(float inForce){
  float outForce = 0;
  uint8_t i;
  
  for (i = 0; i < 10; i++) {
    if (inForce <= inputAxis[i]) {
      if (inForce == inputAxis[i]) {
        outForce = outCurve[i];
        break;
      }
      else {
        outForce = outCurve[i-1] + ((outCurve[i] - outCurve[i-1])*(inForce-inputAxis[i-1]))/(inputAxis[i]-inputAxis[i-1]);
        break;
      }
    }
    else {
      outForce = outCurve[9];
    }
  }
  
  return outForce;
}
*/
float filter2 (float avg, float input, float alpha) {
  avg = (alpha * input) + (1.0 - alpha) * avg;
  return avg;
}

void saturateInteger(int16_t* i, int16_t min, int16_t max) {
  int16_t val = *i;
  if (val < min) val = min;
  else if (val > max) val = max;
  *i = val;
}

void saturateVolatileFloat(volatile float* i, float min, float max) {
  float val = *i;
  if (val < min) val = min;
  else if (val > max) val = max;
  *i = val;
}

void calculateLRSpeeds(float r, float phi) {

  phi = phi * PI / 180.0;
  
  if ((phi >= 0) && (phi < PI / 2.0)) //1st quadrant
  {
      referenceSpeedLeft = r;
      referenceSpeedRight = -r * cosf(2 * phi);
  }
  else if ((phi >= PI / 2.0) && (phi < PI)) //2nd quadrant
  {
      referenceSpeedLeft = -r * cosf(2 * phi);
      referenceSpeedRight = r;
  }
  else if ((phi >= PI) && (phi < 3 * PI / 2.0)) //3rd quadrant
  {
      referenceSpeedLeft = -r;
      referenceSpeedRight = r * cosf(2 * phi);
  }
  else if ((phi >= 3 * PI / 2.0) && (phi <= 2 * PI)) //4th quadrant
  {
      referenceSpeedLeft = r * cosf(2 * phi);
      referenceSpeedRight = -r;
  }
}


/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* StartSensorTask function */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  uint16_t taskCounter = 0;
  float gyroBuffer[3];
  float compassBuffer[3];
  float accelerometerBuffer[3] = {0};
  //float temperatureBuffer[1] = {0};
  float temperatureBufferGyro[1] = {0};
  /* Infinite loop */
  for(;;)
  {
    taskCounter++;
    
    posWheelL1_prev = posWheelL1;
    posWheelL2_prev = posWheelL2;
    posWheelL3_prev = posWheelL3;
    posWheelR1_prev = posWheelR1;
    posWheelR2_prev = posWheelR2;
    posWheelR3_prev = posWheelR3;
    
    spdWheelL1_prev = spdWheelL1;
    spdWheelL2_prev = spdWheelL2;
    spdWheelL3_prev = spdWheelL3;
    spdWheelR1_prev = spdWheelR1;
    spdWheelR2_prev = spdWheelR2;
    spdWheelR3_prev = spdWheelR3;
    
    posWheelL1 = mot1Cntr*toMeterFactor;
    posWheelL2 = mot2Cntr*toMeterFactor;
    posWheelL3 = mot3Cntr*toMeterFactor;
    posWheelR1 = mot4Cntr*toMeterFactor;
    posWheelR2 = mot5Cntr*toMeterFactor;
    posWheelR3 = mot6Cntr*toMeterFactor;
    
    if (fabs(posWheelL1-posWheelL1_prev) > invalidSpeedThreshold) spdWheelL1 = spdWheelL1_prev;
    else spdWheelL1 = (posWheelL1 - posWheelL1_prev);
    if (fabs(posWheelL2-posWheelL2_prev) > invalidSpeedThreshold) spdWheelL2 = spdWheelL2_prev;
    else spdWheelL2 = (posWheelL2 - posWheelL2_prev);
    if (fabs(posWheelL3-posWheelL3_prev) > invalidSpeedThreshold) spdWheelL3 = spdWheelL3_prev;
    else spdWheelL3 = (posWheelL3 - posWheelL3_prev);
    if (fabs(posWheelR1-posWheelR1_prev) > invalidSpeedThreshold) spdWheelR1 = spdWheelR1_prev;
    else spdWheelR1 = (posWheelR1 - posWheelR1_prev);
    if (fabs(posWheelR2-posWheelR2_prev) > invalidSpeedThreshold) spdWheelR2 = spdWheelR2_prev;
    else spdWheelR2 = (posWheelR2 - posWheelR2_prev);
    if (fabs(posWheelR3-posWheelR3_prev) > invalidSpeedThreshold) spdWheelR3 = spdWheelR3_prev;
    else spdWheelR3 = (posWheelR3 - posWheelR3_prev);
    
    spdWheelL1Filtered = filter2(spdWheelL1Filtered, spdWheelL1, 0.35);
    spdWheelL2Filtered = filter2(spdWheelL2Filtered, spdWheelL2, 0.35);
    spdWheelL3Filtered = filter2(spdWheelL3Filtered, spdWheelL3, 0.35);
    spdWheelR1Filtered = filter2(spdWheelR1Filtered, spdWheelR1, 0.35);
    spdWheelR2Filtered = filter2(spdWheelR2Filtered, spdWheelR2, 0.35);
    spdWheelR3Filtered = filter2(spdWheelR3Filtered, spdWheelR3, 0.35);
    
    spdWheelL1Norm = spdWheelL1Filtered * speedMagicConstant;
    spdWheelL2Norm = spdWheelL2Filtered * speedMagicConstant;
    spdWheelL3Norm = spdWheelL3Filtered * speedMagicConstant;
    spdWheelR1Norm = spdWheelR1Filtered * speedMagicConstant;
    spdWheelR2Norm = spdWheelR2Filtered * speedMagicConstant;
    spdWheelR3Norm = spdWheelR3Filtered * speedMagicConstant;
    
    if (taskCounter % 4 == 0){
      /* Read Gyro Angular data */
      BSP_GYRO_GetXYZ(gyroBuffer);
      
      gyroX = gyroBuffer[0];
      gyroY = gyroBuffer[1];
      gyroZ = gyroBuffer[2];
      
      BSP_GYRO_GetTemp(temperatureBufferGyro);
      temperatureGyro = temperatureBufferGyro[0];
      
      /* Read Acceleration*/
      BSP_ACCELERO_GetXYZ(accelerometerBuffer);

      accX = accelerometerBuffer[0];
      accY = accelerometerBuffer[1];
      accZ = accelerometerBuffer[2];
      
      accXFiltered = filter2(accXFiltered, accX, 0.2);
      accYFiltered = filter2(accYFiltered, accY, 0.2);
      accZFiltered = filter2(accZFiltered, accZ, 0.2);
     
      
      /* Read Magnetometer*/
      LSM303DLHC_MagReadXYZ(compassBuffer);
      compassX = compassBuffer[0];
      compassY = compassBuffer[1];
      compassZ = compassBuffer[2];
      
      if (initialCompassCalib == 1){
        compassX_Max = compassX;
        compassX_Min = compassX;
        compassY_Max = compassY;
        compassY_Min = compassY;
        compassZ_Max = compassZ;
        compassZ_Min = compassZ;
        initialCompassCalib = 0;
      }
      
      if (compassX > compassX_Max) compassX_Max = compassX;
      if (compassY > compassY_Max) compassY_Max = compassY;
      if (compassZ > compassZ_Max) compassZ_Max = compassZ;
      if (compassX < compassX_Min) compassX_Min = compassX;
      if (compassY < compassY_Min) compassY_Min = compassY;
      if (compassZ < compassZ_Min) compassZ_Min = compassZ;
      
      
      //The code you quoted first subtracts the minimum raw value from the reading,
      //then divides it by the range between the minimum and maximum raw values,
      //which puts the reading into the range of 0 to 1.
      //It then multiplies the reading by 2 and subtracts 1 to put it in the range of -1 to 1.
      
      saturateVolatileFloat(&compassX, COMPASS_X_MIN, COMPASS_X_MAX);
      saturateVolatileFloat(&compassY, COMPASS_Y_MIN, COMPASS_Y_MAX);
      saturateVolatileFloat(&compassZ, COMPASS_Z_MIN, COMPASS_Z_MAX);
      
      compassX_Norm = (compassX - COMPASS_X_MIN) / (COMPASS_X_MAX - COMPASS_X_MIN) * 2 - 1.0;
      compassY_Norm = (compassY - COMPASS_Y_MIN) / (COMPASS_Y_MAX - COMPASS_Y_MIN) * 2 - 1.0;
      compassZ_Norm = (compassZ - COMPASS_Z_MIN) / (COMPASS_Z_MAX - COMPASS_Z_MIN) * 2 - 1.0;
      
      compassX_NormFiltered = filter2(compassX_NormFiltered, compassX_Norm, 0.3);
      compassY_NormFiltered = filter2(compassY_NormFiltered, compassY_Norm, 0.3);
      compassZ_NormFiltered = filter2(compassZ_NormFiltered, compassZ_Norm, 0.3);
      
      
      fNormAcc = sqrt((accXFiltered*accXFiltered)+(accYFiltered*accYFiltered)+(accZFiltered*accZFiltered));
        
      fSinRoll = accYFiltered/fNormAcc;
      fCosRoll = sqrt(1.0-(fSinRoll * fSinRoll));
      fSinPitch = accXFiltered/fNormAcc;
      fCosPitch = sqrt(1.0-(fSinPitch * fSinPitch));
        
      if ( fSinRoll >0) {
        if (fCosRoll>0) roll = acos(fCosRoll)*180/PI;
        else            roll = acos(fCosRoll)*180/PI + 180;
      }
      else {
        if (fCosRoll>0) roll = acos(fCosRoll)*180/PI + 360;
        else            roll = acos(fCosRoll)*180/PI + 180;
      }
       
      if ( fSinPitch >0) {
        if (fCosPitch>0) pitch = acos(fCosPitch)*180/PI;
        else             pitch = acos(fCosPitch)*180/PI + 180;
      }
      else {
        if (fCosPitch>0) pitch = acos(fCosPitch)*180/PI + 360;
        else             pitch = acos(fCosPitch)*180/PI + 180;
      }

      if (roll >=360)  roll = 360 - roll;
      if (pitch >=360) pitch = 360 - pitch;
      
      //compassX_NormFilteredTilted = compassX_NormFiltered*fCosPitch+compassZ_NormFiltered*fSinPitch;
      //compassY_NormFilteredTilted = compassX_NormFiltered*fSinRoll*fSinPitch+compassY_NormFiltered*fCosRoll-compassZ_NormFiltered*fSinRoll*fCosPitch;
      //xh=cx*cos(ayf)+cy*sin(ayf)*sin(axf)-cz*cos(axf)*sin(ayf);
      //yh=cy*cos(axf)+cz*sin(axf);
      
      compassX_NormFilteredTilted = compassX_NormFiltered*fCosRoll + compassY_NormFiltered*fSinRoll*fSinPitch - compassZ_NormFiltered*fCosPitch*fSinRoll;
      compassY_NormFilteredTilted = compassY_NormFiltered*fCosPitch + compassZ_NormFiltered*fSinPitch;
      
      heading = (float) ((atan2f((float)compassY_NormFilteredTilted,(float)compassX_NormFilteredTilted))*180)/PI;
      heading += 180;
      
      if (heading < 0) heading = heading + 360;
    }
    
    osDelay(10);
  }
  /* USER CODE END StartSensorTask */
}

/* StartControlTask function */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
  for(;;)
  {
    
    
    referenceSpeedWheelL1 = referenceSpeedWheelL2 = referenceSpeedWheelL3 = ABS(referenceSpeedLeft);
    referenceSpeedWheelR1 = referenceSpeedWheelR2 = referenceSpeedWheelR3 = ABS(referenceSpeedRight);
    
    spdErrorWheelL1Before = spdErrorWheelL1;
    spdErrorWheelL1 = referenceSpeedWheelL1 - spdWheelL1Norm;
    spdIntErrorWheelL1 = spdIntErrorWheelL1 + spdErrorWheelL1 + antiWindup * (forceWheelL1 - forceWheelL1BeforeSaturation);
    spdDerivatedErrorWheelL1 = spdErrorWheelL1 - spdErrorWheelL1Before;
    forceWheelL1 = (int)(ctrlP * spdErrorWheelL1 + ctrlI * spdIntErrorWheelL1 + ctrlD * spdDerivatedErrorWheelL1);
    forceWheelL1BeforeSaturation = forceWheelL1;
    
    spdErrorWheelL2Before = spdErrorWheelL2;
    spdErrorWheelL2 = referenceSpeedWheelL2 - spdWheelL2Norm;
    spdIntErrorWheelL2 = spdIntErrorWheelL2 + spdErrorWheelL2 + antiWindup * (forceWheelL2 - forceWheelL2BeforeSaturation);
    spdDerivatedErrorWheelL2 = spdErrorWheelL2 - spdErrorWheelL2Before;
    forceWheelL2 = (int)(ctrlP * spdErrorWheelL2 + ctrlI * spdIntErrorWheelL2 + ctrlD * spdDerivatedErrorWheelL2);
    forceWheelL2BeforeSaturation = forceWheelL2;
    
    spdErrorWheelL3Before = spdErrorWheelL3;
    spdErrorWheelL3 = referenceSpeedWheelL3 - spdWheelL3Norm;
    spdIntErrorWheelL3 = spdIntErrorWheelL3 + spdErrorWheelL3 + antiWindup * (forceWheelL3 - forceWheelL3BeforeSaturation);
    spdDerivatedErrorWheelL3 = spdErrorWheelL3 - spdErrorWheelL3Before;
    forceWheelL3 = (int)(ctrlP * spdErrorWheelL3 + ctrlI * spdIntErrorWheelL3 + ctrlD * spdDerivatedErrorWheelL3);
    forceWheelL3BeforeSaturation = forceWheelL3;
    
    spdErrorWheelR1Before = spdErrorWheelR1;
    spdErrorWheelR1 = referenceSpeedWheelR1 - spdWheelR1Norm;
    spdIntErrorWheelR1 = spdIntErrorWheelR1 + spdErrorWheelR1 + antiWindup * (forceWheelR1 - forceWheelR1BeforeSaturation);
    spdDerivatedErrorWheelR1 = spdErrorWheelR1 - spdErrorWheelR1Before;
    forceWheelR1 = (int)(ctrlP * spdErrorWheelR1 + ctrlI * spdIntErrorWheelR1 + ctrlD * spdDerivatedErrorWheelR1);
    forceWheelR1BeforeSaturation = forceWheelR1;
    
    spdErrorWheelR2Before = spdErrorWheelR2;
    spdErrorWheelR2 = referenceSpeedWheelR2 - spdWheelR2Norm;
    spdIntErrorWheelR2 = spdIntErrorWheelR2 + spdErrorWheelR2 + antiWindup * (forceWheelR2 - forceWheelR2BeforeSaturation);
    spdDerivatedErrorWheelR2 = spdErrorWheelR2 - spdErrorWheelR2Before;
    forceWheelR2 = (int)(ctrlP * spdErrorWheelR2 + ctrlI * spdIntErrorWheelR2 + ctrlD * spdDerivatedErrorWheelR2);
    forceWheelR2BeforeSaturation = forceWheelR2;
    
    spdErrorWheelR3Before = spdErrorWheelR3;
    spdErrorWheelR3 = referenceSpeedWheelR3 - spdWheelR3Norm;
    spdIntErrorWheelR3 = spdIntErrorWheelR3 + spdErrorWheelR3 + antiWindup * (forceWheelR3 - forceWheelR3BeforeSaturation);
    spdDerivatedErrorWheelR3 = spdErrorWheelR3 - spdErrorWheelR3Before;
    forceWheelR3 = (int)(ctrlP * spdErrorWheelR3 + ctrlI * spdIntErrorWheelR3 + ctrlD * spdDerivatedErrorWheelR3);
    forceWheelR3BeforeSaturation = forceWheelR3;
    
    saturateInteger(&forceWheelL1, 0, 75);
    absForceWheelL1 = ABS(forceWheelL1);
    saturateInteger(&forceWheelL2, 0, 75);
    absForceWheelL2 = ABS(forceWheelL2);
    saturateInteger(&forceWheelL3, 0, 75);
    absForceWheelL3 = ABS(forceWheelL3);
    saturateInteger(&forceWheelR1, 0, 75);
    absForceWheelR1 = ABS(forceWheelR1);
    saturateInteger(&forceWheelR2, 0, 75);
    absForceWheelR2 = ABS(forceWheelR2);
    saturateInteger(&forceWheelR3, 0, 75);
    absForceWheelR3 = ABS(forceWheelR3);
    
    if (referenceSpeedLeft > 0.1){
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);   //LEFT FW
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); //LEFT RW
    }
    else if (referenceSpeedLeft < -0.1){
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); //LEFT FW
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);   //LEFT RW
    }
    else {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); //LEFT FW
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);   //LEFT RW
      spdIntErrorWheelL1 -= spdIntErrorWheelL1*0.01;
      spdIntErrorWheelL2 -= spdIntErrorWheelL2*0.01;
      spdIntErrorWheelL3 -= spdIntErrorWheelL3*0.01;
    }
    
    if (referenceSpeedRight > 0.1){
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); //RIGHT RW
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);   //RIGHT FW
    }
    else if (referenceSpeedRight < -0.1){
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);   //RIGHT RW
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); //RIGHT FW
    }
    else {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); //RIGHT RW
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); //RIGHT FW
      spdIntErrorWheelR1 -= spdIntErrorWheelR1*0.01;
      spdIntErrorWheelR2 -= spdIntErrorWheelR2*0.01;
      spdIntErrorWheelR3 -= spdIntErrorWheelR3*0.01;
    }
    
        
    TIM1->CCR1 = absForceWheelR3*4800/100; //RIGHT3
    TIM1->CCR2 = absForceWheelR2*4800/100; //RIGHT2
    TIM1->CCR3 = absForceWheelR1*4800/100; //RIGHT1
    TIM1->CCR4 = absForceWheelL3*4800/100; //LEFT3
    TIM2->CCR3 = absForceWheelL2*4800/100; //LEFT2
    TIM2->CCR2 = absForceWheelL1*4800/100; //LEFT1
    
    /*
    TIM1->CCR1 = (int)(referenceSpeedWheelR3*4800/100*10); //RIGHT3
    TIM1->CCR2 = (int)(referenceSpeedWheelR2*4800/100*10); //RIGHT2
    TIM1->CCR3 = (int)(referenceSpeedWheelR1*4800/100*10); //RIGHT1
    TIM1->CCR4 = (int)(referenceSpeedWheelL3*4800/100*10); //LEFT3
    TIM2->CCR3 = (int)(referenceSpeedWheelL2*4800/100*10); //LEFT2
    TIM2->CCR2 = (int)(referenceSpeedWheelL1*4800/100*10); //LEFT1
*/    if (isServoDown){
        TIM3->CCR1 = 4*20000/100;
        //TIM3->CCR2 = 5*20000/100;
        //TIM3->CCR3 = 5*20000/100;
        //TIM3->CCR4 = 5*20000/100;
    }
    else {
        TIM3->CCR1 = 11*20000/100;
        //TIM3->CCR2 = 10*20000/100;
        //TIM3->CCR3 = 10*20000/100;
        //TIM3->CCR4 = 10*20000/100;
    }
    
    osDelay(10);
  }
  /* USER CODE END StartControlTask */
}

/* StartCommTask function */
void StartCommTask(void const * argument)
{
  /* USER CODE BEGIN StartCommTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_UART_Receive_IT(&huart6,receiveBuffer,3);
    
    if ((receiveBuffer[0] == 0xFF) && (receiveBuffer[1] <=100) && (receiveBuffer[2] <=180)){
      //referenceSpeedOrig = receiveBuffer[1] / 10.0;
      //referenceSpeed = calcSpdRef_LUT(referenceSpeedOrig);
      referenceSpeed = receiveBuffer[1] / 25.0;
      referenceAngle = receiveBuffer[2] * 2.0;
      calculateLRSpeeds(referenceSpeed, referenceAngle);
      timeStamp = HAL_GetTick();
    }    
    else if ((receiveBuffer[0] == 0xFE) && (receiveBuffer[1] <=200) && (receiveBuffer[2] <=128)){
      byte0 = receiveBuffer[0];
      byte1 = receiveBuffer[1];
      byte2 = receiveBuffer[2];
      errorByte = 0;
    }
    else if ((receiveBuffer[1] == 0xFE) && (receiveBuffer[2] <=200) && (receiveBuffer[0] <=128)){
      byte0 = receiveBuffer[1];
      byte1 = receiveBuffer[2];
      byte2 = receiveBuffer[0];
      errorByte = 0;
    }
    else if ((receiveBuffer[2] == 0xFE) && (receiveBuffer[0] <=200) && (receiveBuffer[1] <=128)){
      byte0 = receiveBuffer[2];
      byte1 = receiveBuffer[0];
      byte2 = receiveBuffer[1];
      errorByte = 0;
    }
    else {
      byte0 = 0xAA;
      errorByte = 1;
    }

    if (byte0 == 0xFE){
      isRunning = (byte2 & (1<<4));
      isTurning = (byte2 & (1<<5));
      isHighSpeed = (byte2 & (1<<0));
      isServoDown = (byte2 & (1<<2));
      isReverse = (byte2 & (1<<3));

      potValue = -1 * (byte1 - 100);
      
      if (isRunning && !isTurning){
        /*
        if (isHighSpeed) {
          referenceSpeed = 4;
        }
        else{
          referenceSpeed = 1.5;
        }
        if (isReverse){
          referenceAngle = 360 - (((byte1 / 4.45) + 22.5)*2);
        }
        else{
          referenceAngle = 180 - (((byte1 / 4.45) + 22.5)*2);
        }
        */
        if (isReverse){
          referenceAngle = 270;
        }
        else{
          referenceAngle = 90;
        }
        referenceSpeed = (byte1+45)/100.0;
        saturateVolatileFloat(&referenceSpeed, 0, 2);
      }
      else if ((isTurning && !isRunning) || (isTurning && isRunning)){
        if (byte1 <= 100){
          referenceAngle = 90*2;
        }
        else {
          referenceAngle = 0;
        }
        referenceSpeed = abs(byte1 - 100) / 200.0;
        saturateVolatileFloat(&referenceSpeed, 0, 0.5);
      }
      else{
        referenceSpeed = 0;
        referenceAngle = 0;
      }
      calculateLRSpeeds(referenceSpeed, referenceAngle);
      timeStamp = HAL_GetTick();
      
    }
    else {
      referenceSpeed = 0;
      referenceAngle = 0;
      isRunning = 0;
      isTurning = 0;
      isHighSpeed = 0;
      isServoDown = 0;
    }
    
    timeOutGuard = HAL_GetTick() - timeStamp;
    
    if ((timeOutGuard) > 3000){
      referenceSpeed = 0;
      referenceAngle = 0;
      isRunning = 0;
      isTurning = 0;
      isHighSpeed = 0;
      isServoDown = 0;
    }
    
    osDelay(20);
  }
  /* USER CODE END StartCommTask */
}

/* StartIdleTask function */
void StartIdleTask(void const * argument)
{
  /* USER CODE BEGIN StartIdleTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartIdleTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
