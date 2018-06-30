/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "stm32f411e_discovery.h"
#include "stm32f411e_discovery_accelerometer.h"
#include "stm32f411e_discovery_gyroscope.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define LED_B_Pin GPIO_PIN_5
#define LED_B_GPIO_Port GPIOE
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define LEFT2_EXTI1_Pin GPIO_PIN_1
#define LEFT2_EXTI1_GPIO_Port GPIOC
#define LEFT2_EXTI1_EXTI_IRQn EXTI1_IRQn
#define LEFT3_EXTI2_Pin GPIO_PIN_2
#define LEFT3_EXTI2_GPIO_Port GPIOC
#define LEFT3_EXTI2_EXTI_IRQn EXTI2_IRQn
#define RIGHT1_EXTI3_Pin GPIO_PIN_3
#define RIGHT1_EXTI3_GPIO_Port GPIOC
#define RIGHT1_EXTI3_EXTI_IRQn EXTI3_IRQn
#define RIGHT_PWM2_Pin GPIO_PIN_0
#define RIGHT_PWM2_GPIO_Port GPIOA
#define RIGHT_PWM3_Pin GPIO_PIN_1
#define RIGHT_PWM3_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define RIGHT2_EXTI4_Pin GPIO_PIN_4
#define RIGHT2_EXTI4_GPIO_Port GPIOC
#define RIGHT2_EXTI4_EXTI_IRQn EXTI4_IRQn
#define RIGHT3_EXTI5_Pin GPIO_PIN_5
#define RIGHT3_EXTI5_GPIO_Port GPIOC
#define RIGHT3_EXTI5_EXTI_IRQn EXTI9_5_IRQn
#define SERVO3_Pin GPIO_PIN_0
#define SERVO3_GPIO_Port GPIOB
#define SERVO4_Pin GPIO_PIN_1
#define SERVO4_GPIO_Port GPIOB
#define LEFT_PWM1_Pin GPIO_PIN_9
#define LEFT_PWM1_GPIO_Port GPIOE
#define LEFT_PWM2_Pin GPIO_PIN_11
#define LEFT_PWM2_GPIO_Port GPIOE
#define LEFT_PWM3_Pin GPIO_PIN_13
#define LEFT_PWM3_GPIO_Port GPIOE
#define RIGHT_PWM1_Pin GPIO_PIN_14
#define RIGHT_PWM1_GPIO_Port GPIOE
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LEFT_FW_Pin GPIO_PIN_0
#define LEFT_FW_GPIO_Port GPIOD
#define LEFT_RW_Pin GPIO_PIN_1
#define LEFT_RW_GPIO_Port GPIOD
#define RIGHT_FW_Pin GPIO_PIN_2
#define RIGHT_FW_GPIO_Port GPIOD
#define RIGHT_RW_Pin GPIO_PIN_3
#define RIGHT_RW_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define SERVO1_Pin GPIO_PIN_4
#define SERVO1_GPIO_Port GPIOB
#define SERVO2_Pin GPIO_PIN_5
#define SERVO2_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_7
#define LED_R_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_8
#define LED_G_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define ABS(x)         (x < 0) ? (-x) : x
#define PI     3.14159

#define COMPASS_X_MAX 702
#define COMPASS_X_MIN -358
#define COMPASS_Y_MAX 293
#define COMPASS_Y_MIN -829
#define COMPASS_Z_MAX 845
#define COMPASS_Z_MIN -282
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
