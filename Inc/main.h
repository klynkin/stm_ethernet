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
  * Copyright (c) 2019 STMicroelectronics International N.V. 
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

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Stop_1_Pin GPIO_PIN_2
#define Stop_1_GPIO_Port GPIOE
#define Stop_1_EXTI_IRQn EXTI2_IRQn
#define Stop_2_Pin GPIO_PIN_3
#define Stop_2_GPIO_Port GPIOE
#define Stop_2_EXTI_IRQn EXTI3_IRQn
#define Stop_3_Pin GPIO_PIN_4
#define Stop_3_GPIO_Port GPIOE
#define Stop_3_EXTI_IRQn EXTI4_IRQn
#define PWM_DC1_Pin GPIO_PIN_5
#define PWM_DC1_GPIO_Port GPIOE
#define PWM_DC2_Pin GPIO_PIN_6
#define PWM_DC2_GPIO_Port GPIOE
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define Stop_4_Pin GPIO_PIN_7
#define Stop_4_GPIO_Port GPIOE
#define Stop_4_EXTI_IRQn EXTI9_5_IRQn
#define Stop_5_Pin GPIO_PIN_8
#define Stop_5_GPIO_Port GPIOE
#define Stop_5_EXTI_IRQn EXTI9_5_IRQn
#define Stop_6_Pin GPIO_PIN_9
#define Stop_6_GPIO_Port GPIOE
#define Stop_6_EXTI_IRQn EXTI9_5_IRQn
#define Stop_7_Pin GPIO_PIN_10
#define Stop_7_GPIO_Port GPIOE
#define Stop_7_EXTI_IRQn EXTI15_10_IRQn
#define Stop_8_Pin GPIO_PIN_11
#define Stop_8_GPIO_Port GPIOE
#define Stop_8_EXTI_IRQn EXTI15_10_IRQn
#define Stop_9_Pin GPIO_PIN_12
#define Stop_9_GPIO_Port GPIOE
#define Stop_9_EXTI_IRQn EXTI15_10_IRQn
#define Stop_10_Pin GPIO_PIN_13
#define Stop_10_GPIO_Port GPIOE
#define Stop_10_EXTI_IRQn EXTI15_10_IRQn
#define Stop_11_Pin GPIO_PIN_14
#define Stop_11_GPIO_Port GPIOE
#define Stop_11_EXTI_IRQn EXTI15_10_IRQn
#define Stop_12_Pin GPIO_PIN_15
#define Stop_12_GPIO_Port GPIOE
#define Stop_12_EXTI_IRQn EXTI15_10_IRQn
#define EXT_PWM_2_Pin GPIO_PIN_14
#define EXT_PWM_2_GPIO_Port GPIOB
#define EXT_PWM_1_Pin GPIO_PIN_15
#define EXT_PWM_1_GPIO_Port GPIOB
#define Nextion_TX_Pin GPIO_PIN_8
#define Nextion_TX_GPIO_Port GPIOD
#define Nextion_RX_Pin GPIO_PIN_9
#define Nextion_RX_GPIO_Port GPIOD
#define Encoder_Z1_Pin GPIO_PIN_11
#define Encoder_Z1_GPIO_Port GPIOD
#define Encoder_A1_Pin GPIO_PIN_12
#define Encoder_A1_GPIO_Port GPIOD
#define Encoder_B1_Pin GPIO_PIN_13
#define Encoder_B1_GPIO_Port GPIOD
#define Encoder_Z2_Pin GPIO_PIN_15
#define Encoder_Z2_GPIO_Port GPIOD
#define Encoder_A2_Pin GPIO_PIN_6
#define Encoder_A2_GPIO_Port GPIOC
#define Encoder_B2_Pin GPIO_PIN_7
#define Encoder_B2_GPIO_Port GPIOC
#define EXT_UART_TX_Pin GPIO_PIN_9
#define EXT_UART_TX_GPIO_Port GPIOA
#define EXT_UART_RX_Pin GPIO_PIN_10
#define EXT_UART_RX_GPIO_Port GPIOA
#define DXL_DIR_Pin GPIO_PIN_12
#define DXL_DIR_GPIO_Port GPIOA
#define SYS_JTMS_Pin GPIO_PIN_13
#define SYS_JTMS_GPIO_Port GPIOA
#define SYS_JTCK_Pin GPIO_PIN_14
#define SYS_JTCK_GPIO_Port GPIOA
#define DXL_TX_Pin GPIO_PIN_10
#define DXL_TX_GPIO_Port GPIOC
#define GPIO_2_Pin GPIO_PIN_2
#define GPIO_2_GPIO_Port GPIOD
#define GPIO_3_Pin GPIO_PIN_3
#define GPIO_3_GPIO_Port GPIOD
#define GPIO_4_Pin GPIO_PIN_4
#define GPIO_4_GPIO_Port GPIOD
#define GPIO_5_Pin GPIO_PIN_5
#define GPIO_5_GPIO_Port GPIOD
#define GPIO_6_Pin GPIO_PIN_6
#define GPIO_6_GPIO_Port GPIOD
#define GPIO_7_Pin GPIO_PIN_7
#define GPIO_7_GPIO_Port GPIOD
#define IRD2_TX_Pin GPIO_PIN_5
#define IRD2_TX_GPIO_Port GPIOB
#define USER_LED_Pin GPIO_PIN_0
#define USER_LED_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

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
