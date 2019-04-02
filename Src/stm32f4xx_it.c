/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
extern xSemaphoreHandle xBinarySemaphore0;
extern xSemaphoreHandle xBinarySemaphore1;
extern xSemaphoreHandle xBinarySemaphore2;
extern xSemaphoreHandle xBinarySemaphore3;
extern xSemaphoreHandle xBinarySemaphore4;
extern xSemaphoreHandle xBinarySemaphore5;
extern xSemaphoreHandle xBinarySemaphoreStart;
extern xSemaphoreHandle xBinarySemaphoreStop;
extern xQueueHandle xQueueSwitch;
extern GPIO_InitTypeDef GPIO_InitStruct;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;

extern TIM_HandleTypeDef htim7;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	EXTI->PR |= (1<<2);
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( xBinarySemaphoreStop, &xHigherPriorityTaskWoken );
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles EXTI line3 interrupt.
*/
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	EXTI->PR |= (1<<3);
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( xBinarySemaphoreStart, &xHigherPriorityTaskWoken );
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	EXTI->PR |= (1<<4);
	static int i=0;
	static uint8_t first_intr=0;
	static uint8_t front=0;
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	if (front == 0) //концевик нажат
	{
		front=1;
		xQueueSendToBackFromISR( xQueueSwitch, &i, &xHigherPriorityTaskWoken );
		i=6;
	}
	else	//концевик отжат
	{
		front=0;
		xQueueSendToBackFromISR( xQueueSwitch, &i, &xHigherPriorityTaskWoken );
		i=0;
	}
	if (first_intr==0)
		{
	 	GPIO_InitStruct.Pin = Stop_3_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
		}
	first_intr=1;
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	static int i1=1;
	static uint8_t front1=0;
	static int i2=2;
	static uint8_t front2=0;
	static int i3=3;
	static uint8_t front3=0;
	static uint8_t first_intr1=0;
	static uint8_t first_intr2=0;
	static uint8_t first_intr3=0;
	if (EXTI->PR & (1<<7)) // Прерывание от EXTI7?
	      {  EXTI->PR |= (1<<7);
			static portBASE_TYPE xHigherPriorityTaskWoken;
			xHigherPriorityTaskWoken = pdFALSE;
			if (front1 == 0) //концевик нажат
				{
					front1=1;
					xQueueSendToBackFromISR( xQueueSwitch, &i1, &xHigherPriorityTaskWoken );
					i1=7;
				}
				else	//концевик отжат
				{
					front1=0;
					xQueueSendToBackFromISR( xQueueSwitch, &i1, &xHigherPriorityTaskWoken );
					i1=1;
				}
			if (first_intr1==0)
					{
				 	GPIO_InitStruct.Pin = Stop_4_Pin;
					GPIO_InitStruct.Pull = GPIO_PULLUP;
					GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
					HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
					}
				first_intr1=1;
		}
	else if (EXTI->PR & (1<<8)) // Прерывание от EXTI8?
    	 {  EXTI->PR |= (1<<8);
			static portBASE_TYPE xHigherPriorityTaskWoken;
			xHigherPriorityTaskWoken = pdFALSE;
			if (front2 == 0) //концевик нажат
				{
					front2=1;
					xQueueSendToBackFromISR( xQueueSwitch, &i2, &xHigherPriorityTaskWoken );
					i2=8;
				}
			else	//концевик отжат
				{
					front2=0;
					xQueueSendToBackFromISR( xQueueSwitch, &i2, &xHigherPriorityTaskWoken );
					i2=2;
				}
			if (first_intr2==0)
								{
							 	GPIO_InitStruct.Pin = Stop_5_Pin;
								GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
								GPIO_InitStruct.Pull = GPIO_PULLUP;
								HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
								}
							first_intr2=1;
		 }
	else if (EXTI->PR & (1<<9)) // Прерывание от EXTI9?
	    	 {  EXTI->PR |= (1<<9);
				static portBASE_TYPE xHigherPriorityTaskWoken;
				xHigherPriorityTaskWoken = pdFALSE;
				if (front3 == 0) //концевик нажат
					{
						front3=1;
						xQueueSendToBackFromISR( xQueueSwitch, &i3, &xHigherPriorityTaskWoken );
						i3=9;
					}
					else	//концевик отжат
					{
						front3=0;
						xQueueSendToBackFromISR( xQueueSwitch, &i3, &xHigherPriorityTaskWoken );
						i3=3;
					}
				if (first_intr3==0)
									{
								 	GPIO_InitStruct.Pin = Stop_6_Pin;
									GPIO_InitStruct.Pull = GPIO_PULLUP;
									GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
									HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
									}
								first_intr3=1;
			 }


  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
		static int i4=4;
		static uint8_t front4=0;
		static int i5=5;
		static uint8_t front5=0;
		static uint8_t first_intr4=0;
		static uint8_t first_intr5=0;
	 if (EXTI->PR & (1<<10)) // Прерывание от EXTI7?
		      {  EXTI->PR |= (1<<10);
				static portBASE_TYPE xHigherPriorityTaskWoken;
				xHigherPriorityTaskWoken = pdFALSE;
				if (front4 == 0) //концевик нажат
						{
							front4=1;
							xQueueSendToBackFromISR( xQueueSwitch, &i4, &xHigherPriorityTaskWoken );
							i4=10;
						}
				else	//концевик отжат
					{
							front4=0;
							xQueueSendToBackFromISR( xQueueSwitch, &i4, &xHigherPriorityTaskWoken );
							i4=4;
					}
				if (first_intr4==0)
									{
								 	GPIO_InitStruct.Pin = Stop_7_Pin;
									GPIO_InitStruct.Pull = GPIO_PULLUP;
									GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
									HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
									}
								first_intr4=1;
			}
		else if (EXTI->PR & (1<<11)) // Прерывание от EXTI8?
	    	 {  EXTI->PR |= (1<<11);
				static portBASE_TYPE xHigherPriorityTaskWoken;
				xHigherPriorityTaskWoken = pdFALSE;
				if (front5 == 0) //концевик нажат
						{
							front5=1;
							xQueueSendToBackFromISR( xQueueSwitch, &i5, &xHigherPriorityTaskWoken );
							i5=11;
						}
				else	//концевик отжат
						{
							front5=0;
							xQueueSendToBackFromISR( xQueueSwitch, &i5, &xHigherPriorityTaskWoken );
							i5=5;
						}
				if (first_intr5==0)
									{
								 	GPIO_InitStruct.Pin = Stop_8_Pin;
									GPIO_InitStruct.Pull = GPIO_PULLUP;
									GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
									HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
									}
								first_intr5=1;
			 }

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
* @brief This function handles Ethernet global interrupt.
*/
void ETH_IRQHandler(void)
{
  /* USER CODE BEGIN ETH_IRQn 0 */

  /* USER CODE END ETH_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_IRQn 1 */

  /* USER CODE END ETH_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
