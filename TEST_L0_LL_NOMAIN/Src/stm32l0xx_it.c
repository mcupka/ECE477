/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int tim6_update_count = 0;
int comp_rising = 1;
int steps = 0;
int coulomb_ticks = 0;

int tim6_sel = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */


/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */ 
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
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC, COMP1 and COMP2 interrupts (COMP interrupts through EXTI lines 21 and 22).
  */
void ADC1_COMP_IRQHandler(void)
{
	if (comp_rising == 1) {
			EXTI->IMR &= ~(EXTI_IMR_IM22); //disable interrupt
			EXTI->PR |= EXTI_PR_PIF22; //clear pending flag for interrupt

			EXTI->IMR |= EXTI_IMR_IM21; //enable lower bound interrupt
			comp_rising = 0;
	}
	else {
			EXTI->IMR &= ~(EXTI_IMR_IM21); //disable lower bound interrupt
			EXTI->PR |= EXTI_PR_PIF21; //clear pending flag for interrupt

			//if (GPIOA->ODR & (1 << 5)) GPIOA->ODR &= ~(1 << 5); //turn red led off (debugging)
			//else GPIOA->ODR |= 1 << 5;
			steps++;

			TIM6->CR1 |= TIM_CR1_CEN; //enable timer to count before another step may be counted
	}


  /* USER CODE END ADC1_COMP_IRQn 0 */
  
  /* USER CODE BEGIN ADC1_COMP_IRQn 1 */

  /* USER CODE END ADC1_COMP_IRQn 1 */
}

/* USER CODE BEGIN 1 */


extern int motor_frequency;
void TIM6_DAC_IRQHandler() {
	//clear interrupt pending flag
	TIM6->SR &= ~(TIM_SR_UIF);

	// for the step counter
	if (tim6_sel == 0x0) {
		if (tim6_update_count < 10) tim6_update_count++;
		else {
			//disable TIM6
			TIM6->CR1 &= ~(TIM_CR1_CEN);

			comp_rising = 1;
			EXTI->IMR |= EXTI_IMR_IM22;

			//reset update count
			tim6_update_count = 0;
		}
	}

	else if (tim6_sel == 0x1) {
		//for use when the motor is running. used to see if the motor is stalling

		motor_frequency = TIM22->CNT * 61 / 2;
		TIM22->CNT = 0;

		//GPIOB->ODR &= ~(1 << 12);

	}
}

void USART1_IRQHandler() {
	//clear interrupt pending flag



}

void EXTI4_15_IRQHandler() {
	//interrupt for the coulomb counter (PC15)

	EXTI->PR |= EXTI_PR_PIF15; //clear pending flag for interrupt

	if (GPIOA->ODR & (1 << 5)) GPIOA->ODR &= ~(1 << 5);
	else GPIOA->ODR |= (1 << 5);

	coulomb_ticks++;

}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
