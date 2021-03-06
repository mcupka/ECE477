/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "stm32l0xx_ll_adc.h"
#include "stm32l0xx.h"
#include "stm32l0xx_ll_i2c.h"
#include "stm32l0xx_ll_crs.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_dma.h"
#include "stm32l0xx_ll_spi.h"
#include "stm32l0xx_ll_tim.h"
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


void wait_for_interrupt();
void nano_wait(int);
void test_h_bridge();
void test_encoder();
//void test_comparator();
void test_uart();
void test_coulomb_counter();
void motor_driver_encoder(int);
void start_encoder();
void check_stall();
void enable_frequency_sampler();
void initialize();
void state_not_tight();
void tighten();
void synch();
void state_tight();
void untighten();
void get_eeprom_data();
void write_eeprom_data();
void test_eeprom(int);
void step_counter_enabled(int);

void setup_untightened();
void setup_tightened();

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MFX_IRQ_OUT_Pin LL_GPIO_PIN_13
#define MFX_IRQ_OUT_GPIO_Port GPIOC
#define PC14_OSC32_IN_Pin LL_GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin LL_GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define MCO_Pin LL_GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define MFX_WAKEUP_Pin LL_GPIO_PIN_1
#define MFX_WAKEUP_GPIO_Port GPIOA
#define LD_R_Pin LL_GPIO_PIN_5
#define LD_R_GPIO_Port GPIOA
#define ePD1_RESET_Pin LL_GPIO_PIN_2
#define ePD1_RESET_GPIO_Port GPIOB
#define ePD1_PWR_ENn_Pin LL_GPIO_PIN_10
#define ePD1_PWR_ENn_GPIO_Port GPIOB
#define ePD1_D_C_Pin LL_GPIO_PIN_11
#define ePD1_D_C_GPIO_Port GPIOB
#define NFC_NSS_Pin LL_GPIO_PIN_12
#define NFC_NSS_GPIO_Port GPIOB
#define NFC_SCK_Pin LL_GPIO_PIN_13
#define NFC_SCK_GPIO_Port GPIOB
#define NFC_MISO_Pin LL_GPIO_PIN_14
#define NFC_MISO_GPIO_Port GPIOB
#define NFC_MOSI_Pin LL_GPIO_PIN_15
#define NFC_MOSI_GPIO_Port GPIOB
#define ePD1_BUSY_Pin LL_GPIO_PIN_8
#define ePD1_BUSY_GPIO_Port GPIOA
#define USART_TX_Pin LL_GPIO_PIN_9
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin LL_GPIO_PIN_10
#define USART_RX_GPIO_Port GPIOA
#define USB1_DM_Pin LL_GPIO_PIN_11
#define USB1_DM_GPIO_Port GPIOA
#define USB1_DP_Pin LL_GPIO_PIN_12
#define USB1_DP_GPIO_Port GPIOA
#define SWDIO_Pin LL_GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin LL_GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define ePD1_CS_Pin LL_GPIO_PIN_15
#define ePD1_CS_GPIO_Port GPIOA
#define ePD1_SCK_Pin LL_GPIO_PIN_3
#define ePD1_SCK_GPIO_Port GPIOB
#define LD_G_Pin LL_GPIO_PIN_4
#define LD_G_GPIO_Port GPIOB
#define ePD1_MOSI_Pin LL_GPIO_PIN_5
#define ePD1_MOSI_GPIO_Port GPIOB
#define MFX_I2C_SCL_Pin LL_GPIO_PIN_8
#define MFX_I2C_SCL_GPIO_Port GPIOB
#define MFX_I2C_SDA_Pin LL_GPIO_PIN_9
#define MFX_I2C_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


#define TIM6_SEL_STEP_DEBOUNCER 0x0
#define TIM6_SEL_FREQ_SAMPLER 0x1

#define STATE_INIT 		0
#define STATE_NOTTIGHT 		1 
#define STATE_TIGHT 		2
#define STATE_TIGHTENING 	3
#define STATE_SYNCHING 		4
#define STATE_UNTIGHTENING  5

#define MAX_MAH 52000000
#define MAH_PER_TICK 1707
#define MAH_LEFT_GREEN 42000000 //defined as integers for quicker operations
#define MAH_LEFT_RED 10000000


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
