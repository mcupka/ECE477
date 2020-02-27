/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

//TSC_HandleTypeDef htsc;

//PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TSC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC_Init(void);
*//* USER CODE BEGIN PFP */

void nano_wait(int);
void test_h_bridge();
void test_encoder();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main() {

	//test_h_bridge();
	test_encoder();
}



void test_encoder() {

	//set up timer 21 to count up using an external clock (the pulse train from the encoder)
	//TIM 22 Ch 1 on PB4

	RCC->IOPENR |= RCC_IOPENR_IOPBEN; //enable port b clock
	RCC->APB2ENR |= RCC_APB2ENR_TIM22EN; //enable timer 22 clock

	GPIOB->MODER &= ~(GPIO_MODER_MODE4); //AF mode (10)
	GPIOB->MODER |= GPIO_MODER_MODE4_1;

	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL4);
	GPIOB->AFR[0] |= (0x4 << 16);	//Set the af of pb4 to 4 for TIM22 CH1

	//From page 554 in programming manual
	TIM22->CCMR1 &= ~(TIM_CCMR1_CC1S);
	TIM22->CCMR1 |= TIM_CCMR1_CC1S_0;
	TIM22->CCER &= ~(TIM_CCER_CC1P);
	TIM22->CCER &= ~(TIM_CCER_CC1NP);
	TIM22->SMCR |= TIM_SMCR_SMS;
	TIM22->SMCR &= ~(TIM_SMCR_TS);
	TIM22->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_TS_0;

	TIM22->CCER |= TIM_CCER_CC1E;


	TIM22->ARR = 645000;

	TIM22->CR1 |= TIM_CR1_CEN; //enable counter
	TIM22->EGR |= TIM_EGR_UG; //force update generation


	uint32_t count;
	while (1) {
		nano_wait(100000);
		count = TIM22->CNT;
		if (count > 50000) return;
	}

}


void test_h_bridge() {
	//function to demo using two pwm signals to drive the h bridge chip that we'll use for our design

	//****SET UP GPIO PA5*****//
	//begin by enabling the clock to GPIO port PA5, which is the onboard red led.
	RCC->IOPENR |= RCC_IOPENR_IOPAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODE5);
	GPIOA->MODER |= GPIO_MODER_MODE5_0;
	GPIOA->ODR |= 1 << 5;



	//****SET UP TIM2 CH1 AS PWM ON PA0*****//
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable timer 2 clock

	GPIOA->MODER &= ~(GPIO_MODER_MODE15); //AF mode (10)
	GPIOA->MODER |= GPIO_MODER_MODE15_1;

	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL15);
	GPIOA->AFR[1] |= (0x5 << 28);	//Set the af of pa0 to 2 for TIM2 CH1


	TIM2->PSC = 1 - 1; //TIMER CLOCK = 16MHz / (PSC + 1) = 1MHz
	TIM2->ARR = 40; //8 cycles per transition; period is (8 / 1MHz) * 2 = 16us
	TIM2->CCR1 = 0; //CCRx = 7. since arr = 8, signal will be high for 7 / 8 of period, 87.5 % duty cycle, or 14us high out of 16us period

	//One pwm signal will stay low (0% duty) the other will use pwm to control motor speed. Switching the two switches direction of the motor

	TIM2->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE; //SET oc1m to pwm mode 1. Also enable preload
	TIM2->CCER |= TIM_CCER_CC1E; //enable output on OC1
	TIM2->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN; //Center aligned mode, enable counter
	TIM2->EGR |= TIM_EGR_UG; //force update generation


	nano_wait(100000000);
	nano_wait(100000000);
	nano_wait(100000000);


	//****SET UP TIM2 CH1 AS PWM ON PA5*****//

	GPIOA->MODER &= ~(GPIO_MODER_MODE5); //AF mode (10)
	GPIOA->MODER |= GPIO_MODER_MODE5_1;

	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL5);
	GPIOA->AFR[0] |= 0x5 << (5 * 4);	//Set the af of pa0 to 2 for TIM2 CH1

	GPIOA->MODER &= ~(GPIO_MODER_MODE15);
	GPIOA->MODER |= GPIO_MODER_MODE15_0;
	GPIOA->ODR |= 1 << 15;

	short clockwise = 0;
	int ccrval = 32;
	short up = 0;


	nano_wait(100000000);
	nano_wait(100000000);
	nano_wait(100000000);

	while (1) {

		if (clockwise == 0) {

			GPIOA->MODER &= ~(GPIO_MODER_MODE5);
			GPIOA->MODER |= GPIO_MODER_MODE5_0;
			GPIOA->ODR |= 1 << 5;


			GPIOA->MODER &= ~(GPIO_MODER_MODE15); //AF mode (10)
			GPIOA->MODER |= GPIO_MODER_MODE15_1;
			GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL15);
			GPIOA->AFR[1] |= (0x5 << 28);	//Set the af of pa0 to 2 for TIM2 CH1


		}
		else {


			//****SET UP TIM2 CH1 AS PWM ON PA5*****//

			GPIOA->MODER &= ~(GPIO_MODER_MODE5); //AF mode (10)
			GPIOA->MODER |= GPIO_MODER_MODE5_1;
			GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL5);
			GPIOA->AFR[0] |= 0x5 << (5 * 4);	//Set the af of pa0 to 2 for TIM2 CH1

			GPIOA->MODER &= ~(GPIO_MODER_MODE15);
			GPIOA->MODER |= GPIO_MODER_MODE15_0;
			GPIOA->ODR |= 1 << 15;
		}


		TIM2->CCER &= ~(TIM_CCER_CC1E);
		TIM2->CR1 &= ~(TIM_CR1_CEN);
		TIM2->ARR = 40; //8 cycles per transition; period is (8 / 1MHz) * 2 = 16us
		TIM2->CCR1 = ccrval; //CCRx = 7. since arr = 8, signal will be high for 7 / 8 of period, 87.5 % duty cycle, or 14us high out of 16us period
		TIM2->CR1 |= TIM_CR1_CEN;
		TIM2->CCER |= TIM_CCER_CC1E;

		if (ccrval >= 39) {
			nano_wait(10000000);
			up = 0;

			if (clockwise == 1) clockwise = 0;
			else clockwise = 1;

		}
		if (ccrval <= 0) {
			up = 1;
			nano_wait(10000000);

		}

		//wait longer at full speed
		if (ccrval == 0) for (int a = 0; a < 15; a++) nano_wait(10000000);

		if (up == 1) ccrval += 1;
		else ccrval -= 1;

		nano_wait(10000000);


	}



}
//
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
//
//  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
//  {
//  Error_Handler();
//  }
//  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
//  LL_RCC_HSE_EnableBypass();
//  LL_RCC_HSE_Enable();
//
//   /* Wait till HSE is ready */
//  while(LL_RCC_HSE_IsReady() != 1)
//  {
//
//  }
//  LL_RCC_HSI48_Enable();
//
//   /* Wait till HSI48 is ready */
//  while(LL_RCC_HSI48_IsReady() != 1)
//  {
//
//  }
//  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLL_MUL_12, LL_RCC_PLL_DIV_3);
//  LL_RCC_PLL_Enable();
//
//   /* Wait till PLL is ready */
//  while(LL_RCC_PLL_IsReady() != 1)
//  {
//
//  }
//  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
//  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
//  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
//  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
//
//   /* Wait till System clock is ready */
//  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
//  {
//
//  }
//  LL_SetSystemCoreClock(32000000);
//
//   /* Update the time base */
//  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
//  {
//    Error_Handler();
//  };
//  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
//  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);
//  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_HSI48);
//}
//
///**
//  * @brief ADC Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_ADC_Init(void)
//{
//
//  /* USER CODE BEGIN ADC_Init 0 */
//
//  /* USER CODE END ADC_Init 0 */
//
//  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
//  LL_ADC_InitTypeDef ADC_InitStruct = {0};
//
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* Peripheral clock enable */
//  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
//
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
//  /**ADC GPIO Configuration
//  PA6   ------> ADC_IN6
//  */
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /* ADC interrupt Init */
//  NVIC_SetPriority(ADC1_COMP_IRQn, 0);
//  NVIC_EnableIRQ(ADC1_COMP_IRQn);
//
//  /* USER CODE BEGIN ADC_Init 1 */
//
//  /* USER CODE END ADC_Init 1 */
//  /** Configure Regular Channel
//  */
//  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_6);
//  /** Common config
//  */
//  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
//  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_1RANK;
//  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
//  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
//  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
//  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
//  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_160CYCLES_5);
//  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
//  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
//  LL_ADC_SetCommonFrequencyMode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_FREQ_MODE_HIGH);
//  LL_ADC_DisableIT_EOC(ADC1);
//  LL_ADC_DisableIT_EOS(ADC1);
//  LL_ADC_EnableInternalRegulator(ADC1);
//  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
//  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
//  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
//  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
//  LL_ADC_Init(ADC1, &ADC_InitStruct);
//  /* USER CODE BEGIN ADC_Init 2 */
//
//  /* USER CODE END ADC_Init 2 */
//
//}
//
///**
//  * @brief I2C1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_I2C1_Init(void)
//{
//
//  /* USER CODE BEGIN I2C1_Init 0 */
//
//  /* USER CODE END I2C1_Init 0 */
//
//  LL_I2C_InitTypeDef I2C_InitStruct = {0};
//
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
//  /**I2C1 GPIO Configuration
//  PB8   ------> I2C1_SCL
//  PB9   ------> I2C1_SDA
//  */
//  GPIO_InitStruct.Pin = MFX_I2C_SCL_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
//  LL_GPIO_Init(MFX_I2C_SCL_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = MFX_I2C_SDA_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
//  LL_GPIO_Init(MFX_I2C_SDA_GPIO_Port, &GPIO_InitStruct);
//
//  /* Peripheral clock enable */
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
//
//  /* USER CODE BEGIN I2C1_Init 1 */
//
//  /* USER CODE END I2C1_Init 1 */
//  /** I2C Initialization
//  */
//  LL_I2C_EnableAutoEndMode(I2C1);
//  LL_I2C_DisableOwnAddress2(I2C1);
//  LL_I2C_DisableGeneralCall(I2C1);
//  LL_I2C_EnableClockStretching(I2C1);
//  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
//  I2C_InitStruct.Timing = 0x00707CBB;
//  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
//  I2C_InitStruct.DigitalFilter = 0;
//  I2C_InitStruct.OwnAddress1 = 0;
//  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
//  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
//  LL_I2C_Init(I2C1, &I2C_InitStruct);
//  LL_I2C_SetOwnAddress2(I2C1, 0, LL_I2C_OWNADDRESS2_NOMASK);
//  /* USER CODE BEGIN I2C1_Init 2 */
//
//  /* USER CODE END I2C1_Init 2 */
//
//}
//
///**
//  * @brief SPI1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_SPI1_Init(void)
//{
//
//  /* USER CODE BEGIN SPI1_Init 0 */
//
//  /* USER CODE END SPI1_Init 0 */
//
//  LL_SPI_InitTypeDef SPI_InitStruct = {0};
//
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* Peripheral clock enable */
//  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
//
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
//  /**SPI1 GPIO Configuration
//  PA15   ------> SPI1_NSS
//  PB3   ------> SPI1_SCK
//  PB5   ------> SPI1_MOSI
//  */
//  GPIO_InitStruct.Pin = ePD1_CS_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
//  LL_GPIO_Init(ePD1_CS_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = ePD1_SCK_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
//  LL_GPIO_Init(ePD1_SCK_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = ePD1_MOSI_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
//  LL_GPIO_Init(ePD1_MOSI_GPIO_Port, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN SPI1_Init 1 */
//
//  /* USER CODE END SPI1_Init 1 */
//  /* SPI1 parameter configuration*/
//  SPI_InitStruct.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
//  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
//  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
//  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
//  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
//  SPI_InitStruct.NSS = LL_SPI_NSS_HARD_INPUT;
//  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
//  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
//  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
//  SPI_InitStruct.CRCPoly = 7;
//  LL_SPI_Init(SPI1, &SPI_InitStruct);
//  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
//  /* USER CODE BEGIN SPI1_Init 2 */
//
//  /* USER CODE END SPI1_Init 2 */
//
//}
//
///**
//  * @brief SPI2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_SPI2_Init(void)
//{
//
//  /* USER CODE BEGIN SPI2_Init 0 */
//
//  /* USER CODE END SPI2_Init 0 */
//
//  LL_SPI_InitTypeDef SPI_InitStruct = {0};
//
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* Peripheral clock enable */
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
//
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
//  /**SPI2 GPIO Configuration
//  PB12   ------> SPI2_NSS
//  PB13   ------> SPI2_SCK
//  PB14   ------> SPI2_MISO
//  PB15   ------> SPI2_MOSI
//  */
//  GPIO_InitStruct.Pin = NFC_NSS_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
//  LL_GPIO_Init(NFC_NSS_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = NFC_SCK_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
//  LL_GPIO_Init(NFC_SCK_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = NFC_MISO_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
//  LL_GPIO_Init(NFC_MISO_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = NFC_MOSI_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
//  LL_GPIO_Init(NFC_MOSI_GPIO_Port, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN SPI2_Init 1 */
//
//  /* USER CODE END SPI2_Init 1 */
//  /* SPI2 parameter configuration*/
//  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
//  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
//  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
//  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
//  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
//  SPI_InitStruct.NSS = LL_SPI_NSS_HARD_INPUT;
//  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
//  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
//  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
//  SPI_InitStruct.CRCPoly = 7;
//  LL_SPI_Init(SPI2, &SPI_InitStruct);
//  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
//  /* USER CODE BEGIN SPI2_Init 2 */
//
//  /* USER CODE END SPI2_Init 2 */
//
//}
//
///**
//  * @brief TIM2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM2_Init(void)
//{
//
//  /* USER CODE BEGIN TIM2_Init 0 */
//
//  /* USER CODE END TIM2_Init 0 */
//
//  LL_TIM_InitTypeDef TIM_InitStruct = {0};
//  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
//
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* Peripheral clock enable */
//  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
//
//  /* USER CODE BEGIN TIM2_Init 1 */
//
//  /* USER CODE END TIM2_Init 1 */
//  TIM_InitStruct.Prescaler = 0;
//  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
//  TIM_InitStruct.Autoreload = 2000;
//  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
//  LL_TIM_Init(TIM2, &TIM_InitStruct);
//  LL_TIM_DisableARRPreload(TIM2);
//  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
//  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
//  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
//  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
//  TIM_OC_InitStruct.CompareValue = 500;
//  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
//  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
//  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH1);
//  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
//  LL_TIM_DisableMasterSlaveMode(TIM2);
//  /* USER CODE BEGIN TIM2_Init 2 */
//
//  /* USER CODE END TIM2_Init 2 */
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
//  /**TIM2 GPIO Configuration
//  PA0   ------> TIM2_CH1
//  */
//  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
//  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//}
//
///**
//  * @brief TSC Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TSC_Init(void)
//{
//
//  /* USER CODE BEGIN TSC_Init 0 */
//
//  /* USER CODE END TSC_Init 0 */
//
//  /* USER CODE BEGIN TSC_Init 1 */
//
//  /* USER CODE END TSC_Init 1 */
//  /** Configure the TSC peripheral
//  */
//  htsc.Instance = TSC;
//  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
//  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
//  htsc.Init.SpreadSpectrum = DISABLE;
//  htsc.Init.SpreadSpectrumDeviation = 1;
//  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
//  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
//  htsc.Init.MaxCountValue = TSC_MCV_8191;
//  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
//  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
//  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
//  htsc.Init.MaxCountInterrupt = DISABLE;
//  htsc.Init.ChannelIOs = TSC_GROUP1_IO3|TSC_GROUP3_IO2;
//  htsc.Init.ShieldIOs = 0;
//  htsc.Init.SamplingIOs = TSC_GROUP1_IO4|TSC_GROUP2_IO4|TSC_GROUP3_IO3;
//  if (HAL_TSC_Init(&htsc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TSC_Init 2 */
//
//  /* USER CODE END TSC_Init 2 */
//
//}
//
///**
//  * @brief USART1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART1_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART1_Init 0 */
//
//  /* USER CODE END USART1_Init 0 */
//
//  LL_USART_InitTypeDef USART_InitStruct = {0};
//
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* Peripheral clock enable */
//  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
//
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
//  /**USART1 GPIO Configuration
//  PA9   ------> USART1_TX
//  PA10   ------> USART1_RX
//  */
//  GPIO_InitStruct.Pin = USART_TX_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
//  LL_GPIO_Init(USART_TX_GPIO_Port, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = USART_RX_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
//  LL_GPIO_Init(USART_RX_GPIO_Port, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN USART1_Init 1 */
//
//  /* USER CODE END USART1_Init 1 */
//  USART_InitStruct.BaudRate = 115200;
//  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
//  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
//  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
//  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
//  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
//  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
//  LL_USART_Init(USART1, &USART_InitStruct);
//  LL_USART_ConfigAsyncMode(USART1);
//  LL_USART_Enable(USART1);
//  /* USER CODE BEGIN USART1_Init 2 */
//
//  /* USER CODE END USART1_Init 2 */
//
//}

///**
//  * @brief USB Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USB_PCD_Init(void)
//{
//
//  /* USER CODE BEGIN USB_Init 0 */
//
//  /* USER CODE END USB_Init 0 */
//
//  /* USER CODE BEGIN USB_Init 1 */
//
//  /* USER CODE END USB_Init 1 */
//  hpcd_USB_FS.Instance = USB;
//  hpcd_USB_FS.Init.dev_endpoints = 8;
//  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
//  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
//  hpcd_USB_FS.Init.low_power_enable = DISABLE;
//  hpcd_USB_FS.Init.lpm_enable = DISABLE;
//  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
//  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USB_Init 2 */
//
//  /* USER CODE END USB_Init 2 */
//
//}

///*
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
//  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOH);
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
//
//  /**/
//  LL_GPIO_ResetOutputPin(LD_R_GPIO_Port, LD_R_Pin);
//
//  /**/
//  LL_GPIO_ResetOutputPin(ePD1_RESET_GPIO_Port, ePD1_RESET_Pin);
//
//  /**/
//  LL_GPIO_ResetOutputPin(ePD1_PWR_ENn_GPIO_Port, ePD1_PWR_ENn_Pin);
//
//  /**/
//  LL_GPIO_ResetOutputPin(ePD1_D_C_GPIO_Port, ePD1_D_C_Pin);
//
//  /**/
//  LL_GPIO_ResetOutputPin(LD_G_GPIO_Port, LD_G_Pin);
//
//  /**/
//  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);
//
//  /**/
//  LL_GPIO_SetPinPull(MFX_IRQ_OUT_GPIO_Port, MFX_IRQ_OUT_Pin, LL_GPIO_PULL_NO);
//
//  /**/
//  LL_GPIO_SetPinMode(MFX_IRQ_OUT_GPIO_Port, MFX_IRQ_OUT_Pin, LL_GPIO_MODE_INPUT);
//
//  /**/
//  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
//  EXTI_InitStruct.LineCommand = ENABLE;
//  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
//  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
//  LL_EXTI_Init(&EXTI_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = MFX_WAKEUP_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(MFX_WAKEUP_GPIO_Port, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = LD_R_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(LD_R_GPIO_Port, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = ePD1_RESET_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(ePD1_RESET_GPIO_Port, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = ePD1_PWR_ENn_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(ePD1_PWR_ENn_GPIO_Port, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = ePD1_D_C_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(ePD1_D_C_GPIO_Port, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = ePD1_BUSY_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(ePD1_BUSY_GPIO_Port, &GPIO_InitStruct);
//
//  /**/
//  GPIO_InitStruct.Pin = LD_G_Pin;
//  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//  LL_GPIO_Init(LD_G_GPIO_Port, &GPIO_InitStruct);
//
//}
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
//
///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
