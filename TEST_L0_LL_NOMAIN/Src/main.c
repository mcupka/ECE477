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

#include "stm32l0xx.h"


#include <stdio.h>

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


#define TIM6_SEL_STEP_DEBOUNCER 0x0
#define TIM6_SEL_FREQ_SAMPLER 0x1
#define DIR_TIGHTEN 0x0
#define DIR_UNTIGHTEN 0x1


int motor_frequency = 0;
int motor_up = 0;
int battery_ticks = 0;
int flag_tighten = 0;
int flag_untighten = 0;
int flag_sync = 0;
int stalled = 0;
int step_count = 0;

int state = STATE_INIT; //variable used to hold the state of the machine
extern int tim6_sel;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

//----------------
/* Port mappings:
 * PC15: Coulomb counter interrupt
 *  
*/
//---------------
int main() {

	//while(1) {
	//	RCC->IOPENR |= RCC_IOPENR_IOPBEN;
	//}

	//test_h_bridge();
	//test_encoder();
	//state = STATE_TIGHT;
	//test_comparator();
	//return 0;

	//test_uart();
	//test_coulomb_counter();
	//return 0;

	RCC->IOPENR |= RCC_IOPENR_IOPBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE12);
	GPIOB->MODER |= GPIO_MODER_MODE12_0;
	GPIOB->ODR &= ~(1 << 12);

	GPIOB->MODER &= ~(GPIO_MODER_MODE5);
	GPIOB->MODER |= GPIO_MODER_MODE5_0;
	GPIOB->ODR &= ~(1 << 5);

	initialize();


while (1) {


	motor_frequency = 0;
	motor_up = 0;
	flag_tighten = 0;
	flag_untighten = 0;
	stalled = 0;


	//Heel Pressure Sensor
	GPIOB->MODER &= ~(GPIO_MODER_MODE3);
	GPIOB->MODER |= GPIO_MODER_MODE3_0 | GPIO_MODER_MODE3_1; //11 analog mode
	GPIOB->MODER &= ~(GPIO_MODER_MODE6);
	GPIOB->MODER |= GPIO_MODER_MODE6_0 | GPIO_MODER_MODE6_1;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //comparator shares a clock with the system configuration controller
	COMP2->CSR &= ~(COMP_CSR_COMP2INPSEL);
	COMP2->CSR |= (COMP_CSR_COMP2INPSEL_0 | COMP_CSR_COMP2INPSEL_1); //011 FOR PB6
	COMP2->CSR &= ~(COMP_CSR_COMP2INNSEL);
	COMP2->CSR |= COMP_CSR_COMP2INNSEL_0 | COMP_CSR_COMP2INNSEL_1 | COMP_CSR_COMP2INNSEL_2; //111 FOR PB3
	COMP2->CSR |= COMP_CSR_COMP2POLARITY;
	COMP2->CSR |= COMP_CSR_COMP2EN;
	EXTI->IMR |= EXTI_IMR_IM22;
	EXTI->FTSR |= EXTI_FTSR_FT22;
	NVIC->ISER[0] |= 1 << ADC1_COMP_IRQn;

	//enable sync button interrupt
	GPIOB->MODER &= ~(GPIO_MODER_MODE8);
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //enable clock for sys config
	SYSCFG->EXTICR[2] &= ~(SYSCFG_EXTICR3_EXTI8); //Choose PB9 as EXTI source
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PB; //Choose PB9 as EXTI source
	NVIC->ISER[0] |= (1 << EXTI4_15_IRQn);
	EXTI->RTSR |= EXTI_RTSR_RT8; //Falling edge triggered
	EXTI->FTSR &= ~EXTI_FTSR_FT8;
	EXTI->IMR |= EXTI_IMR_IM8; //unmask the interrupt

	state = STATE_NOTTIGHT;
	flag_tighten = 0;
	flag_sync = 0;
	GPIOB->ODR &= ~(1 << 5);
	GPIOB->ODR &= ~(1 << 12);
	while (flag_tighten == 0) {
		if (flag_sync == 1) sync();
		SCB->SCR &= ~( SCB_SCR_SLEEPDEEP_Msk);
		wait_for_interrupt();
	}

	//disable previous peripherals
	COMP2->CSR &= ~(COMP_CSR_COMP2EN);
	EXTI->IMR &= ~(EXTI_IMR_IM22);
	NVIC->ISER[0] &= ~(1 << ADC1_COMP_IRQn);
	EXTI->IMR &= ~(EXTI_IMR_IM8);

	//Pressure sensor on face of shoe
	GPIOB->MODER &= ~(GPIO_MODER_MODE7);
	GPIOB->MODER |= GPIO_MODER_MODE7_0 | GPIO_MODER_MODE7_1;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //comparator shares a clock with the system configuration controller
	SYSCFG->CFGR3 |= SYSCFG_CFGR3_ENBUFLP_VREFINT_COMP | SYSCFG_CFGR3_EN_VREFINT;
	COMP2->CSR &= ~(COMP_CSR_COMP2INPSEL);
	COMP2->CSR |= COMP_CSR_COMP2INPSEL_2; //100 FOR PB7
	COMP2->CSR &= ~(COMP_CSR_COMP2INNSEL);
	COMP2->CSR |= COMP_CSR_COMP2INNSEL_0 | COMP_CSR_COMP2INNSEL_1 | COMP_CSR_COMP2INNSEL_2; //111 FOR PB3

	//COMP2->CSR |= COMP_CSR_COMP2INNSEL_2 | COMP_CSR_COMP2INNSEL_0; //101 FOR 1/2 VREFINT

	COMP2->CSR |= COMP_CSR_COMP2POLARITY;
	COMP2->CSR |= COMP_CSR_COMP2EN;

	start_encoder();
	motor_driver_encoder(DIR_TIGHTEN);

	GPIOB->ODR |= (1 << 5); //signal that we are in the tightened state

	//Disable previous peripherals
	TIM22->CR1 &= ~TIM_CR1_CEN;
	TIM6->CR1 &= ~TIM_CR1_CEN;
	RCC->APB2ENR &= ~RCC_APB2ENR_TIM22EN;
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;


	//Enable the untighten button interrupt
	GPIOB->MODER &= ~(GPIO_MODER_MODE9);
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //enable clock for sys config
	SYSCFG->EXTICR[2] &= ~(SYSCFG_EXTICR3_EXTI9); //Choose PB9 as EXTI source
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI9_PB; //Choose PB9 as EXTI source
	NVIC->ISER[0] |= (1 << EXTI4_15_IRQn);
	EXTI->RTSR |= EXTI_RTSR_RT9; //Falling edge triggered
	EXTI->FTSR &= ~EXTI_FTSR_FT9;
	EXTI->IMR |= EXTI_IMR_IM9; //unmask the interrupt

	//enable the sync button interrupt
	EXTI->IMR |= EXTI_IMR_IM8;

	//wait for untighten interrupt
	flag_untighten = 0;
	flag_sync = 0;

	GPIOB->ODR &= ~(1 << 5);
	GPIOB->ODR &= ~(1 << 12);
	while (flag_untighten == 0) {
		if (flag_sync == 1) sync();
		SCB->SCR &= ~( SCB_SCR_SLEEPDEEP_Msk);
		wait_for_interrupt();
	}

	//disable the untighten button interrupt
	EXTI->IMR &= ~EXTI_IMR_IM9;
	NVIC->ISER[0] &= ~(1 << EXTI4_15_IRQn);

	motor_up = 0;

	GPIOB->ODR |= (1 << 12);


	//Disable the sync button interrupt
	EXTI->IMR &= ~(EXTI_IMR_IM8);

	GPIOB->ODR &= ~(1 << 5);
	start_encoder();
	motor_driver_encoder(DIR_UNTIGHTEN);

}
	return 0;
}

void write_eeprom_data() {
	//function to write saved data to the eeprom

	//unlock the eeprom
	while ((FLASH->SR & FLASH_SR_BSY) != 0);
	if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0)
	{
	 FLASH->PEKEYR = FLASH_PEKEY1;
	 FLASH->PEKEYR = FLASH_PEKEY2;
	}

	//write to the eeprom
	*(uint32_t *)(DATA_EEPROM_BASE + 0x4) = battery_ticks;
	*(uint32_t *)(DATA_EEPROM_BASE + 0x8) = step_count;

	//lock the eeprom
	while ((FLASH->SR & FLASH_SR_BSY) != 0);
	FLASH->PECR |= FLASH_PECR_PELOCK;
}

void get_eeprom_data() {
	//function to get data from non-volatile memory

	//unlock the eeprom
	while ((FLASH->SR & FLASH_SR_BSY) != 0);
	if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0)
	{
	 FLASH->PEKEYR = FLASH_PEKEY1;
	 FLASH->PEKEYR = FLASH_PEKEY2;
	}
	//read from the eeprom
	battery_ticks = *(uint32_t *)(DATA_EEPROM_BASE + 0x4);
	step_count = *(uint32_t *)(DATA_EEPROM_BASE + 0x8);

	//lock the eeprom
	while ((FLASH->SR & FLASH_SR_BSY) != 0);
	FLASH->PECR |= FLASH_PECR_PELOCK;

}

void sync() {

		int battery_life =(MAX_MAH - (battery_ticks * MAH_PER_TICK)) / (MAX_MAH / 100);

		//communicate over uart with the nrf52 to synch the step count and battery life to the android application
		char test_message[8] = {'s', 'y', (char)100, (char)69, (char)(0x00), (char)(0x00), (char)(0x01), (char)(0xFF)};
		char message[8] = {'s', 'y', (char)battery_life, (char)69, 0, 0, 1, 0xFF};

		//Configure Pins
		RCC->IOPENR |= RCC_IOPENR_IOPAEN;
		GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
		GPIOA->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1;
		GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10);
		GPIOA->AFR[1] |= 0x4 << 4; //usart1_tx pa9
		GPIOA->AFR[1] |= 0x4 << 8; //usart1_rx pa10


		RCC->APB2ENR |= RCC_APB2ENR_USART1EN; //enable usart1 clock

		//Configure USART1 TX
		/* (1) oversampling by 16, 9600 baud */
		/* (2) 8 data bit, 1 start bit, 1 stop bit, no parity */
		USART1->BRR = 2100000 / 115200; /* (1) */
		USART1->CR1 |= USART_CR1_TE | USART_CR1_UE; /* (2) */

		//Send data via TX
		for (int a = 0; a < 8; a++) {
			USART1->TDR = message[a];
			while (!(USART1->ISR & USART_ISR_TC)); //wait for the transfer to be complete
		}

		//Enable the RX
		USART1->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;

		RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
		RCC->APB2ENR &= ~(RCC_APB2ENR_USART1EN);
		RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;


		flag_sync = 0;

}

void untighten() {
	//reverse the motor to untighten the shoe.
	state = STATE_UNTIGHTENING;
	nano_wait(10000000);
	GPIOB->ODR |= 1 << 12;
	start_encoder();
	motor_driver_encoder(DIR_UNTIGHTEN);
	state = STATE_NOTTIGHT;
	return;
}

void state_tight () {
	//setup the pressure sensor to count steps, wait for button press to untighten or to synch the shoe


	//setup button to untighten the shoe. Go to sleep.
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //enable clock for sys config
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI2); //Choose PA2 as EXTI source on EXTI line 2
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA;

	//set up pa2 for the untighten button
	RCC->IOPENR |= RCC_IOPENR_IOPAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODE2);
	GPIOA->MODER |= GPIO_MODER_MODE2_0;
	NVIC_EnableIRQ(EXTI2_3_IRQn);
	EXTI->RTSR |= EXTI_RTSR_RT2; //Falling edge triggered
	EXTI->IMR |= EXTI_IMR_IM2; //unmask the interrupt

	GPIOB->ODR |= 1 << 5;
	while (1) {
		if (flag_untighten == 1) {
			GPIOB->ODR &= ~(1 << 5);
			untighten();
			state = STATE_NOTTIGHT;
			return;
		}
		else wait_for_interrupt();
		nano_wait(10000);
	}
}

void tighten() {
	//setup 2nd pressure sensor and magnetic encoder for tightening procedure. tighten the shoe.
	GPIOB->ODR |= 1 << 12;
	start_encoder();
	motor_driver_encoder(DIR_TIGHTEN);
	state = STATE_TIGHT;
	return;
}

void state_not_tight() {
	int x;

	//set up the pressure sensor to trigger interrupt

	//configure PB3 and PB6 as analog inputs to the comparator 2
	RCC->IOPENR |= RCC_IOPENR_IOPBEN;

	GPIOB->MODER &= ~(GPIO_MODER_MODE3);
	GPIOB->MODER |= GPIO_MODER_MODE3_0 | GPIO_MODER_MODE3_1; //11 analog mode
	GPIOB->MODER &= ~(GPIO_MODER_MODE6);
	GPIOB->MODER |= GPIO_MODER_MODE6_0 | GPIO_MODER_MODE6_1;

	//configure pa4 as analog input for comparator 1
	GPIOA->MODER &= ~(GPIO_MODER_MODE4);
	GPIOA->MODER |= GPIO_MODER_MODE4_0 | GPIO_MODER_MODE4_1;

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //comparator shares a clock with the system configuration controller


	//comarator 2 setup
	COMP2->CSR &= ~(COMP_CSR_COMP2INPSEL);
	COMP2->CSR |= (COMP_CSR_COMP2INPSEL_0 | COMP_CSR_COMP2INPSEL_1); //011 FOR PB6

	COMP2->CSR &= ~(COMP_CSR_COMP2INNSEL);
	COMP2->CSR |= COMP_CSR_COMP2INNSEL_0 | COMP_CSR_COMP2INNSEL_1 | COMP_CSR_COMP2INNSEL_2; //111 FOR PB3

	COMP2->CSR |= COMP_CSR_COMP2POLARITY;

	COMP2->CSR |= COMP_CSR_COMP2EN;

	//enable timer and interrupt that we will use to "debounce" the interrupt for the pressure sensor
	//TIMER 6
	//RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	//enable intrerrupt on exti line 22 (comparator 2)
	EXTI->IMR |= EXTI_IMR_IM22;
	//EXTI->RTSR |= EXTI_RTSR_RT22;
	EXTI->FTSR |= EXTI_FTSR_FT22;

	NVIC->ISER[0] |= 1 << ADC1_COMP_IRQn;


	//setup ADC to poll to make sure that the pressure sensor is actually being pressed enough and it's not just noise that triggers it
	/*RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	RCC->IOPENR |= RCC_IOPENR_IOPAEN;
	RCC->CR |= RCC_CR_HSION;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	while ((RCC->CR & RCC_CR_HSIRDY) == 0);
	//pb1 adc channel 7
	GPIOA->MODER &= ~(GPIO_MODER_MODE7);
	//GPIOA->MODER |= GPIO_MODER_MODE7_0 | GPIO_MODER_MODE7_1; //11 for analog mode

	ADC1->CFGR2 |= ADC_CFGR2_CKMODE;

	if ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
		ADC1->CR |= ADC_CR_ADDIS;
	}

	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->ISR & ADC_ISR_EOCAL) == 0){}
	ADC1->ISR |= ADC_ISR_EOCAL;

	ADC1->ISR |= ADC_ISR_ADRDY;


	ADC1->CR |= ADC_CR_ADEN;
	if ((ADC1->CFGR1 & ADC_CFGR1_AUTOFF) == 0)
	{
		while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {}
	}

	ADC1->CHSELR = ADC_CHSELR_CHSEL7;
	ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2;
	ADC->CCR |= ADC_CCR_VREFEN;

	ADC1->CHSELR = 1 << 0;
    while(!(ADC1->ISR & ADC_ISR_ADRDY)) {};
    ADC1->CR |= ADC_CR_ADSTART;
    while(!(ADC1->ISR & ADC_ISR_EOC)) {};
/*
	while (1) {
		if (ADC1->DR > 400) tighten(); else GPIOB->ODR &= ~(1 << 12);
		ADC1->CR |= ADC_CR_ADSTART;
		//nano_wait(10000);
		while ((ADC1->ISR & ADC_ISR_EOC) == 0) {}
	}
*/
//ADC1->CR &= ~(ADC_CR_ADEN);

    flag_tighten = 0;
	while(1) {
		wait_for_interrupt();
		if (flag_tighten == 0) {
			//GPIOB->ODR &= ~(1 << 12);
		}
		else {
			//x = 1000;
			//for (int i = 0; i < 1000; i++) {
			//	if (COMP2->CSR & COMP_CSR_COMP2VALUE) x -= 1;
			//	nano_wait(10);
			//}
			//if (x > 900) tighten();
			//flag_tighten = 0;
			//EXTI->IMR |= EXTI_IMR_IM22;
			state = STATE_TIGHTENING;
			return;
		}
	}

	return;

}

void initialize() {

	//This function needs to: setup coulbomb counter

	get_eeprom_data(); //get the saved step count and battery life

	//Setup coulbomb counter	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //enable clock for sys config
	SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI15); //Choose PC15 as EXTI source
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PC; 

	RCC->IOPENR |= RCC_IOPENR_IOPCEN;

	GPIOC->MODER &= ~GPIO_MODER_MODE15; //00 for input mode

	NVIC_EnableIRQ(EXTI4_15_IRQn);

	EXTI->FTSR |= EXTI_FTSR_FT15; //Falling edge triggered
	EXTI->IMR |= EXTI_IMR_IM15; //unmask the interrupt

	state = STATE_NOTTIGHT;

	//setup the battery LED indicator
	GPIOC->MODER &= ~(GPIO_MODER_MODE14);
	GPIOC->MODER |= GPIO_MODER_MODE14_0;
	GPIOC->ODR &= ~(1 << 14);

	//setup interrupt to reset the battery charge
	//Setup coulbomb counter
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //enable clock for sys config
	SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI13); //Choose PB13 as EXTI source
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PB;

	RCC->IOPENR |= RCC_IOPENR_IOPBEN;

	GPIOB->MODER &= ~GPIO_MODER_MODE13; //00 for input mode

	EXTI->FTSR |= EXTI_FTSR_FT13; //Falling edge triggered
	EXTI->IMR |= EXTI_IMR_IM13; //unmask the interrupt

	return;
}


void enable_frequency_sampler() {
	//uses timer 6 to sample the frequency.

	motor_frequency = 0;
	tim6_sel = TIM6_SEL_FREQ_SAMPLER;

	//enable the timer 6 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	NVIC->ISER[0] |= 1 << TIM6_DAC_IRQn;

	TIM6->DIER |= TIM_DIER_UIE;

	TIM6->CR1 |= TIM_CR1_CEN;


}

void check_stall() {

	if (motor_frequency > 700) {GPIOB->ODR |= 1 << 12; motor_up = 1;}

	if (motor_frequency < 100 && motor_up == 1) {GPIOB->ODR &= ~(1 << 12); TIM2->CCR1 = 40; stalled = 1;}
	else return;
}

void start_encoder() {
	//function to start the encoder timer counting the pulses

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

	//enable basic timer 6. Used to sample and reset the pulse count and convert it to an approximate frequency
	enable_frequency_sampler();

}

//function to test the motor while fitting it to the shoe. Since I don't have a power supply anymore I'll need to use the encoder to automatically cut power to the motor when it starts to stall
void motor_driver_encoder(int direction) {


	//Going to use PB12 for led feedback for this since pa5 is being used
	RCC->IOPENR |= RCC_IOPENR_IOPBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE12);
	GPIOB->MODER |= GPIO_MODER_MODE12_0;
	GPIOB->ODR &= ~(1 << 12);

	RCC->IOPENR |= RCC_IOPENR_IOPAEN;


	short clockwise = direction;

	if (clockwise == 0x1) {

		GPIOA->MODER &= ~(GPIO_MODER_MODE5);
		GPIOA->MODER |= GPIO_MODER_MODE5_0;
		GPIOA->ODR &= ~(1 << 5);

		GPIOA->MODER &= ~(GPIO_MODER_MODE15);
		GPIOA->MODER |= GPIO_MODER_MODE15_0;
		GPIOA->ODR |= 1 << 15;

		}

	else {

		GPIOA->MODER &= ~(GPIO_MODER_MODE5);
		GPIOA->MODER |= GPIO_MODER_MODE5_0;
		GPIOA->ODR |= 1 << 5;

		GPIOA->MODER &= ~(GPIO_MODER_MODE15);
		GPIOA->MODER |= GPIO_MODER_MODE15_0;
		GPIOA->ODR &= ~(1 << 15);

		}

		stalled = 0;

		nano_wait(100000);

		//turn off the motor if it's stalling out
		while (1) {
			check_stall(); //check for real stall
			if (((GPIOB->IDR & (1 << 9)) == 0) && (direction == DIR_UNTIGHTEN)) stalled = 1; //check for button press if untightening
			if (((COMP2->CSR & COMP_CSR_COMP2VALUE) == 0) && (direction == DIR_TIGHTEN)) { //check for pressure sufficient if tightening
				//account for noise
					if (COMP2->CSR & COMP_CSR_COMP2VALUE);
					else {
							int deb = 0;
							for (int a = 0; a < 1000; a++)
							{
								if (COMP2->CSR & COMP_CSR_COMP2VALUE) deb++;
								nano_wait(10);

							}
							if (deb < 1) {stalled = 1; /*GPIOC->ODR &= ~(1 << 14);*/}
					}
			}

			//nano_wait(10000);
			if (stalled == 1) {

				GPIOA->ODR &= ~(1 << 5);
				GPIOA->ODR &= ~(1 << 15);


				//GPIOA->MODER |= GPIO_MODER_MODE5;
				//GPIOA->MODER |= GPIO_MODER_MODE15;

				stalled = 0;
				GPIOB->ODR &= ~(1 << 12);

				return;
			}
		}


}

void test_coulomb_counter() {

	//Coulomb counter connected to PC15. Using EXTI interrupt to count the pulses sent by the coulomb counter

	//onboard led for testing feedback. pa5
	RCC->IOPENR |= RCC_IOPENR_IOPBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE12);
	GPIOB->MODER |= GPIO_MODER_MODE12_0;
	GPIOB->ODR |= 1 << 12;

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //enable clock for sys config
	SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI15); //Choose PC15 as EXTI source
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PC; //Choose PC15 as EXTI source

	RCC->IOPENR |= RCC_IOPENR_IOPCEN;

	GPIOC->MODER &= ~GPIO_MODER_MODE15; //00 for input mode

	NVIC_EnableIRQ(EXTI4_15_IRQn);


	EXTI->FTSR |= EXTI_FTSR_FT15; //Falling edge triggered
	EXTI->IMR |= EXTI_IMR_IM15; //unmask the interrupt


	while(1) {
		nano_wait(10000);
	}

	//When the coulomb counter pulses, the interrupt will toggle the red led. With a 150 ohm resistor as the load and a 7.2V input voltage, this should happen around once every 12 seconds
	//With 3.3V, should happen about once every 28 seconds
}

void test_uart() {
	//testing uart using the Analog Discovery 2 as a RX/TX in place of the NRF52
	//Uses pins PA9 (STM TX) and PA10 (STM RX)

	char test_message[13] = "Test message\n";
	char test_rx[6] = {0, 0, 0, 0, 0, 0};
	char test_rx_expected[6] = "ledon\n";
	short match = 0;

	//Configure Pins
	RCC->IOPENR |= RCC_IOPENR_IOPAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
	GPIOA->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1;
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10);
	GPIOA->AFR[1] |= 0x4 << 4; //usart1_tx pa9
	GPIOA->AFR[1] |= 0x4 << 8; //usart1_rx pa10


	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; //enable usart1 clock

	//Configure USART1 TX
	/* (1) oversampling by 16, 9600 baud */
	/* (2) 8 data bit, 1 start bit, 1 stop bit, no parity */
	USART1->BRR = 2100000 / 9600; /* (1) */
	USART1->CR1 |= USART_CR1_TE | USART_CR1_UE; /* (2) */

	//Send data via TX
	for (int a = 0; a < 13; a++) {
		USART1->TDR = test_message[a];
		while (!(USART1->ISR & USART_ISR_TC)); //wait for the transfer to be complete
	}

	//Enable the RX
	USART1->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;

	//Receive some data. if it matches the expected string, turn the onboard LED on
	for (int a = 0; a < 6; a++) {
		while (!((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE));
		test_rx[a] = (uint8_t)(USART1->RDR); /* Receive data, clear flag */
	}

	for (int a = 0; a < 6; a++){
		if (test_rx[a] == test_rx_expected[a]) match++;
	}

	if (match == 6) {
		GPIOA->MODER &= ~(GPIO_MODER_MODE5);
		GPIOA->MODER |= GPIO_MODER_MODE5_0;
		GPIOA->ODR |= 1 << 5;
	}

	while (1) {
		nano_wait(10000000);
	}


}

void test_comparator() {


	//onboard led for testing feedback. pa5
	RCC->IOPENR |= RCC_IOPENR_IOPAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODE5);
	GPIOA->MODER |= GPIO_MODER_MODE5_0;
	GPIOA->ODR |= 1 << 5;

	//configure PB3 and PB6 as analog inputs to the comparator 2
	RCC->IOPENR |= RCC_IOPENR_IOPBEN;

	GPIOB->MODER &= ~(GPIO_MODER_MODE3);
	GPIOB->MODER |= GPIO_MODER_MODE3_0 | GPIO_MODER_MODE3_1; //11 analog mode
	GPIOB->MODER &= ~(GPIO_MODER_MODE6);
	GPIOB->MODER |= GPIO_MODER_MODE6_0 | GPIO_MODER_MODE6_1;

	//configure pa4 as analog input for comparator 1
	GPIOA->MODER &= ~(GPIO_MODER_MODE4);
	GPIOA->MODER |= GPIO_MODER_MODE4_0 | GPIO_MODER_MODE4_1;

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //comparator shares a clock with the system configuration controller


	//comarator 2 setup
	COMP2->CSR &= ~(COMP_CSR_COMP2INPSEL);
	COMP2->CSR |= (COMP_CSR_COMP2INPSEL_0 | COMP_CSR_COMP2INPSEL_1); //011 FOR PB6

	COMP2->CSR &= ~(COMP_CSR_COMP2INNSEL);
	COMP2->CSR |= COMP_CSR_COMP2INNSEL_0 | COMP_CSR_COMP2INNSEL_1 | COMP_CSR_COMP2INNSEL_2; //111 FOR PB3

	COMP2->CSR |= COMP_CSR_COMP2POLARITY;

	COMP2->CSR |= COMP_CSR_COMP2EN;


	//comparator 1 setup (lower bound)
	COMP1->CSR |= COMP_CSR_COMP1WM; //+ of comparator is shorted with + of comp 2 (pressure sensor)

	COMP1->CSR &= ~(COMP_CSR_COMP1INNSEL);
	COMP1->CSR |= COMP_CSR_COMP1INNSEL_1; //10 for PA4

	COMP1->CSR &= ~(COMP_CSR_COMP1POLARITY);

	COMP1->CSR |= COMP_CSR_COMP1EN;


	//enable timer and interrupt that we will use to "debounce" the interrupt for the pressure sensor
	//TIMER 6
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;


	NVIC->ISER[0] |= 1 << TIM6_DAC_IRQn;


	//enable intrerrupt on exti line 22 (comparator 2)
	EXTI->IMR |= EXTI_IMR_IM22;
	EXTI->RTSR |= EXTI_RTSR_RT22;

	EXTI->FTSR |= EXTI_FTSR_FT21; //falling edge
	EXTI->IMR &= ~(EXTI_IMR_IM21);

	NVIC->ISER[0] |= 1 << ADC1_COMP_IRQn;


	while (1) {
		nano_wait(10000000);
		nano_wait(10000000);
		nano_wait(10000000);
		nano_wait(10000000);
		GPIOA->ODR |= 1 << 5;
		wait_for_interrupt();
		GPIOA->ODR &= ~(1 << 5);
	}



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

void test_eeprom(int after_power_off) {

	RCC->IOPENR |= RCC_IOPENR_IOPBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE12);
	GPIOB->MODER |= GPIO_MODER_MODE12_0;
	GPIOB->ODR &= ~(1 << 12);
	GPIOB->MODER &= ~(GPIO_MODER_MODE5);
	GPIOB->MODER |= GPIO_MODER_MODE5_0;
	GPIOB->ODR &= ~(1 << 5);


	step_count = 1;

	if (after_power_off == 0) {
		//store data in eeprom
		step_count = 9987;
		write_eeprom_data();
	}
	else {
		get_eeprom_data();
		if (step_count == 9987) GPIOB->ODR |= (1 << 12);
		else GPIOB->ODR |= (1 << 5);
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
