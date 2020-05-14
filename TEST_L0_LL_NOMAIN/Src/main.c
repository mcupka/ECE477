#include "main.h"

#include "stm32l0xx.h"


#include <stdio.h>

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

int main() {


	RCC->IOPENR |= RCC_IOPENR_IOPBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE12);
	GPIOB->MODER |= GPIO_MODER_MODE12_0;
	GPIOB->ODR &= ~(1 << 12);

	GPIOB->MODER &= ~(GPIO_MODER_MODE5);
	GPIOB->MODER |= GPIO_MODER_MODE5_0;
	GPIOB->ODR &= ~(1 << 5);

	initialize();

while (1) {

	setup_untightened();

	while (flag_tighten == 0) {
		if (flag_sync == 1) sync();
		SCB->SCR &= ~( SCB_SCR_SLEEPDEEP_Msk);
		wait_for_interrupt();
	}

	setup_tightened();

	while (flag_untighten == 0) {
		if (flag_sync == 1) sync();
		SCB->SCR &= ~( SCB_SCR_SLEEPDEEP_Msk);
		wait_for_interrupt();
	}



}
	return 0;
}


void setup_untightened() {
	//disable the untighten button interrupt
	EXTI->IMR &= ~EXTI_IMR_IM9;
	NVIC->ISER[0] &= ~(1 << EXTI4_15_IRQn);

	motor_up = 0;

	GPIOB->ODR |= (1 << 12);


	//Disable the sync button interrupt
	EXTI->IMR &= ~(EXTI_IMR_IM8);

	//disable step counter
	step_counter_enabled(0);

	GPIOB->ODR &= ~(1 << 5);
	start_encoder();
	motor_driver_encoder(DIR_UNTIGHTEN);

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

}

void setup_tightened() {

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

	//enable step counter
	state = STATE_TIGHT;
	step_counter_enabled(1);

	GPIOB->ODR &= ~(1 << 5);
	GPIOB->ODR &= ~(1 << 12);

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

extern int steps;
void sync() {

		int battery_life = (MAX_MAH - (battery_ticks * MAH_PER_TICK)) / (MAX_MAH / 100);

		//step count breakdown
		char step_count[4] = {(char)(steps >> 24), (char)((steps << 8) >> 24), (char)((steps << 16) >> 24), (char)((steps << 24) >> 24)};

		//communicate over uart with the nrf52 to synch the step count and battery life to the android application
		char test_message[8] = {'s', 'y', (char)100, (char)69, (char)(0x00), (char)(0x00), (char)(0x01), (char)(0xFF)};
		char message[8] = {'s', 'y', (char)battery_life, (char)100, step_count[0],step_count[1], step_count[2], step_count[3]};

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

void step_counter_enabled(int enabled) {

	if (enabled) {

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

	tim6_sel = 0;
	TIM6->DIER |= TIM_DIER_UIE;

	}
	else {
		RCC->APB1ENR &= ~(RCC_APB1ENR_TIM6EN);
		TIM6->CR1 &= ~(TIM_CR1_CEN); 			//disable timer 6

		EXTI->IMR &= ~EXTI_IMR_IM22;
		EXTI->IMR &= ~EXTI_IMR_IM21;

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
