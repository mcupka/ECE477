#include "main.h"
#include "stm32l0xx_it.h"

int tim6_update_count = 0;
int comp_rising = 1;
int steps = 0;
extern int battery_ticks;
extern int state;
extern int flag_tighten;

int tim6_sel = 0;
int debounce_int = 0;
int debounce_count = 0;


void NMI_Handler(void)
{}

void HardFault_Handler(void)
{
	while (1)
  {}
}

void SVC_Handler(void)
{}

void PendSV_Handler(void)
{}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void ADC1_COMP_IRQHandler(void)
{
	int debounce;

	//count steps if the shoe is tightened already
	if (state == STATE_TIGHT) {
		if (comp_rising == 1) {
				EXTI->IMR &= ~(EXTI_IMR_IM22); //disable interrupt
				EXTI->PR |= EXTI_PR_PIF22; //clear pending flag for interrupt

				EXTI->IMR |= EXTI_IMR_IM21; //enable lower bound interrupt
				comp_rising = 0;
			}
		else {
				EXTI->IMR &= ~(EXTI_IMR_IM21); //disable lower bound interrupt
				EXTI->PR |= EXTI_PR_PIF21; //clear pending flag for interrupt

				steps++;

				TIM6->CR1 |= TIM_CR1_CEN; //enable timer to count before another step may be counted
			}
	}
	
	//detect pressure to tighten shoe if shoe is not tightened
	else if (state == STATE_NOTTIGHT) {
		EXTI->IMR &= ~(EXTI_IMR_IM22);
		//EXTI->IMR &= ~(EXTI_IMR_IM21);
		EXTI->PR |= EXTI_PR_PIF22; //clear pending flag


		int x = 1000;
		for (int i = 0; i < 1000; i++) {
			if (COMP2->CSR & COMP_CSR_COMP2VALUE) x -= 1;
			nano_wait(10);
		}
		if (x > 900) flag_tighten = 1;

		EXTI->IMR |= EXTI_IMR_IM22;

	}

	else {

	return;
	}

}


extern int motor_up;
extern int stalled;
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
		if ((motor_frequency < 600) & (motor_up == 1)) {stalled = 1;}
		TIM22->CNT = 0;

	}

}


extern int flag_untighten;
extern int flag_sync;
void EXTI4_15_IRQHandler() {
	//interrupt for the coulomb counter (PC15)

	if (EXTI->PR & EXTI_PR_PIF15) {
		EXTI->PR |= EXTI_PR_PIF15; //clear pending flag for interrupt

		battery_ticks++;

		if ((MAX_MAH - (battery_ticks * MAH_PER_TICK)) <= MAH_LEFT_RED) //turn on battery indicator led
			GPIOC->ODR |= (1 << 14);
	}
	else if (EXTI->PR & EXTI_PR_PIF13) {
		//battery charge count reset button
		EXTI->PR |= EXTI_PR_PIF13; //clear pending flag for interrupt

		battery_ticks = 0;
		GPIOC->ODR &= ~(1 << 14);

	}
	else if ((EXTI->PR & EXTI_PR_PIF9) != 0) {
		//untighten button
		EXTI->PR |= EXTI_PR_PIF9; //clear pending flag for interrupt
		EXTI->IMR &= ~(EXTI_IMR_IM9);

		//debounce
		int deb = 1;
		while (deb > 0) {
			deb = 0;
			for (int x = 0; x < 100; x++) {
				if ((GPIOB->IDR & (1 << 9)) == 0) deb++;
				nano_wait(10);
			}
		}

		flag_untighten = 1;
		EXTI->IMR |= EXTI_IMR_IM9;
	}
	else if ((EXTI->PR & EXTI_PR_PIF8) != 0) {
		//untighten button
		EXTI->PR |= EXTI_PR_PIF8; //clear pending flag for interrupt
		EXTI->IMR &= ~(EXTI_IMR_IM8);

		//debounce
		int deb = 1;
		while (deb > 0) {
			deb = 0;
			for (int x = 0; x < 100; x++) {
				if ((GPIOB->IDR & (1 << 8)) == 0) deb++;
				nano_wait(10);
			}
		}

		flag_sync = 1;
		EXTI->IMR |= EXTI_IMR_IM8;
	}
}

