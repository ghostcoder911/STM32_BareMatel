/*
 * main.c
 * LED Blink for STM32F446RE Nucleo Board
 * Onboard LED (LD2) is connected to PA5
 */

#include "stm32f446xx.h"

void delay(int t){
	for (int i = t * 8000; i > 0; i--);
}

int main(void)
{
	// Enable GPIOA clock
	GPIOA_CLK_EN();
	
	// Configure PA5 as output (onboard LED LD2 on Nucleo-F446RE)
	// Clear mode bits for pin 5 (bits 10-11)
	GPIOA->MODER &= ~(0x3 << (5 * 2));
	// Set mode to output (01)
	GPIOA->MODER |= (0x1 << (5 * 2));
	
	// Set output type to push-pull (default, bit 5 = 0)
	GPIOA->OTYPER &= ~(1 << 5);
	
	// Set output speed to medium (01)
	GPIOA->OSPEEDR &= ~(0x3 << (5 * 2));
	GPIOA->OSPEEDR |= (0x1 << (5 * 2));
	
	// No pull-up/pull-down (00)
	GPIOA->PUPDR &= ~(0x3 << (5 * 2));
	
	while(1){
		// Toggle PA5 (LED)
		GPIOA->ODR ^= (1 << 5);
		delay(200);
	}
}
