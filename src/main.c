
#include "stm32f103xx.h"

void delay(int t){
	for (int i = t *8000 ;  i > 0; i--);
}

int main(void)
{
	GPIOA_CLK_EN();
	GPIOA->CRL &= ~(0xF << (1 * 4));
	GPIOA->CRL |=  (0x2 << (1 * 4));
	while(1){
		GPIOA->ODR ^= (1 << 1);
		delay(200);
	}
}
