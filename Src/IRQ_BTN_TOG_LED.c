/*
 * IRQ_BTN_TOG_LED.c
 *
 *  Created on: Feb 11, 2024
 *      Author: daher
 */

#include <stdint.h>


#include "stm32f407xx_GPIO_driver.h"


int main(void)
{
	//enable clk port D & B
		GPIO_PeriClkControl(GPIOA,ENABLE);
		GPIO_PeriClkControl(GPIOD,ENABLE);

		//PORT D PIN 12 CONFIG
		GPIO_Handle_t my_GPIOD;
		my_GPIOD.pGPIOx = GPIOD;

		my_GPIOD.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
		my_GPIOD.GPIO_PinConfig.GPIO_PinNum = 12;
		my_GPIOD.GPIO_PinConfig.GPIO_PinOPType = OUT_PP;


		//PORT B PIN 12 CONFIG AS INPUT FROM BUTTOM
		GPIO_Handle_t my_GPIOA;
		my_GPIOA.pGPIOx = GPIOA;

		my_GPIOA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FE;
		my_GPIOA.GPIO_PinConfig.GPIO_PinNum = 5;
		my_GPIOA.GPIO_PinConfig.GPIO_PinPuPdControl = PUPD_DOWN;

		//Init PD12 AND PA0
		GPIO_Init(&my_GPIOA);
		GPIO_Init(&my_GPIOD);


		//IRQ Configuration
		GPIO_IRQConfig(IRQ_EXTI9_5, ENABLE);



	//TOGGLE LED PORTD PIN 12

	while(1){
		GPIO_ToggleOutputPin(GPIOD, 12);
		for(uint32_t i=0 ; i < 300000 ; i++ );

	}

	void EXTI9_5_IRQHandler (void){

		GPIO_IRQHandling(5);
		GPIO_ToggleOutputPin(GPIOD, 12);

	}


	return 0;
}
