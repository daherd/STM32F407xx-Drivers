/*
 * main_SPI.c
 *
 *  Created on: Feb 14, 2024
 *      Author: daher
 */


// Toggle LED When button is pressed

#include <stdint.h>
#include <string.h>


#include "stm32f407xx_GPIO_driver.h"
#include "stm32f407xx_SPI_driver.h"



int main(void)
{
	//char data[] = "HELLLOOOOO WORLD I AM DAHER DAHER MOFDAH";
	uint8_t data =0xFF;
	//uint8_t *pdata= (uint8_t*)&data;

	GPIO_Handle_t SPI_Pins;
	SPI_Handle_t SPI2_Handle;

	GPIO_PeriClkControl(GPIOB,ENABLE);


	SPI_Pins.pGPIOx = GPIOB;
	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = OSPEED_HIGH;
	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFunMode = AF5;
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = OUT_PP;

	//SCK
	SPI_Pins.GPIO_PinConfig.GPIO_PinNum = 13;
	GPIO_Init(&SPI_Pins);

	//MISO
	SPI_Pins.GPIO_PinConfig.GPIO_PinNum = 14;
	GPIO_Init(&SPI_Pins);

	//NSS
	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	SPI_Pins.GPIO_PinConfig.GPIO_PinNum = 12;
	GPIO_Init(&SPI_Pins);

	SPI_PeriClkControl(SPI2,ENABLE);

	SPI2_Handle.pSPIx = SPI2;
	SPI2_Handle.SPI_PinConfig.SPI_BusConfig = FULL_DUPLEX;
	SPI2_Handle.SPI_PinConfig.SPI_DeviceMode = MASTER;
	SPI2_Handle.SPI_PinConfig.SPI_DFF = DFF_8BITS;
	SPI2_Handle.SPI_PinConfig.SPI_CPOL = 0;
	SPI2_Handle.SPI_PinConfig.SPI_CPHA = 0;
	SPI2_Handle.SPI_PinConfig.SPI_SCLK_Speed = SPI_Speed_CLK_DIV2;
	SPI2_Handle.SPI_PinConfig.SPI_SSM = SOFTWARE;
	SPI2_Handle.SPI_PinConfig.SPI_SSI = 1;

	SPI_Init(&SPI2_Handle);

	GPIO_WriteToOutputPin(GPIOB,12,1); //NSS HIGH

	SPI_DataSend(SPI2,&data,1);


	while(1);



	return 0;
}
