/*
 * I2C_MASTER_SEND.c
 *
 *  Created on: Feb 20, 2024
 *      Author: daher
 */


#include "stm32f407xx.h"


int main(void){

	uint8_t data[] = "testing I2C communication";

	GPIO_Handle_t GPIOB_I2C1;

	GPIO_PeriClkControl(GPIOB, ENABLE);

	GPIOB_I2C1.pGPIOx = GPIOB;

	GPIOB_I2C1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_AF;
	GPIOB_I2C1.GPIO_PinConfig.GPIO_PinOPType = OUT_OD;
	GPIOB_I2C1.GPIO_PinConfig.GPIO_PinPuPdControl = PUPD_UP;
	GPIOB_I2C1.GPIO_PinConfig.GPIO_PinSpeed = OSPEED_HIGH;
	GPIOB_I2C1.GPIO_PinConfig.GPIO_PinAltFunMode = AF4;



	//Enable SCL
	GPIOB_I2C1.GPIO_PinConfig.GPIO_PinNum = 6;
	GPIO_Init(&GPIOB_I2C1);

	//Enable SDA
	GPIOB_I2C1.GPIO_PinConfig.GPIO_PinNum = 7;
	GPIO_Init(&GPIOB_I2C1);


	I2C_Handle_t I2C1_Handle;

	I2C_PeriClkControl(I2C1, ENABLE);

	I2C1_Handle.pI2Cx = I2C1;
	I2C1_Handle.I2C_PinConfig.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1_Handle.I2C_PinConfig.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C1_Handle.I2C_PinConfig.I2C_DeviceAddress = 0x68;
	I2C_Init(&I2C1_Handle);

	while (1){
		for(uint32_t i=0 ; i < 600000 ; i++ );
		I2C_MasterSendData(&I2C1_Handle,data,strlen((char*)data),0x68 ,DISABLE);
	}
}
