/*
 * stm32f407xx_GPIO_driver.h
 *
 *  Created on: Feb 8, 2024
 *      Author: daher
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

// GPIO Pin Configuration
typedef struct{
	uint8_t GPIO_PinNum;
	uint8_t GPIO_PinMode;		/*!< @ref mode */
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

// GPIO Handler

typedef struct{
	GPIO_RegDif_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/************************* API Supporter by this Driver ******************************/

//Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDif_t *pGPIOx);

// peripheral clock setup
void GPIO_PeriClkControl(GPIO_RegDif_t *pGPIO, uint8_t EN_DI);

// Read and Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDif_t *pGPIO, uint8_t Pin_Number);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDif_t *pGPIO);
void GPIO_WriteToOutputPin(GPIO_RegDif_t *pGPIO, uint8_t Pin_Number, uint8_t DataWrite);
void GPIO_WriteToOutputPort(GPIO_RegDif_t *pGPIO, uint16_t DataWrite);
void GPIO_ToggleOutputPin(GPIO_RegDif_t *pGPIO, uint8_t Pin_Number);

//Interrupt and ISR Handling
void GPIO_IRQConfig(uint8_t IRQ_Number, uint8_t ENorDI);
void GPIO_IRQPriorityConfig(uint8_t IRQ_Number,uint8_t IRQ_Priority);
void GPIO_IRQHandling(uint8_t IRQ_Number);

//********************GPIO PIN CONFIGURE**************************

/** @defgroup mode
  * @{
  */

//GPIO port mode register
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_AF			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_RE			4	//Interrupt Rising Edge
#define GPIO_MODE_IT_FE			5	//Interrupt Falling Edge
#define GPIO_MODE_IT_FE_RE		6	//Interrupt Rising & Falling Edge
/**
  * @}
  */

//GPIO port output type register
#define OUT_PP				0		//PUSH PULL
#define OUT_OD				1		//OPEN DRAIN

//GPIO port output speed register
#define OSPEED_LOW			0
#define OSPEED_MED			1
#define OSPEED_HIGH			2
#define OSPEED_VHIGH		3

//GPIO port pull-up/pull-down register
#define PUPD_NO				0		//NO PULL UP AND PULL DOWN
#define PUPD_UP				1		//ONLY PULL UP
#define PUPD_DOWN			2		//ONLY PULL DOWN

//GPIO alternate function
#define AF0					0
#define AF1					1
#define AF2					2
#define AF3					3
#define AF4					4
#define AF5					5
#define AF6					6
#define AF7					7
#define AF8					8
#define AF9					9
#define AF10				10
#define AF11				11
#define AF12				12
#define AF13				13
#define AF14				14
#define AF15				15


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
