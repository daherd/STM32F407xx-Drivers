/*
 * stm3232f407xx_GPIO_driver.c
 *
 *  Created on: Feb 8, 2024
 *      Author: daher
 */

#include <stdint.h>
#include "stm32f407xx_GPIO_driver.h"

/**
 ******************************************************************************
 * @function			: GPIO_PeriClkControl

 * @brief				: Enable or Disable Clock for the given GPIO port

 * @param[in]			: Base Address of GPIO Port
 * @param[in]			: Enable or Disable macros
 * @param[in]			:
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */

void GPIO_PeriClkControl(GPIO_RegDif_t *pGPIO, uint8_t EN_DI){

	if (EN_DI == ENABLE){
		if (pGPIO == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if (pGPIO == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if (pGPIO == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if (pGPIO == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if (pGPIO == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if (pGPIO == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if (pGPIO == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if (pGPIO == GPIOH){
			GPIOH_PCLK_EN();
		}
		else if (pGPIO == GPIOI){
			GPIOI_PCLK_EN();
		}
	}
	else if (EN_DI == DISABLE){
		if (pGPIO == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if (pGPIO == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if (pGPIO == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if (pGPIO == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if (pGPIO == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if (pGPIO == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if (pGPIO == GPIOG){
			GPIOG_PCLK_DI();
		}
		else if (pGPIO == GPIOH){
			GPIOH_PCLK_DI();
		}
		else if (pGPIO == GPIOI){
			GPIOI_PCLK_DI();
		}
	}

}


/**
 ******************************************************************************
 * @function			: GPIO_Init

 * @brief				: Initialise specific GPIO Pin

 * @param[in]			: GPIO_Handle, struct that include all pin configuration
 * 						  Pin Mode,Output Speed, Output Mode, Push Pull and Alternate Function
 * @param[in]			:
 * @param[in]			:
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp =0;
	//Config Pin Mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= 3 ){
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));			//clearing
		pGPIOHandle->pGPIOx->MODER |= temp;		//setting
		temp=0;
	}
	// Interrupt Mode

	else{
		// 1. set Interrupt mode raise or falling edge
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RE	){
			EXTI->RTSR |= ( 1 <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));  //SET
			EXTI->FTSR &= ~( 1 <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));  //RESET
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FE){
			EXTI->FTSR |= ( 1 <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));  //SET
			EXTI->RTSR &= ~( 1 <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));  //RESET
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FE_RE){
			EXTI->RTSR |= ( 1 <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
			EXTI->FTSR |= ( 1 <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
		}
		// 2. set System Configuration (EXTIx Port)
		uint32_t temp2 =0;
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum / 4; // SYSCFG Register 0 -> 3
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNum % 4; // SYSCFG Group SET 0 ->3
		SYSCFG_PCLK_EN(); //Enable CLK
		SYSCFG->EXTICR[temp] &= ~(0xF << (4*temp2));		//Clear
		SYSCFG->EXTICR[temp] |= (DECODE_GPIO(pGPIOHandle->pGPIOx) << (4*temp2)); //SET

		// 3. remove mask from required EXTI
		EXTI->IMR |= ( 1 <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));

	}
	//Config Pin Speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));			//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;		//setting
	temp=0;

	//Config Pin pull-up/pull-down
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));			//clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;		//setting
	temp=0;

	//Config Pin output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));				//clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;		//setting
	temp=0;

	//Config Pin alternate function
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_AF){
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode;
		if(temp < 8){
			pGPIOHandle->pGPIOx->AF[0] &= ~((0xF << 4*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));		//clearing
			pGPIOHandle->pGPIOx->AF[0] |= (temp << (4*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));		//setting
		}
		else{
			temp = temp - 8;
			pGPIOHandle->pGPIOx->AF[1] &= ~((0xF << 4*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));		//clearing
			pGPIOHandle->pGPIOx->AF[1] |= (temp << (4*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));		//setting
		}
	}

}

/**
 ******************************************************************************
 * @function			: GPIO_DeInit

 * @brief				: Reset GPIO Register

 * @param[in]			: Base Address Of Required GPIO To Reset
 * @param[in]			:
 * @param[in]			:
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */

void GPIO_DeInit(GPIO_RegDif_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_RESET_REG();
	}
	else if (pGPIOx == GPIOB){
		GPIOB_RESET_REG();
	}
	else if (pGPIOx == GPIOC){
		GPIOC_RESET_REG();
	}
	else if (pGPIOx == GPIOD){
		GPIOD_RESET_REG();
	}
	else if (pGPIOx == GPIOE){
		GPIOE_RESET_REG();
	}
	else if (pGPIOx == GPIOF){
		GPIOF_RESET_REG();
	}
	else if (pGPIOx == GPIOG){
		GPIOG_RESET_REG();
	}
	else if (pGPIOx == GPIOH){
		GPIOH_RESET_REG();
	}
	else if (pGPIOx == GPIOI){
		GPIOI_RESET_REG();
	}
}

/**
 ******************************************************************************
 * @function			: GPIO_ReadFromInputPin

 * @brief				: Read Data From The Given GPIOx Pin

 * @param[in]			: Base Address Of GPIOx
 * @param[in]			: Pin Number
 * @param[in]			:
 *
 * @return				: 1 or 0
 *
 * @note				:

 ******************************************************************************
 */



uint8_t GPIO_ReadFromInputPin(GPIO_RegDif_t *pGPIO, uint8_t Pin_Number){
	return ((uint8_t)(((pGPIO->IDR)>> Pin_Number)&0x0001));

}




/**
 ******************************************************************************
 * @function			: GPIO_ReadFromInputPort

 * @brief				: Read Data From The Given GPIOx (16 bit Data)

 * @param[in]			: Base Address Of GPIOx
 * @param[in]			:
 * @param[in]			:
 *
 * @return				: 16 bit unsigned int
 *
 * @note				:

 ******************************************************************************
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDif_t *pGPIO){
	return (uint16_t)(pGPIO->IDR);
}




/**
 ******************************************************************************
 * @function			: GPIO_WriteToOutputPin

 * @brief				: Write Data To The Given GPIOx Pin

 * @param[in]			: Base Address Of GPIOx
 * @param[in]			: Pin Number
 * @param[in]			: Data Write
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */

void GPIO_WriteToOutputPin(GPIO_RegDif_t *pGPIO, uint8_t Pin_Number, uint8_t DataWrite){
	if(DataWrite == GPIO_PIN_SET){
		pGPIO->ODR |= 1 << Pin_Number; //Write 1
	}
	else{
		pGPIO->ODR &= ~(1 << Pin_Number); //Write 0

	}
}

/**
 ******************************************************************************
 * @function			: GPIO_WriteToOutputPort

 * @brief				: Write Data To The Given GPIOx (16 bits)

 * @param[in]			: Base Address Of GPIOx
 * @param[in]			: Data Write
 * @param[in]			:
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */


void GPIO_WriteToOutputPort(GPIO_RegDif_t *pGPIO, uint16_t DataWrite){
	pGPIO->ODR = DataWrite;
}


/**
 ******************************************************************************
 * @function			: GPIO_ToggleOutputPin

 * @brief				: Toggle selected GPIOx Pin

 * @param[in]			: Base Address Of GPIOx
 * @param[in]			: Pin_Number
 * @param[in]			:
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */



void GPIO_ToggleOutputPin(GPIO_RegDif_t *pGPIO, uint8_t Pin_Number){
	pGPIO->ODR ^= (1 << Pin_Number);
}




/**
 ******************************************************************************
 * @function			: GPIO_IRQConfig

 * @brief				:

 * @param[in]			:
 * @param[in]			:
 * @param[in]			:
 *
 * @return				:
 *
 * @note				:

 ******************************************************************************
 */


void GPIO_IRQConfig(uint8_t IRQ_Number, uint8_t ENorDI){
	if (ENorDI == ENABLE){
		if (IRQ_Number <= 31){
			*NVIC_ISER0 |= (1 << IRQ_Number);
		}
		else if ((IRQ_Number > 31) && (IRQ_Number < 64)){
			*NVIC_ISER1 |= (1 << (IRQ_Number%32));
		}
		else if ((IRQ_Number >= 64) && (IRQ_Number < 96)){
			*NVIC_ISER2 |= (1 << (IRQ_Number%32));
		}
	}
	else{
		if (IRQ_Number <= 31){
			*NVIC_ICER0 |= (1 << IRQ_Number);
		}
		else if ((IRQ_Number > 31) && (IRQ_Number < 64)){
			*NVIC_ICER1 |= (1 << (IRQ_Number%32));
		}
		else if ((IRQ_Number >= 64) && (IRQ_Number < 96)){
			*NVIC_ICER2 |= (1 << (IRQ_Number%32));
		}
	}
}



/**
 ******************************************************************************
 * @function			: GPIO_IRQPriorityConfig

 * @brief				:

 * @param[in]			:
 * @param[in]			:
 * @param[in]			:
 *
 * @return				:
 *
 * @note				:

 ******************************************************************************
 */



void GPIO_IRQPriorityConfig(uint8_t IRQ_Number,uint8_t IRQ_Priority){
	uint8_t PR_Reg = IRQ_Number / 4;
	uint8_t PR_Reg_Section = IRQ_Number % 4;
	uint8_t Shift_Amount = (8 * PR_Reg_Section) + (8 - NUMBER_OF_IMPLEMNTED_BITS);
	*(NVIC_IPR + (PR_Reg)) |= IRQ_Priority << Shift_Amount;
}



/**
 ******************************************************************************
 * @function			: GPIO_IRQHandling

 * @brief				:

 * @param[in]			:
 * @param[in]			:
 * @param[in]			:
 *
 * @return				:
 *
 * @note				:

 ******************************************************************************
 */


void GPIO_IRQHandling(uint8_t Pin_Number){

	if (EXTI->PR & (1 << Pin_Number)){
		EXTI->PR |= (1 << Pin_Number);
	}

}













