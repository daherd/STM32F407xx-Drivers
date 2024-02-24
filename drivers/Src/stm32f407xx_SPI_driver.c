/*
 * stm32f407xx_SPI_driver.c
 *
 *  Created on: Feb 14, 2024
 *      Author: daher
 */

#include <stdint.h>
#include "stm32f407xx_SPI_driver.h"
#include "stm32f407xx_GPIO_driver.h"



/**
 ******************************************************************************
 * @function			: SPI_PeriClkControl

 * @brief				: Enable or Disable Clock for the given SPI port

 * @param[in]			: Base Address of SPI Port
 * @param[in]			: Enable or Disable macros
 * @param[in]			:
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */

void SPI_PeriClkControl(SPI_RegDif_t *pSPI, uint8_t EN_DI){

	if (EN_DI == ENABLE){

		if (pSPI == SPI1){
			SPI1_PCLK_EN();
		}
		else if (pSPI == SPI2){
			SPI2_PCLK_EN();
		}
		else if (pSPI == SPI3){
			SPI3_PCLK_EN();
		}
	}
	else{

		if (pSPI == SPI1){
			SPI1_PCLK_DI();
		}
		else if (pSPI == SPI2){
			SPI2_PCLK_DI();
		}
		else if (pSPI == SPI3){
			SPI3_PCLK_DI();
		}
	}
}


/**
 ******************************************************************************
 * @function			: SPI_Init

 * @brief				: Initialise specific SPI (1 ,2 or 3)

 * @param[in]			: SPI_Handle, struct that include all SPI configuration
 * 						  SPI Mode ,BUS ,DFF ,CPHA ,CPOL , Speed ..
 * @param[in]			:
 * @param[in]			:
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */

void SPI_Init(SPI_Handle_t *pSPIHandle){

	uint32_t tempReg=0;

	//SPI Enable
	//tempReg |= (pSPIHandle->SPI_PinConfig.SPI_SPE << SPI_CR1_SPE);

	//Adjust Clock Phase - bit 1
	tempReg |= (pSPIHandle->SPI_PinConfig.SPI_CPHA << SPI_CR1_CPHA);

	//Adjust Clock Polarity  - bit 0
	tempReg |= (pSPIHandle->SPI_PinConfig.SPI_CPOL << SPI_CR1_CPOL);

	// Adjust Mode (MASTER or SLAVE) - bit 2
	tempReg |= (pSPIHandle->SPI_PinConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	// BR (Baud Rate) Set SCLK - bit 3->5
	tempReg |= (pSPIHandle->SPI_PinConfig.SPI_SCLK_Speed<< SPI_CR1_BR);

	// Adjust SSM (SoftWare or HardWare) - bit 9
	tempReg |= (pSPIHandle->SPI_PinConfig.SPI_SSM<< SPI_CR1_SSM);

	// Adjust SSI - bit 10
	tempReg |= (pSPIHandle->SPI_PinConfig.SPI_SSI << SPI_CR1_SSI);

	// Adjust DFF (8bits or 16bits) - bit 11
	tempReg |= (pSPIHandle->SPI_PinConfig.SPI_DFF<< SPI_CR1_DFF);

	// BUS Configuration

	if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == FULL_DUPLEX){
		tempReg &= ~(0 << SPI_CR1_BIDIMODE);
		tempReg &= ~(0 << SPI_CR1_RXONLY);
	}
	else if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == HALF_DUPLEX){
		tempReg |=(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SIMPLEX){
		tempReg &= ~(0 << SPI_CR1_BIDIMODE);
		tempReg |= (1 << SPI_CR1_RXONLY);
	}


	//Set Register
	pSPIHandle->pSPIx->CR1 = tempReg ;

	// Set CR2 Register
	pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE );


}

/**
 ******************************************************************************
 * @function			: SPI_DeInit

 * @brief				: Reset SPI Registers

 * @param[in]			: Base Address Of Required SPIx To Reset
 * @param[in]			:
 * @param[in]			:
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */

void SPI_DeInit(SPI_RegDif_t *pSPIx){

	if(pSPIx == SPI1){
		SPI1_RESET_REG();
	}
	else if (pSPIx == SPI2){
		SPI2_RESET_REG();
	}
	else if (pSPIx == SPI3){
		SPI3_RESET_REG();
	}
}


/**
 ******************************************************************************
 * @function			: SPI_SPE

 * @brief				: Enable SPI

 * @param[in]			: Base Address Of SPIx
 * @param[in]			: Enable or Disable
 * @param[in]			:
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */

void SPI_SPE (SPI_RegDif_t *pSPI ,uint8_t EN_DI){

	//SPI Enable
	if (EN_DI == ENABLE){
		pSPI->CR1 |= (1 << SPI_CR1_SPE);
		GPIO_WriteToOutputPin(GPIOB,12,0);
	}
	else{
		pSPI->CR1 &= ~(1 << SPI_CR1_SPE);
		GPIO_WriteToOutputPin(GPIOB,12,1);
	}
}



/**
 ******************************************************************************
 * @function			: SPI_DataSend

 * @brief				: send Data from Master

 * @param[in]			: Base Address Of Required SPIx To Reset
 * @param[in]			: Base Address of Data (we want to send)
 * @param[in]			: Lenght of Data in Bytes
 *
 * @return				: None
 *
 * @note				: This is blocking function (Polling)

 ******************************************************************************
 */

void SPI_DataSend (SPI_RegDif_t *pSPI ,uint8_t *pTxBuffer ,uint32_t Len){

	//SPI Enable
	SPI_SPE(pSPI,ENABLE);

	while (Len > 0){

		//check if Tx Buffer empty, empty->continue, busy->wait
		while(!((pSPI->SR >> SPI_SR_TXE)& 0x1));

		if (((pSPI->CR1 >> SPI_CR1_DFF)&0x1) == DFF_16BITS){
			pSPI->DR = *((uint16_t*)pTxBuffer);
			(uint16_t*)pTxBuffer++;
			Len--;
			Len--;
		}
		else {
			pSPI->DR = *pTxBuffer;
			pTxBuffer++;
			Len--;
		}
	}

	//check if SPI busy before disable SPI, not busy->continue, busy->wait
	while((pSPI->SR >> SPI_SR_BSY)& 0x1);
	//SPI Disable
	SPI_SPE(pSPI,DISABLE);
}


/**
 ******************************************************************************
 * @function			: SPI_DataSend

 * @brief				: send Data from Master

 * @param[in]			: Base Address Of Required SPIx To Reset
 * @param[in]			: Base Address of Data (we want to send)
 * @param[in]			: Lenght of Data in Bytes
 *
 * @return				: None
 *
 * @note				: This is blocking function (Polling)

 ******************************************************************************
 */

void SPI_DataRecieve (SPI_RegDif_t *pSPI ,uint8_t *pRxBuffer ,uint32_t Len){

	//SPI Enable
	SPI_SPE(pSPI,ENABLE);

	while (Len > 0){

		//check if Rx Buffer empty, empty->continue, busy->wait
		while((pSPI->SR >> SPI_SR_RXNE)& 0x1);

		if (((pSPI->CR1 >> SPI_CR1_DFF)&0x1) == DFF_16BITS){
			*((uint16_t*)pRxBuffer) = pSPI->DR;
			(uint16_t*)pRxBuffer++;
			Len--;
			Len--;
		}
		else {
			*pRxBuffer = pSPI->DR;
			pRxBuffer++;
			Len--;
		}
	}

	//check if SPI busy before disable SPI, not busy->continue, busy->wait
	while((pSPI->SR >> SPI_SR_BSY)& 0x1);
	//SPI Disable
	SPI_SPE(pSPI,DISABLE);
}


/**
 ******************************************************************************
 * @function			: SPI_IRQConfig

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


void SPI_IRQConfig(uint8_t IRQ_Number, uint8_t ENorDI){
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
 * @function			: SPI_IRQPriorityConfig

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



void SPI_IRQPriorityConfig(uint8_t IRQ_Number,uint8_t IRQ_Priority){
	uint8_t PR_Reg = IRQ_Number / 4;
	uint8_t PR_Reg_Section = IRQ_Number % 4;
	uint8_t Shift_Amount = (8 * PR_Reg_Section) + (8 - NUMBER_OF_IMPLEMNTED_BITS);
	*(NVIC_IPR + (PR_Reg)) |= IRQ_Priority << Shift_Amount;
}


/**
 ******************************************************************************
 * @function			: SPI_DataSend_IT

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

uint8_t SPI_DataSend_IT (SPI_Handle_t *pSPIHandle ,uint8_t *pTxBuffer ,uint32_t Len){

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_TX_BUSY){
		// save Tx address and Len in global variable
		pSPIHandle->pTXBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// mark SPI Status as busy
		pSPIHandle->TxState = SPI_TX_BUSY;

		// Enable TX interrupt TXEIE
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// Data transmission will be handled by ISR code
	}

	return state;
}




/**
 ******************************************************************************
 * @function			: SPI_DataRecieve_IT

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
uint8_t SPI_DataRecieve_IT (SPI_Handle_t *pSPIHandle ,uint8_t *pRxBuffer ,uint32_t Len){

	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_RX_BUSY){
		// save Rx address and Len in global variable
		pSPIHandle->pRXBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// mark SPI Status as busy
		pSPIHandle->RxState = SPI_RX_BUSY;

		// Enable RX interrupt RXNEIE
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// Data transmission will be handled by ISR code
	}

	return state;
}

/**
 ******************************************************************************
 * @function			: SPI_RX and TX Handle

 * @brief				:

 * @param[in]			:
 * @param[in]			:
 * @param[in]			:
 *
 * @return				:
 *
 * @note				:

 ******************************************************************************/

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){

	//This is weak implementation, Application may override this function
}

static void SPI_Handle_IRQ_Tx(SPI_Handle_t *pSPIHandle){

	if (((pSPIHandle->pSPIx->CR1 >> SPI_CR1_DFF)&0x1) == DFF_16BITS){
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTXBuffer);
		(uint16_t*)pSPIHandle->pTXBuffer++;
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
	}
	else {
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTXBuffer);
		pSPIHandle->pTXBuffer++;
		pSPIHandle->TxLen--;
	}

	if (! pSPIHandle->TxLen){
		pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
		pSPIHandle->pTXBuffer =NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_AVAILABLE;
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}
}
static void SPI_Handle_IRQ_Rx(SPI_Handle_t *pSPIHandle){

	if (((pSPIHandle->pSPIx->CR1 >> SPI_CR1_DFF)&0x1) == DFF_16BITS){
		*((uint16_t*)pSPIHandle->pRXBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		pSPIHandle->pRXBuffer--;
		pSPIHandle->pRXBuffer--;
	}
	else {
		*(pSPIHandle->pRXBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->pRXBuffer--;
		pSPIHandle->RxLen--;
	}

	if (! pSPIHandle->RxLen){
		pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
		pSPIHandle->pRXBuffer =NULL;
		pSPIHandle->RxLen = 0;
		pSPIHandle->RxState = SPI_AVAILABLE;
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);

	}
}





/**
 ******************************************************************************
 * @function			: SPI_IRQHandling

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

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){

	uint8_t temp1, temp2;
	// check if TX interrupt
	temp1 = (pSPIHandle->pSPIx->SR >> SPI_SR_TXE) & 1;
	temp2 = (pSPIHandle->pSPIx->CR2 >> SPI_CR2_TXEIE) & 1;

	if (temp1 && temp2){
		SPI_Handle_IRQ_Tx(pSPIHandle);
	}

	// check if RX interrupt
	temp1 = (pSPIHandle->pSPIx->SR >> SPI_SR_RXNE) & 1;
	temp2 = (pSPIHandle->pSPIx->CR2 >> SPI_CR2_RXNEIE) & 1;

	if (temp1 && temp2){
		SPI_Handle_IRQ_Rx(pSPIHandle);
	}

}

