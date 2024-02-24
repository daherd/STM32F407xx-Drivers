/*
 * stm32f407xx_I2C_driver.c
 *
 *  Created on: Feb 19, 2024
 *      Author: daher
 */
#include "stm32f407xx.h"

uint32_t AHB_Prescaler [9] = {2,4,8,16,32,64,128,256,512};
uint8_t APB1_Prescaler [4] = {2,4,8,16};

/**
 ******************************************************************************
 * @function			: I2C_GetFlagStatus

 * @brief				: take position of status in register and returns
 * 						  if it's high or not.

 * @param[in]			: Base Address of I2C Port
 * @param[in]			: flag bit position
 * @param[in]			:
 *
 * @return				: Ture or FALSE
 *
 * @note				:

 ******************************************************************************
 */
uint8_t I2C_GetFlagStatus(I2C_RegDif_t *pI2C, uint32_t Flag_Position){
	if(pI2C->SR1 & (1<<Flag_Position)){
		return TRUE;
	}
	else{
		return FALSE;
	}
}

/*
 * Internal function to send Slave Address + R/W bit
 * */

static void I2C_SendSlaveAddr(I2C_RegDif_t *pI2C, uint8_t SlaveAddr, uint8_t R_W){

	SlaveAddr = SlaveAddr << 1; 		// Move 1 bit for R/W
	if (R_W == WRITE){
		SlaveAddr &= ~(1); 				// clear W/R bit -> set Write
	}
	else{
		SlaveAddr |= 1;					// W/R bit -> set Read
	}
	pI2C->DR = SlaveAddr; 				// Send data
}


void I2C_GenerateStopCondition(I2C_RegDif_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}


/*
 * Clear ADDR
 * */
static void ClearADDRFlag(I2C_Handle_t *pI2CHandle )
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << 0))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				ACK_EN_DI(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}


}


/*
 * Enable Disable ACK
 * */
void ACK_EN_DI(I2C_RegDif_t *pI2C ,uint8_t EN_DI){

	if(EN_DI == ENABLE){
		pI2C->CR1 |= (1 << I2C_CR1_ACK);
	}
	else{
		pI2C->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/*
 * Help Function I2C Master Handle TXE Interrupt
 * */

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle )
{

	if(pI2CHandle->TxLen > 0)
	{
		//1. load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTXBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTXBuffer++;

	}

}



/*
 * Help Function I2C Close Receive Data
 * */

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRXBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_PinConfig.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		ACK_EN_DI(pI2CHandle->pI2Cx,ENABLE);
	}

}

/*
 * Help Function I2C Close Send Data
 * */

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTXBuffer = NULL;
	pI2CHandle->TxLen = 0;
}



/*
 * Help Function I2C Master Handle RXNE Interrupt
 * */

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle )
{
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRXBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;

	}


	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		}

			//read DR
			*pI2CHandle->pRXBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->pRXBuffer++;
			pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0 )
	{
		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(pI2CHandle->Sr == DISABLE)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2 . Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
	}
}



/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void I2C_PeripheralControl(I2C_RegDif_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}

}


/**
 ******************************************************************************
 * @function			: I2C_PeriClkControl

 * @brief				: Enable or Disable Clock for the given I2C port

 * @param[in]			: Base Address of I2C Port
 * @param[in]			: Enable or Disable macros
 * @param[in]			:
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */

void I2C_PeriClkControl(I2C_RegDif_t *pI2C, uint8_t EN_DI){

	if (EN_DI == ENABLE){

		if (pI2C == I2C1){
			I2C1_PCLK_EN();
		}
		else if (pI2C == I2C2){
			I2C2_PCLK_EN();
		}
		else if (pI2C == I2C3){
			I2C3_PCLK_EN();
		}
	}
	else{

		if (pI2C == I2C1){
			I2C1_PCLK_DI();
		}
		else if (pI2C == I2C2){
			I2C2_PCLK_DI();
		}
		else if (pI2C == I2C3){
			I2C3_PCLK_DI();
		}
	}
}


/**
 ******************************************************************************
 * @function			: Help function ,Calculate_APB1_Freq

 * @brief				: Calculate to help us set I2C FREQ

 * @param[in]			:
 * @param[in]			:
 * @param[in]			:
 *
 * @return				: APB1_Freq
 *
 * @note				:

 ******************************************************************************
 */

uint32_t Calculate_APB1_Freq(void){

	uint8_t SysCLK;
	uint16_t AHB_P, APB1_P;
	uint32_t SysCLK_Freq = 16000000;
	SysCLK = ((RCC->CFGR >> 2) &0x3 );

	if (SysCLK == 0){
		SysCLK_Freq = 16000000;
	}
	else if (SysCLK == 1){
		SysCLK_Freq = 8000000;
	}

	AHB_P = ((RCC->CFGR >> 4) &0xF );	//AHB Prescaler

	if (AHB_P < 8){

		AHB_P =1;
	}
	else{
		AHB_P = AHB_Prescaler[8 - AHB_P];
	}

	//tempCLK_Freq = tempCLK_Freq >> AHB_P; // Divide by AHB Prescaler

	APB1_P = ((RCC->CFGR >> 10) &0x7 );	//APB1 Prescaler

	if (AHB_P < 8){

		APB1_P =1;
	}
	else{
		APB1_P = APB1_Prescaler[4 - APB1_P];
	}

	//tempCLK_Freq = tempCLK_Freq >> APB1_P; // Divide by APB1 Prescaler

	//tempCLK_Freq = tempCLK_Freq >> 6;		//remove zeros

	return ((SysCLK_Freq / AHB_P )/ APB1_P);

}


/**
 ******************************************************************************
 * @function			: I2C_Init

 * @brief				: Initialise specific I2C (1 ,2 or 3)

 * @param[in]			: I2C_Handle, struct that include all I2C configuration
 * @param[in]			:
 * @param[in]			:
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */

void I2C_Init(I2C_Handle_t *pI2CHandle){

	uint32_t tempReg = 0, temp=0, PCLK, SCLK;
	uint8_t DC=0;

	//REG_CR1
	//****Enable ACK*****
	pI2CHandle->pI2Cx->CR1= (pI2CHandle->I2C_PinConfig.I2C_ACKControl << I2C_CR1_ACK);

	//REG_CR2
	//****config Speed of serial clock****
	//1.speed of bus connected to I2C
	PCLK = Calculate_APB1_Freq();
	temp = PCLK  / 1000000U;
	pI2CHandle->pI2Cx->CR2 = temp & 0x3F;

	//REG_CCR
	//2.speed of bus connected to I2C
	SCLK= pI2CHandle->I2C_PinConfig.I2C_SCLSpeed;
	if (SCLK <= I2C_SCL_SPEED_SM){		//Check Standard or Fast Mode
		// Standard
		tempReg |= (0 << I2C_CCR_FS);  //Set Standard
		temp = ((PCLK / 2) / SCLK);
		tempReg |= (temp & 0xFFF); 		// Set CCR
	}
	else{
		// Fast
		tempReg |= (1 << I2C_CCR_FS);  //Set Fast
		DC = pI2CHandle->I2C_PinConfig.I2C_FMDutyCycle;
		tempReg |= (DC << I2C_CCR_DUTY); //Set D.C.
		if (!DC){
			temp = ((PCLK / 3) / SCLK);
		}
		else{
			temp = ((PCLK / 25) / SCLK);
		}
		tempReg |= (temp & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempReg;

	//REG_OAR1
	//*****Device (Slave) Address*****
	tempReg |= (pI2CHandle->I2C_PinConfig.I2C_DeviceAddress << 1);
	tempReg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempReg;

	//****Config TRise******
	if (SCLK <= I2C_SCL_SPEED_SM){		//Check Standard or Fast Mode
		//Standard
		tempReg = (PCLK / 1000000U) + 1;
	}
	else{
		//Fast
		tempReg = ((PCLK * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempReg & 0x3F);

}


/**
 ******************************************************************************
 * @function			: I2C_MasterSendData

 * @brief				: Send data from Master to Slave using I2C protocol

 * pI2CHandle[in]		: I2C_Handle, struct that include all I2C configuration
 * @pTxBuffer[in]		: pointer to send data address
 * @Len[in]				: length of data send in bytes
 * @SalveAddr[in]		: Address of the slave
 * @Reapeat_start[in]	: Reapeat_start (ENABLE OR DISABLE)
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SalveAddr, uint8_t Repeat_Start){

	//1.Start (Master send start bit to slave)
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
	//wait until Start bit generated (SB became High)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_SB));

	//2. Send Slave Address with W/R bit (8 bits)
	I2C_SendSlaveAddr(pI2CHandle->pI2Cx, SalveAddr, WRITE);
	// wait until finish sending slave address
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_ADDR));

	//Clear ADDR bit
	ClearADDRFlag(pI2CHandle);

	//3. Start Send data until  Len = 0
	while(Len != 0){
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TXE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//4. check if TXE is Empty and byte transfer is finished BTF to end the program
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_BTF));

	if (Repeat_Start == DISABLE){
		//STOP, finish sending data
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
	}
}


/**
 ******************************************************************************
 * @function			: I2C_MasterReceiveData

 * @brief				: Receive data from Slave using I2C protocol

 * pI2CHandle[in]		: I2C_Handle, struct that include all I2C configuration
 * @pTxBuffer[in]		: pointer to send data address
 * @Len[in]				: length of data send in bytes
 * @SalveAddr[in]		: Address of the slave
 * @Reapeat_start[in]	: Reapeat_start (ENABLE OR DISABLE)
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */


void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SalveAddr, uint8_t Repeat_Start){

	//1.Start (Master send start bit to slave)
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
	//wait until Start bit generated (SB became High)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_SB));

	//2. Send Slave Address with READ bit (8 bits)
	I2C_SendSlaveAddr(pI2CHandle->pI2Cx, SalveAddr, READ);
	// wait until End of address transmission
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_ADDR));

	//3. Start Receiving Data
	if(Len == 1){
		//Disable ACK
		ACK_EN_DI(pI2CHandle->pI2Cx ,DISABLE);
		//Clear ADDR
		ClearADDRFlag(pI2CHandle);
		//wait until RXNE =1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RXNE));
		//Generate Stop Condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
		// Read data and save it
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}
	else if (Len > 1){
		//Clear ADDR
		ClearADDRFlag(pI2CHandle);
		while (Len > 0){
			//wait until RXNE =1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RXNE));
			if (Len == 2){
				//Disable ACK
				ACK_EN_DI(pI2CHandle->pI2Cx ,DISABLE);
				//Generate Stop Condition
				if (Repeat_Start == DISABLE){
					pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
				}
			}
			// Read data and save it
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
			Len--;
		}

	}

	//Enable ACK
	ACK_EN_DI(pI2CHandle->pI2Cx ,ENABLE);

}


/**
 ******************************************************************************
 * @function			: I2C_IRQConfig

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


void I2C_IRQConfig(uint8_t IRQ_Number, uint8_t ENorDI){
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
 * @function			: I2C_IRQPriorityConfig

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



void I2C_IRQPriorityConfig(uint8_t IRQ_Number,uint8_t IRQ_Priority){
	uint8_t PR_Reg = IRQ_Number / 4;
	uint8_t PR_Reg_Section = IRQ_Number % 4;
	uint8_t Shift_Amount = (8 * PR_Reg_Section) + (8 - NUMBER_OF_IMPLEMNTED_BITS);
	*(NVIC_IPR + (PR_Reg)) |= IRQ_Priority << Shift_Amount;
}





/**
 ******************************************************************************
 * @function			: I2C_MasterReceiveData

 * @brief				: Receive data from Slave using I2C protocol

 * pI2CHandle[in]		: I2C_Handle, struct that include all I2C configuration
 * @pTxBuffer[in]		: pointer to send data address
 * @Len[in]				: length of data send in bytes
 * @SalveAddr[in]		: Address of the slave
 * @Reapeat_start[in]	: Reapeat_start (ENABLE OR DISABLE)
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */


uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Reapeat_start)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_TX_BUSY) && (busystate != I2C_RX_BUSY))
	{
		pI2CHandle->pRXBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_RX_BUSY;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Reapeat_start;

		//Implement code to Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERRN);

	}

	return busystate;
}



/**
 ******************************************************************************
 * @function			: I2C_MasterSendData

 * @brief				: Send data from Master to Slave using I2C protocol

 * pI2CHandle[in]		: I2C_Handle, struct that include all I2C configuration
 * @pTxBuffer[in]		: pointer to send data address
 * @Len[in]				: length of data send in bytes
 * @SalveAddr[in]		: Address of the slave
 * @Reapeat_start[in]	: Reapeat_start (ENABLE OR DISABLE)
 *
 * @return				: None
 *
 * @note				:

 ******************************************************************************
 */

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Reapeat_start)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_TX_BUSY) && (busystate != I2C_RX_BUSY))
	{
		pI2CHandle->pTXBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_TX_BUSY;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Reapeat_start;

		//Implement code to Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERRN);

	}

	return busystate;

}


/**
 ******************************************************************************
 * @function			: I2C_EV_IRQHandling

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


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;

	temp1   = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN) ;
	temp2   = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN) ;

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_SendSlaveAddr(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, WRITE);
		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			I2C_SendSlaveAddr(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, READ);
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		// interrupt is generated because of ADDR event
		ClearADDRFlag(pI2CHandle);
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXE is also set .
			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE) )
			{
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0 )
				{
					//1. generate the STOP condition
					if(pI2CHandle->Sr == DISABLE)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2. reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);

				}
			}

		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX )
		{
			;
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode
	if(temp1 && temp3)
	{
		//STOF flag is set
		//Clear the STOPF ( i.e 1) read SR1 2) Write to CR1 )

		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}


	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//TXE flag is set
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			//slave
			//make sure that the slave is really in transmitter mode
		    if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA))
		    {
		    	I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
		    }
		}
	}

	temp3  = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		//check device mode .
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//The device is master

			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}

		}else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}
}






