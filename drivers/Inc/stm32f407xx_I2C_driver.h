/*
 * stm32f407xx_I2C_driver.h
 *
 *  Created on: Feb 19, 2024
 *      Author: daher
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/************************************************************************/

// I2C Pin Configuration
typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_PinConfig_t;



// I2C Handler

typedef struct{
	I2C_RegDif_t 		*pI2Cx;			/*<! Base Address of I2Cx>*/
	I2C_PinConfig_t 	I2C_PinConfig;
	uint8_t 			*pTXBuffer;		/*<! store Tx buffer Address>*/
	uint8_t 			*pRXBuffer;		/*<! store Rx buffer Address>*/
	uint32_t 			TxLen;
	uint32_t 			RxLen;
	uint8_t 			TxRxState;
	uint8_t 			DevAddr;		/*<! Device (Salve) Address>*/
	uint32_t 			RxSize;
	uint8_t 			Sr;				/*<! Repeated Start>*/

}I2C_Handle_t;



/************************* I2C DEFINATIONS ******************************/


/*
 * I2C application states
 */
#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2

/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9

/*
 * @I2C_SCL_Mode
 */
#define I2C_Standard			0
#define I2C_Fast				1


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		100000		//Standard Mode SCL up to 100kHz
#define I2C_SCL_SPEED_FM4K		400000		//Fast Mode SCL up to 400kHz
#define I2C_SCL_SPEED_FM2K		200000		//Fast Mode SCL up to 200kHz


/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0		//Fast Mode Duty Cycle 50%
#define I2C_FM_DUTY_9_16		1		//Fast Mode Duty Cycle 9/16


/*
 * Read / Write
 */
#define WRITE					0
#define READ					1



/*
 * @I2C_STATUS
 */
#define READY							0
#define I2C_TX_BUSY						1
#define I2C_RX_BUSY						2

/************************** API FOR I2C **********************************/

//Init and De-init
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDif_t *pI2Cx);

// peripheral clock setup
void I2C_PeriClkControl(I2C_RegDif_t *pI2C, uint8_t EN_DI);

//Master Send Data Polling
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SalveAddr , uint8_t Repeat_Start);

//Master Receive Data Polling
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SalveAddr, uint8_t Repeat_Start);

//Master Send Data Interrupt
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SalveAddr , uint8_t Repeat_Start);

//Master Receive Data Interrupt
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SalveAddr, uint8_t Repeat_Start);

//Interrupt and ISR Handling
void I2C_IRQConfig(uint8_t IRQ_Number, uint8_t ENorDI);
void I2C_IRQPriorityConfig(uint8_t IRQ_Number,uint8_t IRQ_Priority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
