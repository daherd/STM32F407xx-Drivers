/*
 * stm32f407xx_SPI_driver.h
 *
 *  Created on: Feb 14, 2024
 *      Author: daher
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/************************************************************************/

// SPI Pin Configuration
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
	uint8_t SPI_SSI;
	uint8_t SPI_SCLK_Speed;
	uint8_t SPI_SPE;
}SPI_PinConfig_t;



// SPI Handler

typedef struct{
	SPI_RegDif_t 		*pSPIx;			/*<! Base Address of SPIx>*/
	SPI_PinConfig_t 	SPI_PinConfig;
	uint8_t 			*pTXBuffer;		/*<! store Tx buffer Address>*/
	uint8_t 			*pRXBuffer;		/*<! store Rx buffer Address>*/
	uint32_t 			TxLen;
	uint32_t 			RxLen;
	uint8_t 			TxState;
	uint8_t 			RxState;
}SPI_Handle_t;


/************************** API FOR SPI **********************************/

//Init and De-init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDif_t *pSPIx);

// peripheral clock setup
void SPI_PeriClkControl(SPI_RegDif_t *pSPI, uint8_t EN_DI);

// Send Data blocking
void SPI_DataSend (SPI_RegDif_t *pSPI ,uint8_t *pTxBuffer ,uint32_t Len);

// Read Data blocking
void SPI_DataRecieve (SPI_RegDif_t *pSPI ,uint8_t *pRxBuffer ,uint32_t Len);

//Interrupt
//Interrupt and ISR Handling
void SPI_IRQConfig(uint8_t IRQ_Number, uint8_t ENorDI);
void SPI_IRQPriorityConfig(uint8_t IRQ_Number,uint8_t IRQ_Priority);

// Send Data Interrupt
uint8_t SPI_DataSend_IT (SPI_Handle_t *pSPIHandle ,uint8_t *pTxBuffer ,uint32_t Len);

// Read Data Interrupt
uint8_t SPI_DataRecieve_IT (SPI_Handle_t *pSPIHandle ,uint8_t *pRxBuffer ,uint32_t Len);


/************************* SPI DEFINATIONS ******************************/

/** @defgroup "DeviceMode" x
  * @{
  */
#define SLAVE					0
#define MASTER					1

/**
  * @}
  */


/*
 * @SPI_SSM
 */
#define HARDWARE				0
#define SOFTWARE				1


/*
 * @SPI_DFF
 */
#define DFF_8BITS				0
#define DFF_16BITS				1


/*
 * @SPI_BusConfig
 */

#define FULL_DUPLEX				0
#define HALF_DUPLEX				1
#define	SIMPLEX					2

/*
 * @SPI_CPHA
 */
#define CPHA_LOW				0
#define CPHA_HIGH				1

/*
 * @SPI_SPI_CPOL
 */
#define CPOL_LOW				0
#define CPOL_HIGH				1


/*
 * @SPI_SPI_Speed
 */
#define SPI_Speed_CLK_DIV2			0
#define SPI_Speed_CLK_DIV4			1
#define SPI_Speed_CLK_DIV8			2
#define SPI_Speed_CLK_DIV16			3
#define SPI_Speed_CLK_DIV32			4
#define SPI_Speed_CLK_DIV64			5
#define SPI_Speed_CLK_DIV128		6
#define SPI_Speed_CLK_DIV256		7


/*
 * @SPI_SPE
 */
#define SPI_ENABLE					1
#define SPI_DISABLE					0

/*
 * @SPI_SS_ENABLE
 */
#define SPI_SS_ENABLE					1
#define SPI_SS_DISABLE					0


/*
 * @SPI_STATUS
 */
#define SPI_AVAILABLE					0
#define SPI_TX_BUSY						1
#define SPI_RX_BUSY						2

/*
 * @SPI_APPLICATION EVENTS
 */
#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERR				3



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
