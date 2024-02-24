/*
 * stm32f407xx.h
 *
 *  Created on: Feb 7, 2024
 *      Author: daher
 */

#include<stdint.h>
#include<stddef.h>
#include <string.h>

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

//***************************** ARM Cortex-M4 Memory Address *******************************************

// NVIC Enable Interrupt
#define NVIC_ISER0 			(uint32_t*)0xE000E100
#define NVIC_ISER1 			(uint32_t*)0xE000E104
#define NVIC_ISER2 			(uint32_t*)0xE000E108
#define NVIC_ISER3 			(uint32_t*)0xE000E10C

// NVIC Clear Interrupt
#define NVIC_ICER0 			(uint32_t*)0XE000E180
#define NVIC_ICER1 			(uint32_t*)0XE000E184
#define NVIC_ICER2 			(uint32_t*)0XE000E188
#define NVIC_ICER3 			(uint32_t*)0XE000E18C

// NVIC Priority Register Base Address
#define NVIC_IPR 			(uint32_t*)0xE000E400

//*******************************************************************************************************

// Memory Address
#define FLASH_BASE_ADD		0x08000000U
#define ROM					0x1FFF0000U
#define SRAM1_BASE_ADD		0x20000000U
#define SRAM2_BASE_ADD		0x2001C000U
#define SRAM				SRAM1_BASE_ADD

//AHB_APB Peripheral Address
#define PERI_BASE		0x40000000U
#define APB1_BASE		PERI_BASE
#define APB2_BASE		0x40010000U
#define AHB1_BASE		0x40020000U
#define AHB2_BASE		0x50000000U
#define AHB3_BASE		0x60000000U

//       peripherals in AHB1

//GPIOx Address
#define GPIOA_BASE		(AHB1_BASE + 0x0000)
#define GPIOB_BASE		(AHB1_BASE + 0x0400)
#define GPIOC_BASE		(AHB1_BASE + 0x0800)
#define GPIOD_BASE		(AHB1_BASE + 0x0C00)
#define GPIOE_BASE		(AHB1_BASE + 0x1000)
#define GPIOF_BASE		(AHB1_BASE + 0x1400)
#define GPIOG_BASE		(AHB1_BASE + 0x1800)
#define GPIOH_BASE		(AHB1_BASE + 0x1C00)
#define GPIOI_BASE		(AHB1_BASE + 0x2000)

//RCC Address
#define RCC_BASE		(AHB1_BASE + 0x3800)






//       peripherals in APB1

//SPI Address
#define SPI2_BASE		(APB1_BASE + 0x3800)
#define SPI3_BASE		(APB1_BASE + 0x3C00)

//UART Address
#define USART2_BASE		(APB1_BASE + 0x4400)
#define USART3_BASE		(APB1_BASE + 0x4800)
#define UART4_BASE		(APB1_BASE + 0x4C00)
#define UART5_BASE		(APB1_BASE + 0x5000)

//I2C Address
#define I2C1_BASE		(APB1_BASE + 0x5400)
#define I2C2_BASE		(APB1_BASE + 0x5800)
#define I2C3_BASE		(APB1_BASE + 0x5C00)




//       peripherals in APB2

//SPI Address
#define SPI1_BASE		(APB2_BASE + 0x3000)

//UART Address
#define USART1_BASE		(APB2_BASE + 0x1400)
#define USART6_BASE		(APB2_BASE + 0x1000)

//EXTI Address
#define EXTI_BASE		(APB2_BASE + 0x3C00)

//SYSCFG Address
#define SYSCFG_BASE		(APB2_BASE + 0x3800)

//---------------------------------------------------------------------------------

//GPIO Registers Definition

typedef struct{
	uint32_t MODER;			//Offset 0x00	 GPIO port mode register
	uint32_t OTYPER;		//Offset 0x04	 GPIO port output type register
	uint32_t OSPEEDR;		//Offset 0x08	 GPIO port output speed register
	uint32_t PUPDR;			//Offset 0x0C	 GPIO port pull-up/pull-down register
	uint32_t IDR;			//Offset 0x10	 GPIO port input data register
	uint32_t ODR;			//Offset 0x14	 GPIO port output data register
	uint32_t BSRR;			//Offset 0x18	 GPIO port bit set/reset register
	uint32_t LCKR;			//Offset 0x1C	 GPIO port configuration lock register
	uint32_t AF[2];			//Offset 0x20	 GPIO alternate function AF[1] low register, AF[2] high register
}GPIO_RegDif_t;

//---------------------------------------------------------------------------------

//EXTI Registers Definition

typedef struct{
	uint32_t IMR;				//Offset 0x00	 Interrupt mask register
	uint32_t EMR;				//Offset 0x04	 Event mask register
	uint32_t RTSR;				//Offset 0x08	 Rising trigger selection register
	uint32_t FTSR;				//Offset 0x0C	 Falling trigger selection register
	uint32_t SWIER;				//Offset 0x10	 Software interrupt event register
	uint32_t PR;				//Offset 0x14	 Pending register
}EXTI_RegDif_t;

//---------------------------------------------------------------------------------


//GPIO Registers Definition

typedef struct{
	uint32_t MEMRMP;			//Offset 0x00	 SYSCFG memory remap register
	uint32_t PMC;				//Offset 0x04	 SYSCFG peripheral mode configuration register
	uint32_t EXTICR[4];			//Offset 0x08	 SYSCFG external interrupt configuration 4 registers
}SYSCFG_RegDif_t;



//---------------------------------------------------------------------------------

//SPI Registers Definition

typedef struct{
	uint32_t CR1;				//Offset 0x00	 SPI control register 1
	uint32_t CR2;				//Offset 0x04	 SPI control register 1
	uint32_t SR;				//Offset 0x08	 SPI status register
	uint32_t DR;				//Offset 0x0C	 SPI data register
	uint32_t CRCPR;				//Offset 0x10	 SPI CRC polynomial register
	uint32_t RXCRCR;			//Offset 0x14	 SPI RX CRC register
	uint32_t TXCRCR;			//Offset 0x18	 SPI TX CRC register
	uint32_t I2SCFGR;			//Offset 0x1C	 SPI_I2S configuration register
	uint32_t I2SPR;				//Offset 0x20	 SPI_I2S prescaler register
}SPI_RegDif_t;



//---------------------------------------------------------------------------------

//I2C Registers Definition

typedef struct{
	uint32_t CR1;				//Offset 0x00	 I2C control register 1
	uint32_t CR2;				//Offset 0x04	 I2C control register 1
	uint32_t OAR1;				//Offset 0x08	 I2C Own address register 1
	uint32_t OAR2;				//Offset 0x0C	 I2C Own address register 2
	uint32_t DR;				//Offset 0x10	 I2C Data register
	uint32_t SR1;				//Offset 0x14	 I2C Status register 1
	uint32_t SR2;				//Offset 0x18	 I2C Status register 2
	uint32_t CCR;				//Offset 0x1C	 I2C Clock control register
	uint32_t TRISE;				//Offset 0x20	 I2C TRISE register
	uint32_t FLTR;				//Offset 0x24	 I2C FLTR register
}I2C_RegDif_t;


//---------------------------------------------------------------------------------


//USART Registers Definition

typedef struct{
	uint32_t SR;				//Offset 0x00	 Status register
	uint32_t DR;				//Offset 0x04	 Data register
	uint32_t BRR;				//Offset 0x08	 Baud rate register
	uint32_t CR1;				//Offset 0x0C	 Control register 1
	uint32_t CR2;				//Offset 0x10	 Control register 2
	uint32_t CR3;				//Offset 0x14	 Control register 3
	uint32_t GTPR;				//Offset 0x18	 Guard time and prescaler register
}USART_RegDif_t;


//---------------------------------------------------------------------------------


//RCC Registers Definition

typedef struct{
	uint32_t CR;					//Offset 0x00
	uint32_t RCC_PLLCFGR;			//Offset 0x04
	uint32_t CFGR;					//Offset 0x08
	uint32_t CIR;					//Offset 0x0C
	uint32_t AHB1RSTR;				//Offset 0x10
	uint32_t AHB2RSTR;				//Offset 0x14
	uint32_t AHB3RSTR;				//Offset 0x18
	uint32_t Reserved0;				//Offset 0x1C
	uint32_t APB1RSTR;				//Offset 0x20
	uint32_t APB2RSTR;				//Offset 0x24
	uint32_t Reserved1;				//Offset 0x28
	uint32_t Reserved2;				//Offset 0x2C
	uint32_t AHB1ENR;				//Offset 0x30
	uint32_t AHB2ENR;				//Offset 0x34
	uint32_t AHB3ENR;				//Offset 0x38
	uint32_t Reserved3;				//Offset 0x3C
	uint32_t APB1ENR;				//Offset 0x40
	uint32_t APB2ENR;				//Offset 0x44
	uint32_t Reserved4;				//Offset 0x48
	uint32_t Reserved5;				//Offset 0x4C
	uint32_t AHB1LPENR;				//Offset 0x50
	uint32_t AHB2LPENR;				//Offset 0x54
	uint32_t AHB3LPENR;				//Offset 0x58
	uint32_t Reserved7;				//Offset 0x5C
	uint32_t APB1LPENR;				//Offset 0x60
	uint32_t APB2LPENR;				//Offset 0x64
	uint32_t Reserved8;				//Offset 0x68
	uint32_t Reserved9;				//Offset 0x6C
	uint32_t BDCR;					//Offset 0x70
	uint32_t CSR;					//Offset 0x74
	uint32_t Reserved10;			//Offset 0x78
	uint32_t Reserved11;			//Offset 0x7C
	uint32_t SSCGR;					//Offset 0x80
	uint32_t PLLI2SCFGR;			//Offset 0x84
	uint32_t PLLSAICFGR;			//Offset 0x88
	uint32_t DCKCFGR;				//Offset 0x8C


}RCC_RegDif_t;

/*********************Peripheral definitions******************/


#define GPIOA		((GPIO_RegDif_t*)GPIOA_BASE)
#define GPIOB		((GPIO_RegDif_t*)GPIOB_BASE)
#define GPIOC		((GPIO_RegDif_t*)GPIOC_BASE)
#define GPIOD		((GPIO_RegDif_t*)GPIOD_BASE)
#define GPIOE		((GPIO_RegDif_t*)GPIOE_BASE)
#define GPIOF		((GPIO_RegDif_t*)GPIOF_BASE)
#define GPIOG		((GPIO_RegDif_t*)GPIOG_BASE)
#define GPIOH		((GPIO_RegDif_t*)GPIOH_BASE)
#define GPIOI		((GPIO_RegDif_t*)GPIOI_BASE)

#define RCC			((RCC_RegDif_t*)RCC_BASE)

#define EXTI		((EXTI_RegDif_t*)EXTI_BASE)

#define SYSCFG		((SYSCFG_RegDif_t*)SYSCFG_BASE)

#define SPI1		((SPI_RegDif_t*)SPI1_BASE)
#define SPI2		((SPI_RegDif_t*)SPI2_BASE)
#define SPI3		((SPI_RegDif_t*)SPI3_BASE)

#define I2C1		((I2C_RegDif_t*)I2C1_BASE)
#define I2C2		((I2C_RegDif_t*)I2C2_BASE)
#define I2C3		((I2C_RegDif_t*)I2C3_BASE)

#define USART1		((USART_RegDif_t*)USART1_BASE)
#define USART2		((USART_RegDif_t*)USART2_BASE)
#define USART3		((USART_RegDif_t*)USART3_BASE)
#define UART4		((USART_RegDif_t*)UART4_BASE)
#define UART5		((USART_RegDif_t*)UART5_BASE)
#define USART6		((USART_RegDif_t*)USART6_BASE)

/*********************Clock Enable Function******************/

//GPIOx CLK Enable
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |=(1<<0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |=(1<<1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |=(1<<2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |=(1<<3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |=(1<<4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |=(1<<5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |=(1<<6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |=(1<<7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |=(1<<8))

//SYSCFG CLK Enable
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |=(1<<14))

//SPI CLK Enable
#define SPI1_PCLK_EN()		(RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |=(1<<14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |=(1<<15))

//I2C CLK Enable
#define I2C1_PCLK_EN()		(RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |=(1<<23))

//UART CLK Enable
#define USART1_PCLK_EN()		(RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |=(1<<17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |=(1<<18))
#define UART4_PCLK_EN()			(RCC->APB1ENR |=(1<<19))
#define UART5_PCLK_EN()			(RCC->APB1ENR |=(1<<20))
#define USART6_PCLK_EN()		(RCC->APB2ENR |=(1<<5))


//Clock Disable Function

//GPIOx CLK Disable
#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &=~(1<<0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &=~(1<<1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &=~(1<<2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &=~(1<<3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &=~(1<<4))
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &=~(1<<5))
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &=~(1<<6))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &=~(1<<7))
#define GPIOI_PCLK_DI()			(RCC->AHB1ENR &=~(1<<8))

//SPI CLK Disable
#define SPI1_PCLK_DI()			(RCC->APB2ENR &=~(1<<12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &=~(1<<14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &=~(1<<15))

//I2C CLK Disable
#define I2C1_PCLK_DI()			(RCC->APB1ENR &=~(1<<21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR &=~(1<<22))
#define I2C3_PCLK_DI()			(RCC->APB1ENR &=~(1<<23))

//UART CLK Disable
#define USART1_PCLK_DI()		(RCC->APB2ENR &=~(1<<4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &=~(1<<17))
#define USART3_PCLK_DI()		(RCC->APB1ENR &=~(1<<18))
#define UART4_PCLK_DI()			(RCC->APB1ENR &=~(1<<19))
#define UART5_PCLK_DI()			(RCC->APB1ENR &=~(1<<20))
#define USART6_PCLK_DI()		(RCC->APB2ENR &=~(1<<5))

//General Definitions
#define ENABLE		 			1
#define DISABLE		 			0
#define TRUE		 			1
#define FALSE		 			0
#define SET			 			ENABLE
#define RESET		 			DISABLE
#define GPIO_PIN_SET			ENABLE
#define GPIO_PIN_RESET			DISABLE

//GPIO REGISTER RESET
#define GPIOA_RESET_REG()		do{(RCC->AHB1RSTR |=(1<<0)); RCC->AHB1RSTR &=~(1<<0);}while(0)
#define GPIOB_RESET_REG()		do{(RCC->AHB1RSTR |=(1<<1)); RCC->AHB1RSTR &=~(1<<1);}while(0)
#define GPIOC_RESET_REG()		do{(RCC->AHB1RSTR |=(1<<2)); RCC->AHB1RSTR &=~(1<<2);}while(0)
#define GPIOD_RESET_REG()		do{(RCC->AHB1RSTR |=(1<<3)); RCC->AHB1RSTR &=~(1<<3);}while(0)
#define GPIOE_RESET_REG()		do{(RCC->AHB1RSTR |=(1<<4)); RCC->AHB1RSTR &=~(1<<4);}while(0)
#define GPIOF_RESET_REG()		do{(RCC->AHB1RSTR |=(1<<5)); RCC->AHB1RSTR &=~(1<<5);}while(0)
#define GPIOG_RESET_REG()		do{(RCC->AHB1RSTR |=(1<<6)); RCC->AHB1RSTR &=~(1<<6);}while(0)
#define GPIOH_RESET_REG()		do{(RCC->AHB1RSTR |=(1<<7)); RCC->AHB1RSTR &=~(1<<7);}while(0)
#define GPIOI_RESET_REG()		do{(RCC->AHB1RSTR |=(1<<8)); RCC->AHB1RSTR &=~(1<<8);}while(0)

//SPI REGISTER RESET
#define SPI2_RESET_REG()		do{(RCC->APB1RSTR |=(1<<14)); RCC->APB1RSTR &=~(1<<14);}while(0)
#define SPI3_RESET_REG()		do{(RCC->APB1RSTR |=(1<<15)); RCC->APB1RSTR &=~(1<<15);}while(0)
#define SPI1_RESET_REG()		do{(RCC->APB2RSTR |=(1<<12)); RCC->APB2RSTR &=~(1<<12);}while(0)


//SYSCNF EXTI
#define DECODE_GPIO(X)			((X == GPIOA)?0 :\
								(X == GPIOB)?1 :\
								(X == GPIOC)?2 :\
								(X == GPIOD)?3 :\
								(X == GPIOE)?4 :\
								(X == GPIOF)?5 :\
								(X == GPIOG)?6 :\
								(X == GPIOH)?7 :\
								(X == GPIOI)?8 :0)

//****************** IRQ Vector Table Priority **********************
#define	IRQ_EXTI0				6
#define	IRQ_EXTI1				7
#define	IRQ_EXTI2				8
#define	IRQ_EXTI3				9
#define	IRQ_EXTI4				10
#define	IRQ_EXTI9_5				23
#define	IRQ_EXTI15_10			40

#define	IRQ_SPI1				35
#define	IRQ_SPI2				36
#define	IRQ_SPI3				51

#define IRQ_I2C1_EV     		31
#define IRQ_I2C1_ER     		32
#define IRQ_I2C2_EV     		33
#define IRQ_I2C2_ER     		34

// Number of used bits in ARM priority register (we used 4 bits out of 8 in STM32F4)
# define NUMBER_OF_IMPLEMNTED_BITS 		4


//************* Bit Position Definition of SPI ********************
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7


#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8


//************* Bit Position Definition of I2C ********************

#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15

#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERRN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12

#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15

#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIOMOUT			14

#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY 			1
#define I2C_SR2_TRA 			2
#define I2C_SR2_GENCALL 		4
#define I2C_SR2_DUALF 			7

//************* Bit Position Definition of USART ********************


#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15


#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11


#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9



#include "stm32f407xx_SPI_driver.h"
#include "stm32f407xx_GPIO_driver.h"
#include "stm32f407xx_I2C_driver.h"
#include "stm32f407xx_USART_driver.h"


#endif /* INC_STM32F407XX_H_ */
