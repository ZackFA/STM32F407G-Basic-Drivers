#ifndef INC_STM32F407G_H_
#define INC_STM32F407G_H_
#include <stdint.h>
#include <stddef.h>


#define __vo volatile

/* ARM Cortex Mx Processor NVIC ISERx Register Address */

#define NVIC_ISER0			( (_vo uint32_t*)0xE00E100 )
#define NVIC_ISER1			( (_vo uint32_t*)0xE00E104 )
#define NVIC_ISER2			( (_vo uint32_t*)0xE00E108 )
#define NVIC_ISER3			( (_vo uint32_t*)0xE00E10C )

/* ARM Cortex Mx Processor NVIC ICERx Register Address */

#define NVIC_ICER0			( (_vo uint32_t*)0xE00E180 )
#define NVIC_ICER1			( (_vo uint32_t*)0xE00E184 )
#define NVIC_ICER2			( (_vo uint32_t*)0xE00E188 )
#define NVIC_ICER3			( (_vo uint32_t*)0xE00E18C )

/* ARM Cortex Mx Processor Priority Register Address Calculations */

#define NVIC_PR_BASE_ADDR		( (_vo uint32_t*)0xE00E400 )
#define NO_PR_BITS_IMPLEMENTED 	4

/* base addresses of Flash & SRAM memories */

#define FLASH_BASEADDR		0x0800000U
#define SRAM1_BASEADDR		0x2000000U
#define SRAM2_BASEADDR		0x2001C00U
#define ROM					0x1FFF000U
#define SRAM				SRAM1_BASEADDR

/* AHBx & APBx Bus Peripherals base addresses */

#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIP_BASE		0x50000000U

/* Base addresses of Peripherals that Hang on AHB1 bus */

#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASE + 0x2000)
#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800)

/* Base addresses of Peripherals that Hang on APB1 bus */

#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASE + 0x5000)

/* Base addresses of Peripherals that Hang on APB2 bus */

#define EXT1_BASEADDR		(APB2PERIPH_BASE + 0x3C00)
#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x3800)
#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400)


/*** DEFINING STRUCTURE FOR GPIOx REGISTERS ***/

typedef struct
{
	__vo uint32_t MODER;  			/* GPIO PORT MODE REGISTER, address offset : 0x00 */
	__vo uint32_t OTYPER; 			/* address offset : 0x04 */
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];

}GPIO_RegDef_t;

/*** DEFINING STRUCTURE FOR SPI REGISTERS ***/

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;

/*** DEFINING STRUCTURE FOR I2C REGISTERS ***/

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;

}I2C_RegDef_t;

/*** DEFINING STRUCTURE FOR USART REGISTERS ***/

typedef struct
{

	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;

}USART_RegDef_t;

/*** DEFINING STRUCTURE FOR RCC REGISTERS ***/

typedef struct
{
	__vo uint32_t CR;  			/* RCC REGISTER, address offset : 0x00 */
	__vo uint32_t PLLCFGR; 			/* address offset : 0x04 */
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AH1RSTR;
	__vo uint32_t AH2RSTR;
	__vo uint32_t AH3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;


}RCC_RegDef_t;

/*** DEFINING STRUCTURE FOR EXTI REGISTERS ***/

typedef struct{

	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;



}EXT_RegDef_t;

/*** DEFINING STRUCTURE FOR SYSCFG REGISTERS ***/

typedef struct{

	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t      RESERVED1[1];
	__vo uint32_t CMPCR;
	uint32_t      RESERVED2[2];
	__vo uint32_t CFGR;



}SYSCFG_RegDef_t;

 /*** PERIPHERAL DEFINITIONS ***/

#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASEADDR)

#define RCC					((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI				((EXTI_RegDef_t*)EXT1_BASEADDR)

#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


 /*** CLOCK ENABLE MACROS FOR GPIOx PERIPHERALS ***/

#define GPIOA_PERI_CLK_EN()		 RCC->AHB1ENR |= ( 1 << 0)  /* Left shit to 0th bit position and make it 1 */
#define GPIOB_PERI_CLK_EN()		 RCC->AHB1ENR |= ( 1 << 1)  /* Left shit to 1th bit position and make it 1 */
#define GPIOC_PERI_CLK_EN()		 RCC->AHB1ENR |= ( 1 << 2)  /* Left shit to 2th bit position and make it 1 */
#define GPIOD_PERI_CLK_EN()		 RCC->AHB1ENR |= ( 1 << 3)  /* Left shit to 3th bit position and make it 1 */
#define GPIOE_PERI_CLK_EN()		 RCC->AHB1ENR |= ( 1 << 4)  /* Left shit to 4th bit position and make it 1 */
#define GPIOF_PERI_CLK_EN()		 RCC->AHB1ENR |= ( 1 << 5)  /* Left shit to 5th bit position and make it 1 */
#define GPIOG_PERI_CLK_EN()		 RCC->AHB1ENR |= ( 1 << 6)  /* Left shit to 6th bit position and make it 1 */
#define GPIOH_PERI_CLK_EN()		 RCC->AHB1ENR |= ( 1 << 7)  /* Left shit to 7th bit position and make it 1 */
#define GPIOI_PERI_CLK_EN()		 RCC->AHB1ENR |= ( 1 << 8)  /* Left shit to 8th bit position and make it 1 */

 /*** CLOCK ENABLE MACROS FOR I2Cx PERIPHERALS ***/

#define I2C1_PERI_CLK_EN()		(RCC->APB1ENR |= (1 << 21)) /* Left shit to 21th bit position and make it 1 */
#define I2C2_PERI_CLK_EN()		(RCC->APB1ENR |= (1 << 22)) /* Left shit to 22th bit position and make it 1 */
#define I2C3_PERI_CLK_EN()		(RCC->APB1ENR |= (1 << 23)) /* Left shit to 23th bit position and make it 1 */

 /*** CLOCK ENABLE MACROS FOR SPIx PERIPHERALS ***/

#define SPI1_PERI_CLK_EN()		(RCC->APB2ENR |= (1 << 12)) /* Left shit to 12th bit position and make it 1 */
#define SPI2_PERI_CLK_EN()		(RCC->APB1ENR |= (1 << 14)) /* Left shit to 14th bit position and make it 1 */
#define SPI3_PERI_CLK_EN()		(RCC->APB1ENR |= (1 << 15)) /* Left shit to 15th bit position and make it 1 */
#define SPI4_PERI_CLK_EN()		(RCC->APB2ENR |= (1 << 13)) /* Left shit to 13th bit position and make it 1 */

 /*** CLOCK ENABLE MACROS FOR USARTx PERIPHERALS ***/

#define USART1_PERI_CLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PERI_CLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PERI_CLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PERI_CLK_EN()	    (RCC->APB1ENR |= (1 << 19))
#define UART5_PERI_CLK_EN() 	(RCC->APB1ENR |= (1 << 20))
#define USART6_PERI_CLK_EN()	(RCC->APB1ENR |= (1 << 5))

 /*** CLOCK ENABLE MACROS FOR SYSCFG PERIPHERAL ***/

#define SYSCFG_PERI_CLK_EN()	(RCC->APB2ENR |= (1 << 14))

/*** CLOCK DISABLE MACROS FOR GPIOx PERIPHERAL ***/

#define GPIOA_PERI_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PERI_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PERI_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PERI_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PERI_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PERI_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PERI_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PERI_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PERI_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))

 /*** CLOCK DISABLE MACROS FOR I2Cx PERIPHERALS ***/

#define I2C1_PERI_CLK_DI()		RCC->APB1ENR &= ~(1 << 21)
#define I2C2_PERI_CLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PERI_CLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

 /*** CLOCK DISABLE MACROS FOR SPIx PERIPHERALS ***/

#define SPI1_PERI_CLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PERI_CLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PERI_CLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PERI_CLK_DI()		(RCC->APB2ENR &= ~(1 << 13))

 /*** CLOCK DISABLE MACROS FOR USARTx PERIPHERALS ***/

#define USART1_PERI_CLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PERI_CLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PERI_CLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PERI_CLK_DI()	    (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PERI_CLK_DI() 	(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PERI_CLK_DI()	(RCC->APB1ENR &= ~(1 << 5))

 /*** CLOCK DISABLE MACROS FOR SYSCFG PERIPHERAL ***/

#define SYSCFG_PERI_CLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

 /*** MACROS TO RESET GPIOx PERIPHERALS ***/

#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)

 /** IRQ MACROS **/

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34
#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73

 /*** SOME GENERIC MACROS ***/

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_SET			SET
#define FLAG_RESET			RESET
#define GPIO_BASEADDR_TO_CODE(x)	(  (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
								        (x == GPIOD)?3:\
                                        (x == GPIOE)?4:\
										(x == GPIOF)?5:\
										(x == GPIOG)?6:\
										(x == GPIOH)?7:\
										(x == GPIOH)?8:\) // This macro returns a code( between 0 to 7) for a given GPIO base address(x)

 /*** BIT POSITION DEFINITIONS FOR SPI PERIPHERALS ***/

/*** SPI CR1 REGISTER ***/

#define SPI_CR1_CPHA	0
#define SPI_CR1_CPOL	1
#define SPI_CR1_MSTR	2
#define SPI_CR1_BR		3
#define SPI_CR1_SPE		6
#define SPI_CR1_LSB		7
#define SPI_CR1_SSI		8
#define SPI_CR1_SSM		9
#define SPI_CR1_RXONLY  10
#define SPI_CR1_DFF		11
#define SPI_CR1_CRCNEXT 12
#define SPI_CR1_CRCEN	13
#define SPI_CR1_BIDIOE	14
#define SPI_CR1_BIDIMO	15

/*** SPI CR2 REGISTER ***/

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*** SPI STATUS REGISTER ***/

#define SPI_SR_RXNE		0
#define SPI_SR_TXE		1
#define SPI_SR_CHSIDE	2
#define SPI_SR_UDR		3
#define SPI_SR_CRCERR	4
#define SPI_SR_MODF		5
#define SPI_SR_OVR		6
#define SPI_SR_BSY		7
#define SPI_SR_FRE		8

/*** BIT POSITION DEFINITIONS FOR I2C PERIPHERALS ***/

/*** I2C CR1 REGISTER ***/

#define I2C_CR1_PE			0
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK 		10
#define I2C_CR1_SWRST		15

/*** I2C CR2 REGISTER ***/

#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10

/*** I2C SR1 REGISTER ***/

#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_TIMEOUT		14

/*** I2C SR2 REGISTER ***/

#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_DUALF		7

/*** BIT POSITION DEFINITIONS FOR USART PERIPHERALS ***/

/* USART CR1 REGISTER */

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

/* USART CR2 REGISTER */

#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14

/* USART CR3 REGISTER */

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

/* USART SR REGISTER */

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




#include "stmf407g_GPIO_driver.h"
#include "stm32f407g_SPI_driver.h"
#include "stm32f407g_I2C_driver.h"
#include "stm32f407g_USART_driver.h"
#include "stm32f407g_RCC_driver.h"

#endif /* INC_STM32F407G_H_ */
