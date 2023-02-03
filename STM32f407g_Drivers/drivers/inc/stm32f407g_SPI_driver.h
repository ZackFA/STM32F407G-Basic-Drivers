#ifndef INC_STM32F407G_SPI_DRIVER_H_
#define INC_STM32F407G_SPI_DRIVER_H_

#include "stm32f407g.h"
#define __vo volatile


/*** DEFINING STRUCTURE FOR SPI CONFIGURATIONS ***/

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusCFG;
	uint8_t SPI_SClkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPO_SSM;


}SPI_CFG_t; // Defined all the configurable items for SPI Peripheral

/*** DEFINING STRUCTURE FOR SPI HANDLE ***/

typedef struct
{
	SPI_RegDef_t	*pSPIx; /* Holds the base address of SPIx peripheral */
	SPI_CFG_t		SPICFG;
	uint8_t			*pTxBuffer; /* Stores the application Tx buffer address */
	uint8_t			*pRxBuffer; /* Stores the application Rx buffer address */
	uint32_t		TxLen; /* Stores the Tx Length */
	uint32_t		RxLen; /* Stores the Rx Lengh */
	uint8_t			TxState; /* Stores the Tx state */
	uint8_t			RxState; /* Stores the Rx state */

}SPI_Handle_t;

/* SPI DEVICE MODE MACROS */

#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/* SPI BUS CONFIG MACROS */

#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/* SPI CLOCK SPEED MACROS - REFER TO REFERENCE MANUAL FOR BAUD RATE */

#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define SPI_SCLK_SPEED_DIV16	3
#define SPI_SCLK_SPEED_DIV32	4
#define SPI_SCLK_SPEED_DIV64	5
#define SPI_SCLK_SPEED_DIV128	6
#define SPI_SCLK_SPEED_DIV256	7

/* SPI DFF MACROS */
// Data frame format. By default, 8 bit data frame format is selected for TX/RX

#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

/* SPI CPOL MACROS */
// Clock polarity. 0 By default

#define SPI_CPOL_HIGH		1
#define SPI_CPOL_LOW		0

/* SPI CPHA MACROS */
// Clock Phase. By default first clock transition is the first data capture edge

#define SPI_CPHA_HIGH		1
#define SPI_CPHA_LOW		0

/* SPI SSM MACROS */
// By default SSM is disabled.

#define SPI_SSM_EN		1
#define SPI_SSM_DI		0

/* SPI RELATED STATUS FLAGS MACROS */

#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)

/* SPI APPLICATION STATE MACROS */

#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

/* SPI APPLICATION EVENTS MACROS */

#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRR_ERR	4

/** APIs SUPPORTED BY THIS DRIVER **/

/** Peripheral Clock Setup **/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


/** Init & De-init **/

void SPI_Init(SPI_Handle_t *pSPIHANDLE);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/** Flag **/

uint8_t SPI_GetFlagStat(SPI_RegDef_t *pSPIx, uint32_t Flag);

/* Data send & receive */

void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length); /* Second element is a pointer to the data, in order to store it. The last is the size of the data and it is always in 32 bits */
void SPI_Recieve(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length);
uint8_t SPI_SendINT(SPI_Handle_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length); /* Second element is a pointer to the data, in order to store it. The last is the size of the data and it is always in 32 bits */
uint8_t SPI_RecieveINT(SPI_Handle_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length);


/* IRQ Configuration & ISR Handling */

void SPI_IRQInterrupConfig(uint8_t IRQNum, uint8_t IRQPriority, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/* OTHER PERIPHERAL CONTROL APIs */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


/* APPLICATION CALL BACK API */

void SPI_AppEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif





























