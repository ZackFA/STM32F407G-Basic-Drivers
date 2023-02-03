#ifndef INC_STM32F407G_USART_DRIVER_H_
#define INC_STM32F407G_USART_DRIVER_H_

#include "stm32f407g.h"


/*** DEFINING STRUCTURE FOR USART CONFIGURATIONS ***/

typedef struct
{

	uint8_t  USART_Mode;
	uint32_t USART_Baud;
	uint8_t	 USART_NoOfStopBits;
	uint8_t  USART_WordLength;
	uint8_t  USART_ParityControl;
	uint8_t  USART_HWFlowControl;


}USART_CFG_t;

/*** DEFINING STRUCTURE FOR USART Handle ***/

typedef struct
{

	USART_RegDef_t	*pUSARTx;
	USART_CFG_t		USART_CFG;


}USART_Handle_t;


/* USART MODE MACROS */

#define USART_MODE_ONLY_TX 	0
#define USART_MODE_ONLY_RX 	1
#define USART_MODE_TXRX  	2

/* USART BAUD MACROS */

#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

/* USART PARITY MACROS */

#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE  0

/* USART WORD LENGTH MACROS */

#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/* USART STOPBITS NUMBER MACROS */

#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/* USART HWFLOW MACROS */

#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/* USART FLAGS */

#define USART_FLAG_TXE 			( 1 << USART_SR_TXE)
#define USART_FLAG_RXNE 		( 1 << USART_SR_RXNE)
#define USART_FLAG_TC 			( 1 << USART_SR_TC)

/* Application states */

#define USART_BUSY_IN_RX 	1
#define USART_BUSY_IN_TX 	2
#define USART_READY 		0


#define USART_EVENT_TX_CMPLT   0
#define	USART_EVENT_RX_CMPLT   1
#define	USART_EVENT_IDLE       2
#define	USART_EVENT_CTS        3
#define	USART_EVENT_PE         4
#define	USART_ERR_FE     	   5
#define	USART_ERR_NE    	   6
#define	USART_ERR_ORE    	   7


/** APIs SUPPORTED BY THIS DRIVER **/

/* Peripheral Clock Setup */

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/* Init & De-init */

void USART_Init(USART_Handle_t *pUSARTHANDLE);
void USART_DeInit(USART_Handle_t *pUSARTHANDLE);

/* Data send & receive */

void USART_SendData(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);

/* IRQ Configuration & ISR Handling */

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHANDLE);

/* OTHER PERIPHERAL CONTROL APIs */

void 	USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void	USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void    USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/* Application Callback */

void USART_AppEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv);


















#endif
