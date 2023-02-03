#ifndef INC_STM32F407G_I2C_DRIVER_H_
#define INC_STM32F407G_I2C_DRIVER_H_

#include "stm32f407g.h"


/*** DEFINING STRUCTURE FOR I2C CONFIGURATIONS ***/

typedef struct
{

	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t	 I2C_ACKControl;
	uint8_t  I2C_FMDutyCycle;


}I2C_CFG_t;

/*** DEFINING STRUCTURE FOR I2C Handle ***/

typedef struct
{

	I2C_RegDef_t    *pI2Cx;
	I2C_CFG_t       I2C_CFG;
	uint8_t			*pTxBuffer; /* Stores the application Tx buffer address */
	uint8_t			*pRxBuffer; /* Stores the application Rx buffer address */
	uint32_t		TxLen; /* Stores the Tx Length */
	uint32_t		RxLen; /* Stores the Rx Lengh */
	uint8_t			TxRxState; /* Stores the Communication state */
	uint8_t			DeviceAddress; /* Stores the slave/device address */
	uint32_t		RxSize; /* Stores the Rx size */
	uint8_t			Sr; /* stores repeated start value */

}I2C_Handle_t;


/* I2C SCL MACROS */

#define I2C_SCL_SPEED_SM	100000 //Kbits
#define I2C_SCL_SPEED_FM4k	400000
#define I2C_SCL_SPEED_FM2k	200000

/* I2C ACK CONTROL MACROS */

#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/* I2C ACK CONTROL MACROS */

#define I2C_FMDUTY_2		0
#define I2C_FMDUTY_16_9		1

/* I2C RELATED STATUS FLAGS MACROS */

#define I2C_FLAG_TXE	 (1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE	 (1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB		 (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR	 (1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF	 (1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF	 (1 << I2C_SR1_STOPF)
#define I2C_FLAG_OVR	 (1 << I2C_SR1_OVR)
#define I2C_FLAG_AF 	 (1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO	 (1 << I2C_SR1_ARLO)
#define I2C_FLAG_ADD10	 (1 << I2C_SR1_ADD10)
#define I2C_FLAG_BERR	 (1 << I2C_SR1_BERR)
#define I2C_FLAG_TIMEOUT (1 << I2C_SR1_TIMEOUT)

/* I2C APPLICATION STATUS MACROS */

#define I2C_READY   		0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2

/* I2C APPLICATION EVENTS MACROS */

#define I2C_EV_TX_CMPLT		0
#define I2C_EV_RX_CMPLT		1
#define I2C_EV_STOP			2
#define I2C_ERROR_BERR  	3
#define I2C_ERROR_ARLO  	4
#define I2C_ERROR_AF    	5
#define I2C_ERROR_OVR   	6
#define I2C_ERROR_TIMEOUT	7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9

/** APIs SUPPORTED BY THIS DRIVER **/

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


/** Init & De-init **/

void I2C_Init(I2C_Handle_t *pI2CHANDLE);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/** Flag **/

uint8_t I2C_GetFlagStat(I2C_RegDef_t *pI2Cx, uint32_t Flag);

/* Data send & receive */

void I2C_MasterSend(I2C_Handle_t *pI2CHANDLE, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr);
void I2C_MasterReceive(I2C_Handle_t *pI2CHANDLE, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr);

uint8_t I2C_MasterSendIT(I2C_Handle_t *pI2CHANDLE, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr); // RETURNS THE STATE OF THE APPLICATION (READY OF BUSY IN TX/RX)
uint8_t I2C_MasterReceiveIT(I2C_Handle_t *pI2CHANDLE, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHANDLE);
void I2C_CloseSendData(I2C_Handle_t *pI2CHANDLE);

void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/* IRQ Configuration & ISR Handling */

void I2C_IRQInterrupConfig(uint8_t IRQNum, uint8_t IRQPriority, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHANDLE);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHANDLE);

/* OTHER PERIPHERAL CONTROL APIs */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/* APPLICATION CALL BACK API */

void I2C_AppEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t AppEv);










#endif
