#ifndef INC_STMF407G_GPIO_DRIVER_H_
#define INC_STMF407G_GPIO_DRIVER_H_

#include "stm32f407g.h"
#define __vo volatile


/*** DEFINING STRUCTURE FOR GPIOx CONFIGURATION ***/

typedef struct
{
	uint8_t GPIO_PinNum;
	uint8_t GPIO_PinMode;   /** Possible values range from 0-6 @ GPIO PIN POSSIBLE MODES **/
	uint8_t GPIO_PinSpeed;  /** Possible values range from 0-3 @ GPIO PIN OUTPUT POSSIBLE SPEEDS **/
	uint8_t GPIO_PinPupControl; /** Possible values range from 0-2 @ GPIO PIN PULL UP-DOWN CONFIG **/
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinCFG_t;

/*** DEFINING STRUCTURE FOR GPIOx HANDLE ***/

typedef struct
{
	/** FIRST ELEMENT IS A POINTER TO HOLD THE BASE ADDRESS OF THE GPIO PERIPHERAL **/

	GPIO_RegDef_t *pGPIOx;  /** BASE ADDRESS **/
	GPIO_PinCFG_t GPIO_PinCFG;

}GPIO_Handle_t;

/** GPIO PIN NUMBERS **/

#define GPIO_PIN_NUM0		0
#define GPIO_PIN_NUM1		1
#define GPIO_PIN_NUM2		2
#define GPIO_PIN_NUM3		3
#define GPIO_PIN_NUM4		4
#define GPIO_PIN_NUM5		5
#define GPIO_PIN_NUM6		6
#define GPIO_PIN_NUM7		7
#define GPIO_PIN_NUM8		8
#define GPIO_PIN_NUM9		9
#define GPIO_PIN_NUM10		10
#define GPIO_PIN_NUM11		11
#define GPIO_PIN_NUM12		12
#define GPIO_PIN_NUM13		13
#define GPIO_PIN_NUM14		14
#define GPIO_PIN_NUM15		15

/** GPIO PIN POSSIBLE MODES **/

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4  /* INPUT INTERRUPT FALLING EDGE */
#define GPIO_MODE_IT_RT		5  /* INPUT INTERRUPT RISING EDGE */
#define GPIO_MODE_IT_RFT	6  /* INPUT INTERRUPT RISING-FALLING EDGE */

/** GPIO PIN POSSIBLE OUTPUT TYPES **/

#define GPIO_OP_TYPE_PP		0 /* OUTPUT TYPE PUSH-PULL */
#define GPIO_OP_TYPE_OD		1 /* OUTPUT TYPE OPEN DRAIN */

/** GPIO PIN POSSIBLE OUTPUT SPEEDS **/

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/** GPIO PIN PULL UP-DOWN CONFIG **/

#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2






/** APIs SUPPORTED BY THIS DRIVER **/

/** Peripheral Clock Setup **/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi); /* First param is the base address, the second param to decide whether EN or DI */


/** Init & De-init **/

void GPIO_Init(GPIO_Handle_t *pGPIOHANDLE); /* Takes a pointer to the handler structure in order to initialize the GPIO */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); /* Takes a pointer to the Register define structure in order to de-init the GPIO */

/** Data read & write **/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum); /* Takes the base address and the pin number and returns 0 or 1 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx); /* 16 bits because of port */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t Val); /* No need to return a value, however we write a value of 0 or 1 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOX, uint16_t Val);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);

/** IRQ Config & ISR Handling */

void GPIO_IRQInterrupConfig(uint8_t IRQNum, uint8_t IRQPriority, uint8_t EnorDi); /* Takes IRQ Number as a first param, then Priority and En or DI */
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNum); /* which pin triggered the interrupt */










#endif /* INC_STMF407G_GPIO_DRIVER_H_ */
