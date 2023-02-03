#include "stmf407g_GPIO_driver.h"
#include "stm32f407g.h"
#include <stdint.h>
#include <stdio.h>

/*********************************************************************
 * @fn      		  - Peripheral Clock Setup
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - EN or DI macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi == ENABLE)
	{
		if( (pGPIOx == GPIOA) )
		{
			GPIOA_PERI_CLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PERI_CLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PERI_CLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PERI_CLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PERI_CLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PERI_CLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PERI_CLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PERI_CLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PERI_CLK_EN();
		}
	}
	else{

	}
}

/*********************************************************************
 * @fn      		  - GPIO Initialization Function
 *
 * @brief             - Initializes the GPIO
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHANDLE)
{


	GPIO_PeriClockControl(pGPIOHANDLE->pGPIOx, ENABLE);
	/** 1. CONFIGURING MODE OF THE GPIO **/
		// Non interrupt mode
	uint32_t temp = 0;

	if( pGPIOHANDLE->GPIO_PinCFG.GPIO_PinMode <= GPIO_MODE_ANALOG )
	{
		temp = (pGPIOHANDLE->GPIO_PinCFG.GPIO_PinMode << (2 * pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum)); /** CONFIG THE PIN THAT WILL WORK ON, EVERY PIN HAS 2 BIT SPACE **/
	    pGPIOHANDLE->pGPIOx->MODER &= ~( 0x3 << pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum); /* In case we don't know the bit field is cleared, we clear it by negating 0x3 and reseting it */
		pGPIOHANDLE->pGPIOx->MODER |= temp; /* set the bit field */
		temp = 0;
	}else
	{
		// Interrupt Mode

		if( pGPIOHANDLE->GPIO_PinCFG.GPIO_PinMode == GPIO_MODE_IT_FT ) //falling edge
		{

			EXTI->FTSR |= ( 1 << pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum); // Configure FTSR
			EXTI->RTSR &= ~( 1 << pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum); 		// Clear the corresponding RTSR //

		}else if(pGPIOHANDLE->GPIO_PinCFG.GPIO_PinMode == GPIO_MODE_IT_RT)
		{

			EXTI->RTSR |= ( 1 << pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum); // Configure RTSR
			EXTI->FTSR &= ~( 1 << pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum); // Clear the corresponding FTSR //
		}else if(pGPIOHANDLE->GPIO_PinCFG.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{

			EXTI->RTSR |= ( 1 << pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum); // Configure both FTSR & RTSR
			EXTI->FTSR |= ( 1 << pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum);
		}


		/* Configure the EXTICR Register */

		uint8_t temp1 = pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum / 4;
		uint8_t temp2 = pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHANDLE->pGPIOx);
		SYSCFG_PERI_CLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		/* Configure the EXTI interrupt delivery using IMR */
		EXTI->IMR |=  (1 << pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum);
	}

	/** 2. CONFIGURING SPEED OF THE GPIO **/

	temp = (pGPIOHANDLE->GPIO_PinCFG.GPIO_PinSpeed << (2* pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum));
	pGPIOHANDLE->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum);
	pGPIOHANDLE->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	/** 3. CONFIGURING PULL MODE OF THE GPIO **/

	temp = (pGPIOHANDLE->GPIO_PinCFG.GPIO_PinPupControl << (2* pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum));
	pGPIOHANDLE->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum);
	pGPIOHANDLE->pGPIOx->PUPDR |= temp;
	temp = 0;

	/** 4. CONFIGURING THE OUTPUT MODE OF THE GPIO **/

	temp = (pGPIOHANDLE->GPIO_PinCFG.GPIO_PinOPType <<  pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum);
	pGPIOHANDLE->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum);
	pGPIOHANDLE->pGPIOx->OTYPER |= temp;

	/** 5. CONFIGURING THE ALTERNATE FUNCTIONALITY OF THE GPIO **/

	if(pGPIOHANDLE->GPIO_PinCFG.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
	uint32_t temp1, temp2;
	temp1 = pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum / 8;
	temp2 = pGPIOHANDLE->GPIO_PinCFG.GPIO_PinNum % 8;
	pGPIOHANDLE->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); // 4 bits field (0x1111)
	pGPIOHANDLE->pGPIOx->AFR[temp1] |= (pGPIOHANDLE->GPIO_PinCFG.GPIO_PinAltFunMode << (4 * temp2));

	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

	 if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}

}

/** Data read & write **/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNum) & 0x00000001); // Shift the desired pin to the LSB position, & mask all other bit position with 0 except LSB
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum, uint8_t Val)
{
	if( Val == 1)
	{
		/** WRITE 1 TO THE OUTPUT DATA REGISTER AT THE BIT FIELD CORRESPONDING TO THE PIN NUM **/
		pGPIOx->ODR |= ( 1 << PinNum);


	}else
	{
		/** WRITE 0 O THE OUTPUT DATA REGISTER AT THE BIT FIELD CORRESPONDING TO THE PIN NUM **/
		pGPIOx->ODR &= ~( 1 << PinNum);
	}

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Val)
{
	pGPIOx->ODR = Val;

}

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
	pGPIOx->ODR ^= ( 1 << PinNum);  /** ^ is an XOR operator **/

}

/** IRQ Config & ISR Handlin, This is on a processor level */

void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if( IRQNumber <= 31) // Program ISER0 Register
		{

			*NVIC_ISER0 |= (1 << IRQNumber);


		}else if( IRQNumber > 31 && IRQNumber < 64) // Program ISER1 Register
		{

			*NVIC_ISER1 |= (1 << IRQNumber % 32);

		}else if( IRQNumber >= 64 && IRQNumber < 96) // Program ISER2 Register
		{

			*NVIC_ISER3 |= (1 << IRQNumber % 64);
		}
	}else
	{
		if( IRQNumber <= 31 )
		{

			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{

			*NVIC_ICER1 |= (1 << IRQNumber % 32);

		}else if(IRQNumber >= 64 && IRQNumber < 96 )
		{

			*NVIC_ICER3 |= (1 << IRQNumber % 64);

		}
	}

}


void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority)
{
	//1 First find out the IPR Register
	uint8_t iprx = IRQNum / 4;
	uint8_t iprx_select = IRQNum % 4;
	uint8_t shift_amount = (8 * iprx_select) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= ( IRQPriority << shift_amount); // jumps to IPR register address with + iprx

}


void GPIO_IRQHandling(uint8_t PinNum)
{

	// clear the EXTI PR Register corresponding to the pin number
	if(EXTI->PR & (1 << PinNum))
	{
		// Clear
		EXTI->PR |= (1 << PinNum);

	}

}
