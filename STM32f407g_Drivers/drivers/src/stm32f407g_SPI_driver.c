#include "stm32f407g_SPI_driver.h"
#include "stm32f407g.h"
#include <stdint.h>
#include <stdio.h>

static void	SPI_TXE_INTHandle(SPI_Handle_t *pSPIHandle);
static void	SPI_RXNE_INTHandle(SPI_Handle_t *pSPIHandle);
static void SPI_OVRERR_INTHandle(SPI_Handle_t *pSPIHandle);




void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == DISABLE)
	{
		if(pSPIx == SPI1)
				{
					SPI1_PERI_CLK_DI();

				}else if (pSPIx == SPI2)
				{
					SPI2_PERI_CLK_DI();

				}else if (pSPIx == SPI3)
				{
					SPI3_PERI_CLK_DI();

				}
	}
	else if(EnorDi == DISABLE)
	{
		if(pSPIx == SPI1)
				{
					SPI1_PERI_CLK_DI();

				}else if (pSPIx == SPI2)
				{
					SPI2_PERI_CLK_DI();

				}else if (pSPIx == SPI3)
				{
					SPI3_PERI_CLK_DI();

				}
	}


}


void SPI_Init(SPI_Handle_t *pSPIHANDLE)
{

	// Clock Enable
	SPI_PeriClockControl(pSPIHANDLE->pSPIx, ENABLE);

	// Configure SPI_CR1 Register
	uint32_t tempreg = 0;

	// 1. Device mode

	tempreg |= pSPIHANDLE->SPICFG.SPI_DeviceMode << SPI_CR1_MSTR; // Left shift it to the corresponding register bit position

	// 2. Bus config

	if(pSPIHANDLE->SPICFG.SPI_BusCFG == SPI_BUS_CONFIG_FD)
	{
		// Clear BIDI MODE

		tempreg &= ~( 1 << SPI_CR1_BIDIMODE); // Left shift it to the corresponding register bit position


	}else if(pSPIHANDLE->SPICFG.SPI_BusCFG == SPI_BUS_CONFIG_HD)
	{
		// Set BIDI MODE

		tempreg |= (1 << SPI_CR1_BIDIMODE); // Left shift it to the corresponding register bit position


	}else if(pSPIHANDLE->SPICFG.SPI_BusCFG == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// Clear BIDI MODE

		tempreg &= ~( 1 << SPI_CR1_BIDIMODE); // Left shift it to the corresponding register bit position


		//RXONLY bit must be set

		tempreg &= ~( 1 << SPI_CR1_RXONLY); // Left shift it to the corresponding register bit position


	}

	// 3. SPI SCLK speed config

	tempreg |=  pSPIHANDLE->SPICFG.SPI_SClkSpeed << SPI_CR1_BR; // Left shift it to the corresponding register bit position


	// 4. Configure DFF

	tempreg |=  pSPIHANDLE->SPICFG.SPI_DFF << SPI_CR1_DFF; // Left shift it to the corresponding register bit position


	// 5. Configure CPOL

	tempreg |=  pSPIHANDLE->SPICFG.SPI_CPOL << SPI_CR1_CPOL; // Left shift it to the corresponding register bit position


	// 6. Configure CPHA

	tempreg |=  pSPIHANDLE->SPICFG.SPI_CPHA << SPI_CR1_CPHA; // Left shift it to the corresponding register bit position


	tempreg |=  pSPIHANDLE->SPICFG.SPO_SSM << SPI_CR1_SSM; // Left shift it to the corresponding register bit position


	pSPIHANDLE->pSPIx->CR1 = tempreg;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	 if(pSPIx == SPI1)
		{
			SPI1_REG_RESET();
		}else if(pSPIx == SPI2)
		{
			SPI2_REG_RESET();
		}else if(pSPIx == SPI3)
		{
			SPI3_REG_RESET();
		}


}

uint8_t SPI_GetFlagStat(SPI_RegDef_t *pSPIx, uint32_t Flag)
{

	if(pSPIx->SR & Flag)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}



void SPI_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length)
{

	while(Length > 0)
	{
		// 1. Wait until TXE is set
		while(SPI_GetFlagStat(pSPIx, SPI_TXE_FLAG) == FLAG_RESET); // if the whole expression is 0, then the TXE is not yet set

		// 2. Check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{

			// 16 bit DFF if the expression is true
			// 1. Load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer); // we typecast the TxBuffer in order to receive a 16 Bits data, then defreference it.
			len--;
			len--;
			(uint16_t*)pTxBuffer++; // Increment in order to make it point to the next data.

		}else
		{

			// 8 bit DFF
			pSPIx->DR = *(pTxBuffer);
			len--;
			pTxBuffer++;
		}
	}

}


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{

		pSPIx->CR1 |= (1 << SPI_CR1_SPE);

	}else
	{

		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);

	}

}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{

		pSPIx->CR1 |= (1 << SPI_CR1_SSI);

	}else
	{

		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);

	}

}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{

		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);

	}else
	{

		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);

	}

}


void SPI_Receive(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length)
{

	while(Length > 0)
	{
		// 1. Wait until RXNE is set
		while(SPI_GetFlagStat(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET); // if the whole expression is 0, then the TXE is not yet set

		// 2. Check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{

			// 16 bit DFF if the expression is true
			// 1. Load the data from DR to Rx buffer address
		    *((uint16_t*)pRxBuffer) = pSPIx->DR; // we typecast the TxBuffer in order to receive a 16 Bits data, then defreference it.
			len--;
			len--;
			(uint16_t*)pRxBuffer++; // Increment in order to make it point to the next data address.

		}else
		{

			// 8 bit DFF
			*(pRxBuffer) = pSPIx->DR;
			len--;
			pRxBuffer++;
		}
	}

}



void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnorDi)
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


void SPI_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority)
{
	//1 First find out the IPR Register
	uint8_t iprx = IRQNum / 4;
	uint8_t iprx_select = IRQNum % 4;
	uint8_t shift_amount = (8 * iprx_select) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= ( IRQPriority << shift_amount); // jumps to IPR register address with + iprx

}


uint8_t SPI_SendINT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length)
{

	uint8_t state = pSPIHandle->TxState;

	if( state != SPI_BUSY_IN_TX)
	{

		// 1. Save the Tx buffer address and length information in some global variables
			pSPIHandle->pTxBuffer = pTxBuffer;
			pSPIHandle->TxLen = Length;
			// 2. Mark the SPI state as busy in transmission so no other code can take over the same SPI
			pSPIHandle->TxState = SPI_BUSY_IN_TX;
			// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in Status register
			pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );

	}

	return state;


}



uint8_t SPI_RecieveINT(SPI_Handle_t *pSPIx, uint8_t *pRxBuffer, uint32_t Length)
{

	uint8_t state = pSPIx->RxState;

		if( state != SPI_BUSY_IN_RX)
		{

			// 1. Save the Tx buffer address and length information in some global variables
			pSPIx->RxLen = Length;
			// 2. Mark the SPI state as busy in transmission so no other code can take over the same SPI
			pSPIx->RxState = SPI_BUSY_IN_RX;
			// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in Status register
			pSPIx->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );

		}

		return state;


}




void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

	uint8_t temp1, temp2;

	// 1. Check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		// Must handle TXE
		SPI_TXE_INTHandle(pHandle);

	}

	// 2. Check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		// Must handle TXE
		SPI_RXNE_INTHandle(pHandle);

	}

	// 3. Check for Overrun error
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		// Must handle TXE
		SPI_OVRERR_INTHandle(pHandle);

	}

}

/* HELPER FUNCTIONS */

static void	SPI_TXE_INTHandle(SPI_Handle_t *pSPIHandle)
{

	// Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{

		// 16 bit DFF if the expression is true
		// 1. Load the data into the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer); // we typecast the TxBuffer in order to receive a 16 Bits data, then defreference it.
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++; // Increment in order to make it point to the next data.

	}else
	{

		// 8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if( ! pSPIHandle->TxLen )
	{
		// if TxLen is zero, close SPI transmission and inform that TX is over
		// 1. Deactivate TXEIE / Prevents interrupts from setting up of TXE FLAG
		SPI_CloseTransmission(pSPIHandle);
		SPI_AppEventCallBack(pSPIHandle,SPI_EVENT_TX_COMPLT);


	}

}


static void	SPI_RXNE_INTHandle(SPI_Handle_t *pSPIHandle)
{

	    // Check the DFF bit in CR1
		if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{

			// 16 bit DFF if the expression is true
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;  // we typecast the TxBuffer in order to receive a 16 Bits data, then defreference it.
			pSPIHandle->RxLen -= 2;
			(uint16_t*)pSPIHandle->pRxBuffer--;
			(uint16_t*)pSPIHandle->pRxBuffer--;

		}else
		{

			// 8 bit DFF
			*(pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer++;
		}

		if( ! pSPIHandle->RxLen )
		{

			//Reception is complete, turn off RXNEIE INTERRUPT
			SPI_CloseReception(pSPIHandle);
			SPI_AppEventCallBack(pSPIHandle,SPI_EVENT_RX_COMPLT);


		}

}

static void SPI_OVRERR_INTHandle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	// 1. Clear the OVR flag
	if( pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{

		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;

	}
	(void) temp;
	// 2. Inform the application
	SPI_AppEventCallBack(pSPIHandle,SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{

	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{

	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void) temp;
}

void SPI_AppEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	// WEAK IMPLEMENTATION. APPLICATION MAY OVERRIDE THIS FUNCTION


}







