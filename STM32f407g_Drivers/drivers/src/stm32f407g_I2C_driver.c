#include "stm32f407g_I2C_driver.h"
#include "stm32f407g.h"
#include <stdint.h>
#include <stdio.h>


/*** SOME HELPER FUNCTIONS (PRIVATE FOR THE FILE) ***/
static void I2C_GenerateStartCond(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHANDLE);
static void I2C_GenerateStopCond(I2C_RegDef_t *pI2Cx);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHANDLE);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHANDLE);


static void I2C_GenerateStartCond(I2C_RegDef_t *pI2Cx) // REFER TO REFERENCE MANUAL IN ORDER TO UNDERSTAND HOW TO GENERATE THE START CONDITION
{

	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);

}


static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{

	SlaveAddr = SlaveAddr << 1; // Make space for r/nw bit
	SlaveAddr &= ~(1); // Clear the 0th bit of the address to set the r/nw bit to 0
	pI2Cx->DR = SlaveAddr; // Put the address in the Data register

}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{

	SlaveAddr = SlaveAddr << 1; // Make space for r/nw bit
	SlaveAddr |= 1; // Clear the 0th bit of the address to set the r/nw bit to 0
	pI2Cx->DR = SlaveAddr; // Put the address in the Data register

}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHANDLE)
{

	uint32_t dummy_read;
	// Check for master mode
	if( pI2CHANDLE->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL) )
	{

		// Device is in master mode
		if( pI2CHANDLE->TxRxState == I2C_BUSY_IN_RX )
		{

			if( pI2CHANDLE->RxSize == 1)
			{

				// Firstly disable the ACK
				I2C_ManageAcking(pI2CHANDLE->pI2Cx,DISABLE);

				// Clear the ADDR flag (read SR1, read SR2)
				dummy_read = pI2CHANDLE->pI2Cx->SR1;
				dummy_read = pI2CHANDLE->pI2Cx->SR2;
				(void) dummy_read;

			}

		}else
		{

			// Clear the ADDR flag (read SR1, read SR2)
			dummy_read = pI2CHANDLE->pI2Cx->SR1;
			dummy_read = pI2CHANDLE->pI2Cx->SR2;
			(void) dummy_read;

		}

	}else
	{

		// Device is in slave mode

		// Clear the ADDR flag (read SR1, read SR2)
		dummy_read = pI2CHANDLE->pI2Cx->SR1;
		dummy_read = pI2CHANDLE->pI2Cx->SR2;
		(void) dummy_read;

	}

}

static void I2C_GenerateStopCond(I2C_RegDef_t *pI2Cx) // REFER TO REFERENCE MANUAL IN ORDER TO UNDERSTAND HOW TO GENERATE THE STOP CONDITION
{

	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);

}


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{

	if(EnorDi == DISABLE)
		{
			if(pI2Cx == I2C1)
					{
						I2C1_PERI_CLK_DI();

					}else if (pI2Cx == I2C2)
					{
						I2C2_PERI_CLK_DI();

					}else if (pI2Cx == I2C3)
					{
						I2C3_PERI_CLK_DI();

					}
		}
		else if(EnorDi == DISABLE)
		{
			if(pI2Cx == I2C1)
					{
						I2C1_PERI_CLK_DI();

					}else if (pI2Cx == I2C2)
					{
						I2C2_PERI_CLK_DI();

					}else if (pI2Cx == I2C3)
					{
						I2C3_PERI_CLK_DI();

					}
		}



}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{

		pI2Cx->CR1 |= (1 << I2C_CR1_PE);

	}else
	{

		pI2Cx->CR1 &= ~(1 << 0);

	}

}


void I2C_Init(I2C_Handle_t *pI2CHANDLE)
{

	uint32_t tempreg = 0;

	// Enable the clock for I2Cx peripheral
	I2C_PeriClockControl(pI2CHANDLE->pI2Cx,ENABLE);

	// 1. ACK CONTROL BIT
	tempreg |= pI2CHANDLE->I2C_CFG.I2C_ACKControl << 10;
	pI2CHANDLE->pI2Cx->CR1 = tempreg;

	// 2. CONFIGURE THE FREQ FIELD OF CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHANDLE->pI2Cx->CR2 = (tempreg & 0x3F); // MASK THE BITS AFTER 5

	// 3. CONFIGURE THE DEVICE OWN ADDRESS
	tempreg |= pI2CHANDLE->I2C_CFG.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14 );
	pI2CHANDLE->pI2Cx->OAR1 = tempreg;

	// 4. CCR CALCULATIONS
	uint16_t ccrVal = 0;
	tempreg = 0;

	if( pI2CHANDLE->I2C_CFG.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{

		// STANDARD MODE
		ccrVal = (RCC_GetPCLK1Value() / ( 2 * pI2CHANDLE->I2C_CFG.I2C_SCLSpeed) );
		tempreg |= (ccrVal & 0xFFF);

	}else
	{

		//FAST MODE
		tempreg |= ( 1 << 15 ); // CONFIGURE FAST MODE
		tempreg |= ( pI2CHANDLE->I2C_CFG.I2C_FMDutyCycle << 14 );
		if( pI2CHANDLE->I2C_CFG.I2C_FMDutyCycle == I2C_FMDUTY_2 )
		{

			ccrVal = (RCC_GetPCLK1Value() / ( 3 * pI2CHANDLE->I2C_CFG.I2C_SCLSpeed) );

		}else
		{

			ccrVal = (RCC_GetPCLK1Value() / ( 25 * pI2CHANDLE->I2C_CFG.I2C_SCLSpeed) );

		}

		tempreg |= (ccrVal & 0xFFF);
	}

	pI2CHANDLE->pI2Cx->CCR = tempreg;

	// 5. CONFIGURE TRISE
	if(pI2CHANDLE->I2C_CFG.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{
			//mode is standard mode

			tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

		}else
		{
			//mode is fast mode
			tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

		}

		pI2CHANDLE->pI2Cx->TRISE = (tempreg & 0x3F);
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}

}


uint8_t I2C_GetFlagStat(I2C_RegDef_t *pI2Cx, uint32_t Flag)
{

	if(pI2Cx->SR1 & Flag)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}


void I2C_MasterSend(I2C_Handle_t *pI2CHANDLE, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr) // REFER TO TRANSFER SEQUENCE DIAGRAM FOR MASTER SEND IN REFERENCE MANUAL TO UNDERSTAND HOW MASTER SENDS
{

	// 1. Generate the start condition
	I2C_GenerateStartCond(pI2CHANDLE->pI2Cx);

	// 2. Confirm that the start generation is completed by checking the SB flag in the SR1 (until SB is cleared, SCL will be stretched)
	while( ! I2C_GetFlagStat(pI2CHANDLE->pI2Cx, I2C_FLAG_SB) );

	// 3. Send the address of the salve with r/nw bit set to w(0) (8 bits in total)
	I2C_ExecuteAddressPhaseWrite(pI2CHANDLE->pI2Cx, SlaveAddr);

	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStat(pI2CHANDLE->pI2Cx, I2C_FLAG_ADDR) ); // REFER TO MANUAL IN ORDER TO KNOW HOW TO CLEAR THE ADDR FLAG

	// 5. Clear the ADDR flag according to is software sequence (until ADDR is cleared, SCL will be stretched)
	I2C_ClearADDRFlag(pI2CHANDLE);

	// 6. Send the data until Len becomes 0
	while( Len > 0)
	{

		while ( ! I2C_GetFlagStat(pI2CHANDLE->pI2Cx,I2C_FLAG_TXE) ); // Wait till TXE is set

		pI2CHANDLE->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;

	}

	// 7. When Len becomes 0, wait for TXE = 1 & BTF = 1 before generating the STOP condition.
	// If TXE = 1, BTF = 1 it means that both SR and DR are empty and the next transmission shall begin
	// When BTF = 1, SCL will be stretched
	while ( ! I2C_GetFlagStat(pI2CHANDLE->pI2Cx,I2C_FLAG_TXE) ); // Wait till TXE is set

	while ( ! I2C_GetFlagStat(pI2CHANDLE->pI2Cx,I2C_FLAG_BTF) ); // Wait till BTF is set

	// 8. Generate the STOP condition and master do not need to wait for the completion of the stop condition.
	// Note that generating stop will automatically clear the BTF flag
	I2C_GenerateStopCond(pI2CHANDLE->pI2Cx);

}

void I2C_MasterReceive(I2C_Handle_t *pI2CHANDLE, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr)
{

	// 1. Generate the start condition
	I2C_GenerateStartCond(pI2CHANDLE->pI2Cx);

	// 2. Confirm that start generation is completed by checking the SB flag in the SR1 (until SB is cleared, SCL will be stretched)
	while( ! I2C_GetFlagStat(pI2CHANDLE->pI2Cx, I2C_FLAG_SB) );

	// 3. Send the address of the slave with r/nw bit set to R(1) (8 bits in total)
	I2C_ExecuteAddressPhaseRead(pI2CHANDLE->pI2Cx, SlaveAddr);

	// 4. Wait until address phase is completed by checking the ADDR flag in SR1
	while( ! I2C_GetFlagStat(pI2CHANDLE->pI2Cx, I2C_FLAG_ADDR) );

	// Procedure to read only 1 byte from slave
	if( Len == 1 )
	{

       // Disable ACKING
	   I2C_ManageAcking(pI2CHANDLE->pI2Cx,I2C_ACK_DISABLE);

	   // Clear ADDR flag
	   I2C_ClearADDRFlag(pI2CHANDLE);

	   // Wait until RXNE becomes 1
	   while ( ! I2C_GetFlagStat(pI2CHANDLE->pI2Cx,I2C_FLAG_RXNE) );

	   // Generate STOP condition
	   I2C_GenerateStopCond(pI2CHANDLE->pI2Cx);

	   // Read data into buffer
	   *pRxBuffer = pI2CHANDLE->pI2Cx->DR;


	}

	// Procedure to read more than 1 byte from slave
	if( Len > 1)
	{

		// Clear the ADDR
		I2C_ClearADDRFlag(pI2CHANDLE);

		// Read the data until len becoems zero
		for( uint32_t i = Len; i > 0; i--)
		{

			// Wait until RXNE becomes 1
			while ( ! I2C_GetFlagStat(pI2CHANDLE->pI2Cx,I2C_FLAG_RXNE) );

			if( i == 2) // if only last 2 bytes are remaining
			{

				// Disable ACKING
				I2C_ManageAcking(pI2CHANDLE->pI2Cx,I2C_ACK_DISABLE);

				// Generate STOP condition
				I2C_GenerateStopCond(pI2CHANDLE->pI2Cx);

			}

			// Read the data from data register in to buffer
			*pRxBuffer = pI2CHANDLE->pI2Cx->DR;

			// Increment the buffer address
			pRxBuffer++;

		}

	}


	// re-enable ACKING
	if(pI2CHANDLE->I2C_CFG.I2C_ACKControl == I2C_ACK_ENABLE)
	{

		I2C_ManageAcking(pI2CHANDLE->pI2Cx,I2C_ACK_ENABLE);

	}

}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{

	if( EnorDi == I2C_ACK_ENABLE)
	{

		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);

	}else
	{

		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);

	}

}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}

uint8_t I2C_MasterSendIT(I2C_Handle_t *pI2CHANDLE, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	uint8_t busystate = pI2CHANDLE-> TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHANDLE->pTxBuffer = pTxBuffer;
			pI2CHANDLE->TxLen = Len;
			pI2CHANDLE->TxRxState = I2C_BUSY_IN_TX;
			pI2CHANDLE->DevAddr = SlaveAddr;
			pI2CHANDLE->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_GenerateStartCond(pI2CHANDLE->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHANDLE->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHANDLE->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHANDLE->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;

}

uint8_t I2C_MasterReceiveIT(I2C_Handle_t *pI2CHANDLE, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{

	uint8_t busystate = pI2CHANDLE-> TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHANDLE->pRxBuffer = pRxBuffer;
			pI2CHANDLE->RxLen = Len;
			pI2CHANDLE->TxRxState = I2C_BUSY_IN_TX;
			pI2CHANDLE->RxSize = Len;
			pI2CHANDLE->DevAddr = SlaveAddr;
			pI2CHANDLE->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_GenerateStartCond(pI2CHANDLE->pI2Cx);

			//Implement the code to enable ITBUFEN Control Bit
			pI2CHANDLE->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Implement the code to enable ITEVFEN Control Bit
			pI2CHANDLE->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//Implement the code to enable ITERREN Control Bit
			pI2CHANDLE->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;

}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHANDLE)
{

	if( pI2CHANDLE-> TxLen > 0)
	{

			// 1. Load data into DR
			pI2CHANDLE->pI2Cx->DR = *(pI2CHANDLE->pTxBuffer);

			// 2. Decrement the TxLen
			pI2CHANDLE->TxLen--;

			// 3. Increment the buffer address
			pI2CHANDLE->TxBuffer++;

	}

}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHANDLE)
{
	//We have to do the data reception
	if(pI2CHANDLE->RxSize == 1)
	{
		*pI2CHANDLE->pRxBuffer = pI2CHANDLE->pI2Cx->DR;
		pI2CHANDLE->RxLen--;

	}


	if(pI2CHANDLE->RxSize > 1)
	{
		if(pI2CHANDLE->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageAcking(pI2CHANDLE->pI2Cx,DISABLE);
		}

			//read DR
			*pI2CHANDLE->pRxBuffer = pI2CHANDLE->pI2Cx->DR;
			pI2CHANDLE->pRxBuffer++;
			pI2CHANDLE->RxLen--;
	}

	if(pI2CHANDLE->RxLen == 0 )
	{
		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(pI2CHANDLE->Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHANDLE->pI2Cx);

		//2 . Close the I2C rx
		I2C_CloseReceiveData(pI2CHANDLE);

		//3. Notify the application
		I2C_AppEventCallBack(pI2CHANDLE,I2C_EV_RX_CMPLT);
	}
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHANDLE)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHANDLE->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHANDLE->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHANDLE->TxRxState = I2C_READY;
	pI2CHANDLE->pRxBuffer = NULL;
	pI2CHANDLE->RxLen = 0;
	pI2CHANDLE->RxSize = 0;

	if(pI2CHANDLE->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHANDLE->pI2Cx,ENABLE);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHANDLE)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHANDLE->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHANDLE->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHANDLE->TxRxState = I2C_READY;
	pI2CHANDLE->pTxBuffer = NULL;
	pI2CHANDLE->TxLen = 0;
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data) // Write the data to be send to the DR
{

	pI2C->DR = data;

}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C) // Return the desired data
{

	return (uint8_t) pI2C->DR;

}


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHANDLE)
{

	// Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHANDLE->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVEN); // Checking for the corresponding bit field
	temp2 = pI2CHANDLE->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHANDLE->pI2Cx->SR1 & ( 1 << I2C_SR1_SB); // Check for whether the SB flag is set or not

	// 1. Handle for interrupt generated by SB event (only in master mode)
	if( temp1 && temp3) // if both values are true, then only the SB flag is set
	{

		// this interrupt is generated because of SB event, for slave mode SB is always zero
		// Lets execute the address phase in this block
		if( pI2CHANDLE->TxRxState == I2C_BUSY_IN_TX )
		{

			I2C_ExecuteAddressPhaseWrite(pI2CHANDLE->pI2Cx,pI2CHANDLE->DevAddr);

		}else if ( pI2CHANDLE->TxRxState == I2C_BUSY_IN_TX )
		{

			I2C_ExecuteAddressPhaseRead(pI2CHANDLE->pI2Cx,pI2CHANDLE->DevAddr);

		}


	}

	temp3 = pI2CHANDLE->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);
	// 2. Handle for interrupt generated by ADDR event (for master mode : address is sent, for slave mode : address matched with own address)
	if( temp1 && temp3)
	{

		// ADDR is set
		I2C_ClearADDRFlag(pI2CHANDLE);

	}

	temp3 = pI2CHANDLE->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);
	// 3. Handle for interrupt generated by BTF (byte transfer finished) event
	if( temp1 && temp3)
	{

		// BTF is set
		if( pI2CHANDLE->TxRxState == I2C_BUSY_IN_TX )
		{
			// make sure that TXE is also set
			if( pI2CHANDLE->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE) )
			{

				// BTF, TXE = 1
				if( pI2CHANDLE->TxLen == 0)
				{
				// CLose the transmission
				// 1. Generate STOP condition
					if( pI2CHANDLE->Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCond(pI2CHANDLE->pI2Cx);

					// 2. Reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHANDLE);

					// 3. Notify application that transmission is complete
					I2C_AppEventCallBack(pI2CHANDLE,I2C_EV_TX_CMPLT);
				}
			}

		}else if ( pI2CHANDLE->TxRxState == I2C_BUSY_IN_RX )
		{

			;

		}

	}

	temp3 = pI2CHANDLE->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);
	// 4. Handle for interrupt generated by STOPF event
	if( temp1 && temp3)
	{

		// STOPF is set
		// Clear the STOP -> Read SR1 -> Write to CR1. (SR1 is already read)
		pI2CHANDLE->pI2Cx->CR1 |= 0x0000;

		// Notify application that stop is detected
		I2C_AppEventCallBack(pI2CHANDLE,I2C_EV_STOP);


	}

	temp3 = pI2CHANDLE->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);
	// 5. Handle for interrupt generated by TXE event
	if( temp1 && temp2 && temp3)
	{

		// Check for device mode (master or slave)
		if( pI2CHANDLE->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL) )
		{

			// TXE flag is set
			// We have to do the data transmission
			if( pI2CHANDLE->TxRxState == I2C_BUSY_IN_TX)
			{

				I2C_MasterHandleTXEInterrupt(pI2CHANDLE);

			}


		}else
		{

			// For slave
			// make sure that the slave is in transmitter mode
			if( pI2CHANDLE->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA) )
			{

				I2C_AppEvenCallBack(pI2CHANDLE,I2C_EV_DATA_REQ);

			}


		}



	}
	temp3 = pI2CHANDLE->pI2Cx->SR1 & ( 1 << RXNE);
	// 6. Handle for interrupt generated by RXNE event
	if( temp1 && temp2 && temp3)
	{
		   //check device mode .
		if(pI2CHANDLE->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			//The device is master

			//RXNE flag is set
			if(pI2CHANDLE->TxRxState == I2C_BUSY_IN_RX)
		    {

			I2C_MasterHandleRXNEInterrupt(pI2CHANDLE);

		    }

		}else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(pI2CHANDLE->pI2Cx->SR2 & ( 1 << I2C_SR2_TRA)))
		    {

				I2C_AppEventCallBack(pI2CHANDLE,I2C_EV_DATA_RCV);

		    }
		}


	}



}



void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHANDLE)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHANDLE->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
	temp1 = (pI2CHANDLE->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHANDLE->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_AppEventCallBack(pI2CHANDLE,I2C_ERROR_BERR);
	}

	/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHANDLE->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHANDLE->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_AppEventCallBack(pI2CHANDLE,I2C_ERROR_ARLO);

	}

	/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHANDLE->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHANDLE->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_AppEventCallBack(pI2CHANDLE,I2C_ERROR_AF);
	}

	/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHANDLE->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHANDLE->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_AppEventCallBack(pI2CHANDLE,I2C_ERROR_OVR);
	}

	/***********************Check for Time out error************************************/
	temp1 = (pI2CHANDLE->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHANDLE->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_AppEventCallBack(pI2CHANDLE,I2C_ERROR_TIMEOUT);
	}

}











