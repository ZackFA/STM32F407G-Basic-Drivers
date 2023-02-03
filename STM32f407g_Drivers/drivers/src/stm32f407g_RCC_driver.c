#include "stm32f407g_RCC_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 32, 64, 128, 256, 512};
uint16_t APB1_PreScaler[4] = {2, 4, 8, 16};

uint32_t RCC_GetPCLK1Value(void)
{

	uint32_t pClk1, SystemClk;

	uint8_t	ClkSource, temp, ahbp, apb1p;

	ClkSource = (RCC->CFGR >> 2) & 0x3; // Mask out all other bits except bit 0 and 1 where our value is stored

	if( ClkSource == 0)
	{

		SystemClk = 16000000;

	}else if( ClkSource == 1)
	{

		SystemClk = 8000000;

	}else if( ClkSource == 2)
	{

		SystemClk = RCC_GetPLLOutputClock();

	}

	// for AHB
	temp = ((RCC->CFGR >> 4) & 0xF);

	if( temp < 8 )
	{

		ahbp = 1;

	}else
	{

		ahbp = AHB_PreScaler[temp-8];

	}

	// for APB1
	temp = ((RCC->CFGR >> 10) & 0x7);

		if( temp < 4 )
		{

			apb1p = 1;

		}else
		{

			apb1p = AHB_PreScaler[temp-4];

		}

	pClk1 = ( SystemClk / ahbp ) / apb1p;

	return pClk1;

}

uint32_t RCC_GetPCLK2Value(void)
{

	uint32_t pClk2, SystemClk = 0;

	uint8_t	ClkSource, temp, ahbp, apb1p;

	ClkSource = (RCC->CFGR >> 2) & 0x3; // Mask out all other bits except bit 0 and 1 where our value is stored

	if( ClkSource == 0)
	{

		SystemClk = 16000000;

	}else if( ClkSource == 1)
	{

		SystemClk = 8000000;

	}

	// for AHB
	temp = ((RCC->CFGR >> 4) & 0xF);

	if( temp < 0x08 )
	{

		ahbp = 1;

	}else
	{

		ahbp = AHB_PreScaler[temp-8];

	}

	// for APB1
	temp = ((RCC->CFGR >> 13) & 0x7);

		if( temp < 0x04 )
		{

			apb1p = 1;

		}else
		{

			apb1p = AHB_PreScaler[temp-4];

		}

	pClk2 = ( SystemClk / ahbp ) / apb1p;

	return pClk1;

}

uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}
