#include "stm32f407g.h"
#include "stm32f407g_SPI_driver.h"
#include <string.h>

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinCFG.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinCFG.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinCFG.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinCFG.GPIO_PinPupControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinCFG.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCL PIN
	SPIPins.GPIO_PinCFG.GPIO_PinNum = GPIO_PIN_NUM13;
	GPIO_Init(&SPIPins);

	// MOSI PIN
	SPIPins.GPIO_PinCFG.GPIO_PinNum = GPIO_PIN_NUM15;
	GPIO_Init(&SPIPins);

	// MISO PIN
	SPIPins.GPIO_PinCFG.GPIO_PinNum = GPIO_PIN_NUM14;
	GPIO_Init(&SPIPins);

	// NSS PIN
	SPIPins.GPIO_PinCFG.GPIO_PinNum = GPIO_PIN_NUM12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{

	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPICFG.SPI_BusCFG = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPICFG.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPICFG.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPICFG.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPICFG.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPICFG.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPICFG.SPO_SSM = SPI_SSM_EN; // Software slave management enabled for NSS

	SPI_Init(&SPI2Handle);

}

int main()
{

	char user_data[] = "Hello World";

	SPI2_GPIOInits(); // Initialize the GPIO pins to behave as SPI2 pins
	SPI2_Inits(); // Initialize SPI2 Perpipheral parameters
	SPI_SSIConfig(SPI2,ENABLE);
	// Enable SPI2 peripheral before sending data
	SPI_PeripheralControl(SPI2,ENABLE);

	SPI_Send(SPI2, (uint8_t*)user_data, strlen(user_data));

	// Confirm that the SPI is not busy
	while( SPI_GetFlagStat(SPI2,SPI_BUSY_FLAG) ); // if it returns 1 then SPI is busy, if it returns 0 then the loop will break and it will disable the peripheral after data sending

	// Disable the SPI2 peripheral after sending data
	SPI_PeripheralControl(SPI2,DISABLE);


	while(1);

	return 0;

}
