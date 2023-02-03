#include "stm32f407g.h"
#include "stm32f407g_SPI_driver.h"
#include <string.h>

/** MACROS TO BE USED IN THE FILE **/

#define CMD_LED_CTRL	0x50
#define CMD_SENSOR_READ 0x51
#define CMD_LED_READ	0x52
#define CMD_PRINT		0x53
#define CMD_ID_READ		0x54

#define LED_ON			1
#define LED_OFF			0

#define ANALOG_PIN0		0
#define ANALOG_PIN1		1
#define ANALOG_PIN2		2
#define ANALOG_PIN3		3
#define ANALOG_PIN4		4

#define LED_PIN			9

void delay(void)
{

	for( uint32_t i = 0; i < 500000/2 ; i++);

}



void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinCFG.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinCFG.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinCFG.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinCFG.GPIO_PinPupControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinCFG.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK PIN
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
	SPI2Handle.SPICFG.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV8; // SCLK of 2MHz
	SPI2Handle.SPICFG.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPICFG.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPICFG.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPICFG.SPO_SSM = SPI_SSM_DI; // Hardware slave management enabled for NSS

	SPI_Init(&SPI2Handle);

}


void GPIO_ButtonInit(void)
{

	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinCFG.GPIO_PinNum = GPIO_PIN_NUM0;
	GPIOBtn.GPIO_PinCFG.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinCFG.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinCFG.GPIO_PinPupControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{

	if( ackbyte == 0xF5)
	{
		//ack
		return 1;

	}

	return 0;
}



int main()
{

	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	GPIO_ButtonInit(); // Initialize pin num 0 for a button input

	SPI2_GPIOInits(); // Initialize the GPIO pins to behave as SPI2 pins
	SPI2_Inits(); // Initialize SPI2 Perpipheral parameters

	SPI_SSOEConfig(SPI2,ENABLE); // Make SSOE 1 does NSS output enable. When SPE = 1 nss will be low, when SPE = 0, NSS wil be high

	while(1)
	{

		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NUM0) ); // wait until button is pressed

		delay(); // to avoid debounce related issues

		// Enable SPI2 peripheral before sending data
		SPI_PeripheralControl(SPI2,ENABLE);

		// 1. Command number one : CMD_LED_CTRL pin no(1) value(1)
		uint8_t cmdCode = CMD_LED_CTRL;
		uint8_t ackByte;
		uint8_t args[2];

		// Send command
		SPI_Send(SPI2,&cmdCode,1);
		// Do a dummy read to clear off RXNE
		SPI_Recieve(SPI2,&dummy_read,1);
		// Send dummy byte to fetch the response from the slave
		SPI_Send(SPI2,&dummy_write,1);
		// Read the ack byte
		SPI_Recieve(SPI2,&ackByte,1);

		if( SPI_VerifyResponse(ackByte) )
		{

			// Send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_Send(SPI2,args,1);

		}

		// 2. Command number two : CMD_SENSOR_READ analog pin number(1)

		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NUM0) ); // wait until button is pressed

		delay(); // to avoid debounce related issues

		cmdCode = CMD_SENSOR_READ;

		// Send command
		SPI_Send(SPI2,&cmdCode,1);
		// Do a dummy read to clear off RXNE
		SPI_Recieve(SPI2,&dummy_read,1);
		// Send dummy byte to fetch the response from the slave
		SPI_Send(SPI2,&dummy_write,1);
		// Read the ack byte
		SPI_Recieve(SPI2,&ackByte,1);

		if( SPI_VerifyResponse(ackByte) )
		{

			// Send arguments
			args[0] = ANALOG_PIN0;
			SPI_Send(SPI2,args,1);
		}

		// Do a dummy read to clear off RXNE
		SPI_Recieve(SPI2,&dummy_read,1);
		// Send dummy byte to fetch the response from the slave
		SPI_Send(SPI2,&dummy_write,1);

		delay(); // Insert some delay in order to give the slave time to be ready with the data

		uint8_t analogRead;
		// Read the analog value
		SPI_Recieve(SPI2,&analogRead,1);


		// Confirm that the SPI is not busy
		while( SPI_GetFlagStat(SPI2,SPI_BUSY_FLAG) ); // If it returns 1 then SPI is busy, if it returns 0 then the loop will break and it will disable the peripheral after data sending

		// Disable the SPI2 peripheral after sending data
		SPI_PeripheralControl(SPI2,DISABLE);
	}


	return 0;

}
