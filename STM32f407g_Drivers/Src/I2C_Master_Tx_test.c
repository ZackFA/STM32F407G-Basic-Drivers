#include <stdio.h>
#include <string.h>
#include "stm32f407g.h"


#define MY_ADDRESS		0x61
#define SLAVE_ADDRESS	0x68 // CAN BE READ FROM ARDUINO SERIAL MONITOR


uint8_t data[] = "We are testing I2C Master TX\n"; // Don't send or receive more than 32 bytes in single I2C transaction for arduino


void delay(void)
{

	for( uint32_t i = 0; i < 500000/2 ; i++);

}

/*** SOME NOTES TO CONSIDER FOR THIS APPLICATION
 *
 * 		PB6 ----> SCL
 * 		PB9 ----> SDA
 *
 */




void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinCFG.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinCFG.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinCFG.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinCFG.GPIO_PinPupControl = GPIO_PU;
	I2CPins.GPIO_PinCFG.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK PIN
	I2CPins.GPIO_PinCFG.GPIO_PinNum = GPIO_PIN_NUM6;
	GPIO_Init(&I2CPins);

	// SDA PIN
	I2CPins.GPIO_PinCFG.GPIO_PinNum = GPIO_PIN_NUM9;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void)
{

	I2C_Handle_t I2C1Handle;

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_CFG.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_CFG.I2C_DeviceAddress = MY_ADDRESS;
	I2C1Handle.I2C_CFG.I2C_FMDutyCycle = I2C_FMDUTY_2;
	I2C1Handle.I2C_CFG.I2C_SCLSpeed = I2C_SCL_SPEED_SM; // 100Khz

	I2C_Init(&I2C1Handle);

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

int main(void)
{

	I2C_Handle_t I2C1Handle;

	// Button Pin Init
	GPIO_ButtonInit();

	// I2C Pin Inits
	I2C1_GPIOInits();

	// I2C Peripheral Inits
	I2C1_Inits();

	// Enable the I2C Peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	// Wait for button press
	while(1)
	{

		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NUM0) ); // wait until button is pressed

		delay(); // to avoid debounce related issues

		// Send data to the slave
		I2C_MasterSend(&I2C1Handle,data,strlen((char*)data), SLAVE_ADDRESS);

	}



}
