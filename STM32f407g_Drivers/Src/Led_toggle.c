#include "stm32f407g.h"



void delay(void)
{
	for (uint32_t i = 0; i < 50000; i++);
}



int main(void)
{

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinCFG.GPIO_PinNum = GPIO_PIN_NUM12;
	GpioLed.GPIO_PinCFG.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinCFG.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinCFG.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinCFG.GPIO_PinPupControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&GpioLed);

	while(1){

		GPIO_TogglePin(GPIOD,GPIO_PIN_NUM12);
		delay();
	}

}
