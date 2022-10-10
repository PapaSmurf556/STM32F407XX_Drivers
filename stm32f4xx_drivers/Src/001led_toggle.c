/*
 * 001led_toggle.c
 *
 *  Created on: 25 Aug 2022
 *      Author: nxf77381
 */

#include "../drivers/Inc/stm32f407xx.h"


void delay()
{
	for(uint32_t volatile i = 0; i <500000; i++);


}

int main()
{
	GPIO_Handle_t GpioLed;


	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PUSH_PULL;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NPU_NPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_HIGH;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
		delay();
	}
}
