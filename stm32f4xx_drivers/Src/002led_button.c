/*
 * 002led_button.c
 *
 *  Created on: 25 Aug 2022
 *      Author: nxf77381
 */


#include "../drivers/Inc/stm32f407xx.h"



int main()
{
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioButton;

	//---------------------------------------------------------------
	// GPIO LED Pin Config Init
	//---------------------------------------------------------------
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PUSH_PULL;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NPU_NPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_HIGH;

	//---------------------------------------------------------------
	// GPIO Button Pin Config Init
	//---------------------------------------------------------------
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NPU_NPD;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_HIGH;

	//---------------------------------------------------------------
	// GPIO Peripheral Clocks Enable
	//---------------------------------------------------------------
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOD, ENABLE);


	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0))
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		else
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	}
}
