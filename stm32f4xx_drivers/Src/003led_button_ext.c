/*
 * 003led_button_ext.c
 *
 *  Created on: 26 Aug 2022
 *      Author: nxf77381
 */


#include "../drivers/Inc/stm32f407xx.h"

void delay()
{
	for(uint32_t volatile i = 0; i <500000; i++);
}

int main()
{
	GPIO_Handle_t GpioLed, GpioButton;

	//---------------------------------------------------------------
	// GPIO LED Pin Config Init
	//---------------------------------------------------------------
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PUSH_PULL;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NPU_NPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);

	//---------------------------------------------------------------
	// GPIO Button Pin Config Init
	//---------------------------------------------------------------
	GpioButton.pGPIOx = GPIOD;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INT_FALL;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	//GpioButton.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PUSH_PULL;
	//GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_HIGH;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULL_UP;


	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioButton);


	GPIO_IRQPriorityConfig(IRQ_EXTI9_5, 15)
	GPIO_IRQInterruptConfig(IRQ_EXTI9_5, ENABLE);


	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	while(1)
	{
		if(!GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_7))
		{
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_8);
			delay();
		}

	}
}
