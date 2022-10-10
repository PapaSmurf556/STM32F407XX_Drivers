/*
 * 009i2c_tx_testing.c
 *
 *  Created on: 19 Sep 2022
 *      Author: Victor
 */

#include<stdio.h>
#include<string.h>
#include "../drivers/Inc/stm32f407xx.h"

#define MY_ADDR			0x61
#define SLAVE_ADDR		0x68
void delay()
{
	for(uint32_t volatile i = 0; i <500000; i++);
}

I2C_Handle_t I2C1Handle;

uint8_t some_data[] = "We are testing I2C master Tx\n"; // Arduino Wire library limits how many bytes can be received or transmitted in one go to 32 bytes

void GPIO_ButtonInit()
{
	GPIO_Handle_t GpioButton;
	memset(&GpioButton,0,sizeof(GpioButton));
	//---------------------------------------------------------------
	// GPIO Button Pin Config Init
	//---------------------------------------------------------------
	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NPU_NPD;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_HIGH;

	GPIO_Init(&GpioButton);
}

void I2C1_GPIOInit()
{
	GPIO_Handle_t I2CBus;
	memset(&I2CBus,0,sizeof(I2CBus));


	I2CBus.pGPIOx = GPIOB;
	I2CBus.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CBus.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_OPEN_DRAIN;
	I2CBus.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULL_UP;
	I2CBus.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CBus.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_HIGH;

	// SCL
	I2CBus.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CBus);

	// SDA
	I2CBus.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_Init(&I2CBus);
}

void I2C1_Init()
{
	memset(&I2C1Handle,0,sizeof(I2C1Handle));

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;		// doesn't matter (not slave)
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;  // doesn't matter
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

int main()
{
	//1. Initialise GPIO button
	GPIO_ButtonInit();

	//2. Initialise I2C1 GPIO pins
	I2C1_GPIOInit();

	//3. Initialise I2C1 peripheral
	I2C1_Init();

	//4. Enable I2C1 peripheral
	I2C_PeripheralControl(&I2C1Handle, ENABLE);

	while(1)
	{
		//5. Wait for button press
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay();

		//5. Send some data
		I2C_MasterSendData(&I2C1Handle, some_data, strlen((char *)some_data),SLAVE_ADDR);

	}

	return 0;
}
