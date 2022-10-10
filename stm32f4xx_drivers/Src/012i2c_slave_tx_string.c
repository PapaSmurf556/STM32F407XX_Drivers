/*
 * 012i2c_slave_tx_string.c
 *
 *  Created on: 28 Sep 2022
 *      Author: Victor
 */

#include<stdio.h>
#include<string.h>
#include "../drivers/Inc/stm32f407xx.h"

#define MY_ADDR				0x69
#define SLAVE_ADDR			0x68



void delay()
{
	for(uint32_t volatile i = 0; i <500000; i++);
}

void I2C_delay()
{
	for(uint32_t volatile i = 0; i <1; i++);
}


I2C_Handle_t I2C1Handle;

uint8_t const buffer[] = "hihihihihihihhihiihihihiihihihihihihihhihiihihihiihihihihihihihhihiihihihiihihihihihihihhihiihihihiihihihihihihihhihiihihihiihihihihihihihhihiihihihiihihihihihihihhihiihihihiihihihihihihihhihiihihihiihihihihihihihhihiihihihiihihihihihihihhihi\n";


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
	I2CBus.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_MED;

	// SCL
	I2CBus.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CBus);

	// SDA
	I2CBus.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
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

	I2C_IRQInterruptConfig(IRQ_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_I2C1_ER, ENABLE);

	//4. Enable I2C1 peripheral
	I2C_PeripheralControl(&I2C1Handle, ENABLE);

	I2C_SlaveCallbackControl(&I2C1Handle, ENABLE);

	while(1)
	{

	}

	return 0;
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t Application_Event)
{
	static uint8_t len = strlen((char *) buffer);

	static uint8_t count = 0;

	static uint8_t command = 0;

	if(Application_Event == I2C_EV_TX_CMPLT)
	{

	}else if(Application_Event == I2C_EV_RX_CMPLT)
	{

	}else if(Application_Event == I2C_EV_STOP)
	{

	}else if(Application_Event == I2C_ERROR_AF)
	{
		if(command == 0x52)
			count = 0;
		command = 0;
	}else if(Application_Event == I2C_ERROR_ARLO)
	{

	}else if(Application_Event == I2C_ERROR_BERR)
	{

	}else if(Application_Event == I2C_ERROR_OVR)
	{

	}else if(Application_Event == I2C_ERROR_TIMEOUT)
	{

	}else if(Application_Event == I2C_EV_DATA_REQ)
	{
		if(command == 0x51)
		{
			I2C_SlaveSendData(I2C1Handle.pI2Cx, len);
		}
		else if(command == 0x52)
		{
			I2C_SlaveSendData(I2C1Handle.pI2Cx, buffer[count]);
			count++;
		}
	}else if(Application_Event == I2C_EV_DATA_RCV)
	{
		command = I2C_SlaveReceiveData(I2C1Handle.pI2Cx);
	}
}



void I2C1_EV_IRQHandler()
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler()
{
	I2C_ER_IRQHandling(&I2C1Handle);
}


