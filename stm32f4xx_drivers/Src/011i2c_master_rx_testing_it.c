/*
 * 011i2c_master_rx_testing_it.c
 *
 *  Created on: 26 Sep 2022
 *      Author: Victor
 */

#include<stdio.h>
#include<string.h>
#include "../drivers/Inc/stm32f407xx.h"

#define MY_ADDR				0x61
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

uint8_t some_data[500];

uint8_t len;

uint8_t command1 = 0x51;
uint8_t command2 = 0x52;

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
	I2C1Handle.Mode = I2C_MODE_MASTER;
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

	while(1)
	{
		//Wait for button press
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay();


		//Send COMMAND 1 BYTE
		//I2C_MasterSendData(&I2C1Handle, &command1, 1 ,SLAVE_ADDR,I2C_STOP);
		while(I2C_MasterSendDataIT(&I2C1Handle, &command1, 1 ,SLAVE_ADDR,I2C_NO_STOP) != I2C_READY);

		//I2C_delay();

		//Get length
		while(I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR,I2C_NO_STOP) != I2C_READY);

		printf("The length of the message is - %u\n",len);

		//Send COMMAND 2 BYTE
		while(I2C_MasterSendDataIT(&I2C1Handle, &command2, 1 ,SLAVE_ADDR,I2C_NO_STOP) != I2C_READY);

		//I2C_delay();

		//Get all data
		while(I2C_MasterReceiveDataIT(&I2C1Handle, some_data, 2, SLAVE_ADDR,I2C_STOP) != I2C_READY);

		printf("Communication is complete\n");

	}

	return 0;
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t Application_Event)
{
	if(Application_Event == I2C_EV_TX_CMPLT)
	{
		printf("TX is complete\n");
	}else if(Application_Event == I2C_EV_RX_CMPLT)
	{
		printf("RX is complete\n");
	}else if(Application_Event == I2C_ERROR_AF)
	{
		printf("ERROR : ACK failure\n");
		//in master ACK failure happens when slave fails to send ACK for a byte
		//sent from the master.

		//Generate STOP condition
		I2C_GenerateStopCondition(I2C1Handle.pI2Cx);

		//Hang in infinite loop
		while(1);

	}else if(Application_Event == I2C_ERROR_ARLO)
	{

	}else if(Application_Event == I2C_ERROR_BERR)
	{

	}else if(Application_Event == I2C_ERROR_OVR)
	{

	}else if(Application_Event == I2C_ERROR_TIMEOUT)
	{

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
