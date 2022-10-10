/*
 * 007spi_cmd_handling.c
 *
 *  Created on: 15 Sep 2022
 *      Author: Victor
 */

/*
 * PB12 - SPI2_NSS
 * PB13 - SPI2_SCK
 * PB14 - SPI2_MISO
 * PB15 - SPI2_MOSI
 * Alt mode - 5
 */


#include <stdio.h>
#include <string.h>
#include "../drivers/Inc/stm32f407xx.h"

//Command codes
#define COMMAND_LED_CTRL			0x50
#define COMMAND_SENSOR_READ			0x51
#define COMMAND_LED_READ			0x52
#define COMMAND_PRINT				0x53
#define COMMAND_ID_READ				0x54

#define LED_ON						1
#define LED_OFF						0

//Arduino Analog Pins
#define ANALOG_PIN0					0
#define ANALOG_PIN1					1
#define ANALOG_PIN2					2
#define ANALOG_PIN3					3
#define ANALOG_PIN4					4
#define ANALOG_PIN5					5

#define LED_PIN						9

void delay()
{
	for(uint32_t volatile i = 0; i <500000; i++);
}


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

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
		return 1;
	else
		return 0;
}

void SPI2_GPIOInit()
{
	GPIO_Handle_t SPIPins;

	memset(&SPIPins,0,sizeof(SPIPins));

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PUSH_PULL;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NPU_NPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_VHIGH;
	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Init()
{
	SPI_Handle_t SPI2Handle;

	memset(&SPI2Handle,0,sizeof(SPI2Handle));

	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DIS; // Hardware Slave management

	SPI_Init(&SPI2Handle);

}

int main()
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;
	//char user_data[] = "Hello_world";
	//uint8_t dataLen = strlen(user_data);

	GPIO_ButtonInit();

	// this function is used to initialise GPIO pins to act as SPI2 pins
	SPI2_GPIOInit();

	// this function is used to initialise SPI2 peripheral parameters
	SPI2_Init();

	/*
	 * making SSOE bit 1 enables NSS output
	 * (when SPE=1 , NSS is pulled to low)
	 * (when SPE=0, NSS is pulled to high)
	 */
	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay();

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		// 1. CMD_LED_CTRL <pin no(1)> <value(1)>

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		SPI_SendData(SPI2, &commandcode,1);
		// dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//send a dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write,1);
		SPI_ReceiveData(SPI2, &ackbyte,1);

		if(SPI_VerifyResponse(ackbyte))
		{
			//send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2,args,2);
		}
		//end of command CMD_LED_CTRL

		// 2. CMD_SENSOR_READ <analog pin number(1)>
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0));

		delay();

		commandcode = COMMAND_SENSOR_READ;
		uint8_t Sensor_Data;

		SPI_SendData(SPI2, &commandcode,1);
		// dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//send a dummy byte to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write,1);
		SPI_ReceiveData(SPI2, &ackbyte,1);

		if(SPI_VerifyResponse(ackbyte))
		{
			//send arguments
			args[0] = ANALOG_PIN0;

			SPI_SendData(SPI2,args,1);
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//insert some delay so that slave can be ready with the data
			delay();

			SPI_SendData(SPI2, &dummy_write,1);
			SPI_ReceiveData(SPI2, &Sensor_Data, 1);
		}

		//end of command CMD_SENSOR_READ



		//lets confirm SPI is not busy
		while(SPI_GetFLagStatus(SPI2,SPI_BUSY_FLAG));

		// Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);
	}
	return 0;
}

