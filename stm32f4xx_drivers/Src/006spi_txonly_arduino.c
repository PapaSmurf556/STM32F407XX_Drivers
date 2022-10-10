/*
 * 006spi_txonly_arduino.c
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

void delay()
{
	for(uint32_t volatile i = 0; i <500000; i++);
}


void UserButton_GPIOInit()
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

	//There are no slaves, so we can choose not to use NSS and MISO and not initialise them

	/*
	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIPins);
	*/
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
	char user_data[] = "Hello_world";
	uint8_t dataLen = strlen(user_data);

	UserButton_GPIOInit();

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

		//first send length information
		SPI_SendData(SPI2, &dataLen,1);

		SPI_SendData(SPI2, (uint8_t *) user_data, strlen(user_data));

		//lets confirm SPI is not busy
		while(SPI_GetFLagStatus(SPI2,SPI_BUSY_FLAG));

		// Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);
	}
	return 0;
}
