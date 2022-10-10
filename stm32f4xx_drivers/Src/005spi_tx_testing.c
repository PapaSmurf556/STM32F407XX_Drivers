/*
 * 005spi_tx_testing.c
 *
 *  Created on: 10 Sep 2022
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

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);
	*/
}

void SPI2_Init()
{
	SPI_Handle_t SPI2Handle;

	memset(&SPI2Handle,0,sizeof(SPI2Handle));

	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN; // software slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);

}

int main()
{
	char user_data[] = "Hello_world";

	// this function is used to initialise GPIO pins to act as SPI2 pins
	SPI2_GPIOInit();

	// this function is used to initialise SPI2 peripheral parameters
	SPI2_Init();

	// this function enables the SSI bit (since Software management is active, SSI is tied to NSS bit, so pulling it to Vcc allows to be in master mode)
	SPI_SSIConfig(SPI2,ENABLE);

	// enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	SPI_SendData(SPI2, (uint8_t *) user_data, strlen(user_data));

	//lets confirm SPI is not busy
	while(SPI_GetFLagStatus(SPI2,SPI_BUSY_FLAG));

	SPI_SSIConfig(SPI2,DISABLE);

	SPI_PeripheralControl(SPI2,DISABLE);
	while(1);
}
