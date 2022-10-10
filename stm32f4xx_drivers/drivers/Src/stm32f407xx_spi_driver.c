/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 8 Sep 2022
 *      Author: Victor
 */

#include "stm32f407xx_spi_driver.h"

//private helper function prototypes
static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handler(SPI_Handle_t *pSPIHandle);

uint8_t SPI_GetFLagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SPI_SR & FlagName)
		return FLAG_SET;
	else
		return FLAG_RESET;
}

/****************************************************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 *****************************************************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDis)
{
	if (EnorDis == ENABLE)
	{
		if (pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if (pSPIx == SPI2)
			SPI2_PCLK_EN();
		else if (pSPIx == SPI3)
			SPI3_PCLK_EN();
	}
	else
	{
		if (pSPIx == SPI1)
			SPI1_PCLK_DIS();
		else if (pSPIx == SPI2)
			SPI2_PCLK_DIS();
		else if (pSPIx == SPI3)
			SPI3_PCLK_DIS();
	}
}

/****************************************************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function takes the info from SPI_PinConfig_t and writes into registers within pSPIx structure
 *
 * @param[in]         - base address of the SPI Handle which contains the base address of SPIx peripheral and user created SPI_Pin_Config struct
 *
 * @return            -  none
 *
 * @Note              -  none
 *****************************************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//0. Enable peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);

	//first lets configure SPI_CR1 register

	uint32_t tempreg = 0;

	//1. Configure the device mode
	tempreg |= (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	//2. Configure the bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);

	}

	//3. Configure the SCLK speed
	//Reset
	tempreg &= ~(7 << SPI_CR1_BR);
	//Set
	tempreg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);

	//4. Configure the DFF (shift register width)
	tempreg |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	//5. Configure the CPOL
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	//6. Configure the CPHA
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

	//6. Configure the SSM
	tempreg |= (pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->SPI_CR1 = tempreg;

}

/****************************************************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function resets the SPI peripheral registers to its default states
 *
 * @param[in]         - base address of the SPI peripheral
 *
 * @return            -  none
 *
 * @Note              -  none
 *****************************************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		SPI1_REG_RESET();
	else if(pSPIx == SPI2)
		SPI2_REG_RESET();
	else if(pSPIx == SPI3)
		SPI3_REG_RESET();
}

/****************************************************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - Sends data (Blocking API)
 *
 * @param[in]         - (uint32_t *)base address of the SPI peripheral
 * @param[in]  		  - (uint8_t *) base address of the TxBuffer
 * @param[in]  		  - (uint32_t) Length of the TxBuffer
 *
 * @return            - none
 *
 * @Note              - This is blocking call
 *****************************************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. Wait until TXE is set
		while( SPI_GetFLagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check DFF bit
		if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. Load the data into DR
			pSPIx->SPI_DR = *((uint16_t *) pTxBuffer);
			//2. Decrease the length 2 times
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			//8 bit DFF
			//1. Load the data into DR
			pSPIx->SPI_DR = *pTxBuffer;
			//2. Decrease the length 1 time
			Len--;
			pTxBuffer++;
		}
	}
}

/****************************************************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             - Sends data (Non-Blocking API)
 *
 * @param[in]         - (uint32_t *)base address of the SPI Handle
 * @param[in]  		  - (uint8_t *) base address of the TxBuffer
 * @param[in]  		  - (uint32_t) Length of the TxBuffer
 *
 * @return            - none
 *
 * @Note              - This is a non-blocking call
 *****************************************************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that no other code can take over
		//   same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);

		//4. Data Transmission will be handled by the ISR code (will implement later)
	}
	return state;
}

/****************************************************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - Receives data (Blocking API)
 *
 * @param[in]         - (uint32_t *)base address of the SPI peripheral
 * @param[in]  		  - (uint8_t *) base address of the RxBuffer
 * @param[in]  		  - (uint32_t) Length of the RxBuffer
 *
 * @return            - none
 *
 * @Note              - This is blocking call
 *****************************************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. Wait until RXNE is set
		while( SPI_GetFLagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);

		//2. Check DFF bit
		if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. Load the data from DR to RxBuffer address
			*((uint16_t *) pRxBuffer) = pSPIx->SPI_DR;
			//2. Decrease the length 2 times
			Len--;
			Len--;
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			//8 bit DFF
			//1. Load the data from DR to RxBuffer address
			*pRxBuffer = pSPIx->SPI_DR;
			//2. Decrease the length 1 time
			Len--;
			pRxBuffer++;
		}
	}
}

/****************************************************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             - Receives data (Non-Blocking API)
 *
 * @param[in]         - (uint32_t *)base address of the SPI Handle
 * @param[in]  		  - (uint8_t *) base address of the RxBuffer
 * @param[in]  		  - (uint32_t) Length of the RxBuffer
 *
 * @return            - none
 *
 * @Note              - This is a non-blocking call
 *****************************************************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

		if(state != SPI_BUSY_IN_RX)
		{
			//1. Save the Rx buffer address and Len information in some global variables
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = Len;

			//2. Mark the SPI state as busy in reception so that no other code can take over
			//   same SPI peripheral until reception is over
			pSPIHandle->RxState = SPI_BUSY_IN_RX;

			//3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR
			pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);

			//4. Data Reception will be handled by the ISR code (will implement later)
		}
		return state;
}

/****************************************************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             - This enables interrupt on NVIC peripheral and sets the priority
 *
 * @param[in]         - The IRQ number of the interrupt
 * @param[in]         - Enable or Disable boolean
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber >= 32 && IRQNumber <= 63)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber <= 95)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber  % 64);
		}
	}
	else if(EnorDis == DISABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber >= 32 && IRQNumber <= 63)
		{
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber <= 95)
		{
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}

}
/****************************************************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - This sets the priority of an interrupt
 *
 * @param[in]         - The IRQ number of the interrupt
 * @param[in]         - The priority of the interrupt
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// 1. FInd out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);
}

/****************************************************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - Clears the EXTI_PR register corresponding to the pin number
 *
 * @param[in]         - The pin number
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	//1. Check for TXE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1<< SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1<< SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handler(pSPIHandle);
	}

	//2. Check for RXNE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1<< SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1<< SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handler(pSPIHandle);
	}
	//3. Check for OVR flag
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1<< SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1<< SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		//handle RXNE
		spi_ovr_err_interrupt_handler(pSPIHandle);
	}
}
/****************************************************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - Enables or Disables SPI peripheral
 *
 * @param[in]         - base address of SPI peripheral
 * @param[in]		  - ENABLE or DISABLE macro
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
	else if(EnorDis == DISABLE)
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);

}

/****************************************************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - Enables or Disables SPI peripheral SSI bit
 *
 * @param[in]         - base address of SPI peripheral
 * @param[in]		  - ENABLE or DISABLE macro
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
	else if(EnorDis == DISABLE)
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
}

/****************************************************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             - Enables or Disables SPI peripheral SSOE bit
 *
 * @param[in]         - base address of SPI peripheral
 * @param[in]		  - ENABLE or DISABLE macro
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
		pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);
	else if(EnorDis == DISABLE)
		pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
}

static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
	//Check DFF bit
	if (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		//1. Load the data into DR
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t *) pSPIHandle->pTxBuffer);
		//2. Decrease the length 2 times
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8 bit DFF
		//1. Load the data into DR
		pSPIHandle->pSPIx->SPI_DR = *(pSPIHandle->pTxBuffer);
		//2. Decrease the length 1 time
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen)
	{
		// If TxLen is zero, close the spi transmission and inform application that TX is over

		//this prevents interrupts from setting up the TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}

//private helper function implementation
static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
	//Check DFF bit
	if (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		//1. Read data from DR
		*((uint16_t *) pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->SPI_DR;
		//2. Decrease the length 2 times
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t *)pSPIHandle->pRxBuffer++;
	}
	else
	{
		//8 bit DFF
		//1.  Read data from DR
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->SPI_DR;
		//2. Decrease the length 1 time
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(!pSPIHandle->RxLen)
	{
		// If TxLen is zero, close the spi transmission and inform application that TX is over

		//this prevents interrupts from setting up the TXE flag
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}

static void spi_ovr_err_interrupt_handler(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. Clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp;

	//2. Inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;
	(void)temp;
}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t Application_Event)
{
	// This is a weak implementation
}

