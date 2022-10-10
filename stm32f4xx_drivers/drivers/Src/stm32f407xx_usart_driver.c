/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: 29 Sep 2022
 *      Author: Victor
 */

#include "stm32f407xx_usart_driver.h"


/****************************************************************************************************
 * @fn      		  - USART_GetFLagStatus
 *
 * @brief             - This function returns state of selected USART flag
 *
 * @param[in]         - base address of the USART peripheral
 * @param[in]         - Flagname macro
 *
 * @return            - Flag state (FLAG_SET/FLAG_RESET)
 *
 * @Note              - none
 *****************************************************************************************************/
uint8_t USART_GetFLagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	if (pUSARTx->SR & FlagName)
		return FLAG_SET;
	else
		return FLAG_RESET;
}

/****************************************************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             - This function clears the selected USART flag
 *
 * @param[in]         - base address of the USART peripheral
 * @param[in]         - Flagname macro
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	pUSARTx->SR &= ~FlagName;
}

/****************************************************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             - This function enables or disables USART peripheral
 *
 * @param[in]         - base address of the USART peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	else
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
}





/****************************************************************************************************
 * @fn      		  - USART_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given USART peripheral
 *
 * @param[in]         - base address of the USART peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDis)
{
	if (EnorDis == ENABLE)
	{
		if (pUSARTx == USART1)
			USART1_PCLK_EN();
		else if (pUSARTx == USART2)
			USART2_PCLK_EN();
		else if (pUSARTx == USART3)
			USART3_PCLK_EN();
		else if (pUSARTx == UART4)
			UART4_PCLK_EN();
		else if (pUSARTx == UART5)
			UART5_PCLK_EN();
		else if (pUSARTx == USART6)
			USART6_PCLK_EN();
	}
	else
	{
		if (pUSARTx == USART1)
			USART1_PCLK_DIS();
		else if (pUSARTx == USART2)
			USART2_PCLK_DIS();
		else if (pUSARTx == USART3)
			USART3_PCLK_DIS();
		else if (pUSARTx == UART4)
			UART4_PCLK_DIS();
		else if (pUSARTx == UART5)
			UART5_PCLK_DIS();
		else if (pUSARTx == USART6)
			USART6_PCLK_DIS();
	}
}


/****************************************************************************************************
 * @fn      		  - USART_IRQInterruptConfig
 *
 * @brief             - This function enables or disables Interrupt for NVIC peripheral corresponding
 * @brief			  - to IRQ number
 *
 * @param[in]         - IRQ position number
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis)
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
 * @fn      		  - USART_IRQPriorityConfig
 *
 * @brief             - This function sets the priority of an interrupt in the NVIC peripheral
 *
 * @param[in]         - IRQ position number
 * @param[in]         - IRQ priority
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);
}

