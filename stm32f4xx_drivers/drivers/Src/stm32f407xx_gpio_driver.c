/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 24 Aug 2022
 *      Author: nxf77381
 */


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx.h"



/****************************************************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 *****************************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDis)
{
	if (EnorDis == ENABLE)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_EN();
	}
	else
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_DIS();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_DIS();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_DIS();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_DIS();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_DIS();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_DIS();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_DIS();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_DIS();
		else if(pGPIOx == GPIOI)
			GPIOI_PCLK_DIS();
	}
}

/****************************************************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function takes the info from GPIO_PinConfig_t and writes into registers within pGPIOx structure
 * 					  - It clears the bitfields first then writes into them
 *
 * @param[in]         - base address of the GPIO Handle which contains the base address of GPIOx peripheral and user created GPIO_Pin_Config struct
 *
 * @return            -  none
 *
 * @Note              -  none
 *****************************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint64_t temp = 0;

	//0. Enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);

	//1. Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Non Interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(3 << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else
	{
		// Interrupt mode (To be done later)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FALL)
		{
			//Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_RISE)
		{
			//1. Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INT_FALL_RISE)
		{
			//1. Configure both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		SYSCFG_PCLK_EN();
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		SYSCFG->EXTICR[temp1] &= ~(0xF << (temp2 * 4));
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[temp1] |= (portcode << (temp2 * 4));


		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	//2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3 << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//3. Configure the PUPD settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4. Configure the OPType
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5. Configure the Alt Functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//Configure the alt func registers
		temp = ((uint64_t)pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->AFR &= ~(0xF << (4 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->AFR |= temp;
		temp = 0;
	}
}

/****************************************************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function resets the GPIO peripheral port registers to its default states
 *
 * @param[in]         - base address of the GPIO peripheral
 *
 * @return            -  none
 *
 * @Note              -  none
 *****************************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	else if(pGPIOx == GPIOC)
		GPIOC_REG_RESET();
	else if(pGPIOx == GPIOD)
		GPIOD_REG_RESET();
	else if(pGPIOx == GPIOE)
		GPIOE_REG_RESET();
	else if(pGPIOx == GPIOF)
		GPIOF_REG_RESET();
	else if(pGPIOx == GPIOG)
		GPIOG_REG_RESET();
	else if(pGPIOx == GPIOH)
		GPIOH_REG_RESET();
	else if(pGPIOx == GPIOI)
		GPIOI_REG_RESET();

}

/****************************************************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This reads and returns the data from a GPIO peripheral pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]  		  - pin number of the GPIO peripheral port
 *
 * @return            - 8 bit value
 *
 * @Note              -  none
 *****************************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);
	return value;
}
/****************************************************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This reads and returns the data from a GPIO peripheral port
 *
 * @param[in]         - base address of the GPIO peripheral
 *
 * @return            - 16 bit value
 *
 * @Note              -  none
 *****************************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = pGPIOx->IDR;
	return value;
}
/****************************************************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This writes data into a GPIO peripheral pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number of the GPIO peripheral port
 * @param[in]         - Value to be written
 *
 * @return            - none
 *
 * @Note              -  none
 *****************************************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
		pGPIOx->ODR |= (1 << PinNumber);
	else if (Value == GPIO_PIN_RESET)
		pGPIOx->ODR &= ~(1 << PinNumber);

}
/****************************************************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This writes data into a GPIO peripheral port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - Value to be written
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}
/****************************************************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This toggles GPIO peripheral port pin
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number of the GPIO peripheral port
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/****************************************************************************************************
 * @fn      		  - GPIO_IRQConfig
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis)
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// 1. FInd out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);
}

/****************************************************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - Clears the EXTI_PR register corresponding to the pin number
 *
 * @param[in]         - The pin number
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the EXTI_PR register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		EXTI->PR |= ( 1 << PinNumber);
	}

}

