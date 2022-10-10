/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: 24 Aug 2022
 *      Author: nxf77381
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

// ------------------------------------------------------------------------------
// Gonfiguration Structure for GPIO pin
// ------------------------------------------------------------------------------


typedef struct
{
	uint8_t GPIO_PinNumber;				/*!< possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;				/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;				/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;		/*!< possible values from @GPIO_PIN_PU_PD_CONFIG >*/
	uint8_t GPIO_PinOPType;				/*!< possible values from @GPIO_PIN_OUT_TYPE >*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

// ------------------------------------------------------------------------------
// Handle Structure for GPIO pin
// ------------------------------------------------------------------------------

typedef struct
{
	GPIO_RegDef_t *pGPIOx; 				// This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;		// This holds GPIO pin configuration settings
}GPIO_Handle_t;

// ------------------------------------------------------------------------------
// @GPIO_PIN_NUMBERS
// GPIO pin numbers
// ------------------------------------------------------------------------------

#define GPIO_PIN_0					0
#define GPIO_PIN_1					1
#define GPIO_PIN_2					2
#define GPIO_PIN_3					3
#define GPIO_PIN_4					4
#define GPIO_PIN_5					5
#define GPIO_PIN_6					6
#define GPIO_PIN_7					7
#define GPIO_PIN_8					8
#define GPIO_PIN_9					9
#define GPIO_PIN_10					10
#define GPIO_PIN_11					11
#define GPIO_PIN_12					12
#define GPIO_PIN_13					13
#define GPIO_PIN_14					14
#define GPIO_PIN_15					15

// ------------------------------------------------------------------------------
// @GPIO_PIN_MODES
// GPIO pin possible modes
// ------------------------------------------------------------------------------

#define GPIO_MODE_IN 				0
#define GPIO_MODE_OUT 				1
#define GPIO_MODE_ALTFN 			2
#define GPIO_MODE_ANALOG 			3
#define GPIO_MODE_INT_FALL			4
#define GPIO_MODE_INT_RISE			5
#define GPIO_MODE_INT_FALL_RISE		6

// ------------------------------------------------------------------------------
// @GPIO_PIN_OUT_TYPE
// GPIO pin possible output types
// ------------------------------------------------------------------------------

#define GPIO_OUT_TYPE_PUSH_PULL		0
#define GPIO_OUT_TYPE_OPEN_DRAIN	1

// ------------------------------------------------------------------------------
// @GPIO_PIN_SPEED
// GPIO pin possible output speeds
// ------------------------------------------------------------------------------

#define GPIO_OUT_SPEED_LOW			0
#define GPIO_OUT_SPEED_MED			1
#define GPIO_OUT_SPEED_HIGH			2
#define GPIO_OUT_SPEED_VHIGH		3


// ------------------------------------------------------------------------------
// @GPIO_PIN_PU_PD_CONFIG
// GPIO pin possible Pull-Up/Pull-Down configurations
// ------------------------------------------------------------------------------

#define GPIO_NPU_NPD		0
#define GPIO_PULL_UP		1
#define GPIO_PULL_DOWN		2


// ---------------------------------------------------------------APIs supported by this driver------------------------------------------------------------------------------------


// ------------------------------------------------------------------------------
// Peripheral Clock Setup
// ------------------------------------------------------------------------------

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDis);

// ------------------------------------------------------------------------------
// Init and De-init
// ------------------------------------------------------------------------------
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// ------------------------------------------------------------------------------
// Data Read and Write
// ------------------------------------------------------------------------------

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// ------------------------------------------------------------------------------
// IRQ Configuration and ISR handling
// ------------------------------------------------------------------------------

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
