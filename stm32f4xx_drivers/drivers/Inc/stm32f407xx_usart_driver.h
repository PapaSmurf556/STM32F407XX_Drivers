/*
 * stm32f407xx_usart_driver.h
 *
 *  Created on: 29 Sep 2022
 *      Author: Victor
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for USART peripheral
 */
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baudrate;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

/*
 * Handle structure for USART peripheral
 */
typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
}USART_Handle_t;

/*
 * USART related status flags definitions
 */
#define USART_FLAG_PE			(1 << USART_SR_PE)
#define USART_FLAG_FE			(1 << USART_SR_FE)
#define USART_FLAG_NF			(1 << USART_SR_NF)
#define USART_FLAG_ORE			(1 << USART_SR_ORE)
#define USART_FLAG_IDLE			(1 << USART_SR_IDLE)
#define USART_FLAG_RXNE			(1 << USART_SR_RXNE)
#define USART_FLAG_TC			(1 << USART_SR_TC)
#define USART_FLAG_TXE			(1 << USART_SR_TXE)
#define USART_FLAG_LBD			(1 << USART_SR_LBD)
#define USART_FLAG_CTS			(1 << USART_SR_CTS)



// ----------------------------------------APIs supported by this driver---------------------------- ------------

/*
 * Peripheral Clock Setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDis);

/*
 * Init and De-Init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


/*
 * Data Send and Receive
 */
void USART_TX(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer,uint32_t Len);
void USART_RX(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer,uint32_t Len);

/*
 * Interrupt Configuration and ISR Handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void USART_EV_IRQHandling(USART_Handle_t *pUSARTHandle);
void USART_ER_IRQHandling(USART_Handle_t *pUSARTHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDis);
uint8_t USART_GetFLagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t FlagName);


#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
