/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 17 Sep 2022
 *      Author: Victor
 */

#include "stm32f407xx_i2c_driver.h"

void I2C_SlaveCallbackControl(I2C_Handle_t *pI2CHandle, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		// Enable Event, Error and Buffer interrupts
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2CHandle->Mode = I2C_MODE_SLAVE;
	}
	else if(EnorDis == DISABLE)
	{
		// Disable Event, Error and Buffer interrupts
		pI2CHandle->pI2Cx->I2C_CR2 &= (1 << I2C_CR2_ITEVTEN);
		pI2CHandle->pI2Cx->I2C_CR2 &= (1 << I2C_CR2_ITERREN);
		pI2CHandle->pI2Cx->I2C_CR2 &= (1 << I2C_CR2_ITBUFEN);
		pI2CHandle->Mode = I2C_MODE_MASTER;
	}
}

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
	else if(EnorDis == DISABLE)
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
}
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;
	pI2Cx->I2C_DR = SlaveAddr;
}


static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->I2C_DR = SlaveAddr;
}

static void I2C_CLearADDRFLag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead;
	//check for device mode
	if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX && pI2CHandle->RxSize == 1)
		{
			//first disable the ACKing and then clear the ADDR flag
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

			//CLear ADDR flag (Read SR1 and SR2)
			dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
			dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
			(void)dummyRead;
		}
		else
		{
			//CLear ADDR flag (Read SR1 and SR2)
			dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
			dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
			(void)dummyRead;
		}
	}
	else
	{
		//device is in slave mode

		//CLear ADDR flag (Read SR1 and SR2)
		dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
		dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
		(void)dummyRead;
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVTEN Control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVTEN Control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

//uint32_t RCC_GetPLLOutputClock(){} //TBD

uint32_t RCC_GetPCLK1Value()
{
	uint8_t clksource;
	uint8_t AHBPrescaler;
	uint8_t APB1Prescaler;

	uint32_t SystemClk = 0;
	uint32_t AHBClock;
	uint32_t APB1Clock;

	clksource = ((RCC->CFGR >> 2) & 0x3);

	AHBPrescaler = ((RCC->CFGR >> 4) & 0xF);
	APB1Prescaler = ((RCC->CFGR >> 10) & 7);

	if(clksource == 0)
		SystemClk = 16000000;
	else if(clksource == 1)
		SystemClk = 8000000;
	//else if(clksource == 2)
		//SystemClk = RCC_GetPLLOutputClock(); //TBD

	if (!(AHBPrescaler & 8))
	{
		if((AHBPrescaler & 7) < 4)
			AHBClock = SystemClk / (2^((AHBPrescaler & 7)+1));
		else
			AHBClock = SystemClk / (2^((AHBPrescaler & 7)+2));
	}
	else
		AHBClock = SystemClk;


	if (!(APB1Prescaler & 4))
	{
		APB1Clock = AHBClock / (2^((APB1Prescaler & 3)+1));
	}
	else
		APB1Clock = AHBClock;

	return APB1Clock;
}



/****************************************************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given I2C peripheral
 *
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 *****************************************************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDis)
{
	if (EnorDis == ENABLE)
	{
		if (pI2Cx == I2C1)
			I2C1_PCLK_EN();
		else if (pI2Cx == I2C2)
			I2C2_PCLK_EN();
		else if (pI2Cx == I2C3)
			I2C3_PCLK_EN();
	}
	else
	{
		if (pI2Cx == I2C1)
			I2C1_PCLK_DIS();
		else if (pI2Cx == I2C2)
			I2C2_PCLK_DIS();
		else if (pI2Cx == I2C3)
			I2C3_PCLK_DIS();
	}
}


/****************************************************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - This function takes the info from I2C_PinConfig_t and writes into registers within pI2Cx structure
 *
 * @param[in]         - base address of the I2C Handle which contains the base address of I2Cx peripheral and user created I2C_Pin_Config struct
 *
 * @return            -  none
 *
 * @Note              -  none
 *****************************************************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	//uint32_t tempregCR1 = 0;
	uint32_t tempregCR2 = 0;
	uint32_t tempregOAR1 = 0;
	uint32_t tempregCCR = 0;
	uint32_t tempregTRISE = 0;
	//0. Enable peripheral clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//1. Configure the mode (standard or fast)

	//2. Configure the speed of the serial clock (SCL)
		//FREQ
	tempregCR2  |= (((RCC_GetPCLK1Value()/1000000) & 0x3F) << I2C_CR2_FREQ);
		//CCR
	uint16_t ccrvalue = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Standard mode
		// T(high,SCL) = CCR * T(APB1CLK)
		// T(low,SCL) = CCR * T(APB1CLK)
		// T(SCL) = 2CCR * T(APB1CLK)
		// CCR = T(SCL) / (2*T(APB1CLK))  --> CCR = Freq(APB1CLK) / (2*Freq(SCL))
		ccrvalue = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
	}
	else
	{
		// Fast mode
		tempregOAR1 |= (1 << 15);
		tempregOAR1 |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
			ccrvalue = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		else if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9)
			ccrvalue = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
	}
	tempregCCR |= (ccrvalue & 0xFFF);

	//3. Configure the device address (applicable when the device is a slave)
	tempregOAR1 |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempregOAR1 |= (1 << 14); // Unusual, one bit of reserved bits must be kept at 1 by software

	//4. Enable the Acking (acknowledgement) !!!NEEDS to be done after PE = 1!!!
	//tempregCR1  |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);

	//5. Configure the rise time for I2C pins
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
	tempregTRISE |= (RCC_GetPCLK1Value()/1000000) + 1;
	}
	else
	{
	tempregTRISE |= ((RCC_GetPCLK1Value()*300) / 1000000000) + 1;
	}
	tempregTRISE &= 0x3F;


	//6. Initialise all temp registers
	pI2CHandle->pI2Cx->I2C_CR2 = tempregCR2;
	pI2CHandle->pI2Cx->I2C_OAR1 = tempregOAR1;
	pI2CHandle->pI2Cx->I2C_CCR = tempregCCR;
	pI2CHandle->pI2Cx->I2C_TRISE = tempregTRISE;
}

/****************************************************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             - This function resets the I2C peripheral registers to its default states
 *
 * @param[in]         - base address of the I2C peripheral
 *
 * @return            -  none
 *
 * @Note              -  none
 *****************************************************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
		I2C1_REG_RESET();
	else if(pI2Cx == I2C2)
		I2C2_REG_RESET();
	else if(pI2Cx == I2C3)
		I2C3_REG_RESET();
}

uint8_t I2C_GetFLagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->I2C_SR1 & FlagName)
		return FLAG_SET;
	else
		return FLAG_RESET;
}



/****************************************************************************************************
 * @fn      		  - I2C_MasterSendData
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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,uint32_t Len, uint8_t SlaveAddr, uint8_t SR)
{
	//1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that START generation is completed by checking the SB flag in the SR1
	//	 NOTE: Until SB is cleared, SCL will be stretched (pulled to LOW)
	while(!I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4. Confirm that address phase is completed by checking ADDR flag in the SR1
	while(!I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear the ADDR flag according to its software sequence
	//	 NOTE: Until ADDR is cleared, SCL will be stretched (pulled to LOW)
	I2C_CLearADDRFLag(pI2CHandle);

	//6. Send the data until Len becomes 0
	while(Len > 0)
	{
		while(!I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->I2C_DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	//7. When the Len becomes 0, wait for TXE=1 and BTF=1 before generating STOP condition
	//   NOTE: TXE=1, BTF=1, means that both SR(shift register) and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
	while(!I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(!I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));


	//8. Generate STOP condition and master needs not to wait for the completion of STOP condition.
	//   NOTE: generating STOP, automatically clears BTF
	if(SR == I2C_STOP)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

/****************************************************************************************************
 * @fn      		  - I2C_MasterReceiveData
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
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t Len, uint8_t SlaveAddr, uint8_t SR)
{
	//1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that START generation is completed by checking the SB flag in the SR1
	//	 NOTE: Until SB is cleared, SCL will be stretched (pulled to LOW)
	while(!I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to r(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4. Wait until the address phase is completed by checking ADDR flag in the SR1
	while(!I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

		//Clear the ADDR flag
		I2C_CLearADDRFLag(pI2CHandle);

		//Generate STOP condition
		if(SR == I2C_STOP)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Wait until RXNE=1
		while(!I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//Read data into the RxBuffer
		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;

	}
	else if(Len > 1)
	{
		//Clear the ADDR flag
		I2C_CLearADDRFLag(pI2CHandle);

		while(Len > 0)
		{
			//Wait until RXNE=1
			while(!I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if (Len == 2)
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//Generate STOP condition
				if(SR == I2C_STOP)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//Read data into the RxBuffer
			*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
			pRxBuffer++;
			Len--;
		}
	}

	//Re-enable Acking
	I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
}

/*
 * Helper function for generating I2C Start Condition
 */
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}

/****************************************************************************************************
 * @fn      		  - I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis)
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
 * @fn      		  - I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// 1. Find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_amount);
}

/****************************************************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             - This Enables or Disables the I2C peripheral
 *
 * @param[in]         - base address of the I2C peripheral
 * @param[in]         - Enable or Disable macro
 *
 * @return            - none
 *
 * @Note              - none
 *****************************************************************************************************/
void I2C_PeripheralControl(I2C_Handle_t *pI2CHandle, uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		pI2CHandle->pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
		pI2CHandle->pI2Cx->I2C_CR1 |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	}
	else if(EnorDis == DISABLE)
		pI2CHandle->pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
}

/****************************************************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             - I2C Master Data transmission via interrupt events
 *
 * @param[in]         - Base address of the I2C Handle
 * @param[in]         - Base address of TxBuffer
 * @param[in]         - Length of data to be sent
 * @param[in]         - Slave device address
 * @param[in]         - I2C_STOP or I2C_NO_STOP selections
 *
 *
 * @return            - I2C Application State
 *
 * @Note              - Non blocking API
 *****************************************************************************************************/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer,uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t state = pI2CHandle->TxRxState;

	if (state == I2C_READY)
	{
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable Event, Error and Buffer interrupts
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);
	}

	return state;
}

/****************************************************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             - I2C Master Data reception via interrupt events
 *
 * @param[in]         - Base address of the I2C Handle
 * @param[in]         - Base address of RxBuffer
 * @param[in]         - Length of data to be received
 * @param[in]         - Slave device address
 * @param[in]         - I2C_STOP or I2C_NO_STOP selections
 *
 *
 * @return            - I2C Application State
 *
 * @Note              - Non blocking API
 *****************************************************************************************************/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t state = pI2CHandle->TxRxState;

	if (state == I2C_READY)
	{
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;
		pI2CHandle->RxSize = Len;

		// Generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable Event, Error and Buffer interrupts
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);
	}

	return state;
}


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->I2C_DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t) pI2Cx->I2C_DR;
}


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = (pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITEVTEN));
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITBUFEN));
	temp3 = I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB);

	//1. Handle for Interrupt generated by SB event
	//	 Note: SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because SB for slave is always zero

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 2)
			{
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				pI2CHandle->pI2Cx->I2C_CR1 |= (1 << I2C_CR1_POS);
			}
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
	}

	temp3 = I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR);
	//2. Handle for Interrupt generated by ADDR event
	//	 Note: when master mode - address is sent
	//	 Note: when slave  mode - address is matched with own address
	if(temp1 && temp3)
	{
		//The interrupt is generated because of ADDR event
		I2C_CLearADDRFLag(pI2CHandle);
	}

	temp3 = I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF);
	//3. Handle for Interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure TXE is also set
			if(I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE) && pI2CHandle->TxLen == 0)
			{
				//BTF and TXE = 1

				//1. Generate STOP condition
				if(pI2CHandle->Sr == I2C_STOP)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

				//2. Reset all the member elements of the handle structure
				I2C_CloseSendData(pI2CHandle);

				//3. Notify the application about transmission complete
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);

			}
		}
	}

	temp3 = I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF);
	//4. Handle for Interrupt generated by STOPF event
	//	 Note: Stop detection flag is applicable only in slave mode
	if(temp1 && temp3)
	{
		//STOPF flag is set

		//Clear the STOPF flag -> | 1)Read SR1 2)Write to CR1 |

		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE);
	//5. Handle for Interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		//check for device mode (if it is master)
		if(pI2CHandle->Mode == I2C_MODE_MASTER)
		{
			//TxE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX && pI2CHandle->TxLen > 0)
			{
				//1. Load data into DR
				pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);
				//2. Decrement TxLen
				pI2CHandle->TxLen--;
				//3. Increment TxBuffer address
				pI2CHandle->pTxBuffer++;
			}
		}else
		{
			//slave
			//make sure that the slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA))
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
		}
	}

	temp3 = I2C_GetFLagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE);
	//6. Handle for Interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{

		//check device mode
		if(pI2CHandle->Mode == I2C_MODE_MASTER)
		{
			//device mode is master

			//RxNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX && pI2CHandle->RxLen > 0)
			{
				if(pI2CHandle->RxSize == 1)
				{
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
					pI2CHandle->RxLen--;
				}
				else if(pI2CHandle->RxSize > 2)
				{
					if(pI2CHandle->RxLen == 3)
					{
						//clear the ack bit
						I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

						//read DR
						*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
						pI2CHandle->pRxBuffer++;
						pI2CHandle->RxLen--;

						if(pI2CHandle->Sr == I2C_STOP)
							I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					else
					{
						//read DR
						*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
						pI2CHandle->pRxBuffer++;
						pI2CHandle->RxLen--;
					}
				}
				else if(pI2CHandle->RxSize == 2)
				{
					if(pI2CHandle->RxLen == 2)
					{
						if(pI2CHandle->Sr == I2C_STOP)
							I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

						//read DR
						*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
						pI2CHandle->pRxBuffer++;
						pI2CHandle->RxLen--;
					}
					else
					{
						//read DR
						*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
						pI2CHandle->pRxBuffer++;
						pI2CHandle->RxLen--;
					}
				}

				if(pI2CHandle->RxLen == 0)
				{
					//close the I2C data reception and notify the application

					//1. generate the stop condition
					if(pI2CHandle->Sr == I2C_STOP && pI2CHandle->RxSize == 1)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

					//2 . Close the I2C rx
					I2C_CloseReceiveData(pI2CHandle);

					pI2CHandle->pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_POS);

					//4. Notify the application
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
				}
			}
		}
		else
		{
			//slave
			//make sure that the slave is really in receiver mode
			if(!(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA)))
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
		}
	}
}
/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	    I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t Application_Event)
{
	// This is a weak implementation
}

