/*
 * stm32f4xx.h
 *
 *  Created on: Aug 23, 2022
 *      Author: nxf77381
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile

/*************************************PROCCESSOR SPECIFIC MACROS****************************************/


/*
 * ARM Cortex M4 NVIC_ISER register addresses
 */
#define NVIC_ISER0							( (__vo uint32_t *)0xE000E100 )
#define NVIC_ISER1							( (__vo uint32_t *)0xE000E104 )
#define NVIC_ISER2							( (__vo uint32_t *)0xE000E108 )
#define NVIC_ISER3							( (__vo uint32_t *)0xE000E10C )

/*
 * ARM Cortex M4 NVIC_ICER register addresses
 */

#define NVIC_ICER0							( (__vo uint32_t *)0XE000E180 )
#define NVIC_ICER1							( (__vo uint32_t *)0XE000E184 )
#define NVIC_ICER2							( (__vo uint32_t *)0XE000E188 )
#define NVIC_ICER3							( (__vo uint32_t *)0XE000E18C )

/*
 * ARM Cortex M4 NVIC_ICER register addresses
 */

#define NVIC_IPR_BASE_ADDR					( (__vo uint32_t *)0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED 				4
/*
 * Base Address of FLASH and SRAM
 */
#define FLASH_BASE_ADDR						0x08000000UL	//512
#define SRAM1_BASE_ADDR						0x20000000UL	//112KB
#define SRAM2_BASE_ADDR						0x2001C000UL	//16KB
#define   ROM_BASE_ADDR						0x1FFF0000UL	//30KB
#define  SRAM_BASE_ADDR						SRAM1_BASE_ADDR

// -------------------------------------------
// Base addresses of APBx & AHBx peripherals
// -------------------------------------------

#define 	 PERIPH_BASE_ADDR				APB1_PERIPH_BASE_ADDR
#define APB1_PERIPH_BASE_ADDR				0x40000000UL
#define APB2_PERIPH_BASE_ADDR				0x40010000UL
#define AHB1_PERIPH_BASE_ADDR				0x40020000UL
#define AHB2_PERIPH_BASE_ADDR				0x50000000UL
#define AHB3_PERIPH_BASE_ADDR				0xA0000000UL


// ----------------------------------------------------------------------------
// Base addresses of AHB1 peripherals
// ----------------------------------------------------------------------------

#define			  GPIOA_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x0000)
#define 		  GPIOB_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x0400)
#define 		  GPIOC_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x0800)
#define 		  GPIOD_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x0C00)
#define 		  GPIOE_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x1000)
#define 		  GPIOF_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x1400)
#define 		  GPIOG_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x1800)
#define 		  GPIOH_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x1C00)
#define 		  GPIOI_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x2000)
#define 		  GPIOJ_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x2400)
#define   		  GPIOK_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x2800)
#define  	   	  	CRC_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x3000)
#define  		  	RCC_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x3800)
#define FLASH_INTERFACE_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x3C00)
#define			BKPSRAM_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x4000)
#define 		   DMA1_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x6000)
#define 		   DMA2_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x6400)
#define    ETHERNET_MAC_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x8000)
#define 		  DMA2D_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0xB000)
#define 	 USB_OTG_HS_BASE_ADDR			((AHB1_PERIPH_BASE_ADDR) + 0x20000)

// -------------------------------------------
// Base addresses of AHB2 peripherals
// -------------------------------------------

#define		 USB_OTG_FS_BASE_ADDR			((AHB2_PERIPH_BASE_ADDR) + 0x00000)
#define 		   DCMI_BASE_ADDR			((AHB2_PERIPH_BASE_ADDR) + 0x50000)
#define 		   CRYP_BASE_ADDR			((AHB2_PERIPH_BASE_ADDR) + 0x60000)
#define 		   HASH_BASE_ADDR			((AHB2_PERIPH_BASE_ADDR) + 0x60400)
#define 		    RNG_BASE_ADDR			((AHB2_PERIPH_BASE_ADDR) + 0x60800)

// -------------------------------------------
// Base addresses of APB1 peripherals
// -------------------------------------------

#define			   TIM2_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x0000)
#define 		   TIM3_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x0400)
#define 		   TIM4_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x0800)
#define 		   TIM5_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x0C00)
#define 		   TIM6_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x1000)
#define 		   TIM7_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x1400)
#define 		  TIM12_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x1800)
#define 		  TIM13_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x1C00)
#define 		  TIM14_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x2000)
#define 		RTC_BKP_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x2800)
#define   		   WWDG_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x2C00)
#define  	   	   IWDG_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x3000)
#define  		I2S2EXT_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x3400)
#define 	  SPI2_I2S2_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x3800)
#define		  SPI3_I2S3_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x3C00)
#define 		I2S3EXT_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x4000)
#define 		 USART2_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x4400)
#define    		 USART3_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x4800)
#define 		  UART4_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x4C00)
#define 	 	  UART5_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x5000)
#define 		   I2C1_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x5400)
#define 	 	   I2C2_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x5800)
#define 		   I2C3_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x5C00)
#define 	 	   CAN1_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x6400)
#define 	 	   CAN2_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x6800)
#define 	 	    PWR_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x7000)
#define 	 	    DAC_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x7400)
#define 	 	  UART7_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x7800)
#define 	 	  UART8_BASE_ADDR			((APB1_PERIPH_BASE_ADDR) + 0x7C00)

// -------------------------------------------
// Base addresses of APB2 peripherals
// -------------------------------------------

#define			   TIM1_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x0000)
#define 		   TIM8_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x0400)
#define 		 USART1_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x1000)
#define 		 USART6_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x1400)
#define  ADC1_ADC2_ADC3_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x2000)
#define 		   SDIO_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x2C00)
#define 		   SPI1_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x3000)
#define 		   SPI4_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x3400)
#define 		 SYSCFG_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x3800)
#define 		   EXTI_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x3C00)
#define   		   TIM9_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x4000)
#define  	   	  TIM10_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x4400)
#define  		  TIM11_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x4800)
#define 	       SPI5_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x5000)
#define		  	   SPI6_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x5400)
#define 		   SAI1_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x5800)
#define 		LCD_TFT_BASE_ADDR			((APB2_PERIPH_BASE_ADDR) + 0x6800)


// ------------------------------------------------------------------------------
// Peripheral register definition structures
// ------------------------------------------------------------------------------

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint64_t AFR;

}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;

}RCC_RegDef_t;

/*
 * Peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t SPI_CR1;
	__vo uint32_t SPI_CR2;
	__vo uint32_t SPI_SR;
	__vo uint32_t SPI_DR;
	__vo uint32_t SPI_CRCPR;
	__vo uint32_t SPI_RXCRCR;
	__vo uint32_t SPI_TXCRCR;
	__vo uint32_t SPI_I2SCFGR;
	__vo uint32_t SPI_I2SPR;
}SPI_RegDef_t;

/*
 * Peripheral register definition structure for I2C
 */
typedef struct
{
	__vo uint32_t I2C_CR1;
	__vo uint32_t I2C_CR2;
	__vo uint32_t I2C_OAR1;
	__vo uint32_t I2C_OAR2;
	__vo uint32_t I2C_DR;
	__vo uint32_t I2C_SR1;
	__vo uint32_t I2C_SR2;
	__vo uint32_t I2C_CCR;
	__vo uint32_t I2C_TRISE;
	__vo uint32_t I2C_FLTR;
}I2C_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;


/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;
}SYSCFG_RegDef_t;

/*
 * Peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_RegDef_t;

//GPIO_RegDef_t *pGPIOA = ((GPIO_RegDef_t *) GPIOA_BASE_ADDR);

// ------------------------------------------------------------------------------
// Peripheral definitions
// ------------------------------------------------------------------------------

#define GPIOA								((GPIO_RegDef_t *) GPIOA_BASE_ADDR)
#define GPIOB								((GPIO_RegDef_t *) GPIOB_BASE_ADDR)
#define GPIOC								((GPIO_RegDef_t *) GPIOC_BASE_ADDR)
#define GPIOD								((GPIO_RegDef_t *) GPIOD_BASE_ADDR)
#define GPIOE								((GPIO_RegDef_t *) GPIOE_BASE_ADDR)
#define GPIOF								((GPIO_RegDef_t *) GPIOF_BASE_ADDR)
#define GPIOG								((GPIO_RegDef_t *) GPIOG_BASE_ADDR)
#define GPIOH								((GPIO_RegDef_t *) GPIOH_BASE_ADDR)
#define GPIOI								((GPIO_RegDef_t *) GPIOI_BASE_ADDR)

#define SPI1								((SPI_RegDef_t *) SPI1_BASE_ADDR)
#define SPI2								((SPI_RegDef_t *) SPI2_I2S2_BASE_ADDR)
#define SPI3								((SPI_RegDef_t *) SPI3_I2S3_BASE_ADDR)


#define I2C1								((I2C_RegDef_t *) I2C1_BASE_ADDR)
#define I2C2								((I2C_RegDef_t *) I2C2_BASE_ADDR)
#define I2C3								((I2C_RegDef_t *) I2C3_BASE_ADDR)

#define USART1								((USART_RegDef_t *) USART1_BASE_ADDR)
#define USART2								((USART_RegDef_t *) USART2_BASE_ADDR)
#define USART3								((USART_RegDef_t *) USART3_BASE_ADDR)
#define UART4								((USART_RegDef_t *) UART4_BASE_ADDR)
#define UART5								((USART_RegDef_t *) UART5_BASE_ADDR)
#define USART6								((USART_RegDef_t *) USART6_BASE_ADDR)



#define RCC									((RCC_RegDef_t *) RCC_BASE_ADDR)

#define EXTI								((EXTI_RegDef_t *) EXTI_BASE_ADDR)
#define SYSCFG								((SYSCFG_RegDef_t *) SYSCFG_BASE_ADDR)

// ------------------------------------------------------------------------------
// Clock Enable Macros for GPIOx Peripherals
// ------------------------------------------------------------------------------

#define GPIOA_PCLK_EN()						(  RCC->AHB1ENR |= (1 <<  0) )
#define GPIOB_PCLK_EN()						(  RCC->AHB1ENR |= (1 <<  1) )
#define GPIOC_PCLK_EN()						(  RCC->AHB1ENR |= (1 <<  2) )
#define GPIOD_PCLK_EN()						(  RCC->AHB1ENR |= (1 <<  3) )
#define GPIOE_PCLK_EN()						(  RCC->AHB1ENR |= (1 <<  4) )
#define GPIOF_PCLK_EN()						(  RCC->AHB1ENR |= (1 <<  5) )
#define GPIOG_PCLK_EN()						(  RCC->AHB1ENR |= (1 <<  6) )
#define GPIOH_PCLK_EN()						(  RCC->AHB1ENR |= (1 <<  7) )
#define GPIOI_PCLK_EN()						(  RCC->AHB1ENR |= (1 <<  8) )


// ------------------------------------------------------------------------------
// Clock Disable Macros for GPIOx Peripherals
// ------------------------------------------------------------------------------

#define GPIOA_PCLK_DIS()					(  RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DIS()					(  RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DIS()					(  RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DIS()					(  RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DIS()					(  RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DIS()					(  RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DIS()					(  RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DIS()					(  RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DIS()					(  RCC->AHB1ENR &= ~(1 << 8) )


// ------------------------------------------------------------------------------
// Clock Enable Macros for I2Cx Peripherals
// ------------------------------------------------------------------------------

#define I2C1_PCLK_EN()						(  RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()						(  RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()						(  RCC->APB1ENR |= (1 << 23) )

// ------------------------------------------------------------------------------
// Clock Disable Macros for I2Cx Peripherals
// ------------------------------------------------------------------------------

#define I2C1_PCLK_DIS()						(  RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DIS()						(  RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DIS()						(  RCC->APB1ENR &= ~(1 << 23) )

// ------------------------------------------------------------------------------
// Clock Enable Macros for SPIx Peripherals
// ------------------------------------------------------------------------------

#define SPI1_PCLK_EN()						(  RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()						(  RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()						(  RCC->APB1ENR |= (1 << 15) )

// ------------------------------------------------------------------------------
// Clock Disable Macros for SPIx Peripherals
// ------------------------------------------------------------------------------

#define SPI1_PCLK_DIS()						(  RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DIS()						(  RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DIS()						(  RCC->APB1ENR &= ~(1 << 15) )

// ------------------------------------------------------------------------------
// Clock Enable Macros for USARTx Peripherals
// ------------------------------------------------------------------------------

#define USART1_PCLK_EN()					(  RCC->APB2ENR |= (1 <<  4) )
#define USART2_PCLK_EN()					(  RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()					(  RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN()						(  RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()						(  RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN()					(  RCC->APB2ENR |= (1 <<  5) )

// ------------------------------------------------------------------------------
// Clock Disable Macros for USARTx Peripherals
// ------------------------------------------------------------------------------

#define USART1_PCLK_DIS()					(  RCC->APB2ENR &= ~(1 <<  4) )
#define USART2_PCLK_DIS()					(  RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DIS()					(  RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCLK_DIS()					(  RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DIS()					(  RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_DIS()					(  RCC->APB2ENR &= ~(1 <<  5) )

// ------------------------------------------------------------------------------
// Clock Enable Macros for SYSCFG Peripheral
// ------------------------------------------------------------------------------

#define SYSCFG_PCLK_EN()					(  RCC->APB2ENR |= (1 << 14) )

// ------------------------------------------------------------------------------
// Clock Disable Macros for SYSCFG Peripheral
// ------------------------------------------------------------------------------

#define SYSCFG_PCLK_DIS()					(  RCC->APB2ENR &= ~(1 << 14) )

// ------------------------------------------------------------------------------
// Macros for GPIOx peripheral ports Reset
// ------------------------------------------------------------------------------

#define GPIOA_REG_RESET()					do{  (RCC->AHB1RSTR |= (1 << 0)) ; (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()					do{  (RCC->AHB1RSTR |= (1 << 1)) ; (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()					do{  (RCC->AHB1RSTR |= (1 << 2)) ; (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()					do{  (RCC->AHB1RSTR |= (1 << 3)) ; (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()					do{  (RCC->AHB1RSTR |= (1 << 4)) ; (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()					do{  (RCC->AHB1RSTR |= (1 << 5)) ; (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()					do{  (RCC->AHB1RSTR |= (1 << 6)) ; (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()					do{  (RCC->AHB1RSTR |= (1 << 7)) ; (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()					do{  (RCC->AHB1RSTR |= (1 << 8)) ; (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

#define GPIO_BASEADDR_TO_CODE(x)		   ((x == GPIOA) ? 0 :\
											(x == GPIOB) ? 1 :\
											(x == GPIOC) ? 2 :\
											(x == GPIOD) ? 3 :\
											(x == GPIOE) ? 4 :\
											(x == GPIOF) ? 5 :\
											(x == GPIOG) ? 6 :\
											(x == GPIOH) ? 7 :\
											(x == GPIOI) ? 8 :0)

// ------------------------------------------------------------------------------
// Macros for SPI peripheral Reset
// ------------------------------------------------------------------------------

#define SPI1_REG_RESET()					do{  (RCC->APB2RSTR |= (1 << 12)) ; (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()					do{  (RCC->APB1RSTR |= (1 << 14)) ; (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()					do{  (RCC->APB1RSTR |= (1 << 15)) ; (RCC->APB1RSTR &= ~(1 << 15)); }while(0)

// ------------------------------------------------------------------------------
// Macros for I2C peripheral Reset
// ------------------------------------------------------------------------------

#define I2C1_REG_RESET()					do{  (RCC->APB1RSTR |= (1 << 21)) ; (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()					do{  (RCC->APB1RSTR |= (1 << 22)) ; (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()					do{  (RCC->APB1RSTR |= (1 << 23)) ; (RCC->APB1RSTR &= ~(1 << 23)); }while(0)

// ------------------------------------------------------------------------------
// Macros for USART peripheral Reset
// ------------------------------------------------------------------------------

#define USART1_REG_RESET()					do{  (RCC->APB2RSTR |= (1 << 4)) ; 	(RCC->APB1RSTR &= ~(1 << 4)); }while(0)
#define USART2_REG_RESET()					do{  (RCC->APB1RSTR |= (1 << 17)) ; (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART3_REG_RESET()					do{  (RCC->APB1RSTR |= (1 << 18)) ; (RCC->APB1RSTR &= ~(1 << 18)); }while(0)
#define UART4_REG_RESET()					do{  (RCC->APB1RSTR |= (1 << 19)) ; (RCC->APB1RSTR &= ~(1 << 19)); }while(0)
#define UART5_REG_RESET()					do{  (RCC->APB1RSTR |= (1 << 20)) ; (RCC->APB1RSTR &= ~(1 << 20)); }while(0)
#define USART6_REG_RESET()					do{  (RCC->APB2RSTR |= (1 << 5)) ; 	(RCC->APB1RSTR &= ~(1 << 5)); }while(0)

/*
 * IRQ (Interrupt request) numbers for STM32F407xx MCU
 */

#define IRQ_EXTI0							6
#define IRQ_EXTI1							7
#define IRQ_EXTI2							8
#define IRQ_EXTI3							9
#define IRQ_EXTI4							10
#define IRQ_EXTI9_5							23
#define IRQ_EXTI15_10						40

#define IRQ_SPI1							35
#define IRQ_SPI2							36
#define IRQ_SPI3							51

#define IRQ_I2C1_EV							31
#define IRQ_I2C1_ER							32
#define IRQ_I2C2_EV							33
#define IRQ_I2C2_ER							34
#define IRQ_I2C3_EV							72
#define IRQ_I2C3_ER							73

#define IRQ_USART1							37
#define IRQ_USART2							38
#define IRQ_USART3							39
#define IRQ_UART4							52
#define IRQ_UART5							53
#define IRQ_USART6							71



// ------------------------------------------------------------------------------
// Generic Macros
// ------------------------------------------------------------------------------

#define ENABLE  			1
#define DISABLE 			0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define HIGH				1
#define LOW					0
#define FLAG_SET			SET
#define FLAG_RESET			RESET



/**************************************************************************************
 * Bit position macros for SPI peripheral
 **************************************************************************************/

/*
 * Bit position definitions for SPI_CR1 register
 */
#define SPI_CR1_CPHA						0
#define SPI_CR1_CPOL						1
#define SPI_CR1_MSTR						2
#define SPI_CR1_BR							3
#define SPI_CR1_SPE							6
#define SPI_CR1_LSBFIRST					7
#define SPI_CR1_SSI							8
#define SPI_CR1_SSM							9
#define SPI_CR1_RXONLY						10
#define SPI_CR1_DFF							11
#define SPI_CR1_CRCNEXT						12
#define SPI_CR1_CRCEN						13
#define SPI_CR1_BIDIOE						14
#define SPI_CR1_BIDIMODE					15

/*
 * Bit position definitions for SPI_CR2 register
 */
#define SPI_CR2_RXDMAEN						0
#define SPI_CR2_TXDMAEN						1
#define SPI_CR2_SSOE						2
#define SPI_CR2_FRF							4
#define SPI_CR2_ERRIE						5
#define SPI_CR2_RXNEIE						6
#define SPI_CR2_TXEIE						7

/*
 * Bit position definitions for SPI_SR register
 */
#define SPI_SR_RXNE							0
#define SPI_SR_TXE							1
#define SPI_SR_CHSIDE						2
#define SPI_SR_UDR							3
#define SPI_SR_CRCERR						4
#define SPI_SR_MODF							5
#define SPI_SR_OVR							6
#define SPI_SR_BSY							7
#define SPI_SR_FRE							8

/**************************************************************************************
 * Bit position macros for I2C peripheral
 **************************************************************************************/

/*
 * Bit position definitions for I2C_CR1 register
 */
#define I2C_CR1_PE								0
#define I2C_CR1_SMBUS							1
#define I2C_CR1_SMBTYPE							3
#define I2C_CR1_ENARP							4
#define I2C_CR1_ENPEC							5
#define I2C_CR1_ENGC							6
#define I2C_CR1_NOSTRETCH						7
#define I2C_CR1_START							8
#define I2C_CR1_STOP							9
#define I2C_CR1_ACK								10
#define I2C_CR1_POS								11
#define I2C_CR1_PEC								12
#define I2C_CR1_ALERT							13
#define I2C_CR1_SWRST							15

/*
 * Bit position definitions for I2C_CR2 register
 */
#define I2C_CR2_FREQ							0
#define I2C_CR2_ITERREN							8
#define I2C_CR2_ITEVTEN							9
#define I2C_CR2_ITBUFEN							10
#define I2C_CR2_DMAEN							11
#define I2C_CR2_LAST							12

/*
 * Bit position definitions for I2C_SR1 register
 */
#define I2C_SR1_SB								0
#define I2C_SR1_ADDR							1
#define I2C_SR1_BTF								2
#define I2C_SR1_ADD10							3
#define I2C_SR1_STOPF							4
#define I2C_SR1_RXNE							6
#define I2C_SR1_TXE								7
#define I2C_SR1_BERR							8
#define I2C_SR1_ARLO							9
#define I2C_SR1_AF								10
#define I2C_SR1_OVR								11
#define I2C_SR1_PECERR							12
#define I2C_SR1_TIMEOUT							14
#define I2C_SR1_SMBALERT						15

/*
 * Bit position definitions for I2C_SR2 register
 */
#define I2C_SR2_MSL								0
#define I2C_SR2_BUSY							1
#define I2C_SR2_TRA								2
#define I2C_SR2_GENCALL							4
#define I2C_SR2_SMBDEFAULT						5
#define I2C_SR2_SMBHOST							6
#define I2C_SR2_DUALF							7
#define I2C_SR2_PEC					 			8

/*
 * Bit position definitions for I2C_CCR register
 */
#define I2C_CCR_CCR								0
#define I2C_CCR_DUTY							14
#define I2C_CCR_F_S								15

/**************************************************************************************
 * Bit position macros for USART peripheral
 **************************************************************************************/

/*
 * Bit position definitions for USART_CR1 register
 */
#define USART_CR1_SBK							0
#define USART_CR1_RWU							1
#define USART_CR1_RE							2
#define USART_CR1_TE							3
#define USART_CR1_IDLEIE						4
#define USART_CR1_RXNEIE						5
#define USART_CR1_TCIE							6
#define USART_CR1_TXEIE					 		7
#define USART_CR1_PEIE							8
#define USART_CR1_PS							9
#define USART_CR1_PCE							10
#define USART_CR1_WAKE					 		11
#define USART_CR1_M					 			12
#define USART_CR1_UE							13
#define USART_CR1_OVER8							15

/*
 * Bit position definitions for USART_CR2 register
 */
#define USART_CR2_ADD							0
#define USART_CR2_LBDL							5
#define USART_CR2_LBDIE							6
#define USART_CR2_LBCL							8
#define USART_CR2_CPHA							9
#define USART_CR2_CPOL							10
#define USART_CR2_CLKEN							11
#define USART_CR2_STOP					 		12
#define USART_CR2_LINEN							14

/*
 * Bit position definitions for USART_CR3 register
 */
#define USART_CR3_EIE							0
#define USART_CR3_IREN							1
#define USART_CR3_IRLP							2
#define USART_CR3_HDSEL							3
#define USART_CR3_NACK							4
#define USART_CR3_SCEN							5
#define USART_CR3_DMAR							6
#define USART_CR3_DMAT					 		7
#define USART_CR3_RTSE							8
#define USART_CR3_CTSE							9
#define USART_CR3_CTSIE						 	10
#define USART_CR3_ONEBIT						11

/*
 * Bit position definitions for USART_SR register
 */
#define USART_SR_PE								0
#define USART_SR_FE								1
#define USART_SR_NF								2
#define USART_SR_ORE							3
#define USART_SR_IDLE							4
#define USART_SR_RXNE							5
#define USART_SR_TC								6
#define USART_SR_TXE					 		7
#define USART_SR_LBD							8
#define USART_SR_CTS							9

/*
 * Bit position definitions for USART_BRR register
 */
#define USART_BRR_DIVFRACTION					0
#define USART_BRR_DIVMANTISSA					4



// ------------------------------------------------------------------------------
// Header file includes
// ------------------------------------------------------------------------------

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"

#endif /* INC_STM32F407XX_H_ */
