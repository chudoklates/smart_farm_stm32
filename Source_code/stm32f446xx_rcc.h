/**
  ******************************************************************************
  * @file 		stm32f446xx_rcc.h
  * @author  	Kamil Mezynski
  * @version 	v1.3.0
  * @date    	25-March-2020
  * @brief   	This file contains macros and functions prototypes for the RCC
  * 			peripheral for STM32F446xx MCU
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F446XX_RCC_H
#define __STM32F446XX_RCC_H

/* Define to ensure compilation in C -----------------------------------------*/
#ifdef __cplusplus
	extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Enumeration types and macros ----------------------------------------------*/

/** @defgroup RCC clock frequencies structure
  * @{
  */
typedef struct {
	uint32_t SYSCLK_Freq;	/* SYSCLK clock frequency in Hz */
	uint32_t HCLK_Freq;		/* HCLK clock frequency in Hz */
	uint32_t PCLK1_Freq;	/* PCLK1 clock frequency in Hz */
	uint32_t PCLK2_Freq;	/* PCLK2 clock frequency in Hz */
} RCC_Clocks_TypeDef;
/**
  * @}
  */

/** @defgroup RCC HSE configuration
  * @{
  */
#define RCC_HSE_ON        				((uint32_t)0x00010000)
#define RCC_HSE_Bypass					((uint32_t)0x00050000)
#define RCC_HSE_OFF                   	~RCC_HSE_Bypass
/**
  * @}
  */

/** @defgroup RCC PLL configuration
  * @{
  */
#define RCC_PLL_ON						((uint32_t)0x01000000)
#define RCC_PLL_OFF						~RCC_PLL_ON
#define RCC_PLLI2S_ON					((uint32_t)0x04000000)
#define RCC_PLLI2S_OFF					~RCC_PLLI2S_ON
#define RCC_PLLSAI_ON					((uint32_t)0x10000000)
#define RCC_PLLSAI_OFF					~RCC_PLLSAI_ON
/**
  * @}
  */

/** @defgroup RCC PLL clock source
  * @{
  */
#define RCC_PLL_Source_HSI				((uint32_t)0x00000000)
#define RCC_PLL_Source_HSE				((uint32_t)0x00400000)
/**
  * @}
  */

/** @defgroup RCC System Clock source
  * @{
  */
#define RCC_SYSCLK_Source_HSI			((uint32_t)0x00000000)
#define RCC_SYSCLK_Source_HSE			((uint32_t)0x00000001)
#define RCC_SYSCLK_Source_PLLPCLK		((uint32_t)0x00000002)
#define RCC_SYSCLK_Source_PLLRCLK		((uint32_t)0x00000003)
/**
  * @}
  */

/** @defgroup RCC AHB clock source
  * @{
  */
#define RCC_SYSCLK_Div_1				((uint32_t)0x00000000)
#define RCC_SYSCLK_Div_2				((uint32_t)0x00000080)
#define RCC_SYSCLK_Div_4				((uint32_t)0x00000090)
#define RCC_SYSCLK_Div_8				((uint32_t)0x000000A0)
#define RCC_SYSCLK_Div_16				((uint32_t)0x000000B0)
#define RCC_SYSCLK_Div_64				((uint32_t)0x000000C0)
#define RCC_SYSCLK_Div_128				((uint32_t)0x000000D0)
#define RCC_SYSCLK_Div_256				((uint32_t)0x000000E0)
#define RCC_SYSCLK_Div_512				((uint32_t)0x000000F0)

/**
  * @}
  */

/** @defgroup RCC APB1 and APB2 clock source
  * @{
  */
#define RCC_HCLK_Div_1					((uint32_t)0x00000000)
#define RCC_HCLK_Div_2					((uint32_t)0x00001000)
#define RCC_HCLK_Div_4					((uint32_t)0x00001400)
#define RCC_HCLK_Div_8					((uint32_t)0x00001800)
#define RCC_HCLK_Div_16					((uint32_t)0x00001C00)
/**
  * @}
  */

/** @defgroup RCC AHB1 peripherals
  * @{
  */
#define RCC_AHB1_GPIOA             		((uint32_t)0x00000001)
#define RCC_AHB1_GPIOB             		((uint32_t)0x00000002)
#define RCC_AHB1_GPIOC             		((uint32_t)0x00000004)
#define RCC_AHB1_GPIOD             		((uint32_t)0x00000008)
#define RCC_AHB1_GPIOE             		((uint32_t)0x00000010)
#define RCC_AHB1_GPIOF             		((uint32_t)0x00000020)
#define RCC_AHB1_GPIOG             		((uint32_t)0x00000040)
#define RCC_AHB1_GPIOH             		((uint32_t)0x00000080)
#define RCC_AHB1_CRC               		((uint32_t)0x00001000)
#define RCC_AHB1_BKPSRAM           		((uint32_t)0x00040000)
#define RCC_AHB1_DMA1              		((uint32_t)0x00200000)
#define RCC_AHB1_DMA2              		((uint32_t)0x00400000)
#define RCC_AHB1_OTG_HS            		((uint32_t)0x20000000)
#define RCC_AHB1_OTG_HS_ULPI       		((uint32_t)0x40000000)
/**
  * @}
  */

/** @defgroup RCC flags
  * @{
  */
#define RCC_FLAG_HSIRDY                  ((uint8_t)0x01)	/* flags in CR register */
#define RCC_FLAG_HSERDY                  ((uint8_t)0x11)
#define RCC_FLAG_PLLRDY                  ((uint8_t)0x19)
#define RCC_FLAG_PLLI2SRDY               ((uint8_t)0x1B)
#define RCC_FLAG_PLLSAIRDY               ((uint8_t)0x1D)
#define RCC_FLAG_LSERDY                  ((uint8_t)0x21)	/* flag in BDCR register */
#define RCC_FLAG_LSIRDY                  ((uint8_t)0x41)	/* flags in CSR register */
#define RCC_FLAG_BORRST                  ((uint8_t)0x59)
#define RCC_FLAG_PINRST                  ((uint8_t)0x5A)
#define RCC_FLAG_PORRST                  ((uint8_t)0x5B)
#define RCC_FLAG_SFTRST                  ((uint8_t)0x5C)
#define RCC_FLAG_IWDGRST                 ((uint8_t)0x5D)
#define RCC_FLAG_WWDGRST                 ((uint8_t)0x5E)
#define RCC_FLAG_LPWRRST                 ((uint8_t)0x5F)
/**
  *	@}
  */

/* Functions prototypes ----------------------------------------------------- */
/* RCC global configuration function ******************************************/
void RCC_config(void);
void RCC_deinit(void);

/* Internal, external, PLL, CSS and MCO clocks configuration functions ********/
void RCC_HSE_config(uint32_t RCC_HSE_New_State);
ErrorStatus RCC_HSE_startup_status(void);
void RCC_HCLK_config(uint32_t RCC_SYSCLK_Div);
void RCC_PCLK1_config(uint32_t RCC_HCLK_Div);
void RCC_PCLK2_config(uint32_t RCC_HCLK_Div);
void RCC_PLL_state(uint32_t RCC_PLL_New_State);
void RCC_PLL_config(uint32_t RCC_PLL_Source, uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ, uint32_t PLLR);
void RCC_SYSCLK_config(uint32_t RCC_SYSCLK_source);

/* System, AHB and APB busses clocks configuration functions ******************/
uint8_t RCC_get_SYSCLK_source(void);
void RCC_get_clocks_freq(RCC_Clocks_TypeDef *RCC_Clocks);

/* Peripheral clock configuration functions ***********************************/
void RCC_AHB1_reset(uint32_t RCC_AHB1_Periph, FunctionalState New_State);
void RCC_AHB2_reset(uint32_t RCC_AHB2_Periph, FunctionalState New_State);
void RCC_AHB3_reset(uint32_t RCC_AHB3_Periph, FunctionalState New_State);
void RCC_APB1_reset(uint32_t RCC_APB1_Periph, FunctionalState New_State);
void RCC_APB2_reset(uint32_t RCC_APB2_Periph, FunctionalState New_State);

void RCC_AHB1_set(uint32_t RCC_AHB1_Periph, FunctionalState New_State);
void RCC_AHB2_set(uint32_t RCC_AHB2_Periph, FunctionalState New_State);
void RCC_AHB3_set(uint32_t RCC_AHB3_Periph, FunctionalState New_State);
void RCC_APB1_set(uint32_t RCC_APB1_Periph, FunctionalState New_State);
void RCC_APB2_set(uint32_t RCC_APB2_Periph, FunctionalState New_State);

void RCC_AHB1_LP_set(uint32_t RCC_AHB1_Periph, FunctionalState New_State);
void RCC_AHB2_LP_set(uint32_t RCC_AHB2_Periph, FunctionalState New_State);
void RCC_AHB3_LP_set(uint32_t RCC_AHB3_Periph, FunctionalState New_State);
void RCC_APB1_LP_set(uint32_t RCC_APB1_Periph, FunctionalState New_State);
void RCC_APB2_LP_set(uint32_t RCC_APB2_Periph, FunctionalState New_State);

/* RCC flags management functions *********************************************/
FlagStatus RCC_get_flag_status(uint8_t RCC_Flag);

#ifdef __cpluslus
}
#endif

#endif /* __STM32F446XX_RCC_H */

/* END OF FILE ****************************************************************/
