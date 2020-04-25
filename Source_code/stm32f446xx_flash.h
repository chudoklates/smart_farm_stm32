/**
  ******************************************************************************
  * @file 		stm32f446xx_flash.h
  * @author  	Kamil Mezynski
  * @version 	v1.0.0
  * @date    	23-March-2020
  * @brief   	This file contains macros and functions prototypes for the FLASH
  * 			memory interface for STM32F446xx MCU
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F446XX_FLASH_H
#define __STM32F446XX_FLASH_H

/* Define to ensure compilation in C -----------------------------------------*/
#ifdef __cplusplus
	extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Enumeration types and macros ----------------------------------------------*/

/** @defgroup FLASH latency cycles
  * @{
  */
#define FLASH_Latency_0					((uint8_t)0x0000)
#define FLASH_Latency_1					((uint8_t)0x0001)
#define FLASH_Latency_2					((uint8_t)0x0002)
#define FLASH_Latency_3					((uint8_t)0x0003)
#define FLASH_Latency_4					((uint8_t)0x0004)
#define FLASH_Latency_5					((uint8_t)0x0005)
#define FLASH_Latency_6					((uint8_t)0x0006)
#define FLASH_Latency_7					((uint8_t)0x0007)
#define FLASH_Latency_8					((uint8_t)0x0008)
#define FLASH_Latency_9					((uint8_t)0x0009)
#define FLASH_Latency_10				((uint8_t)0x000A)
#define FLASH_Latency_11				((uint8_t)0x000B)
#define FLASH_Latency_12				((uint8_t)0x000C)
#define FLASH_Latency_13				((uint8_t)0x000D)
#define FLASH_Latency_14				((uint8_t)0x000E)
#define FLASH_Latency_15				((uint8_t)0x000F)
/**
  * @}
  */

/* Functions prototypes ----------------------------------------------------- */
/* FLASH interface configuration functions ************************************/
void FLASH_set_latency(uint8_t FLASH_Latency);

#ifdef __cpluslus
}
#endif

#endif /* __STM32F446XX_FLASH_H */

/* END OF FILE ****************************************************************/
