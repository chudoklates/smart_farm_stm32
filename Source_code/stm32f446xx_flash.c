/**
  ******************************************************************************
  * @file 		stm32f446xx_flash.c
  * @author  	Kamil Mezynski
  * @version 	v1.0.0
  * @date    	23-March-2020
  * @brief   	This file contains macros and functions prototypes for the FLASH
  * 			memory interface for STM32F446xx MCU
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f446xx_flash.h"

/* Functions declarations --------------------------------------------------- */

/** @defgroup FLASH interface configuration functions
  * @{
================================================================================
                  	##### FLASH interface configuration #####
================================================================================
  */

/** @brief	Sets the code latency value
  *	@param 	FLASH_Latency: specifies the FLASH latency cycles
  *			This parameter can be FLASH_Latency_x where x can be (0..15)
  */
void FLASH_set_latency(uint8_t FLASH_Latency) {
	FLASH->ACR |= FLASH_Latency;
}

/**
  * @}
  */

/* END OF FILE ****************************************************************/
