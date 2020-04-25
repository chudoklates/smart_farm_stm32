/**
  ******************************************************************************
  * @file 		stm32f446xx_rcc.c
  * @author  	Kamil Mezynski
  * @version 	v1.3.0
  * @date    	25-March-2020
  * @brief   	This file contains macros and functions prototypes for the RCC
  * 			peripheral for STM32F446xx MCU
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f446xx_rcc.h"
#include "stm32f446xx_flash.h"

/* Functions declarations --------------------------------------------------- */

/**	@defgroup RCC global configuration function
  *	@{
================================================================================
                  	  ##### RCC global configuration #####
================================================================================
  */

void RCC_config(void) {
	/* Reset RCC to the default reset state */
	RCC_deinit();

	/* Enable HSE clock */
	RCC_HSE_config(RCC_HSE_ON);

	/* Wait for HSE clock to be ready */
	ErrorStatus HSE_startup_status;
	HSE_startup_status = RCC_HSE_startup_status();

	if (HSE_startup_status == SUCCESS) {

		/* Set flash latency */
		FLASH_set_latency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLK_config(RCC_SYSCLK_Div_1);

		/* PCLK2 = HCLK */
		RCC_PCLK2_config(RCC_HCLK_Div_1);

		/* PCLK1 = HCLK / 2 */
		RCC_PCLK1_config(RCC_HCLK_Div_2);

		/* PLLCLK = ((HSE / PLLM) * PLLN) / PLLP = ((8 / 4) * 72) / 2 = 72MHz */
		uint32_t pllm = 4, plln = 72, pllp = 2, pllq = 2, pllr = 2;
		RCC_PLL_config(RCC_PLL_Source_HSE, pllm, plln, pllp, pllq, pllr);

		/* Enable PLL */
		RCC_PLL_state(ENABLE);

		/* Wait for PLL to be ready */
		while (RCC_get_flag_status(RCC_FLAG_PLLRDY) == RESET);

		/* Set PLLPCLK as SYSCLK clock source */
		RCC_SYSCLK_config(RCC_SYSCLK_Source_PLLPCLK);

		/* Wait for PLL to be SYSCLK clock source */
		while (RCC_get_SYSCLK_source() != 0x08);
	}

}

void RCC_deinit(void) {
	/* Enable HSI clock */
	RCC->CR |= ((uint32_t)0x00000001);

	/* Disable HSE, CSS, PLLI2S and PLLSAI clocks */
	RCC->CR &= ((uint32_t)0xEAF6FFFF);

	/* Disable HSE bypass */
	RCC->CR &= ((uint32_t)0xFFFBFFFF);

	/* Reset PLLCFGR register */
	RCC->PLLCFGR = 0x24003010;

	/* Reset CFGR register */
	RCC->CFGR = 0x00000000;

	/* Disable all interrupts */
	RCC->CIR = 0x00000000;

	/* Reset DCKCFGR register */
	RCC->DCKCFGR = 0x00000000;

	/* Reset DCKCFGR2 register */
	RCC->DCKCFGR2 = 0x00000000;
}

/**
  * @}
  */

/**	@defgroup Internal, external, PLL, CSS and MCO clocks configuration functions
  *	@{
================================================================================
     ##### Internal, external, PLL, CSS and MCO clocks configuration #####
================================================================================
  */

void RCC_HSE_config(uint32_t RCC_HSE_New_State) {
	/* Reset HSEON and HSEBYP bits before configuring the HSE */
	RCC->CR &= RCC_HSE_OFF;

	/* Set new HSE configuration */
	if (RCC_HSE_New_State != RCC_HSE_OFF) {
		RCC->CR |= RCC_HSE_New_State;
	}
}

ErrorStatus RCC_HSE_startup_status(void) {
	uint32_t startup_counter = 0;
	FlagStatus hse_status = RESET;

	/* Wait until HSE is ready without exceeding timeout */
	do {
		hse_status = RCC_get_flag_status(RCC_FLAG_HSERDY);
		++startup_counter;
	} while ((startup_counter != HSE_STARTUP_TIMEOUT) && (hse_status == RESET));

	ErrorStatus status = ERROR;
	if (hse_status != RESET) {
		status = SUCCESS;
	} else {
		status = ERROR;
	}
	return status;
}

void RCC_HCLK_config(uint32_t RCC_SYSCLK_Div) {
	uint32_t tmp_reg = 0;

	/* Store current CFGR register value */
	tmp_reg = RCC->CFGR;

	/* Clear HPRE bits */
	tmp_reg &= ~RCC_CFGR_HPRE;

	/* Set HPRE bits with RCC_SYSCLK_Div value */
	tmp_reg |= RCC_SYSCLK_Div;

	/* Store new value to CFGR register */
	RCC->CFGR |= tmp_reg;
}

void RCC_PCLK1_config(uint32_t RCC_HCLK_Div) {
	uint32_t tmp_reg = 0;

	/* Store current CFGR register value */
	tmp_reg = RCC->CFGR;

	/* Clear PPRE1 bits */
	tmp_reg &= ~RCC_CFGR_PPRE1;

	/* Set PPRE1 bits with RCC_HCLK_Div value */
	tmp_reg |= RCC_HCLK_Div;

	/* Store new value to CFGR register */
	RCC->CFGR |= tmp_reg;

}

void RCC_PCLK2_config(uint32_t RCC_HCLK_Div) {
	uint32_t tmp_reg = 0;

	/* Store current CFGR register value */
	tmp_reg = RCC->CFGR;

	/* Clear PPRE2 bits */
	tmp_reg &= ~RCC_CFGR_PPRE2;

	/* Set PPRE2 bits with RCC_HCLK_Div value */
	tmp_reg |= RCC_HCLK_Div << 3;

	/* Store new value to CFGR register */
	RCC->CFGR |= tmp_reg;
}

void RCC_PLL_state(uint32_t RCC_PLL_New_State) {
	/* Reset PLLON bit */
	RCC->CR &= RCC_PLL_OFF;

	/* Set PLLON bit */
	if (RCC_PLL_New_State != RCC_PLL_OFF) {
		RCC->CR |= RCC_PLL_ON;
	}
}

void RCC_PLL_config(uint32_t RCC_PLL_Source, uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ, uint32_t PLLR) {
	/* Reset PLLON bit before configuring the PLL */
	RCC->CR &= RCC_PLL_OFF;

	RCC->PLLCFGR = PLLM | (PLLN << 6) | (((PLLP >> 1) -1) << 16) | (RCC_PLL_Source) | (PLLQ << 24) | (PLLR << 28);
	RCC->CR |= RCC_PLL_ON;
}

void RCC_SYSCLK_config(uint32_t RCC_SYSCLK_source) {
	uint32_t tmp_reg = 0;

	/* Store current CFGR register value */
	tmp_reg = RCC->CFGR;

	/* Clear SW bits */
	tmp_reg &= ~RCC_CFGR_SW;

	/* Set SW bits with RCC_SYSCLK_source value */
	tmp_reg |= RCC_SYSCLK_source;

	/* Store new value to CFGR register */
	RCC->CFGR = tmp_reg;
}

/**
  * @}
  */

/**	@defgroup System, AHB and APB busses clocks configuration functions
  *	@{
================================================================================
    	  ##### System, AHB and APB busses clocks configuration #####
================================================================================
  */

/**	@brief  Returns system clock source driver
  * @param  None
  * @retval The clock source of system clock. The returned value can be:
  *         0x00: HSI used as system clock
  *         0x04: HSE used as system clock
  *         0x08: PLL_P used as system clock
  *         0x0C: PLL_R used as system clock
  */
uint8_t RCC_get_SYSCLK_source(void) {
	return ((uint8_t) RCC->CFGR & RCC_CFGR_SWS);
}

void RCC_get_clocks_freq(RCC_Clocks_TypeDef *RCC_Clocks) {
	/* Get SYSCLK source */
	uint32_t tmp_reg = RCC_get_SYSCLK_source();
	uint32_t pll_source = 0, pllm = 2, pll_vco = 0, pllp = 2, pllr = 2;

	switch (tmp_reg) {
	case 0x00:					/* HSI used as system clock source */
		RCC_Clocks->SYSCLK_Freq = HSI_VALUE;
		break;

	case 0x04:					/* HSE used as system clock source */
		RCC_Clocks->SYSCLK_Freq = HSE_VALUE;
		break;

	case 0x08:					/* PLL_P used as system clock source */
		/*	PLL_VCO = (HSI_VALUE or HSE_VALUE / PLLM) * PLLN
		 *	SYSCLK = PLL_VCO / PLLP
		 */
		pll_source = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
		pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

		if (pll_source != 0) {	/* HSE used as PLL_P clock source */
			pll_vco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
		} else {				/* HSI used as PLL_P clock source */
			pll_vco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
		}

		pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) * 2;
		RCC_Clocks->SYSCLK_Freq = pll_vco / pllp;
		break;

	case 0x0C:					/* PLL_R used as system clock source */
		/*	PLL_VCO = (HSI_VALUE or HSE_VALUE / PLLM) * PLLN
		 *	SYSCLK = PLL_VCO / PLLP
		 */
		pll_source = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
		pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

		if (pll_source != 0) {	/* HSE used as PLL_P clock source */
			pll_vco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
		} else {				/* HSI used as PLL_P clock source */
			pll_vco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
		}

		pllr = (RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 28;
//		pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 28) + 1) * 2;
		RCC_Clocks->SYSCLK_Freq = pll_vco / pllr;
		break;

	default:
		RCC_Clocks->SYSCLK_Freq = HSI_VALUE;
		break;
	}

	/* Get HCLK, PCLK1, PCLK2 clocks frequencies */
	uint32_t presc;
	uint8_t AHB_APB_Prescalers[16] = {0, 0, 0, 0, 1, 2, 3, 4,
										1, 2, 3, 4, 6, 7, 8, 9};
	/* Get HCLK prescaler */
	tmp_reg = RCC->CFGR & RCC_CFGR_HPRE;
	tmp_reg = tmp_reg >> 4;
	presc = AHB_APB_Prescalers[tmp_reg];
	/* Evaluate HCLK clock frequency */
	RCC_Clocks->HCLK_Freq = RCC_Clocks->SYSCLK_Freq >> presc;

	/* Get PCLK1 prescaler */
	tmp_reg = RCC->CFGR & RCC_CFGR_PPRE1;
	tmp_reg = tmp_reg >> 10;
	presc = AHB_APB_Prescalers[tmp_reg];
	/* Evaluate PCLK1 clock frequency */
	RCC_Clocks->PCLK1_Freq = RCC_Clocks->HCLK_Freq >> presc;

	/* Get PCLK2 prescaler */
	tmp_reg = RCC->CFGR & RCC_CFGR_PPRE2;
	tmp_reg = tmp_reg >> 13;
	presc = AHB_APB_Prescalers[tmp_reg];
	/* Evaluate PCLK2 clock frequency */
	RCC_Clocks->PCLK2_Freq = RCC_Clocks->HCLK_Freq >> presc;
}

/**
  * @}
  */

/**	@defgroup Peripheral clock configuration functions
  *	@{
================================================================================
    				##### Peripheral clock configuration #####
================================================================================
  */

void RCC_AHB1_reset(uint32_t RCC_AHB1_Periph, FunctionalState New_State) {

}

void RCC_AHB2_reset(uint32_t RCC_AHB2_Periph, FunctionalState New_State) {

}

void RCC_AHB3_reset(uint32_t RCC_AHB3_Periph, FunctionalState New_State) {

}

void RCC_APB1_reset(uint32_t RCC_APB1_Periph, FunctionalState New_State) {

}

void RCC_APB2_reset(uint32_t RCC_APB2_Periph, FunctionalState New_State) {

}

void RCC_AHB1_set(uint32_t RCC_AHB1_Periph, FunctionalState New_State) {

}

void RCC_AHB2_set(uint32_t RCC_AHB2_Periph, FunctionalState New_State) {

}

void RCC_AHB3_set(uint32_t RCC_AHB3_Periph, FunctionalState New_State) {

}

void RCC_APB1_set(uint32_t RCC_APB1_Periph, FunctionalState New_State) {

}

void RCC_APB2_set(uint32_t RCC_APB2_Periph, FunctionalState New_State) {

}

void RCC_AHB1_LP_set(uint32_t RCC_AHB1_Periph, FunctionalState New_State) {

}

void RCC_AHB2_LP_set(uint32_t RCC_AHB2_Periph, FunctionalState New_State) {

}

void RCC_AHB3_LP_set(uint32_t RCC_AHB3_Periph, FunctionalState New_State) {

}

void RCC_APB1_LP_set(uint32_t RCC_APB1_Periph, FunctionalState New_State) {

}

void RCC_APB2_LP_set(uint32_t RCC_APB2_Periph, FunctionalState New_State) {

}

/**
  * @}
  */


/**	@defgroup RCC flags management functions
  *	@{
================================================================================
    					##### RCC flags management #####
================================================================================
  */

FlagStatus RCC_get_flag_status(uint8_t RCC_Flag) {
	/* Get the RCC register index */
	uint32_t reg_index = RCC_Flag >> 5;
	uint32_t reg_status = 0;
	if (reg_index == 0) {			/* Flag is in CR register */
		reg_status = RCC->CR;
	} else if (reg_index == 1) {	/* Flag is in BDCR register */
		reg_status = RCC->BDCR;
	} else if (reg_index == 2) {	/* Flag is in CSR register */
		reg_status = RCC->CSR;
	}

	/* Get the flag position */
	uint8_t flag_mask = 0x1F;		/* 0x1F = 0001 1111*/
	uint32_t flag_pos = RCC_Flag & flag_mask;
	FlagStatus bit_status = RESET;

	if ((reg_status & (1 << flag_pos)) != (uint32_t)RESET) {
		bit_status = SET;
	} else {
		bit_status = RESET;
	}

	return bit_status;
}

/**
  * @}
  */


/* END OF FILE ****************************************************************/
