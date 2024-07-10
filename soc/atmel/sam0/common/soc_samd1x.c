/*
 * Copyright (c) 2017 Google LLC.
 * Copyright (c) 2023 Ionut Catalin Pavel <iocapa@iocapa.com>
 * Copyright (c) 2023 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Atmel SAMD MCU series initialization code
 */

/* The CPU clock will be configured to the DT requested value,
 * and run via DFLL48M.
 *
 * Reference -> GCLK Gen 1 -> DFLL48M -> GCLK Gen 0 -> GCLK_MAIN
 *
 * GCLK Gen 0 -> GCLK_MAIN
 * GCLK Gen 1 -> DFLL48M (variable)
 * GCLK Gen 2 -> WDT @ 32768 Hz
 * GCLK Gen 3 -> ADC @ 8 MHz
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <soc.h>
#include <cmsis_core.h>

#ifdef FUSES_OSC32KCAL_ADDR
#define FUSES_OSC32K_CAL_ADDR		FUSES_OSC32KCAL_ADDR
#define FUSES_OSC32K_CAL_Pos		FUSES_OSC32KCAL_Pos
#define FUSES_OSC32K_CAL_Msk		FUSES_OSC32KCAL_Msk
#endif

#if !CONFIG_SOC_ATMEL_SAMD_OSC32K || CONFIG_SOC_ATMEL_SAMD_DEFAULT_AS_MAIN
#define osc32k_init()
#else
static inline void osc32k_init(void)
{
	uint32_t cal;

	/* Get calibration value */
	cal = (*((uint32_t *)FUSES_OSC32K_CAL_ADDR)
	    & FUSES_OSC32K_CAL_Msk) >> FUSES_OSC32K_CAL_Pos;

	SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_CALIB(cal)
			    | SYSCTRL_OSC32K_STARTUP(0x5) /* 34 cycles / ~1ms */
			    | SYSCTRL_OSC32K_RUNSTDBY
			    | SYSCTRL_OSC32K_EN32K
			    | SYSCTRL_OSC32K_ENABLE;

	while (!SYSCTRL->PCLKSR.bit.OSC32KRDY) {
	}
}
#endif

static inline void DFLL_Initialize( void )
{
	    /****************** DFLL Initialization  *********************************/

    SYSCTRL->DFLLCTRL.reg &= (uint16_t)(~SYSCTRL_DFLLCTRL_ONDEMAND_Msk);

    while((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY_Msk) != SYSCTRL_PCLKSR_DFLLRDY_Msk)
    {
        /* Waiting for the Ready state */
    }

    /* Load Calibration Value */
    uint8_t calibCoarse = (uint8_t)(((*((uint32_t*)0x00806020U + 1U)) >> 26U ) & 0x3fU);
    calibCoarse = (((calibCoarse) == 0x3FU) ? 0x1FU : (calibCoarse));

    SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE((uint32_t)calibCoarse) | SYSCTRL_DFLLVAL_FINE((uint32_t)512U);

    while((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY_Msk) != SYSCTRL_PCLKSR_DFLLRDY_Msk)
    {
        /* Waiting for the Ready state */
    }

    /* Configure DFLL */
    SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE_Msk ;

    while((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY_Msk) != SYSCTRL_PCLKSR_DFLLRDY_Msk)
    {
        /* Waiting for DFLL to be ready */
    }
}
static inline void GCLK0_Initialize( void )
{
	GCLK->GENCTRL.reg = GCLK_GENCTRL_SRC(7U) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_ID(0U);

    while((GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) == GCLK_STATUS_SYNCBUSY)
    {
        /* wait for the Generator 0 synchronization */
    }
}
static inline void GCLK1_Initialize( void )
{
	GCLK->GENCTRL.reg = GCLK_GENCTRL_SRC(3U) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_ID(1U);

    while((GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) == GCLK_STATUS_SYNCBUSY)
    {
        /* wait for the Generator 1 synchronization */
    }
}


void z_arm_platform_init(void)
{
	NVMCTRL->CTRLB.bit.RWS = NVMCTRL_CTRLB_RWS(3UL);

	//PORT_Initialize();
	 // Configure PA16 as output
//    PORT->Group[0].DIR.reg = 0x10000U;
//    PORT->Group[0].OUT.reg = 0x4000U;
//    PORT->Group[0].PINCFG[14].reg = 0x5U;
//    PORT->Group[0].PINCFG[16].reg = 0x0U;

//    PORT->Group[0].PMUX[7].reg = 0x0U;
//    PORT->Group[0].PMUX[8].reg = 0x0U;
   //PORT->Group[0].OUTSET.reg = ((uint32_t)1U << 16U);
   //SYSCTRL->OSC32K.reg = 0x0U;
    osc32k_init();
    
	DFLL_Initialize();

	GCLK0_Initialize();

	GCLK1_Initialize();

	
	/* Selection of the Generator and write Lock for EIC */
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(5U) | GCLK_CLKCTRL_GEN(0x0U)  | GCLK_CLKCTRL_CLKEN;

    /* Configure the APBC Bridge Clocks */
    PM->APBCMASK.reg |= 0x11cU;
	// Enable the clock for the PORT module
    PM->APBBMASK.reg |= PM_APBBMASK_PORT(3);
    /* Disable RC oscillator */
    //SYSCTRL->OSC8M.reg = 0x0U;
   
 }

