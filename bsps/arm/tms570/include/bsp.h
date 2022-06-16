/**
 * @file
 *
 * @ingroup RTEMSBSPsARMTMS570
 *
 * @brief Global BSP definitions.
 */

/*
 * Copyright (c) 2014 Premysl Houdek <kom541000@gmail.com>
 *
 * Google Summer of Code 2014 at
 * Czech Technical University in Prague
 * Zikova 1903/4
 * 166 36 Praha 6
 * Czech Republic
 *
 * Based on LPC24xx and LPC1768 BSP
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_TMS570_BSP_H
#define LIBBSP_ARM_TMS570_BSP_H

/**
 * @defgroup RTEMSBSPsARMTMS570 TMS570
 *
 * @ingroup RTEMSBSPsARM
 *
 * @brief TMS570 Board Support Package.
 *
 * @{
 */

#include <bspopts.h>

#define BSP_FEATURE_IRQ_EXTENSION

#ifndef ASM

#include <rtems.h>
#include <bsp/default-initial-extension.h>

#if defined(ARM_TMS570LC4357)
    /// On the LC43 devboard, the High-Speed-External oscillator ("OSC") is 16MHz
    #define BSP_OSCILATOR_CLOCK 16000000
    /// (TM) TODO: This shouldnt be hardcoded. It should be solved for based on PLL register config
    ///     and external oscillator input frequency. This value should be equal to the HCLK domain frequency
    #define BSP_PLL_OUT_CLOCK 150000000
#elif defined(ARM_TMS570LS3137)
    #define BSP_OSCILATOR_CLOCK 8000000
    #define BSP_PLL_OUT_CLOCK 160000000
#else
    #warning "Unknown or no TMS570 BSP variant defined"
#endif

#endif /* ASM */

/* @} */

#endif /* LIBBSP_ARM_TMS570_BSP_H */
