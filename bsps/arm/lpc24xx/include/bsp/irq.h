/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSImplClassicIntr
 *
 * @brief LPC24XX interrupt definitions.
 */

/*
 * Copyright (c) 2008-2012 embedded brains GmbH.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LIBBSP_ARM_LPC24XX_IRQ_H
#define LIBBSP_ARM_LPC24XX_IRQ_H

#ifndef ASM
  #include <rtems.h>
  #include <rtems/irq.h>
  #include <rtems/irq-extension.h>
#endif

/**
 * @addtogroup RTEMSImplClassicIntr
 *
 * @{
 */

#ifdef ARM_MULTILIB_ARCH_V4
  #define LPC24XX_IRQ_WDT 0
  #define LPC24XX_IRQ_SOFTWARE 1
  #define LPC24XX_IRQ_ARM_CORE_0 2
  #define LPC24XX_IRQ_ARM_CORE_1 3
  #define LPC24XX_IRQ_TIMER_0 4
  #define LPC24XX_IRQ_TIMER_1 5
  #define LPC24XX_IRQ_UART_0 6
  #define LPC24XX_IRQ_UART_1 7
  #define LPC24XX_IRQ_PWM 8
  #define LPC24XX_IRQ_I2C_0 9
  #define LPC24XX_IRQ_SPI_SSP_0 10
  #define LPC24XX_IRQ_SSP_1 11
  #define LPC24XX_IRQ_PLL 12
  #define LPC24XX_IRQ_RTC 13
  #define LPC24XX_IRQ_EINT_0 14
  #define LPC24XX_IRQ_EINT_1 15
  #define LPC24XX_IRQ_EINT_2 16
  #define LPC24XX_IRQ_EINT_3 17
  #define LPC24XX_IRQ_ADC_0 18
  #define LPC24XX_IRQ_I2C_1 19
  #define LPC24XX_IRQ_BOD 20
  #define LPC24XX_IRQ_ETHERNET 21
  #define LPC24XX_IRQ_USB 22
  #define LPC24XX_IRQ_CAN 23
  #define LPC24XX_IRQ_SD_MMC 24
  #define LPC24XX_IRQ_DMA 25
  #define LPC24XX_IRQ_TIMER_2 26
  #define LPC24XX_IRQ_TIMER_3 27
  #define LPC24XX_IRQ_UART_2 28
  #define LPC24XX_IRQ_UART_3 29
  #define LPC24XX_IRQ_I2C_2 30
  #define LPC24XX_IRQ_I2S 31

  #define BSP_INTERRUPT_VECTOR_COUNT 32
#else
  #define LPC24XX_IRQ_WDT 0
  #define LPC24XX_IRQ_TIMER_0 1
  #define LPC24XX_IRQ_TIMER_1 2
  #define LPC24XX_IRQ_TIMER_2 3
  #define LPC24XX_IRQ_TIMER_3 4
  #define LPC24XX_IRQ_UART_0 5
  #define LPC24XX_IRQ_UART_1 6
  #define LPC24XX_IRQ_UART_2 7
  #define LPC24XX_IRQ_UART_3 8
  #define LPC24XX_IRQ_PWM_1 9
  #define LPC24XX_IRQ_I2C_0 10
  #define LPC24XX_IRQ_I2C_1 11
  #define LPC24XX_IRQ_I2C_2 12
  #define LPC24XX_IRQ_SPI_SSP_0 14
  #define LPC24XX_IRQ_SSP_1 15
  #define LPC24XX_IRQ_PLL 16
  #define LPC24XX_IRQ_RTC 17
  #define LPC24XX_IRQ_EINT_0 18
  #define LPC24XX_IRQ_EINT_1 19
  #define LPC24XX_IRQ_EINT_2 20
  #define LPC24XX_IRQ_EINT_3 21
  #define LPC24XX_IRQ_ADC_0 22
  #define LPC24XX_IRQ_BOD 23
  #define LPC24XX_IRQ_USB 24
  #define LPC24XX_IRQ_CAN 25
  #define LPC24XX_IRQ_DMA 26
  #define LPC24XX_IRQ_I2S 27
  #define LPC24XX_IRQ_ETHERNET 28
  #define LPC24XX_IRQ_SD_MMC 29
  #define LPC24XX_IRQ_MCPWM 30
  #define LPC24XX_IRQ_QEI 31
  #define LPC24XX_IRQ_PLL_ALT 32
  #define LPC24XX_IRQ_USB_ACTIVITY 33
  #define LPC24XX_IRQ_CAN_ACTIVITY 34
  #define LPC24XX_IRQ_UART_4 35
  #define LPC24XX_IRQ_SSP_2 36
  #define LPC24XX_IRQ_LCD 37
  #define LPC24XX_IRQ_GPIO 38
  #define LPC24XX_IRQ_PWM 39
  #define LPC24XX_IRQ_EEPROM 40

  #define BSP_INTERRUPT_VECTOR_COUNT 41
#endif

#define LPC24XX_IRQ_PRIORITY_VALUE_MIN 0
#ifdef ARM_MULTILIB_ARCH_V4
  #define LPC24XX_IRQ_PRIORITY_VALUE_MAX 15
#else
  #define LPC24XX_IRQ_PRIORITY_VALUE_MAX 31
#endif
#define LPC24XX_IRQ_PRIORITY_COUNT (LPC24XX_IRQ_PRIORITY_VALUE_MAX + 1)
#define LPC24XX_IRQ_PRIORITY_HIGHEST LPC24XX_IRQ_PRIORITY_VALUE_MIN
#define LPC24XX_IRQ_PRIORITY_LOWEST LPC24XX_IRQ_PRIORITY_VALUE_MAX

#ifndef ASM

void lpc24xx_irq_set_priority(rtems_vector_number vector, unsigned priority);

unsigned lpc24xx_irq_get_priority(rtems_vector_number vector);

#endif /* ASM */

/** @} */

#endif /* LIBBSP_ARM_LPC24XX_IRQ_H */
