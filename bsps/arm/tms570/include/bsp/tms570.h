/* This file is generated by make_central_header.py */
/* Current script's version can be found at: */
/* https://github.com/AoLaD/rtems-tms570-utils/tree/headers/headers/python */

/*
 * Copyright (c) 2014-2015, Premysl Houdek <kom541000@gmail.com>
 *
 * Czech Technical University in Prague
 * Zikova 1903/4
 * 166 36 Praha 6
 * Czech Republic
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
*/
#ifndef LIBBSP_ARM_TMS570
#define LIBBSP_ARM_TMS570

#include <bsp/tms570-pins.h>

#include <bsp/ti_herc/reg_adc.h>
#include <bsp/ti_herc/reg_ccmsr.h>
#include <bsp/ti_herc/reg_crc.h>
#include <bsp/ti_herc/reg_dcan.h>
#include <bsp/ti_herc/reg_dcc.h>
#include <bsp/ti_herc/reg_dma.h>
#include <bsp/ti_herc/reg_dmm.h>
#include <bsp/ti_herc/reg_efuse.h>
#include <bsp/ti_herc/reg_emacc.h>
#include <bsp/ti_herc/reg_emacm.h>
#include <bsp/ti_herc/reg_emif.h>
#include <bsp/ti_herc/reg_esm.h>
#include <bsp/ti_herc/reg_flash.h>
#include <bsp/ti_herc/reg_flex_ray.h>
#include <bsp/ti_herc/reg_gio.h>
#include <bsp/ti_herc/reg_htu.h>
#include <bsp/ti_herc/reg_i2c.h>
#include <bsp/ti_herc/reg_iomm.h>
#include <bsp/ti_herc/reg_lin.h>
#include <bsp/ti_herc/reg_mdio.h>
#include <bsp/ti_herc/reg_n2het.h>
#include <bsp/ti_herc/reg_pbist.h>
#include <bsp/ti_herc/reg_pll.h>
#include <bsp/ti_herc/reg_pmm.h>
#include <bsp/ti_herc/reg_rti.h>
#include <bsp/ti_herc/reg_rtp.h>
#include <bsp/ti_herc/reg_sci.h>
#include <bsp/ti_herc/reg_tcr.h>
#include <bsp/ti_herc/reg_tcram.h>
#include <bsp/ti_herc/reg_vim.h>
#include <bsp/ti_herc/reg_pom.h>
#include <bsp/ti_herc/reg_spi.h>
#include <bsp/ti_herc/reg_stc.h>
#include <bsp/ti_herc/reg_sys.h>
#include <bsp/ti_herc/reg_sys2.h>
#include <bsp/ti_herc/reg_pcr.h>

#include <bspopts.h>

#define TMS570_ADC1 (*(volatile tms570_adc_t*)0xFFF7C000)
#define TMS570_ADC2 (*(volatile tms570_adc_t*)0xFFF7C200)
#define TMS570_CCMSR (*(volatile tms570_ccmsr_t*)0xFFFFF600)
#define TMS570_CRC (*(volatile tms570_crc_t*)0xFE000000)
#if TMS570_VARIANT == 4357
#define TMS570_CRC2 (*(volatile tms570_crc_t*)0xFB000000)
#endif
#define TMS570_DCAN1 (*(volatile tms570_dcan_t*)0xFFF7DC00)
#define TMS570_DCAN2 (*(volatile tms570_dcan_t*)0xFFF7DE00)
#define TMS570_DCAN3 (*(volatile tms570_dcan_t*)0xFFF7E000)
#if TMS570_VARIANT == 4357
#define TMS570_DCAN4 (*(volatile tms570_dcan_t*)0xFFF7E200)
#endif
#define TMS570_DCC1 (*(volatile tms570_dcc_t*)0xFFFFEC00)
#define TMS570_DCC2 (*(volatile tms570_dcc_t*)0xFFFFF400)
#define TMS570_DMA (*(volatile tms570_dma_t*)0xFFFFF000)
#define TMS570_DMM (*(volatile tms570_dmm_t*)0xFFFFF700)
#define TMS570_EFUSE (*(volatile tms570_efuse_t*)0xFFF8C01C)
#define TMS570_EMACC (*(volatile tms570_emacc_t*)0xFCF78800)
#define TMS570_EMACM (*(volatile tms570_emacm_t*)0xFCF78000)
#define TMS570_EMIF (*(volatile tms570_emif_t*)0xFCFFE800)
#define TMS570_ESM (*(volatile tms570_esm_t*)0xFFFFF500)
#define TMS570_FLASH (*(volatile tms570_flash_t*)0xFFF87000)
#define TMS570_FLEX_RAY (*(volatile tms570_flex_ray_t*)0xFFF7C800)
#define TMS570_GIO (*(volatile tms570_gio_t*)0xFFF7BC00)
#define TMS570_GIO_PORTA (*(volatile tms570_gio_port_t*)0xFFF7BC34)
#define TMS570_GIO_PORTB (*(volatile tms570_gio_port_t*)0xFFF7BC54)
#define TMS570_GIO_PORTC (*(volatile tms570_gio_port_t*)0xFFF7BC74)
#define TMS570_GIO_PORTD (*(volatile tms570_gio_port_t*)0xFFF7BC94)
#define TMS570_GIO_PORTE (*(volatile tms570_gio_port_t*)0xFFF7BCB4)
#define TMS570_GIO_PORTF (*(volatile tms570_gio_port_t*)0xFFF7BCD4)
#define TMS570_GIO_PORTG (*(volatile tms570_gio_port_t*)0xFFF7BCF4)
#define TMS570_GIO_PORTH (*(volatile tms570_gio_port_t*)0xFFF7BD14)
#define TMS570_HTU1 (*(volatile tms570_htu_t*)0xFFF7A400)
#define TMS570_HTU2 (*(volatile tms570_htu_t*)0xFFF7A500)
#define TMS570_I2C (*(volatile tms570_i2c_t*)0xFFF7D400)
#if TMS570_VARIANT == 4357
#define TMS570_IOMM (*(volatile tms570_iomm_t*)0xFFFF1C00)
#else
#define TMS570_IOMM (*(volatile tms570_iomm_t*)0xFFFFEA00)
#endif
#define TMS570_PINMUX ((volatile uint32_t*)TMS570_IOMM.PINMMR)
#define TMS570_LIN (*(volatile tms570_lin_t*)0xFFF7E400)
#if TMS570_VARIANT == 4357
#define TMS570_LIN2 (*(volatile tms570_lin_t*)0xFFF7E600)
#endif
#define TMS570_MDIO (*(volatile tms570_mdio_t*)0xFCF78900)
#define TMS570_NHET1 (*(volatile tms570_nhet_t*)0xFFF7B800)
#define TMS570_NHET2 (*(volatile tms570_nhet_t*)0xFFF7B900)
#define TMS570_PBIST (*(volatile tms570_pbist_t*)0xFFFFE400)
#define TMS570_PLL (*(volatile tms570_pll_t*)0xFFFFE100)
#define TMS570_PMM (*(volatile tms570_pmm_t*)0xFFFF0000)
#define TMS570_RTI (*(volatile tms570_rti_t*)0xFFFFFC00)
#define TMS570_RTP (*(volatile tms570_rtp_t*)0xFFFFFA00)
#define TMS570_SCI (*(volatile tms570_sci_t*)0xFFF7E500)
#if TMS570_VARIANT == 4357
#define TMS570_SCI2 (*(volatile tms570_sci_t*)0xFFF7E700)
#endif
#define TMS570_TCR (*(volatile tms570_tcr_t*)0xFFF7C800)
#define TMS570_TCRAM1 (*(volatile tms570_tcram_t*)0xFFFFF800)
#define TMS570_TCRAM2 (*(volatile tms570_tcram_t*)0xFFFFF900)
#define TMS570_VIM (*(volatile tms570_vim_t*)0xFFFFFDEC)
#define TMS570_POM (*(volatile tms570_pom_t*)0xFFA04000)
#define TMS570_SPI1 (*(volatile tms570_spi_t*)0xFFF7F400)
#define TMS570_SPI2 (*(volatile tms570_spi_t*)0xFFF7F600)
#define TMS570_SPI3 (*(volatile tms570_spi_t*)0xFFF7F800)
#define TMS570_SPI4 (*(volatile tms570_spi_t*)0xFFF7FA00)
#define TMS570_SPI5 (*(volatile tms570_spi_t*)0xFFF7FC00)
#define TMS570_STC (*(volatile tms570_stc_t*)0xFFFFE600)
#define TMS570_SYS1 (*(volatile tms570_sys1_t*)0xFFFFFF00)
#define TMS570_SYS2 (*(volatile tms570_sys2_t*)0xFFFFE100)
#if TMS570_VARIANT == 4357
#define TMS570_PCR1 (*(volatile tms570_pcr_t*)0xFFFF1000)
#define TMS570_PCR2 (*(volatile tms570_pcr_t*)0xFCFF1000)
#define TMS570_PCR3 (*(volatile tms570_pcr_t*)0xFFF78000)
#else
#define TMS570_PCR1 (*(volatile tms570_pcr_t*)0xFFFFE000)
#endif
#endif /* LIBBSP_ARM_TMS570 */
