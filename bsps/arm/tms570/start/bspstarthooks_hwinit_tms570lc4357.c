/** @file
 * Modified from TI HalCoGen and rtems LS3137 source
 */
/*
 * Copyright (C) 2022 Airbus U.S. Space & Defense, Inc
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

#include <stdint.h>
#include <stdbool.h>
#include <bsp/tms570.h>
#include <bsp/tms570-pinmux.h>
#include <bsp/tms570_selftest.h>
#include <bsp/tms570_hwinit.h>
#include <bsp/ti_herc/errata_SSWF021_45.h>
#include <bsp/tms570_selftest_parity.h>
#include <bsp/start.h>

typedef enum Tms570ClockSources {
  TMS570_CLK_SRC_OSC  = 0x01, ///< External high-speed oscillator as clock source
  TMS570_CLK_SRC_PLL1 = 0x02, 
  TMS570_CLK_SRC_RESERVED = 0x04, ///< reserved. not tied to actual clock source
  TMS570_CLK_SRC_EXT_CLK1 = 0x08, 
  TMS570_CLK_SRC_LOW_FREQ_LPO = 0x10, 
  TMS570_CLK_SRC_HIGH_FREQ_LPO = 0x20, 
  TMS570_CLK_SRC_PLL2 = 0x40, 
  TMS570_CLK_SRC_EXT_CLK2 = 0x80, 
};

/**
 * @brief Setup all system PLLs (HCG:setupPLL)
 *
 */
void tms570_pll_init( void )
{
  uint32_t pll12_dis = TMS570_CLK_SRC_PLL1 | TMS570_CLK_SRC_PLL2;

  /* Disable PLL1 and PLL2 */
  TMS570_SYS1.CSDISSET = pll12_dis;

  /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Hardware status bit read check" */
  while ( ( TMS570_SYS1.CSDIS & pll12_dis ) != pll12_dis ) {
    /* Wait */
  }

  /* Clear Global Status Register */
  TMS570_SYS1.GLBSTAT = 0x301U;

  // Configure PLL control registers

  /**   - Setup pll control register 1:
   *     - Setup reset on oscillator slip
   *     - Setup bypass on pll slip
   *     - setup Pll output clock divider to max before Lock
   *     - Setup reset on oscillator fail
   *     - Setup reference clock divider
   *     - Setup Pll multiplier
   */
  TMS570_SYS1.PLLCTL1 =  (uint32_t)0x00000000U
                      |  (uint32_t)0x40000000U
                      |  (uint32_t)((uint32_t)0x1FU << 24U)
                      |  (uint32_t)0x00000000U
                      |  (uint32_t)((uint32_t)(4U - 1U)<< 16U)
                      |  (uint32_t)(0x4A00U);

  /**   - Setup pll control register 2
   *     - Setup spreading rate
   *     - Setup bandwidth adjustment
   *     - Setup internal Pll output divider
   *     - Setup spreading amount
   */
  TMS570_SYS1.PLLCTL2 =  ((uint32_t)255U << 22U)
                      |  ((uint32_t)7U << 12U)
                      |  ((uint32_t)(1U - 1U) << 9U)
                      |  61U;

  // Initialize Pll2

  /**   - Setup pll2 control register :
   *     - setup Pll output clock divider to max before Lock
   *     - Setup reference clock divider
   *     - Setup internal Pll output divider
   *     - Setup Pll multiplier
   */
  TMS570_SYS2.PLLCTL3 = ((uint32_t)(1U - 1U) << 29U)
                      | ((uint32_t)0x1FU << 24U)
                      | ((uint32_t)(8U - 1U)<< 16U)
                      | (0x9500U);

  // Enable PLL(s) to start up or Lock
  // Enable all clock sources except the following
  TMS570_SYS1.CSDIS = (TMS570_CLK_SRC_EXT_CLK2 | TMS570_CLK_SRC_EXT_CLK1 | TMS570_CLK_SRC_RESERVED);
}

/**
 * @brief Adjust Low-Frequency (LPO) oscillator (HCG:trimLPO)
 *
 */
/* SourceId : SYSTEM_SourceId_002 */
/* DesignId : SYSTEM_DesignId_002 */
/* Requirements : HL_SR468 */
void tms570_trim_lpo_init( void )
{
  /** @b Initialize Lpo: */
  /** Load TRIM values from OTP if present else load user defined values */
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Hardware status bit read check" */
  TMS570_SYS1.LPOMONCTL = TMS570_SYS1_LPOMONCTL_BIAS_ENABLE |
                          TMS570_SYS1_LPOMONCTL_OSCFRQCONFIGCNT * 0 |
                          TMS570_SYS1_LPOMONCTL_HFTRIM( 16 ) |
                          16; /* LFTRIM  */
}

/* FIXME */
enum tms570_flash_power_modes {
  TMS570_FLASH_SYS_SLEEP = 0U,     /**< Alias for flash bank power mode sleep   */
  TMS570_FLASH_SYS_STANDBY = 1U,   /**< Alias for flash bank power mode standby */
  TMS570_FLASH_SYS_ACTIVE = 3U     /**< Alias for flash bank power mode active  */
};

enum tms570_system_clock_source {
  TMS570_SYS_CLK_SRC_OSC = 0U,          /**< Alias for oscillator clock Source                */
  TMS570_SYS_CLK_SRC_PLL1 = 1U,         /**< Alias for Pll1 clock Source                      */
  TMS570_SYS_CLK_SRC_EXTERNAL1 = 3U,    /**< Alias for external clock Source                  */
  TMS570_SYS_CLK_SRC_LPO_LOW = 4U,      /**< Alias for low power oscillator low clock Source  */
  TMS570_SYS_CLK_SRC_LPO_HIGH = 5U,     /**< Alias for low power oscillator high clock Source */
  TMS570_SYS_CLK_SRC_PLL2 = 6U,         /**< Alias for Pll2 clock Source                      */
  TMS570_SYS_CLK_SRC_EXTERNAL2 = 7U,    /**< Alias for external 2 clock Source                */
  TMS570_SYS_CLK_SRC_VCLK = 9U          /**< Alias for synchronous VCLK1 clock Source         */
};

/**
 * @brief Setup Flash memory parameters and timing (HCG:setupFlash)
 */
/* SourceId : SYSTEM_SourceId_003 */
/* DesignId : SYSTEM_DesignId_003 */
/* Requirements : HL_SR457 */
void tms570_flash_init( void )
{
  /** - Setup flash read mode, address wait states and data wait states */
  TMS570_FLASH.FRDCNTL = TMS570_FLASH_FRDCNTL_RWAIT( 3 ) |
                         TMS570_FLASH_FRDCNTL_ASWSTEN |
                         TMS570_FLASH_FRDCNTL_ENPIPE;

  /** - Setup flash access wait states for bank 7 */
  TMS570_FLASH.FSMWRENA = TMS570_FLASH_FSMWRENA_WR_ENA( 0x5 );
  TMS570_FLASH.EEPROMCONFIG = TMS570_FLASH_EEPROMCONFIG_EWAIT( 3 ) |
                              TMS570_FLASH_EEPROMCONFIG_AUTOSUSP_EN * 0 |
                              TMS570_FLASH_EEPROMCONFIG_AUTOSTART_GRACE( 2 );

  /** - Disable write access to flash state machine registers */
  TMS570_FLASH.FSMWRENA = TMS570_FLASH_FSMWRENA_WR_ENA( 0xA );

  /** - Setup flash bank power modes */
  TMS570_FLASH.FBFALLBACK = TMS570_FLASH_FBFALLBACK_BANKPWR7(TMS570_FLASH_SYS_ACTIVE)
    | TMS570_FLASH_FBFALLBACK_BANKPWR1(TMS570_FLASH_SYS_ACTIVE)
    | TMS570_FLASH_FBFALLBACK_BANKPWR0(TMS570_FLASH_SYS_ACTIVE);
}

/**
 * @brief Power-up all peripherals and enable their clocks (HCG:periphInit)
 */
/* SourceId : SYSTEM_SourceId_004 */
/* DesignId : SYSTEM_DesignId_004 */
/* Requirements : HL_SR470 */
void tms570_periph_init( void )
{
  /** - Disable Peripherals before peripheral powerup*/
  TMS570_SYS1.CLKCNTL &= ~TMS570_SYS1_CLKCNTL_PENA;

  /** - Release peripherals from reset and enable clocks to all peripherals */
  /** - Power-up all peripherals */
  TMS570_PCR1.PSPWRDWNCLR0 = 0xFFFFFFFFU;
  TMS570_PCR1.PSPWRDWNCLR1 = 0xFFFFFFFFU;
  TMS570_PCR1.PSPWRDWNCLR2 = 0xFFFFFFFFU;
  TMS570_PCR1.PSPWRDWNCLR3 = 0xFFFFFFFFU;

  TMS570_PCR2.PSPWRDWNCLR0 = 0xFFFFFFFFU;
  TMS570_PCR2.PSPWRDWNCLR1 = 0xFFFFFFFFU;
  TMS570_PCR2.PSPWRDWNCLR2 = 0xFFFFFFFFU;
  TMS570_PCR2.PSPWRDWNCLR3 = 0xFFFFFFFFU;

  TMS570_PCR3.PSPWRDWNCLR0 = 0xFFFFFFFFU;
  TMS570_PCR3.PSPWRDWNCLR1 = 0xFFFFFFFFU;
  TMS570_PCR3.PSPWRDWNCLR2 = 0xFFFFFFFFU;
  TMS570_PCR3.PSPWRDWNCLR3 = 0xFFFFFFFFU;

  /** - Enable Peripherals */
  TMS570_SYS1.CLKCNTL |= TMS570_SYS1_CLKCNTL_PENA;
}

/**
 * @brief Setup chip clocks including to wait for PLLs locks (HCG:mapClocks)
 *
 */
/* SourceId : SYSTEM_SourceId_005 */
/* DesignId : SYSTEM_DesignId_005 */
/* Requirements : HL_SR469 */
void tms570_map_clock_init( void )
{
  uint32_t sys_csvstat, sys_csdis;

  TMS570_SYS2.HCLKCNTL = 1U;

  /** @b Initialize @b Clock @b Tree: */
  /** - Disable / Enable clock domain */
  TMS570_SYS1.CDDIS = ( 0U << 4U ) |  /* AVCLK 1 ON */
                      ( 1U << 5U ) |  /* AVCLK 2 OFF */
                      ( 0U << 8U ) |  /* VCLK3 ON */
                      ( 0U << 9U ) |  /* VCLK4 ON */
                      ( 0U << 10U ) | /* AVCLK 3 ON */
                      ( 0U << 11U );  /* AVCLK 4 ON */

  /* Work Around for Errata SYS#46:
   * Despite this being a LS3137 errata, hardware testing on the LC4357 indicates this wait is still necessary
   */
  sys_csvstat = TMS570_SYS1.CSVSTAT;
  sys_csdis = TMS570_SYS1.CSDIS;

  while ( ( sys_csvstat & ( ( sys_csdis ^ 0xFFU ) & 0xFFU ) ) !=
          ( ( sys_csdis ^ 0xFFU ) & 0xFFU ) ) {
    sys_csvstat = TMS570_SYS1.CSVSTAT;
    sys_csdis = TMS570_SYS1.CSDIS;
  }

  TMS570_SYS1.GHVSRC =  ((uint32_t)TMS570_SYS_CLK_SRC_PLL1 << 24U)
                      | ((uint32_t)TMS570_SYS_CLK_SRC_PLL1 << 16U)
                      | ((uint32_t)TMS570_SYS_CLK_SRC_PLL1 << 0U);

  /** - Setup RTICLK1 and RTICLK2 clocks */
  TMS570_SYS1.RCLKSRC = ((uint32_t)1U << 24U)        /* RTI2 divider (Not applicable for lock-step device)  */
                      | ((uint32_t)TMS570_SYS_CLK_SRC_VCLK << 16U) /* RTI2 clock source (Not applicable for lock-step device) */
                      | ((uint32_t)1U << 8U)         /* RTI1 divider */
                      | ((uint32_t)TMS570_SYS_CLK_SRC_VCLK << 0U); /* RTI1 clock source */

  /** - Setup asynchronous peripheral clock sources for AVCLK1 and AVCLK2 */
  TMS570_SYS1.VCLKASRC =  ((uint32_t)TMS570_SYS_CLK_SRC_VCLK << 8U)
                        | ((uint32_t)TMS570_SYS_CLK_SRC_VCLK << 0U);

  /** - Setup synchronous peripheral clock dividers for VCLK1, VCLK2, VCLK3 */
  TMS570_SYS1.CLKCNTL  = (TMS570_SYS1.CLKCNTL & 0xF0FFFFFFU)
                        | ((uint32_t)1U << 24U);
  TMS570_SYS1.CLKCNTL  = (TMS570_SYS1.CLKCNTL & 0xFFF0FFFFU)
                        | ((uint32_t)1U << 16U);

  TMS570_SYS2.CLK2CNTRL = (TMS570_SYS2.CLK2CNTRL & 0xFFFFFFF0U)
                        | ((uint32_t)1U << 0U);

  TMS570_SYS2.VCLKACON1 =   ((uint32_t)(1U - 1U) << 24U)
                          | ((uint32_t)0U << 20U)
                          | ((uint32_t)TMS570_SYS_CLK_SRC_VCLK << 16U)
                          | ((uint32_t)(1U - 1U) << 8U)
                          | ((uint32_t)0U << 4U)
                          | ((uint32_t)TMS570_SYS_CLK_SRC_VCLK << 0U);

  /* Now the PLLs are locked and the PLL outputs can be sped up */
  /* The R-divider was programmed to be 0xF. Now this divider is changed to programmed value */
  TMS570_SYS1.PLLCTL1 = (TMS570_SYS1.PLLCTL1 & 0xE0FFFFFFU) | (uint32_t)((uint32_t)(1U - 1U) << 24U);
  /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> " Clear and write to the volatile register " */
  TMS570_SYS2.PLLCTL3 = (TMS570_SYS2.PLLCTL3 & 0xE0FFFFFFU) | (uint32_t)((uint32_t)(1U - 1U) << 24U);

  /* Enable/Disable Frequency modulation */
  TMS570_SYS1.PLLCTL2 |= 0x00000000U;
}

#define PBIST_March13N_SP        0x00000008U  /**< March13 N Algo for 1 Port mem */

BSP_START_TEXT_SECTION void bsp_start_hook_0( void )
{
   /*
   * Initialize CPU RAM.
   * This function uses the system module's hardware for auto-initialization of memories and their
   * associated protection schemes. The CPU RAM is initialized by setting bit 0 of the MSIENA register.
   * Hence the value 0x1 passed to the function.
   * This function will initialize the entire CPU RAM and the corresponding ECC locations.
   */
  tms570_memory_init( 0x1U );
  while (_errata_SSWF021_45_both_plls(10) != 0) {
    // (TM) TODO: proper error handling (run off oscillator instead of PLL?)
    for (int ii = INT32_MAX; ii > 0; ii--) {
      ;
    }
  }

  /* check for power-on reset condition */
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Hardware status bit read check" */
  if ( ( TMS570_SYS1.SYSESR & TMS570_SYS1_SYSESR_PORST ) != 0U ) {
    /* clear all reset status flags */
    TMS570_SYS1.SYSESR = 0xFFFFU;

    /* continue with normal start-up sequence */
  }
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Hardware status bit read check" */
  else if ( ( TMS570_SYS1.SYSESR & TMS570_SYS1_SYSESR_OSCRST ) != 0U ) {
    /* Reset caused due to oscillator failure.
       Add user code here to handle oscillator failure */
  }
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Hardware status bit read check" */
  else if ( ( TMS570_SYS1.SYSESR & TMS570_SYS1_SYSESR_WDRST ) != 0U ) {
    /* Reset caused due
     *  1) windowed watchdog violation - Add user code here to handle watchdog violation.
     *  2) ICEPICK Reset - After loading code via CCS / System Reset through CCS
     */
    /* Check the WatchDog Status register */
    if ( TMS570_RTI.WDSTATUS != 0U ) {
      /* Add user code here to handle watchdog violation. */
      /* Clear the Watchdog reset flag in Exception Status register */
      TMS570_SYS1.SYSESR = TMS570_SYS1_SYSESR_WDRST;
    } else {
      /* Clear the ICEPICK reset flag in Exception Status register */
      TMS570_SYS1.SYSESR = TMS570_SYS1_SYSESR_WDRST;
    }
  }
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Hardware status bit read check" */
  else if ( ( TMS570_SYS1.SYSESR & TMS570_SYS1_SYSESR_CPURST ) != 0U ) {
    /* Reset caused due to CPU reset.
       CPU reset can be caused by CPU self-test completion, or
       by toggling the "CPU RESET" bit of the CPU Reset Control Register. */

    /* clear all reset status flags */
    TMS570_SYS1.SYSESR = TMS570_SYS1_SYSESR_CPURST;
  }
  /*SAFETYMCUSW 139 S MR:13.7 <APPROVED> "Hardware status bit read check" */
  else if ( ( TMS570_SYS1.SYSESR & TMS570_SYS1_SYSESR_SWRST ) != 0U ) {
    /* Reset caused due to software reset.
       Add user code to handle software reset. */
  } else {
    /* Reset caused by nRST being driven low externally.
       Add user code to handle external reset. */
  }

  _coreEnableEventBusExport_();

  /*
   * Check if there were ESM group3 errors during power-up.
   * These could occur during eFuse auto-load or during reads from flash OTP
   * during power-up. Device operation is not reliable and not recommended
   * in this case.
   * An ESM group3 error only drives the nERROR pin low. An external circuit
   * that monitors the nERROR pin must take the appropriate action to ensure that
   * the system is placed in a safe state, as determined by the application.
   */
  if ( ( TMS570_ESM.SR[ 2 ] ) != 0U ) {
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "for(;;) can be removed by adding "# if 0" and "# endif" in the user codes above and below" */
    for (;; ) {
    }           /* Wait */
  }

  /* Initialize System - Clock, Flash settings with Efuse self check */
  tms570_system_hw_init();

  /*
   * Run a diagnostic check on the memory self-test controller.
   * This function chooses a RAM test algorithm and runs it on an on-chip ROM.
   * The memory self-test is expected to fail. The function ensures that the PBIST controller
   * is capable of detecting and indicating a memory self-test failure.
   */
  tms570_pbist_self_check();

  tms570_run_pbist_test(STC_ROM_PBIST_RAM_GROUP, ( (uint32_t) PBIST_TripleReadSlow | (uint32_t) PBIST_TripleReadFast ));
  tms570_run_pbist_test(PBIST_ROM_PBIST_RAM_GROUP, ( (uint32_t) PBIST_TripleReadSlow | (uint32_t) PBIST_TripleReadFast ));

  if ( !tms570_running_from_tcram() ) {
    /*
     * The next sequence tests TCRAM, main TMS570 system operation RAM area.
     * The tests are destructive, lead the first to fill memory by 0xc5c5c5c5
     * and then to clear it to zero. The sequence is obliviously incompatible
     * with RTEMS image running from TCRAM area (code clears itself).
     *
     * But TCRAM clear leads to overwrite of stack which is used to store
     * value of bsp_start_hook_0 call return address from link register.
     *
     * If the bsp_start_hook_0 by jump to bsp_start_hook_0_done
     * then generated C code does not use any variable which
     * is stores on stack and code works OK even that memory
     * is cleared during bsp_start_hook_0 execution.
     *
     * The last assumption is a little fragile in respect to
     * code and compiler changes.
     */

    /* Disable RAM ECC before doing PBIST for Main RAM */
    _coreDisableRamEcc_();

    /* Run PBIST on CPU RAM.
     * The PBIST controller needs to be configured separately for single-port and dual-port SRAMs.
     * The CPU RAM is a single-port memory. The actual "RAM Group" for all on-chip SRAMs is defined in the
     * device datasheet.
     */
    tms570_run_pbist_test(0x08300020U, PBIST_March13N_SP);

    /*
     * Enable ECC checking for TCRAM accesses.
     * This function enables the CPU's ECC logic for accesses to B0TCM and B1TCM.
     */
    _coreEnableRamEcc_();
  } /* end of the code skipped for tms570_running_from_tcram() */

  /* Start PBIST on all dual-port memories */
  /* NOTE : Please Refer DEVICE DATASHEET for the list of Supported Dual port Memories.
     PBIST test performed only on the user selected memories in HALCoGen's GUI SAFETY INIT tab.
   */
  tms570_pbist_run( (uint32_t) 0x00000000U | /* EMAC RAM */
    (uint32_t) 0x00000000U |                 /* USB RAM */
    (uint32_t) 0x00000800U |                 /* DMA RAM */
    (uint32_t) 0x00000200U |                 /* VIM RAM */
    (uint32_t) 0x00000040U |                 /* MIBSPI1 RAM */
    (uint32_t) 0x00000080U |                 /* MIBSPI3 RAM */
    (uint32_t) 0x00000100U |                 /* MIBSPI5 RAM */
    (uint32_t) 0x00000004U |                 /* CAN1 RAM */
    (uint32_t) 0x00000008U |                 /* CAN2 RAM */
    (uint32_t) 0x00000010U |                 /* CAN3 RAM */
    (uint32_t) 0x00000400U |                 /* ADC1 RAM */
    (uint32_t) 0x00020000U |                 /* ADC2 RAM */
    (uint32_t) 0x00001000U |                 /* HET1 RAM */
    (uint32_t) 0x00040000U |                 /* HET2 RAM */
    (uint32_t) 0x00002000U |                 /* HTU1 RAM */
    (uint32_t) 0x00080000U |                 /* HTU2 RAM */
    (uint32_t) 0x00004000U |                 /* RTP RAM */
    (uint32_t) 0x00008000U,                  /* FRAY RAM */
    (uint32_t) PBIST_March13N_DP );

  if ( !tms570_running_from_tcram() ) {

    /* Test the CPU ECC mechanism for RAM accesses.
     * The checkBxRAMECC functions cause deliberate single-bit and double-bit errors in TCRAM accesses
     * by corrupting 1 or 2 bits in the ECC. Reading from the TCRAM location with a 2-bit error
     * in the ECC causes a data abort exception. The data abort handler is written to look for
     * deliberately caused exception and to return the code execution to the instruction
     * following the one that caused the abort.
     */
    // {TM) TODO: currently causes boot failure
    // tms570_check_tcram_ecc();

    /* Wait for PBIST for CPU RAM to be completed */
    /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Hardware status bit read check" */
    while ( !tms570_pbist_is_test_completed() ) {
    }                                                  /* Wait */

    /* Check if CPU RAM passed the self-test */
    if ( !tms570_pbist_is_test_passed() ) {
      /* CPU RAM failed the self-test.
       * Need custom handler to check the memory failure
       * and to take the appropriate next step.
       */
      tms570_pbist_fail();
    }

  } /* end of the code skipped for tms570_running_from_tcram() */

  /* Disable PBIST clocks and disable memory self-test mode */
  tms570_pbist_stop();

  /* Release the MibSPI1 modules from local reset.
   * This will cause the MibSPI1 RAMs to get initialized along with the parity memory.
   */
  TMS570_SPI1.GCR0 = TMS570_SPI_GCR0_nRESET;

  /* Release the MibSPI3 modules from local reset.
   * This will cause the MibSPI3 RAMs to get initialized along with the parity memory.
   */
  TMS570_SPI3.GCR0 = TMS570_SPI_GCR0_nRESET;

  /* Release the MibSPI5 modules from local reset.
   * This will cause the MibSPI5 RAMs to get initialized along with the parity memory.
   */
  TMS570_SPI5.GCR0 = TMS570_SPI_GCR0_nRESET;

  /* Enable parity on selected RAMs */
  tms570_enable_parity();

  /* Initialize all on-chip SRAMs except for MibSPIx RAMs
   * The MibSPIx modules have their own auto-initialization mechanism which is triggered
   * as soon as the modules are brought out of local reset.
   */
  /* The system module auto-init will hang on the MibSPI RAM if the module is still in local reset.
   */
  /* NOTE : Please Refer DEVICE DATASHEET for the list of Supported Memories and their channel numbers.
            Memory Initialization is perfomed only on the user selected memories in HALCoGen's GUI SAFETY INIT tab.
   */
  tms570_memory_init( ( (uint32_t) 1U << 1U ) |  /* DMA RAM */
    ( (uint32_t) 1U << 2U ) |                /* VIM RAM */
    ( (uint32_t) 1U << 5U ) |                /* CAN1 RAM */
    ( (uint32_t) 1U << 6U ) |                /* CAN2 RAM */
    ( (uint32_t) 1U << 10U ) |               /* CAN3 RAM */
    ( (uint32_t) 1U << 8U ) |                /* ADC1 RAM */
    ( (uint32_t) 1U << 14U ) |               /* ADC2 RAM */
    ( (uint32_t) 1U << 3U ) |                /* HET1 RAM */
    ( (uint32_t) 1U << 4U ) |                /* HTU1 RAM */
    ( (uint32_t) 1U << 15U ) |               /* HET2 RAM */
    ( (uint32_t) 1U << 16U )                 /* HTU2 RAM */
  );

  /* Disable parity */
  tms570_disable_parity();

  /*
   * Test the parity protection mechanism for peripheral RAMs
   * Refer DEVICE DATASHEET for the list of Supported Memories
   * with parity.
   */

  tms570_selftest_par_run( tms570_selftest_par_list,
    tms570_selftest_par_list_size );

#if 0
  /*
   * RTEMS VIM initialization is implemented by the function
   * bsp_interrupt_facility_initialize(). RTEMS does not
   * gain performance from use of vectors targets provided
   * directly by VIM. RTEMS require to route all interrupts
   * through _ARMV4_Exception_interrupt handler.
   *
   * But actual RTEMS VIM initialization lefts some registers
   * default values untouched. All registers values should be
   * ensured/configured in future probably.
   */

  /* Enable IRQ offset via Vic controller */
  _coreEnableIrqVicOffset_();

  /* Initialize VIM table */
  vimInit();
#endif

  /* Configure system response to error conditions signaled to the ESM group1 */
  tms570_esm_init();

  // (TM): see notes about not running _mpuInit_ below in bsp_start_hook_1
  // {TM) TODO: _cacheEnable_ currently causes boot failure
  // _cacheEnable_();

#if 1
  /*
   * Do not depend on link register to be restored to
   * correct value from stack. If TCRAM self test is enabled
   * the all stack content is zeroed there.
   */
  bsp_start_hook_0_done();
#endif
}
