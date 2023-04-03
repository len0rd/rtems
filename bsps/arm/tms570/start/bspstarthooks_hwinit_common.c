/**
 * Hardware initialization functions that are common to all 'hwinit' bsp variants
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
#include <bsp/tms570.h>
#include <bsp/tms570-pinmux.h>
#include <bsp/tms570_selftest.h>
#include <bsp/tms570_hwinit.h>
#include <bsp/start.h>
#include <bsp/tms570_selftest_parity.h>

/**
 * @brief TMS570 system hardware initialization (HCG:systemInit)
 *
 */
/* SourceId : SYSTEM_SourceId_006 */
/* DesignId : SYSTEM_DesignId_006 */
/* Requirements : HL_SR471 */
void tms570_system_hw_init( void )
{
  uint32_t efc_check_status;

  /* Configure PLL control registers and enable PLLs.
   * The PLL takes (127 + 1024 * NR) oscillator cycles to acquire lock.
   * This initialization sequence performs all the tasks that are not
   * required to be done at full application speed while the PLL locks.
   */
  tms570_pll_init();

  /* Run eFuse controller start-up checks and start eFuse controller ECC self-test.
   * This includes a check for the eFuse controller error outputs to be stuck-at-zero.
   */
  efc_check_status = tms570_efc_check();

  /* Enable clocks to peripherals and release peripheral reset */
  tms570_periph_init();

  /* Configure device-level multiplexing and I/O multiplexing */
  tms570_pinmux_init();

  /* Enable external memory interface */
  TMS570_SYS1.GPREG1 |= TMS570_SYS1_GPREG1_EMIF_FUNC;

  if ( efc_check_status == 0U ) {
    /* Wait for eFuse controller self-test to complete and check results */
    if ( tms570_efc_check_self_test() == false ) { /* eFuse controller ECC logic self-test failed */
      bsp_selftest_fail_notification( EFCCHECK_FAIL1 );           /* device operation is not reliable */
    }
  } else if ( efc_check_status == 2U ) {
    /* Wait for eFuse controller self-test to complete and check results */
    if ( tms570_efc_check_self_test() == false ) { /* eFuse controller ECC logic self-test failed */
      bsp_selftest_fail_notification( EFCCHECK_FAIL1 );           /* device operation is not reliable */
    } else {
      bsp_selftest_fail_notification( EFCCHECK_FAIL2 );
    }
  } else {
    /* Empty */
  }

  /** - Set up flash address and data wait states based on the target CPU clock frequency
   * The number of address and data wait states for the target CPU clock frequency are specified
   * in the specific part's datasheet.
   */
  tms570_flash_init();

  /** - Configure the LPO such that HF LPO is as close to 10MHz as possible */
  tms570_trim_lpo_init();

  /// (TM) Per Errata EMIF#5 on the LC4357B, emif sdram should be configured when the clock
  ///   rate is still <40Mhz (ie: when running off OSC before switching to PLL). So this method should
  ///   be getting called BEFORE tms570_map_clock_init is called
  tms570_emif_sdram_init();

  /** - Wait for PLLs to start up and map clock domains to desired clock sources */
  tms570_map_clock_init();

  /** - set ECLK pins functional mode */
  TMS570_SYS1.SYSPC1 = 0U;

  /** - set ECLK pins default output value */
  TMS570_SYS1.SYSPC4 = 0U;

  /** - set ECLK pins output direction */
  TMS570_SYS1.SYSPC2 = 1U;

  /** - set ECLK pins open drain enable */
  TMS570_SYS1.SYSPC7 = 0U;

  /** - set ECLK pins pullup/pulldown enable */
  TMS570_SYS1.SYSPC8 = 0U;

  /** - set ECLK pins pullup/pulldown select */
  TMS570_SYS1.SYSPC9 = 1U;

  /** - Setup ECLK */
  TMS570_SYS1.ECPCNTL = TMS570_SYS1_ECPCNTL_ECPSSEL * 0 |
                        TMS570_SYS1_ECPCNTL_ECPCOS * 0 |
                        TMS570_SYS1_ECPCNTL_ECPDIV( 8 - 1 );
}

/**
 * Helper method that will run a pbist test and blocks until it finishes
 * Reduces code duplication in start system start hooks
 */
void tms570_run_pbist_test(uint32_t region, uint32_t flags)
{
  /* Run PBIST on region */
  tms570_pbist_run( region, flags );

  /* Wait for PBIST for region to be completed */
  /*SAFETYMCUSW 28 D MR:NA <APPROVED> "Hardware status bit read check" */
  while (!tms570_pbist_is_test_completed()) {
  }                                                  /* Wait */

  /* Check if PBIST on region passed the self-test */
  if (!tms570_pbist_is_test_passed()) {
    /* PBIST and region failed the self-test.
     * Need custom handler to check the memory failure
     * and to take the appropriate next step.
     */
    tms570_pbist_fail();
  }

  /* Disable PBIST clocks and disable memory self-test mode */
  tms570_pbist_stop();
}

int tms570_running_from_tcram( void )
{
  void *fncptr = (void*)tms570_system_hw_init;
  return ( fncptr >= (void*)TMS570_TCRAM_START_PTR ) &&
         ( fncptr < (void*)TMS570_TCRAM_WINDOW_END_PTR );
}

int tms570_running_from_sdram( void )
{
  void *fncptr = (void*)tms570_system_hw_init;
  return ( ( (void*)fncptr >= (void*)TMS570_SDRAM_START_PTR ) &&
           ( (void*)fncptr < (void*)TMS570_SDRAM_WINDOW_END_PTR ) );
}


BSP_START_TEXT_SECTION void bsp_start_hook_1( void )
{
  /* At this point we can use objects outside the .start section  */
#if 0
  /* Do not run attempt to initialize MPU when code is running from SDRAM */
  if ( !tms570_running_from_sdram() ) {
    /*
     * MPU background areas setting has to be overlaid
     * if execution of code is required from external memory/SDRAM.
     * This region is non executable by default.
     */
    _mpuInit_();
  }
#endif

  rtems_cache_enable_instruction();
  rtems_cache_enable_data();

  bsp_start_copy_sections();
  bsp_start_clear_bss();
}

/*
 * Chip specific list of peripherals which should be tested
 * for functional RAM parity reporting
 */
const tms570_selftest_par_desc_t *const
tms570_selftest_par_list[] = {
  &tms570_selftest_par_het1_desc,
  &tms570_selftest_par_htu1_desc,
  &tms570_selftest_par_het2_desc,
  &tms570_selftest_par_htu2_desc,
  &tms570_selftest_par_adc1_desc,
  &tms570_selftest_par_adc2_desc,
  &tms570_selftest_par_can1_desc,
  &tms570_selftest_par_can2_desc,
  &tms570_selftest_par_can3_desc,
  &tms570_selftest_par_vim_desc,
  &tms570_selftest_par_dma_desc,
  &tms570_selftest_par_spi1_desc,
  &tms570_selftest_par_spi3_desc,
  &tms570_selftest_par_spi5_desc,
};

const int tms570_selftest_par_list_size =
  RTEMS_ARRAY_SIZE( tms570_selftest_par_list );
