/**
 * @file
 *
 * @ingroup RTEMSBSPsARMTMS570
 *
 * @brief Initialization of external memory/SDRAM interface.
 */

#include <stdint.h>
#include <bsp/tms570.h>
#include <bsp/tms570_hwinit.h>

/// (TM) TODO: EMIF timings can be tightened up for LC4357. Look at Halcogen configuration
///	Errata EMIF#5 on the LC4357B should also be considered when deciding
///	when this method is called
void tms570_emif_sdram_init( void )
{
    uint32_t dummy;

    /* Do not run attempt to initialize SDRAM when code is running from it */
    if ( ( (void*)tms570_emif_sdram_init >= (void*)TMS570_SDRAM_START_PTR ) &&
            ( (void*)tms570_emif_sdram_init <= (void*)TMS570_SDRAM_WINDOW_END_PTR ) )
        return;

    // Following the initialization procedure as described in EMIF-errata #5 for the tms570lc43
    // at EMIF clock rates >= 40Mhz
    // Note step one of this procedure is running this EMIF initialization sequence before PLL
    // and clocks are mapped/enabled
    // For additional details on startup procedure see tms570lc43 TRM s21.2.5.5.B

    // Set SDRAM timings. These are dependent on the EMIF CLK rate, which = VCLK3
    // Set these based on the final EMIF clock rate once PLL & VCLK is enabled
    TMS570_EMIF.SDTIMR  = (uint32_t)1U << 27U|
                (uint32_t)0U << 24U|
                (uint32_t)0U << 20U|
                (uint32_t)0U << 19U|
                (uint32_t)1U << 16U|
                (uint32_t)1U << 12U|
                (uint32_t)1U << 8U|
                (uint32_t)0U << 4U;

    /* Minimum number of ECLKOUT cycles from Self-Refresh exit to any command */
    // Also set this based on the final EMIF clk
    TMS570_EMIF.SDSRETR = 2;
    // Program the RR Field of SDRCR to provide 200us of initialization time
    // Per Errata#5, for EMIF startup, set this based on the non-VLCK3 clk rate.
    // The Errata is this register must be calculated as `SDRCR = 200us * EMIF_CLK`
    //  (typically this would be `SDRCR = (200us * EMIF_CLK) / 8` ) 
    //  Since the PLL's arent enabled yet, EMIF_CLK would be EXT_OSCIN / 2
    TMS570_EMIF.SDRCR = 1600;

    TMS570_EMIF.SDCR   = ((uint32_t)0U << 31U)|
            ((uint32_t)1U << 14U)|
            ((uint32_t)2U << 9U)|
            ((uint32_t)1U << 8U)|
            ((uint32_t)2U << 4U)|
            ((uint32_t)0); // pagesize = 256

    // Read of SDRAM memory location causes processor to wait until SDRAM Initialization completes
    dummy = *(volatile uint32_t*)TMS570_SDRAM_START_PTR;
    (void) dummy;

    // Program the RR field to the default Refresh Interval of the SDRAM
    // Program this to the correct interval for the VCLK3/EMIF_CLK rate
    // Do this in the typical way per TRM: SDRCR = ((200us * EMIF_CLK) / 8) + 1
    TMS570_EMIF.SDRCR = 1251;

    /* Place the EMIF in Self Refresh Mode For Clock Change          */
	/* Must only write to the upper byte of the SDCR to avoid        */
	/* a second initialization sequence                              */
	/* The byte address depends on endian (0x3U in LE, 0x00 in BE32) */
    *((volatile unsigned char *)(&TMS570_EMIF.SDCR) + 0x0U) = 0x80;
}
