/** File Info
 * \copyright (c) 2023 Airbus U.S. Space and Defense
 * \author 
 * \date   2023-03-13
 *
 * Functions that must be implemented on a per-board basis for any hwinit bsp variants
 * These configure MCU peripherals that are specific to a particular board
 */
#ifndef __TMS570_HWINIT_BOARD_H__
#define __TMS570_HWINIT_BOARD_H__

/// @brief Initialize the External Memory InterFace (EMIF) peripheral
void tms570_emif_sdram_init(void);

/// @brief Initialize PLLs source divider/multipliers
void tms570_pll_init(void);

/// @brief Initialize the tms570 Global Clock Manager (GCM) registers which sub-divide the input
///     clock source (generally PLL) into the various peripheral clocks (VCLK1-3, etc)
void tms570_map_clock_init(void);

/// @brief Initialize the tms570 PINMUX peripheral. This maps signals to pin terminals 
void tms570_pinmux_init(void);

#endif /* __TMS570_HWINIT_BOARD_H__ */
