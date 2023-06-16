/**
 * @file
 *
 * @ingroup RTEMSBSPsARMTMS570
 *
 * @brief Select pin mapping according to selected chip.
 *        Defaults to TMS570LS3137ZWT for now.
 */
#ifndef __TMS570_PINS_H__
#define __TMS570_PINS_H__
#include <bspopts.h>

#if TMS570_VARIANT == 4357
    #include <bsp/tms570lc4357-pins.h>
#elif TMS570_VARIANT == 3137
    #include <bsp/tms570ls3137zwt-pins.h>
#else
    #warning "Unknown or no TMS570 BSP variant defined"
#endif

#endif /* __TMS570_PINS_H__ */
