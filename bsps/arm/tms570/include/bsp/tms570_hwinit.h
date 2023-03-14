/// Common hardware initialization functions that must be implemented on all 'hwinit' bsp variants
#ifndef LIBBSP_ARM_TMS570_HWINIT_H
#define LIBBSP_ARM_TMS570_HWINIT_H
#include "tms570_hwinit_board.h"
#include <rtems.h>

#define TMS570_TCRAM_START_PTR       ( (void *) ( 0x08000000U ) )
#define TMS570_TCRAM_WINDOW_END_PTR  ( (void *) ( 0x08080000U ) )

#define TMS570_SDRAM_START_PTR      ( (void *) ( 0x80000000U ) )
#define TMS570_SDRAM_WINDOW_END_PTR ( (void *) ( 0xA0000000U ) )

/* Ti TMS570 core setup implemented in assembly */
void _esmCcmErrorsClear_( void );
void _coreEnableEventBusExport_( void );
void _errata_CORTEXR4_66_( void );
void _errata_CORTEXR4_57_( void );
void _coreEnableRamEcc_( void );
void _coreDisableRamEcc_( void );
void _mpuInit_( void );
void _mpuDisable_( void );

void tms570_memory_init( uint32_t ram );
void tms570_trim_lpo_init( void );
void tms570_flash_init( void );
void tms570_periph_init( void );

/// @brief Perform base hardware initialization. Calls other initialization methods
void tms570_system_hw_init( void );
void tms570_esm_init( void );
int tms570_running_from_tcram( void );
int tms570_running_from_sdram( void );
void tms570_run_pbist_test( uint32_t region, uint32_t flags );

#endif /* LIBBSP_ARM_TMS570_HWINIT_H */
