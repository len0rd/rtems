/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsAArch64XilinxZynqMP
 *
 * @brief This source file contains the default MMU tables and setup.
 */

/*
 * Copyright (C) 2021 On-Line Applications Research Corporation (OAR)
 * Written by Kinsey Moore <kinsey.moore@oarcorp.com>
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

#include <bsp.h>
#include <bsp/start.h>
#include <bsp/aarch64-mmu.h>
#include <libcpu/mmu-vmsav8-64.h>

BSP_START_DATA_SECTION static const aarch64_mmu_config_entry
zynqmp_mmu_config_table[] = {
  AARCH64_MMU_DEFAULT_SECTIONS,
  {
    .begin = 0xf9000000U,
    .end = 0xf9100000U,
    .flags = AARCH64_MMU_DEVICE
  }, {
    .begin = 0xfd000000U,
    .end = 0xffc00000U,
    .flags = AARCH64_MMU_DEVICE
  }, {
    .begin = 0x80000000U,
    .end = 0x80100000U,
    .flags = 0
  }
};

/*
 * Make weak and let the user override.
 */
BSP_START_TEXT_SECTION void
zynqmp_setup_mmu_and_cache( void ) __attribute__ ((weak));

BSP_START_TEXT_SECTION void
zynqmp_setup_mmu_and_cache( void )
{
  aarch64_mmu_setup();

  aarch64_mmu_setup_translation_table(
    &zynqmp_mmu_config_table[ 0 ],
    RTEMS_ARRAY_SIZE( zynqmp_mmu_config_table )
  );

  aarch64_mmu_enable();
}

/*
 * Make weak and let the user override.
 */
BSP_START_TEXT_SECTION void zynqmp_setup_secondary_cpu_mmu_and_cache( void )
__attribute__ ( ( weak ) );

BSP_START_TEXT_SECTION void zynqmp_setup_secondary_cpu_mmu_and_cache( void )
{
  /* Perform basic MMU setup */
  aarch64_mmu_setup();

  /* Use the existing root page table already configured by CPU0 */
  _AArch64_Write_ttbr0_el1( (uintptr_t) bsp_translation_table_base );

  aarch64_mmu_enable();
}
