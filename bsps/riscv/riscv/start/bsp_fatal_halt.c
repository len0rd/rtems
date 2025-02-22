/*
 * Copyright (c) 2018 embedded brains GmbH
 *
 * Copyright (c) 2015 University of York.
 * Hesham Almatary <hesham@alumni.york.ac.uk>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <bsp/riscv.h>
#include <bsp/fdt.h>
#include <rtems/score/cpuimpl.h>

#include <libfdt.h>

void _CPU_Fatal_halt( uint32_t source, CPU_Uint32ptr error )
{
  const char *fdt;
  int node;
  volatile uint32_t *sifive_test;

  fdt = bsp_fdt_get();

#ifdef RISCV_ENABLE_HTIF_SUPPORT
  node = fdt_node_offset_by_compatible(fdt, -1, "ucb,htif0");

  if (node >= 0) {
    htif_poweroff();
  }
#endif

#if RISCV_ENABLE_MPFS_SUPPORT != 0
  for(;;);
#endif

  node = fdt_node_offset_by_compatible(fdt, -1, "sifive,test0");
  sifive_test = riscv_fdt_get_address(fdt, node);

  while (true) {
    if (sifive_test != NULL) {
      *sifive_test = 0x5555;
    }
  }
}
