/* SPDX-License-Identifier: BSD-2-Clause */

/*
 * Copyright (c) 2011, 2020 embedded brains GmbH.  All rights reserved.
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

#include <bspopts.h>
#include <rtems/score/percpu.h>
#include <bsp/vectors.h>

#ifdef PPC_EXC_CONFIG_USE_FIXED_HANDLER

#define SCRATCH_0_REGISTER r0
#define SCRATCH_1_REGISTER r3
#define SCRATCH_2_REGISTER r4
#define SCRATCH_3_REGISTER r5
#define SCRATCH_4_REGISTER r6
#define SCRATCH_5_REGISTER r7
#define SCRATCH_6_REGISTER r8
#define SCRATCH_7_REGISTER r9
#define SCRATCH_8_REGISTER r10
#define SCRATCH_9_REGISTER r11
#define SCRATCH_10_REGISTER r12
#define FRAME_REGISTER r14

#define SCRATCH_0_OFFSET GPR0_OFFSET
#define SCRATCH_1_OFFSET GPR3_OFFSET
#define SCRATCH_2_OFFSET GPR4_OFFSET
#define SCRATCH_3_OFFSET GPR5_OFFSET
#define SCRATCH_4_OFFSET GPR6_OFFSET
#define SCRATCH_5_OFFSET GPR7_OFFSET
#define SCRATCH_6_OFFSET GPR8_OFFSET
#define SCRATCH_7_OFFSET GPR9_OFFSET
#define SCRATCH_8_OFFSET GPR10_OFFSET
#define SCRATCH_9_OFFSET GPR11_OFFSET
#define SCRATCH_10_OFFSET GPR12_OFFSET
#define FRAME_OFFSET PPC_EXC_INTERRUPT_FRAME_OFFSET

#ifdef RTEMS_PROFILING
.macro GET_TIME_BASE REG
#if defined(__PPC_CPU_E6500__)
	mfspr \REG, FSL_EIS_ATBL
#elif defined(ppc8540)
	mfspr	\REG, TBRL
#else /* ppc8540 */
	mftb	\REG
#endif /* ppc8540 */
.endm
#endif /* RTEMS_PROFILING */

	.global	ppc_exc_min_prolog_async_tmpl_normal
	.global ppc_exc_interrupt

ppc_exc_min_prolog_async_tmpl_normal:

	stwu	r1, -PPC_EXC_INTERRUPT_FRAME_SIZE(r1)
	PPC_REG_STORE	SCRATCH_1_REGISTER, SCRATCH_1_OFFSET(r1)
	li	SCRATCH_1_REGISTER, 0xffff8000

	/*
	 * We store the absolute branch target address here.  It will be used
	 * to generate the branch operation in ppc_exc_make_prologue().
	 */
	.int	ppc_exc_interrupt

ppc_exc_interrupt:

	/* Save non-volatile FRAME_REGISTER */
	PPC_REG_STORE	FRAME_REGISTER, FRAME_OFFSET(r1)

#ifdef RTEMS_PROFILING
	/* Get entry instant */
	GET_TIME_BASE	FRAME_REGISTER
	stw	FRAME_REGISTER, PPC_EXC_INTERRUPT_ENTRY_INSTANT_OFFSET(r1)
#endif /* RTEMS_PROFILING */

#ifdef __SPE__
	/* Enable SPE */
	mfmsr	FRAME_REGISTER
	oris	FRAME_REGISTER, FRAME_REGISTER, MSR_SPE >> 16
	mtmsr	FRAME_REGISTER
	isync

	/*
	 * Save high order part of SCRATCH_1_REGISTER here.  The low order part
	 * was saved in the minimal prologue.
	 */
	evmergehi	SCRATCH_1_REGISTER, SCRATCH_1_REGISTER, FRAME_REGISTER
	PPC_REG_STORE	FRAME_REGISTER, GPR3_OFFSET(r1)
#endif

#if defined(PPC_MULTILIB_FPU) || defined(PPC_MULTILIB_ALTIVEC)
	/* Enable FPU and/or AltiVec */
	mfmsr	FRAME_REGISTER
#ifdef PPC_MULTILIB_FPU
	ori	FRAME_REGISTER, FRAME_REGISTER, MSR_FP
#endif
#ifdef PPC_MULTILIB_ALTIVEC
	oris	FRAME_REGISTER, FRAME_REGISTER, MSR_VE >> 16
#endif
	mtmsr	FRAME_REGISTER
	isync
#endif

	/* Move frame pointer to non-volatile FRAME_REGISTER */
	mr	FRAME_REGISTER, r1

	/*
	 * Save volatile registers.  The SCRATCH_1_REGISTER has been saved in
	 * minimum prologue.
	 */
	PPC_GPR_STORE	SCRATCH_0_REGISTER, SCRATCH_0_OFFSET(r1)
#ifdef __powerpc64__
	PPC_GPR_STORE	r2, GPR2_OFFSET(r1)
	LA32	r2, .TOC.
#endif
	PPC_GPR_STORE	SCRATCH_2_REGISTER, SCRATCH_2_OFFSET(r1)
	GET_SELF_CPU_CONTROL	SCRATCH_2_REGISTER
	PPC_GPR_STORE	SCRATCH_3_REGISTER, SCRATCH_3_OFFSET(r1)
	PPC_GPR_STORE	SCRATCH_4_REGISTER, SCRATCH_4_OFFSET(r1)
	PPC_GPR_STORE	SCRATCH_5_REGISTER, SCRATCH_5_OFFSET(r1)
	PPC_GPR_STORE	SCRATCH_6_REGISTER, SCRATCH_6_OFFSET(r1)
	PPC_GPR_STORE	SCRATCH_7_REGISTER, SCRATCH_7_OFFSET(r1)
	PPC_GPR_STORE	SCRATCH_8_REGISTER, SCRATCH_8_OFFSET(r1)
	PPC_GPR_STORE	SCRATCH_9_REGISTER, SCRATCH_9_OFFSET(r1)
	PPC_GPR_STORE	SCRATCH_10_REGISTER, SCRATCH_10_OFFSET(r1)

	/* Load ISR nest level and thread dispatch disable level */
	lwz	SCRATCH_3_REGISTER, PER_CPU_ISR_NEST_LEVEL(SCRATCH_2_REGISTER)
	lwz	SCRATCH_4_REGISTER, PER_CPU_THREAD_DISPATCH_DISABLE_LEVEL(SCRATCH_2_REGISTER)

	/* Save SRR0, SRR1, CR, XER, CTR, and LR */
	mfsrr0	SCRATCH_0_REGISTER
	mfsrr1	SCRATCH_5_REGISTER
	mfcr	SCRATCH_6_REGISTER
	mfxer	SCRATCH_7_REGISTER
	mfctr	SCRATCH_8_REGISTER
	mflr	SCRATCH_9_REGISTER
	PPC_REG_STORE	SCRATCH_0_REGISTER, SRR0_FRAME_OFFSET(r1)
	PPC_REG_STORE	SCRATCH_5_REGISTER, SRR1_FRAME_OFFSET(r1)
	stw	SCRATCH_6_REGISTER, EXC_CR_OFFSET(r1)
	stw	SCRATCH_7_REGISTER, EXC_XER_OFFSET(r1)
	PPC_REG_STORE	SCRATCH_8_REGISTER, EXC_CTR_OFFSET(r1)
	PPC_REG_STORE	SCRATCH_9_REGISTER, EXC_LR_OFFSET(r1)

#ifdef __SPE__
	/* Save SPEFSCR and ACC */
	mfspr	SCRATCH_0_REGISTER, FSL_EIS_SPEFSCR
	evxor	SCRATCH_5_REGISTER, SCRATCH_5_REGISTER, SCRATCH_5_REGISTER
	evmwumiaa	SCRATCH_5_REGISTER, SCRATCH_5_REGISTER, SCRATCH_5_REGISTER
	stw	SCRATCH_0_REGISTER, PPC_EXC_SPEFSCR_OFFSET(r1)
	evstdd	SCRATCH_5_REGISTER, PPC_EXC_ACC_OFFSET(r1)
#endif

	/* Save volatile AltiVec context */
#ifdef PPC_MULTILIB_ALTIVEC
#ifdef __PPC_VRSAVE__
	mfvrsave	SCRATCH_0_REGISTER
	cmpwi	SCRATCH_0_REGISTER, 0
	bne	.Laltivec_save

.Laltivec_save_continue:
#else /* __PPC_VRSAVE__ */
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(0)
	stvx	v0, r1, SCRATCH_0_REGISTER
	mfvscr	v0
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(1)
	stvx	v1, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(2)
	stvx	v2, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(3)
	stvx	v3, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(4)
	stvx	v4, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(5)
	stvx	v5, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(6)
	stvx	v6, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(7)
	stvx	v7, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(8)
	stvx	v8, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(9)
	stvx	v9, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(10)
	stvx	v10, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(11)
	stvx	v11, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(12)
	stvx	v12, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(13)
	stvx	v13, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(14)
	stvx	v14, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(15)
	stvx	v15, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(16)
	stvx	v16, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(17)
	stvx	v17, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(18)
	stvx	v18, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(19)
	stvx	v19, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VSCR_OFFSET
	stvewx	v0, r1, SCRATCH_0_REGISTER
#endif /* __PPC_VRSAVE__ */
#endif /* PPC_MULTILIB_ALTIVEC */

#ifdef PPC_MULTILIB_FPU
	/* Save volatile FPU context */
	stfd	f0, PPC_EXC_MIN_FR_OFFSET(0)(r1)
	mffs	f0
	stfd	f1, PPC_EXC_MIN_FR_OFFSET(1)(r1)
	stfd	f2, PPC_EXC_MIN_FR_OFFSET(2)(r1)
	stfd	f3, PPC_EXC_MIN_FR_OFFSET(3)(r1)
	stfd	f4, PPC_EXC_MIN_FR_OFFSET(4)(r1)
	stfd	f5, PPC_EXC_MIN_FR_OFFSET(5)(r1)
	stfd	f6, PPC_EXC_MIN_FR_OFFSET(6)(r1)
	stfd	f7, PPC_EXC_MIN_FR_OFFSET(7)(r1)
	stfd	f8, PPC_EXC_MIN_FR_OFFSET(8)(r1)
	stfd	f9, PPC_EXC_MIN_FR_OFFSET(9)(r1)
	stfd	f10, PPC_EXC_MIN_FR_OFFSET(10)(r1)
	stfd	f11, PPC_EXC_MIN_FR_OFFSET(11)(r1)
	stfd	f12, PPC_EXC_MIN_FR_OFFSET(12)(r1)
	stfd	f13, PPC_EXC_MIN_FR_OFFSET(13)(r1)
	stfd	f0, PPC_EXC_MIN_FPSCR_OFFSET(r1)
#endif

	/* Increment ISR nest level and thread dispatch disable level */
	cmpwi	SCRATCH_3_REGISTER, 0
#ifdef RTEMS_PROFILING
	cmpwi	cr2, SCRATCH_3_REGISTER, 0
#endif
	addi	SCRATCH_3_REGISTER, SCRATCH_3_REGISTER, 1
	addi	SCRATCH_4_REGISTER, SCRATCH_4_REGISTER, 1
	stw	SCRATCH_3_REGISTER, PER_CPU_ISR_NEST_LEVEL(SCRATCH_2_REGISTER)
	stw	SCRATCH_4_REGISTER, PER_CPU_THREAD_DISPATCH_DISABLE_LEVEL(SCRATCH_2_REGISTER)

	/* Switch stack if necessary */
	mfspr	SCRATCH_0_REGISTER, SPRG1
	iselgt	r1, r1, SCRATCH_0_REGISTER

	/* Call fixed high level handler */
	bl	bsp_interrupt_dispatch
	PPC64_NOP_FOR_LINKER_TOC_POINTER_RESTORE

#ifdef RTEMS_PROFILING
	/* Update profiling data if necessary */
	bne	cr2, .Lprofiling_done
	GET_SELF_CPU_CONTROL	r3
	lwz	r4, PPC_EXC_INTERRUPT_ENTRY_INSTANT_OFFSET(FRAME_REGISTER)
	GET_TIME_BASE	r5
	bl	_Profiling_Outer_most_interrupt_entry_and_exit
	PPC64_NOP_FOR_LINKER_TOC_POINTER_RESTORE
.Lprofiling_done:
#endif /* RTEMS_PROFILING */

	/* Load some per-CPU variables */
	GET_SELF_CPU_CONTROL	SCRATCH_1_REGISTER
	lbz	SCRATCH_0_REGISTER, PER_CPU_DISPATCH_NEEDED(SCRATCH_1_REGISTER)
	lwz	SCRATCH_5_REGISTER, PER_CPU_ISR_DISPATCH_DISABLE(SCRATCH_1_REGISTER)
	lwz	SCRATCH_6_REGISTER, PER_CPU_THREAD_DISPATCH_DISABLE_LEVEL(SCRATCH_1_REGISTER)
	lwz	SCRATCH_3_REGISTER, PER_CPU_ISR_NEST_LEVEL(SCRATCH_1_REGISTER)

	/*
	 * Switch back to original stack (FRAME_REGISTER == r1 if we are still
	 * on the IRQ stack) and restore FRAME_REGISTER.
	 */
	mr	r1, FRAME_REGISTER
	PPC_REG_LOAD	FRAME_REGISTER, FRAME_OFFSET(r1)

	/* Decrement levels and determine thread dispatch state */
	xori	SCRATCH_0_REGISTER, SCRATCH_0_REGISTER, 1
	or	SCRATCH_0_REGISTER, SCRATCH_0_REGISTER, SCRATCH_5_REGISTER
	subi	SCRATCH_4_REGISTER, SCRATCH_6_REGISTER, 1
	or.	SCRATCH_0_REGISTER, SCRATCH_0_REGISTER, SCRATCH_4_REGISTER
	subi	SCRATCH_3_REGISTER, SCRATCH_3_REGISTER, 1

	/* Store thread dispatch disable and ISR nest levels */
	stw	SCRATCH_4_REGISTER, PER_CPU_THREAD_DISPATCH_DISABLE_LEVEL(SCRATCH_1_REGISTER)
	stw	SCRATCH_3_REGISTER, PER_CPU_ISR_NEST_LEVEL(SCRATCH_1_REGISTER)

	/*
	 * Check thread dispatch necessary, ISR dispatch disable and thread
	 * dispatch disable level.
	 */
	bne	.Lthread_dispatch_done

	/* Thread dispatch */
.Ldo_thread_dispatch:

	/* Set ISR dispatch disable and thread dispatch disable level to one */
	li	SCRATCH_0_REGISTER, 1
	stw	SCRATCH_0_REGISTER, PER_CPU_ISR_DISPATCH_DISABLE(SCRATCH_1_REGISTER)
	stw	SCRATCH_0_REGISTER, PER_CPU_THREAD_DISPATCH_DISABLE_LEVEL(SCRATCH_1_REGISTER)

	/*
	 * Call _Thread_Do_dispatch(), this function will enable interrupts.
	 * The r3 is SCRATCH_1_REGISTER.
	 */
	mfmsr	r4
	ori	r4, r4, MSR_EE
	bl	_Thread_Do_dispatch
	PPC64_NOP_FOR_LINKER_TOC_POINTER_RESTORE

	/* Disable interrupts */
	wrteei	0

	/* SCRATCH_1_REGISTER is volatile, we must set it again */
	GET_SELF_CPU_CONTROL	SCRATCH_1_REGISTER

	/* Check if we have to do the thread dispatch again */
	lbz	SCRATCH_0_REGISTER, PER_CPU_DISPATCH_NEEDED(SCRATCH_1_REGISTER)
	cmpwi	SCRATCH_0_REGISTER, 0
	bne	.Ldo_thread_dispatch

	/* We are done with thread dispatching */
	li	SCRATCH_0_REGISTER, 0
	stw	SCRATCH_0_REGISTER, PER_CPU_ISR_DISPATCH_DISABLE(SCRATCH_1_REGISTER)

.Lthread_dispatch_done:

	/* Restore volatile AltiVec context */
#ifdef PPC_MULTILIB_ALTIVEC
#ifdef __PPC_VRSAVE__
	mfvrsave	SCRATCH_0_REGISTER
	cmpwi	SCRATCH_0_REGISTER, 0
	bne	.Laltivec_restore

.Laltivec_restore_continue:
#else /* __PPC_VRSAVE__ */
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VSCR_OFFSET
	lvewx	v0, r1, SCRATCH_0_REGISTER
	mtvscr	v0
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(0)
	lvx	v0, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(1)
	lvx	v1, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(2)
	lvx	v2, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(3)
	lvx	v3, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(4)
	lvx	v4, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(5)
	lvx	v5, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(6)
	lvx	v6, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(7)
	lvx	v7, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(8)
	lvx	v8, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(9)
	lvx	v9, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(10)
	lvx	v10, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(11)
	lvx	v11, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(12)
	lvx	v12, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(13)
	lvx	v13, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(14)
	lvx	v14, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(15)
	lvx	v15, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(16)
	lvx	v16, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(17)
	lvx	v17, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(18)
	lvx	v18, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(19)
	lvx	v19, r1, SCRATCH_0_REGISTER
#endif /* __PPC_VRSAVE__ */
#endif /* PPC_MULTILIB_ALTIVEC */

#ifdef PPC_MULTILIB_FPU
	/* Restore volatile FPU context */
	lfd	f0, PPC_EXC_MIN_FPSCR_OFFSET(r1)
	mtfsf	0xff, f0
	lfd	f0, PPC_EXC_MIN_FR_OFFSET(0)(r1)
	lfd	f1, PPC_EXC_MIN_FR_OFFSET(1)(r1)
	lfd	f2, PPC_EXC_MIN_FR_OFFSET(2)(r1)
	lfd	f3, PPC_EXC_MIN_FR_OFFSET(3)(r1)
	lfd	f4, PPC_EXC_MIN_FR_OFFSET(4)(r1)
	lfd	f5, PPC_EXC_MIN_FR_OFFSET(5)(r1)
	lfd	f6, PPC_EXC_MIN_FR_OFFSET(6)(r1)
	lfd	f7, PPC_EXC_MIN_FR_OFFSET(7)(r1)
	lfd	f8, PPC_EXC_MIN_FR_OFFSET(8)(r1)
	lfd	f9, PPC_EXC_MIN_FR_OFFSET(9)(r1)
	lfd	f10, PPC_EXC_MIN_FR_OFFSET(10)(r1)
	lfd	f11, PPC_EXC_MIN_FR_OFFSET(11)(r1)
	lfd	f12, PPC_EXC_MIN_FR_OFFSET(12)(r1)
	lfd	f13, PPC_EXC_MIN_FR_OFFSET(13)(r1)
#endif

#ifdef __SPE__
	/* Load SPEFSCR and ACC */
	lwz	SCRATCH_3_REGISTER, PPC_EXC_SPEFSCR_OFFSET(r1)
	evldd	SCRATCH_4_REGISTER, PPC_EXC_ACC_OFFSET(r1)
#endif

	/*
	 * We must clear reservations here, since otherwise compare-and-swap
	 * atomic operations with interrupts enabled may yield wrong results.
	 * A compare-and-swap atomic operation is generated by the compiler
	 * like this:
	 *
	 *   .L1:
	 *     lwarx  r9, r0, r3
	 *     cmpw   r9, r4
	 *     bne-   .L2
	 *     stwcx. r5, r0, r3
	 *     bne-   .L1
	 *   .L2:
	 *
	 * Consider the following scenario.  A thread is interrupted right
	 * before the stwcx.  The interrupt updates the value using a
	 * compare-and-swap sequence.  Everything is fine up to this point.
	 * The interrupt performs now a compare-and-swap sequence which fails
	 * with a branch to .L2.  The current processor has now a reservation.
	 * The interrupt returns without further stwcx.  The thread updates the
	 * value using the unrelated reservation of the interrupt.
	 */
	li	SCRATCH_0_REGISTER, FRAME_OFFSET
	stwcx.	SCRATCH_0_REGISTER, r1, SCRATCH_0_REGISTER

	/* Load SRR0, SRR1, CR, XER, CTR, and LR */
	PPC_REG_LOAD	SCRATCH_5_REGISTER, SRR0_FRAME_OFFSET(r1)
	PPC_REG_LOAD	SCRATCH_6_REGISTER, SRR1_FRAME_OFFSET(r1)
	lwz	SCRATCH_7_REGISTER, EXC_CR_OFFSET(r1)
	lwz	SCRATCH_8_REGISTER, EXC_XER_OFFSET(r1)
	PPC_REG_LOAD	SCRATCH_9_REGISTER, EXC_CTR_OFFSET(r1)
	PPC_REG_LOAD	SCRATCH_10_REGISTER, EXC_LR_OFFSET(r1)

	/* Restore volatile registers */
	PPC_GPR_LOAD	SCRATCH_0_REGISTER, SCRATCH_0_OFFSET(r1)
#ifdef __powerpc64__
	PPC_GPR_LOAD	r2, GPR2_OFFSET(r1)
#endif
	PPC_GPR_LOAD	SCRATCH_1_REGISTER, SCRATCH_1_OFFSET(r1)
	PPC_GPR_LOAD	SCRATCH_2_REGISTER, SCRATCH_2_OFFSET(r1)

#ifdef __SPE__
	/* Restore SPEFSCR and ACC */
	mtspr	FSL_EIS_SPEFSCR, SCRATCH_3_REGISTER
	evmra	SCRATCH_4_REGISTER, SCRATCH_4_REGISTER
#endif

	/* Restore volatile registers */
	PPC_GPR_LOAD	SCRATCH_3_REGISTER, SCRATCH_3_OFFSET(r1)
	PPC_GPR_LOAD	SCRATCH_4_REGISTER, SCRATCH_4_OFFSET(r1)

	/* Restore SRR0, SRR1, CR, CTR, XER, and LR plus volatile registers */
	mtsrr0	SCRATCH_5_REGISTER
	PPC_GPR_LOAD	SCRATCH_5_REGISTER, SCRATCH_5_OFFSET(r1)
	mtsrr1	SCRATCH_6_REGISTER
	PPC_GPR_LOAD	SCRATCH_6_REGISTER, SCRATCH_6_OFFSET(r1)
	mtcr	SCRATCH_7_REGISTER
	PPC_GPR_LOAD	SCRATCH_7_REGISTER, SCRATCH_7_OFFSET(r1)
	mtxer	SCRATCH_8_REGISTER
	PPC_GPR_LOAD	SCRATCH_8_REGISTER, SCRATCH_8_OFFSET(r1)
	mtctr	SCRATCH_9_REGISTER
	PPC_GPR_LOAD	SCRATCH_9_REGISTER, SCRATCH_9_OFFSET(r1)
	mtlr	SCRATCH_10_REGISTER
	PPC_GPR_LOAD	SCRATCH_10_REGISTER, SCRATCH_10_OFFSET(r1)

	/* Pop stack */
	addi	r1, r1, PPC_EXC_INTERRUPT_FRAME_SIZE

	/* Return */
	rfi

#if defined(PPC_MULTILIB_ALTIVEC) && defined(__PPC_VRSAVE__)
.Laltivec_save:

	/*
	 * Let X be VRSAVE, calculate:
	 *
	 * Y = 0x77777777
	 * Z = X & Y
	 * Z = Z + Y
	 * X = X | Z
	 *
	 * Afterwards, we have in X for each group of four VR registers:
	 *
	 * 0111b, if VRSAVE group of four registers == 0
	 * 1XXXb, if VRSAVE group of four registers != 0
	 */
	lis	SCRATCH_5_REGISTER, 0x7777
	ori	SCRATCH_5_REGISTER, SCRATCH_5_REGISTER, 0x7777
	and	SCRATCH_6_REGISTER, SCRATCH_0_REGISTER, SCRATCH_5_REGISTER
	add	SCRATCH_6_REGISTER, SCRATCH_5_REGISTER, SCRATCH_6_REGISTER
	or	SCRATCH_0_REGISTER, SCRATCH_0_REGISTER, SCRATCH_6_REGISTER
	mtcr	SCRATCH_0_REGISTER

	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(0)
	stvx	v0, r1, SCRATCH_0_REGISTER

	/* Move VCSR to V0 */
	mfvscr	v0

	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(1)
	stvx	v1, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(2)
	stvx	v2, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(3)
	stvx	v3, r1, SCRATCH_0_REGISTER

	/* Save VCSR using V0 */
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VSCR_OFFSET
	stvewx	v0, r1, SCRATCH_0_REGISTER

	bf	4, .Laltivec_save_v8
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(4)
	stvx	v4, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(5)
	stvx	v5, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(6)
	stvx	v6, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(7)
	stvx	v7, r1, SCRATCH_0_REGISTER

.Laltivec_save_v8:

	bf	8, .Laltivec_save_v12
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(8)
	stvx	v8, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(9)
	stvx	v9, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(10)
	stvx	v10, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(11)
	stvx	v11, r1, SCRATCH_0_REGISTER

.Laltivec_save_v12:

	bf	12, .Laltivec_save_v16
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(12)
	stvx	v12, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(13)
	stvx	v13, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(14)
	stvx	v14, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(15)
	stvx	v15, r1, SCRATCH_0_REGISTER

.Laltivec_save_v16:

	bf	16, .Laltivec_save_continue
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(16)
	stvx	v16, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(17)
	stvx	v17, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(18)
	stvx	v18, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(19)
	stvx	v19, r1, SCRATCH_0_REGISTER

	b	.Laltivec_save_continue

.Laltivec_restore:

	/* Load VCSR using V0 */
	li	SCRATCH_5_REGISTER, PPC_EXC_MIN_VSCR_OFFSET
	lvewx	v0, r1, SCRATCH_5_REGISTER

	/* See comment at .Laltivec_save */
	lis	SCRATCH_5_REGISTER, 0x7777
	ori	SCRATCH_5_REGISTER, SCRATCH_5_REGISTER, 0x7777
	and	SCRATCH_6_REGISTER, SCRATCH_0_REGISTER, SCRATCH_5_REGISTER
	add	SCRATCH_6_REGISTER, SCRATCH_5_REGISTER, SCRATCH_6_REGISTER
	or	SCRATCH_0_REGISTER, SCRATCH_0_REGISTER, SCRATCH_6_REGISTER
	mtcr	SCRATCH_0_REGISTER

	/* Restore VCR using V0 */
	mtvscr	v0

	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(0)
	lvx	v0, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(1)
	lvx	v1, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(2)
	lvx	v2, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(3)
	lvx	v3, r1, SCRATCH_0_REGISTER

	bf	4, .Laltivec_restore_v8
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(4)
	lvx	v4, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(5)
	lvx	v5, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(6)
	lvx	v6, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(7)
	lvx	v7, r1, SCRATCH_0_REGISTER

.Laltivec_restore_v8:

	bf	8, .Laltivec_restore_v12
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(8)
	lvx	v8, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(9)
	lvx	v9, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(10)
	lvx	v10, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(11)
	lvx	v11, r1, SCRATCH_0_REGISTER

.Laltivec_restore_v12:

	bf	12, .Laltivec_restore_v16
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(12)
	lvx	v12, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(13)
	lvx	v13, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(14)
	lvx	v14, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(15)
	lvx	v15, r1, SCRATCH_0_REGISTER

.Laltivec_restore_v16:

	bf	16, .Laltivec_restore_continue
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(16)
	lvx	v16, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(17)
	lvx	v17, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(18)
	lvx	v18, r1, SCRATCH_0_REGISTER
	li	SCRATCH_0_REGISTER, PPC_EXC_MIN_VR_OFFSET(19)
	lvx	v19, r1, SCRATCH_0_REGISTER

	b	.Laltivec_restore_continue
#endif /* PPC_MULTILIB_ALTIVEC && __PPC_VRSAVE__ */

/* Symbol provided for debugging and tracing */
ppc_exc_interrupt_end:

#endif /* PPC_EXC_CONFIG_USE_FIXED_HANDLER */
