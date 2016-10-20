/****************************************************************************
 * arch/xtensa/src/mips32/xtensa_dumpstate.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/board/board.h>
#include <arch/chip/core-isa.h>

#include "sched/sched.h"
#include "xtensa.h"

#ifdef CONFIG_DEBUG_ALERT

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_getsp
 ****************************************************************************/

/* I don't know if the builtin to get SP is enabled */

static inline uint32_t xtensa_getsp(void)
{
  register uint32_t sp;
#warning Missing logic
  return sp;
}

/****************************************************************************
 * Name: xtensa_stackdump
 ****************************************************************************/

static void xtensa_stackdump(uint32_t sp, uint32_t stack_base)
{
  uint32_t stack ;

  for (stack = sp & ~0x1f; stack < stack_base; stack += 32)
    {
      uint32_t *ptr = (uint32_t *)stack;
      _alert("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
             stack, ptr[0], ptr[1], ptr[2], ptr[3],
             ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}

/****************************************************************************
 * Name: xtensa_registerdump
 ****************************************************************************/

static inline void xtensa_registerdump(void)
{
  uint32_ *regs = CURRENT_REGS;
  /* Are user registers available from interrupt processing? */

  if (regs != NULL)
    {
      _alert("   PC: %08lx    PS: %08lx\n",
             (unsigned long)regs[REG_PC], (unsigned long)regs[REG_PS]);
      _alert("   A0: %08lx    A1: %08lx    A2: %08lx    A3: %08lx\n",
             (unsigned long)regs[REG_A0], (unsigned long)regs[REG_A1],
             (unsigned long)regs[REG_A2], (unsigned long)regs[REG_A3]);
      _alert("   A4: %08lx    A5: %08lx    A6: %08lx    A7: %08lx\n",
             (unsigned long)regs[REG_A4], (unsigned long)regs[REG_A5],
             (unsigned long)regs[REG_A6], (unsigned long)regs[REG_A7]);
      _alert("   A8: %08lx    A9: %08lx   A10: %08lx   A11: %08lx\n",
             (unsigned long)regs[REG_A8], (unsigned long)regs[REG_A9],
             (unsigned long)regs[REG_A10], (unsigned long)regs[REG_A11]);
      _alert("  A12: %08lx   A13: %08lx   A14: %08lx   A15: %08lx\n",
             (unsigned long)regs[REG_A12], (unsigned long)regs[REG_A13],
             (unsigned long)regs[REG_A14], (unsigned long)regs[REG_A15]);
      _alert("  SAR: %08lx CAUSE: %08lx VADDR: %08lx\n",
             (unsigned long)regs[REG_SAR], (unsigned long)regs[REG_EXCCAUSE],
             (unsigned long)regs[REG_EXCVADDR]);
#ifdef XTENSA_HAVE_LOOPS
      _alert(" LBEG: %08lx  LEND: %08lx  LCNT: %08lx\n",
             (unsigned long)regs[REG_LBEG], (unsigned long)regs[REG_LEND],
             (unsigned long)regs[REG_LCOUNT]);
#endif
#ifndef CONFIG_XTENSA_CALL0_ABI
      _alert(" TMP0: %08lx  TMP1: %08lx  TMP2: %08lx\n",
             (unsigned long)regs[REG_TMP0], (unsigned long)regs[REG_TMP1],
             (unsigned long)regs[REG_TMP2]);
#endif
#ifdef CONFIG_XTENSA_USE_SWPRI
      _alert(" VPRI: %08lx\n",
             (unsigned long)regs[REG_VPRI]);
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_dumpstate
 ****************************************************************************/

void xtensa_dumpstate(void)
{
  struct tcb_s *rtcb = this_task();
  uint32_t sp = xtensa_getsp();
  uint32_t ustackbase;
  uint32_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  uint32_t istackbase;
  uint32_t istacksize;
#endif

  /* Get the limits on the user stack memory */

  if (rtcb->pid == 0)
    {
      ustackbase = g_idle_topstack - 4;
      ustacksize = CONFIG_IDLETHREAD_STACKSIZE;
    }
  else
    {
      ustackbase = (uint32_t)rtcb->adj_stack_ptr;
      ustacksize = (uint32_t)rtcb->adj_stack_size;
    }

  /* Get the limits on the interrupt stack memory */

#warning REVISIT interrupt stack
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  istackbase = (uint32_t)&g_intstackbase;
  istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~3) - 4;

  /* Show interrupt stack info */

  _alert("sp:     %08x\n", sp);
  _alert("IRQ stack:\n");
  _alert("  base: %08x\n", istackbase);
  _alert("  size: %08x\n", istacksize);

  /* Does the current stack pointer lie within the interrupt
   * stack?
   */

  if (sp <= istackbase && sp > istackbase - istacksize)
    {
      /* Yes.. dump the interrupt stack */

      xtensa_stackdump(sp, istackbase);

      /* Extract the user stack pointer which should lie
       * at the base of the interrupt stack.
       */

      sp = g_intstackbase;
      _alert("sp:     %08x\n", sp);
    }

  /* Show user stack info */

  _alert("User stack:\n");
  _alert("  base: %08x\n", ustackbase);
  _alert("  size: %08x\n", ustacksize);
#else
  _alert("sp:         %08x\n", sp);
  _alert("stack base: %08x\n", ustackbase);
  _alert("stack size: %08x\n", ustacksize);
#endif

  /* Dump the user stack if the stack pointer lies within the allocated user
   * stack memory.
   */

  if (sp > ustackbase || sp <= ustackbase - ustacksize)
    {
#if !defined(CONFIG_ARCH_INTERRUPTSTACK) || CONFIG_ARCH_INTERRUPTSTACK < 4
      _alert("ERROR: Stack pointer is not within allocated stack\n");
#endif
    }
  else
    {
      xtensa_stackdump(sp, ustackbase);
    }

  /* Then dump the registers (if available) */

  xtensa_registerdump();
}

#endif /* CONFIG_ARCH_STACKDUMP */
