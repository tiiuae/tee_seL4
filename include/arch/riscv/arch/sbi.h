/* SPDX-License-Identifier: BSD-3-Clause */

/* Copyright (c) 2010-2017, The Regents of the University of California
 * (Regents).  All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Regents nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
 * SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING
 * OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF REGENTS HAS
 * BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, PROVIDED
 * HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
 * MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 */

/* This file is copied from RISC-V tools/linux project, it might change for
 * new spec releases.
 */

#pragma once


#include <stdint.h>
#include <config.h>

/* See https://github.com/riscv/riscv-sbi-doc/blob/master/riscv-sbi.adoc for
 * details about these command codes, they are the "legacy extensions"
 * introduced by BBL and supported by OpenSBI.
 */
#define SBI_SET_TIMER 0
#define SBI_CONSOLE_PUTCHAR 1
#define SBI_CONSOLE_GETCHAR 2
#define SBI_CLEAR_IPI 3
#define SBI_SEND_IPI 4
#define SBI_REMOTE_FENCE_I 5
#define SBI_REMOTE_SFENCE_VMA 6
#define SBI_REMOTE_SFENCE_VMA_ASID 7
#define SBI_SHUTDOWN 8
/* The values 9 - 15 are reserved. */

#ifdef CONFIG_HSS_IHC_SYSCALL
/* From linux/drivers/mailbox/mailbox-miv-ihc.c */
#define SBI_EXT_VENDOR_START                0x09000000
#define MICROCHIP_TECHNOLOGY_MVENDOR_ID     0x029

#define SBI_EXT_MICROCHIP_TECHNOLOGY \
            (SBI_EXT_VENDOR_START | MICROCHIP_TECHNOLOGY_MVENDOR_ID)

enum {
    SBI_EXT_IHC_INIT = 0x0,
    SBI_EXT_IHC_TX = 0x1,
    SBI_EXT_IHC_RX = 0x2,
};

/* From linux/arch/riscv/include/asm/sbi.h */
typedef struct {
    long error;
    long value;
} sbiret_t;

/* From linux/arch/riscv/kernel/sbi.c */
static inline sbiret_t sbi_ecall(word_t ext, word_t fid,
            word_t arg0, word_t arg1, word_t arg2,
            word_t arg3, word_t arg4,
            word_t arg5)
{
    sbiret_t ret;

    register word_t a0 asm("a0") = arg0;
    register word_t a1 asm("a1") = arg1;
    register word_t a2 asm("a2") = arg2;
    register word_t a3 asm("a3") = arg3;
    register word_t a4 asm("a4") = arg4;
    register word_t a5 asm("a5") = arg5;
    register word_t a6 asm("a6") = fid;
    register word_t a7 asm("a7") = ext;


    asm volatile ("ecall"
              : "+r" (a0), "+r" (a1)
              : "r" (a2), "r" (a3), "r" (a4), "r" (a5), "r" (a6), "r" (a7)
              : "memory");

    ret.error = a0;
    ret.value = a1;

    return ret;
}

static inline sbiret_t sbi_ihc_call(word_t cmd, word_t channel, word_t buff_addr)
{
    if (cmd >= SBI_EXT_IHC_INIT && cmd <= SBI_EXT_IHC_RX)
        return sbi_ecall(SBI_EXT_MICROCHIP_TECHNOLOGY, cmd, channel, buff_addr, 0, 0, 0, 0);

    /* Silently do nothing for unknown cmds */
    sbiret_t ret = { 0 };
    return ret;
}

#endif /* CONFIG_HSS_IHC_SYSCALL */

static inline word_t sbi_call(word_t cmd,
                              word_t arg_0,
                              word_t arg_1,
                              word_t arg_2)
{
    register word_t a0 asm("a0") = arg_0;
    register word_t a1 asm("a1") = arg_1;
    register word_t a2 asm("a2") = arg_2;
    register word_t a7 asm("a7") = cmd;
    register word_t result asm("a0");
    asm volatile("ecall"
                 : "=r"(result)
                 : "r"(a0), "r"(a1), "r"(a2), "r"(a7)
                 : "memory");
    return result;
}

/* Lazy implementations until SBI is finalized */
#define SBI_CALL_0(which) sbi_call(which, 0, 0, 0)
#define SBI_CALL_1(which, arg0) sbi_call(which, arg0, 0, 0)
#define SBI_CALL_2(which, arg0, arg1) sbi_call(which, arg0, arg1, 0)

static inline void sbi_console_putchar(int ch)
{
    SBI_CALL_1(SBI_CONSOLE_PUTCHAR, ch);
}

static inline int sbi_console_getchar(void)
{
    return (int)(SBI_CALL_0(SBI_CONSOLE_GETCHAR));
}

static inline void sbi_set_timer(unsigned long long stime_value)
{
#if __riscv_xlen == 32
    SBI_CALL_2(SBI_SET_TIMER, stime_value, stime_value >> 32);
#else
    SBI_CALL_1(SBI_SET_TIMER, stime_value);
#endif
}

static inline void sbi_shutdown(void)
{
    SBI_CALL_0(SBI_SHUTDOWN);
}

static inline void sbi_clear_ipi(void)
{
    SBI_CALL_0(SBI_CLEAR_IPI);
}

static inline void sbi_send_ipi(const unsigned long *hart_mask)
{
    SBI_CALL_1(SBI_SEND_IPI, (word_t)hart_mask);
}

static inline void sbi_remote_fence_i(const unsigned long *hart_mask)
{
    SBI_CALL_1(SBI_REMOTE_FENCE_I, (word_t)hart_mask);
}

static inline void sbi_remote_sfence_vma(const unsigned long *hart_mask,
                                         unsigned long start,
                                         unsigned long size)
{
    SBI_CALL_1(SBI_REMOTE_SFENCE_VMA, (word_t)hart_mask);
}

static inline void sbi_remote_sfence_vma_asid(const unsigned long *hart_mask,
                                              unsigned long start,
                                              unsigned long size,
                                              unsigned long asid)
{
    SBI_CALL_1(SBI_REMOTE_SFENCE_VMA_ASID, (word_t)hart_mask);
}

