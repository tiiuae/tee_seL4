/*
 * Copyright 2014, General Dynamics C4 Systems
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <config.h>
#include <mode/types.h>
#include <stdint.h>

typedef unsigned long word_t;
typedef signed long sword_t;
/* for printf() formatting */
#define _seL4_word_fmt  l

typedef word_t vptr_t;
typedef word_t paddr_t;
typedef word_t pptr_t;
typedef word_t cptr_t;
typedef word_t node_id_t;
typedef word_t cpu_id_t;
typedef word_t dom_t;

typedef uint8_t  hw_asid_t;

enum hwASIDConstants {
    hwASIDMax = 255,
    hwASIDBits = 8
};

/* for libsel4 headers that the kernel shares */
typedef word_t seL4_Word;
typedef cptr_t seL4_CPtr;
typedef uint32_t seL4_Uint32;
typedef uint16_t seL4_Uint16;
typedef uint8_t seL4_Uint8;
typedef node_id_t seL4_NodeId;
typedef dom_t seL4_Domain;
typedef paddr_t seL4_PAddr;

typedef struct kernel_frame {
    paddr_t paddr;
    pptr_t pptr;
    int armExecuteNever;
    int userAvailable;
} kernel_frame_t;
