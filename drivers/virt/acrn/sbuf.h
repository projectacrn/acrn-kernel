// SPDX-License-Identifier: GPL-2.0
/*
 * ACRN shared buffer
 *
 * Copyright (c) 2022 Intel Corporation. All rights reserved.
 *
 * Authors:
 * 	Li Fei <fei1.li@intel.com>
 * 	Chen Conghui <conghui.chen@intel.com>
 */

#ifndef SHARED_BUF_H
#define SHARED_BUF_H

#include <linux/types.h>
#include <asm/acrn.h>
#include "acrn_drv.h"

static inline void sbuf_clear_flags(shared_buf_t *sbuf, uint64_t flags)
{
	sbuf->flags &= ~flags;
}

static inline void sbuf_set_flags(shared_buf_t *sbuf, uint64_t flags)
{
	sbuf->flags = flags;
}

static inline void sbuf_add_flags(shared_buf_t *sbuf, uint64_t flags)
{
	sbuf->flags |= flags;
}

shared_buf_t *sbuf_allocate(uint32_t ele_num, uint32_t ele_size);
void sbuf_free(shared_buf_t *sbuf);
int sbuf_get(shared_buf_t *sbuf, uint8_t *data);
shared_buf_t *sbuf_check_valid(uint32_t ele_num, uint32_t ele_size,
				void *vaddr);
shared_buf_t *sbuf_construct(uint32_t ele_num, uint32_t ele_size,
				void *vaddr);
void sbuf_deconstruct(shared_buf_t *sbuf);
int acrn_sbuf_setup(uint16_t vm_id, uint16_t vcpu_id, __u32 sbuf_id, __u64 gpa);
void *acrn_sbuf_get_data_ptr(shared_buf_t *sbuf);
void acrn_sbuf_move_next(struct shared_buf *sbuf);
#endif /* SHARED_BUF_H */
