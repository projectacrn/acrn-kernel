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

#include <linux/io.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/hypervisor.h>
#include <asm/acrn.h>
#include "sbuf.h"

static inline bool sbuf_is_empty(shared_buf_t *sbuf)
{
	return (sbuf->head == sbuf->tail);
}

static inline __u32 sbuf_next_ptr(__u32 pos,
		__u32 span, __u32 scope)
{
	pos += span;
	pos = (pos >= scope) ? (pos - scope) : pos;
	return pos;
}

void *acrn_sbuf_get_data_ptr(shared_buf_t *sbuf)
{
	if (sbuf_is_empty(sbuf))
		return NULL;

	return (void *)sbuf + SBUF_HEAD_SIZE + sbuf->head;
}

void acrn_sbuf_move_next(shared_buf_t *sbuf)
{
	sbuf->head = sbuf_next_ptr(sbuf->head, sbuf->ele_size, sbuf->size);
}

int acrn_sbuf_setup(uint16_t vm_id, uint16_t vcpu_id, __u32 sbuf_id, __u64 gpa)
{
	struct acrn_sbuf_param *asp;
	int ret;

	if (x86_hyper_type != X86_HYPER_ACRN)
		return -ENODEV;

	asp = kzalloc(sizeof(*asp), GFP_KERNEL);
	if (!asp)
		return -ENOMEM;
	asp->vcpu_id = vcpu_id;
	asp->sbuf_id = sbuf_id;
	asp->gpa = gpa;

	ret = hcall_set_sbuf(vm_id, virt_to_phys(asp));
	kfree(asp);
	return ret;
}
