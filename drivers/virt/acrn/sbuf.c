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

#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/hypervisor.h>
#include <asm/pgtable.h>
#include <asm/acrn.h>
#include "sbuf.h"
#include "acrn_drv.h"


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


static inline uint32_t sbuf_calculate_allocate_size(uint32_t ele_num,
						uint32_t ele_size)
{
	uint64_t sbuf_allocate_size;

	sbuf_allocate_size = ele_num * ele_size;
	sbuf_allocate_size +=  SBUF_HEAD_SIZE;
	if (sbuf_allocate_size > SBUF_MAX_SIZE) {
		pr_err("num=0x%x, size=0x%x exceed 0x%llx!\n",
			ele_num, ele_size, SBUF_MAX_SIZE);
		return 0;
	}

	/* align to PAGE_SIZE */
	return (sbuf_allocate_size + PAGE_SIZE - 1) & PAGE_MASK;
}

shared_buf_t *sbuf_allocate(uint32_t ele_num, uint32_t ele_size)
{
	shared_buf_t *sbuf;
	struct page *page;
	uint32_t sbuf_allocate_size;

	if (!ele_num || !ele_size) {
		pr_err("invalid parameter %s!\n", __func__);
		return NULL;
	}

	sbuf_allocate_size = sbuf_calculate_allocate_size(ele_num, ele_size);
	if (!sbuf_allocate_size)
		return NULL;

	page = alloc_pages(GFP_KERNEL | __GFP_ZERO,
					get_order(sbuf_allocate_size));
	if (page == NULL) {
		pr_err("failed to alloc pages!\n");
		return NULL;
	}

	sbuf = phys_to_virt(page_to_phys(page));
	sbuf->ele_num = ele_num;
	sbuf->ele_size = ele_size;
	sbuf->size = ele_num * ele_size;
	sbuf->magic = SBUF_MAGIC;
	pr_info("ele_num=0x%x, ele_size=0x%x allocated!\n",
		ele_num, ele_size);
	return sbuf;
}
EXPORT_SYMBOL(sbuf_allocate);

void sbuf_free(shared_buf_t *sbuf)
{
	uint32_t sbuf_allocate_size;

	if ((sbuf == NULL) || sbuf->magic != SBUF_MAGIC) {
		pr_err("invalid parameter %s\n", __func__);
		return;
	}

	sbuf_allocate_size = sbuf_calculate_allocate_size(sbuf->ele_num,
						sbuf->ele_size);
	if (!sbuf_allocate_size)
		return;

	sbuf->magic = 0;
	__free_pages((struct page *)virt_to_page(sbuf),
			get_order(sbuf_allocate_size));
}
EXPORT_SYMBOL(sbuf_free);

int sbuf_get(shared_buf_t *sbuf, uint8_t *data)
{
	const void *from;

	if ((sbuf == NULL) || (data == NULL))
		return -EINVAL;

	if (sbuf_is_empty(sbuf)) {
		/* no data available */
		return 0;
	}

	from = (void *)sbuf + SBUF_HEAD_SIZE + sbuf->head;

	memcpy(data, from, sbuf->ele_size);

	sbuf->head = sbuf_next_ptr(sbuf->head, sbuf->ele_size, sbuf->size);

	return sbuf->ele_size;
}
EXPORT_SYMBOL(sbuf_get);

shared_buf_t *sbuf_check_valid(uint32_t ele_num, uint32_t ele_size,
				void *vaddr)
{
	shared_buf_t *sbuf;

	if (!ele_num || !ele_size || !vaddr)
		return NULL;

	sbuf = (shared_buf_t *)vaddr;

	if ((sbuf->magic == SBUF_MAGIC) &&
		(sbuf->ele_num == ele_num) &&
		(sbuf->ele_size == ele_size)) {
		return sbuf;
	}

	return NULL;
}
EXPORT_SYMBOL(sbuf_check_valid);

shared_buf_t *sbuf_construct(uint32_t ele_num, uint32_t ele_size,
				void *vaddr)
{
	shared_buf_t *sbuf;

	if (!ele_num || !ele_size || !vaddr)
		return NULL;

	sbuf = (shared_buf_t *)vaddr;

	memset(sbuf, 0, SBUF_HEAD_SIZE);
	sbuf->magic = SBUF_MAGIC;
	sbuf->ele_num = ele_num;
	sbuf->ele_size = ele_size;
	sbuf->size = ele_num * ele_size;
	pr_info("construct sbuf at 0x%llx.\n", (unsigned long long)sbuf);
	return sbuf;
}
EXPORT_SYMBOL(sbuf_construct);

void sbuf_deconstruct(shared_buf_t *sbuf)
{
	if (sbuf == NULL)
		return;

	sbuf->magic = 0;
}
EXPORT_SYMBOL(sbuf_deconstruct);

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
EXPORT_SYMBOL(acrn_sbuf_setup);

