/*
 * virtio and hyperviosr service module (VHM): memory map
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright (c) 2017 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * BSD LICENSE
 *
 * Copyright (C) 2017 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Jason Chen CJ <jason.cj.chen@intel.com>
 *
 */

#ifndef __ACRN_VHM_MM_H__
#define __ACRN_VHM_MM_H__

#include <linux/vhm/vhm_ioctl_defs.h>
#include <linux/vhm/vhm_vm_mngt.h>

#define	MMU_MEM_ATTR_READ	0x00000001
#define	MMU_MEM_ATTR_WRITE	0x00000002
#define	MMU_MEM_ATTR_EXECUTE	0x00000004
#define MMU_MEM_ATTR_WB_CACHE   0x00000040
#define MMU_MEM_ATTR_WT_CACHE   0x00000080
#define MMU_MEM_ATTR_UNCACHED   0x00000100
#define MMU_MEM_ATTR_WC         0x00000200

#define MMU_MEM_ATTR_ALL	0x00000007
#define MMU_MEM_ATTR_WP		0x00000005
#define MMU_MEM_ATTR_ALL_WB	0x00000047
#define MMU_MEM_ATTR_ALL_WC	0x00000207

void *map_guest_phys(unsigned long vmid, u64 uos_phys, size_t size);
int unmap_guest_phys(unsigned long vmid, u64 uos_phys);
int set_mmio_map(unsigned long vmid, unsigned long guest_gpa,
	unsigned long host_gpa, unsigned long len, int prot);
int unset_mmio_map(unsigned long vmid, unsigned long guest_gpa,
	unsigned long host_gpa, unsigned long len, int prot);
int update_mem_map(unsigned long vmid, unsigned long guest_gpa,
	unsigned long host_gpa, unsigned long len, int prot);

int vhm_dev_mmap(struct file *file, struct vm_area_struct *vma);

int check_guest_mem(struct vhm_vm *vm);
void free_guest_mem(struct vhm_vm *vm);

int alloc_guest_memseg(struct vhm_vm *vm, struct vm_memseg *memseg);
int map_guest_memseg(struct vhm_vm *vm, struct vm_memmap *memmap);

#endif
