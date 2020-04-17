/*
 * hypercall definition
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
 */

#ifndef ACRN_HV_DEFS_H
#define ACRN_HV_DEFS_H

/*
 * Common structures for ACRN/VHM/DM
 */
#include "acrn_common.h"

/*
 * Common structures for HV/VHM
 */

#define _HC_ID(x, y) (((x)<<24)|(y))

#define HC_ID 0x80UL

/* general */
#define HC_ID_GEN_BASE               0x0UL
#define HC_GET_API_VERSION          _HC_ID(HC_ID, HC_ID_GEN_BASE + 0x00)
#define HC_SOS_OFFLINE_CPU          _HC_ID(HC_ID, HC_ID_GEN_BASE + 0x01)
/* this is the temporally added hypercall.
 * after HYPERVISOR_CALLBACK_VECTOR is switched in both kernel and hypervisor,
 * this will be removed.
 */
#define HC_SET_CALLBACK_VECTOR		_HC_ID(HC_ID, HC_ID_GEN_BASE + 0x02)
#define HC_GET_PLATFORM_INFO        _HC_ID(HC_ID, HC_ID_GEN_BASE + 0x03)

/* VM management */
#define HC_ID_VM_BASE               0x10UL
#define HC_CREATE_VM                _HC_ID(HC_ID, HC_ID_VM_BASE + 0x00)
#define HC_DESTROY_VM               _HC_ID(HC_ID, HC_ID_VM_BASE + 0x01)
#define HC_START_VM                 _HC_ID(HC_ID, HC_ID_VM_BASE + 0x02)
#define HC_PAUSE_VM                 _HC_ID(HC_ID, HC_ID_VM_BASE + 0x03)
#define HC_CREATE_VCPU              _HC_ID(HC_ID, HC_ID_VM_BASE + 0x04)
#define HC_RESET_VM                 _HC_ID(HC_ID, HC_ID_VM_BASE + 0x05)
#define HC_SET_VCPU_REGS            _HC_ID(HC_ID, HC_ID_VM_BASE + 0x06)

/* IRQ and Interrupts */
#define HC_ID_IRQ_BASE              0x20UL
#define HC_INJECT_MSI               _HC_ID(HC_ID, HC_ID_IRQ_BASE + 0x03)
#define HC_VM_INTR_MONITOR          _HC_ID(HC_ID, HC_ID_IRQ_BASE + 0x04)
#define HC_SET_IRQLINE              _HC_ID(HC_ID, HC_ID_IRQ_BASE + 0x05)

/* DM ioreq management */
#define HC_ID_IOREQ_BASE            0x30UL
#define HC_SET_IOREQ_BUFFER         _HC_ID(HC_ID, HC_ID_IOREQ_BASE + 0x00)
#define HC_NOTIFY_REQUEST_FINISH    _HC_ID(HC_ID, HC_ID_IOREQ_BASE + 0x01)

/* Guest memory management */
#define HC_ID_MEM_BASE              0x40UL
#define HC_VM_GPA2HPA               _HC_ID(HC_ID, HC_ID_MEM_BASE + 0x01)
#define HC_VM_SET_MEMORY_REGIONS    _HC_ID(HC_ID, HC_ID_MEM_BASE + 0x02)
#define HC_VM_WRITE_PROTECT_PAGE    _HC_ID(HC_ID, HC_ID_MEM_BASE + 0x03)

/* PCI assignment*/
#define HC_ID_PCI_BASE              0x50UL
#define HC_ASSIGN_PTDEV             _HC_ID(HC_ID, HC_ID_PCI_BASE + 0x00)
#define HC_DEASSIGN_PTDEV           _HC_ID(HC_ID, HC_ID_PCI_BASE + 0x01)
#define HC_VM_PCI_MSIX_REMAP        _HC_ID(HC_ID, HC_ID_PCI_BASE + 0x02)
#define HC_SET_PTDEV_INTR_INFO      _HC_ID(HC_ID, HC_ID_PCI_BASE + 0x03)
#define HC_RESET_PTDEV_INTR_INFO    _HC_ID(HC_ID, HC_ID_PCI_BASE + 0x04)
#define HC_ASSIGN_PCIDEV            _HC_ID(HC_ID, HC_ID_PCI_BASE + 0x05)
#define HC_DEASSIGN_PCIDEV          _HC_ID(HC_ID, HC_ID_PCI_BASE + 0x06)

/* DEBUG */
#define HC_ID_DBG_BASE              0x60UL
#define HC_SETUP_SBUF               _HC_ID(HC_ID, HC_ID_DBG_BASE + 0x00)
#define HC_SETUP_HV_NPK_LOG         _HC_ID(HC_ID, HC_ID_DBG_BASE + 0x01)
#define HC_PROFILING_OPS            _HC_ID(HC_ID, HC_ID_DBG_BASE + 0x02)
#define HC_GET_HW_INFO              _HC_ID(HC_ID, HC_ID_DBG_BASE + 0x03)

/* Power management */
#define HC_ID_PM_BASE               0x80UL
#define HC_PM_GET_CPU_STATE         _HC_ID(HC_ID, HC_ID_PM_BASE + 0x00)
#define HC_PM_SET_SSTATE_DATA       _HC_ID(HC_ID, HC_ID_PM_BASE + 0x01)

#define ACRN_DOM0_VMID (0UL)
#define ACRN_INVALID_VMID (-1)
#define ACRN_INVALID_HPA (-1UL)

/* Generic memory attributes */
#define	MEM_ACCESS_READ                 0x00000001
#define	MEM_ACCESS_WRITE                0x00000002
#define	MEM_ACCESS_EXEC	                0x00000004
#define	MEM_ACCESS_RWX			(MEM_ACCESS_READ | MEM_ACCESS_WRITE | \
						MEM_ACCESS_EXEC)
#define MEM_ACCESS_RIGHT_MASK           0x00000007
#define	MEM_TYPE_WB                     0x00000040
#define	MEM_TYPE_WT                     0x00000080
#define	MEM_TYPE_UC                     0x00000100
#define	MEM_TYPE_WC                     0x00000200
#define	MEM_TYPE_WP                     0x00000400
#define MEM_TYPE_MASK                   0x000007C0

struct vm_memory_region {
#define MR_ADD		0
#define MR_DEL		2
	uint32_t type;

	/* IN: mem attr */
	uint32_t prot;

	/* IN: beginning guest GPA to map */
	uint64_t gpa;

	/* IN: VM0's GPA which foreign gpa will be mapped to */
	uint64_t vm0_gpa;

	/* IN: size of the region */
	uint64_t size;
} __attribute__((aligned(8)));

struct set_regions {
	/*IN: vmid for this hypercall */
	uint16_t vmid;

	/** Reserved */
	uint16_t reserved[3];

	/* IN: multi memmaps numbers */
	uint32_t mr_num;

	/* IN:
	 * the gpa of memmaps buffer, point to the memmaps array:
	 *  	struct memory_map memmap_array[memmaps_num]
	 * the max buffer size is one page.
	 */
	uint64_t regions_gpa;
} __aligned(8);

struct wp_data {
	/** set page write protect permission.
	 *  ture: set the wp; flase: clear the wp
	 */
	uint8_t set;

	/** Reserved */
	uint64_t pad:56;

	/** the guest physical address of the page to change */
	uint64_t gpa;
} __aligned(8);

struct sbuf_setup_param {
	uint16_t pcpu_id;
	uint16_t reserved;
	uint32_t sbuf_id;
	uint64_t gpa;
} __aligned(8);

struct hv_npk_log_param {
	/* the setup command for the hypervisor NPK log */
	uint16_t cmd;

	/* the setup result for the hypervisor NPK log */
	uint16_t res;

	/* the loglevel for the hypervisor NPK log */
	uint16_t loglevel;

	/* Reserved */
	uint16_t reserved;

	/* the MMIO address for the hypervisor NPK log */
	uint64_t mmio_addr;
} __aligned(8);

struct acrn_hw_info {
	uint16_t cpu_num; /* Physical CPU number */
	uint16_t reserved[3];
} __aligned(8);

struct vm_gpa2hpa {
	uint64_t gpa;		/* IN: gpa to translation */
	uint64_t hpa;		/* OUT: -1 means invalid gpa */
} __aligned(8);

struct hc_ptdev_irq {
#define IRQ_INTX 0
#define IRQ_MSI 1
#define IRQ_MSIX 2
	uint32_t type;
	uint16_t virt_bdf;	/* IN: Device virtual BDF# */
	uint16_t phys_bdf;	/* IN: Device physical BDF# */
	union {
		struct {
			uint32_t virt_pin;	/* IN: virtual IOAPIC pin */
			uint32_t phys_pin;	/* IN: physical IOAPIC pin */
			bool pic_pin;		/* IN: pin from PIC? */
			uint8_t reserved[3];	/* Reserved */
		} intx;
		struct {
			/* IN: vector count of MSI/MSIX */
			uint32_t vector_cnt;
		} msix;
	};
} __aligned(8);

struct hc_api_version {
	uint32_t major_version;
	uint32_t minor_version;
} __aligned(8);

struct hc_platform_info {
	/** Hardware Information */
	/** Physical CPU number */
	uint16_t cpu_num;

	/** version of this structure */
	uint16_t version;

	/** Align the size of version & hardware info to 128Bytes. */
	uint8_t reserved0[124];

	/** Configuration Information */
	/** Maximum vCPU number for one VM. */
	uint16_t max_vcpus_per_vm;

	/** Maximum Kata container number in SOS VM */
	uint8_t max_kata_containers;

	uint8_t reserved1[7];

	/** Number of configured VMs */
	uint16_t max_vms;

	/**
	 * The size of acrn_vm_config is various on different platforms.
	 * This is the size of this struct which is used for the caller
	 * to parse the vm_configs array.
	 */
	uint32_t vm_config_entry_size;

	/**
	 * Address to an array of struct acrn_vm_config, containing all
	 * the configurations of all VMs. VHM treats it as an opague data
	 * structure.
	 *
	 * The size of one array element is vm_config_entry_size while
	 * the number of elements is max_vms.
	 */
	uint64_t vm_configs_addr;

	/** Align the size of Configuration info to 128Bytes. */
	uint8_t reserved3[104];
} __aligned(8);

enum profiling_cmd_type {
	PROFILING_MSR_OPS = 0,
	PROFILING_GET_VMINFO,
	PROFILING_GET_VERSION,
	PROFILING_GET_CONTROL_SWITCH,
	PROFILING_SET_CONTROL_SWITCH,
	PROFILING_CONFIG_PMI,
	PROFILING_CONFIG_VMSWITCH,
	PROFILING_GET_PCPUID,
	PROFILING_GET_STATUS,
};
#endif /* ACRN_HV_DEFS_H */
