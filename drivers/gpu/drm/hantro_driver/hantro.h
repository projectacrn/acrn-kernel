/*
 *    Hantro driver public header file.
 *
 *    Copyright (c) 2017, VeriSilicon Inc.
 *
 *    This program is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU General Public License
 *    as published by the Free Software Foundation; either version 2
 *    of the License, or (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You may obtain a copy of the GNU General Public License
 *    Version 2 or later at the following locations:
 *    http://www.opensource.org/licenses/gpl-license.html
 *    http://www.gnu.org/copyleft/gpl.html
 */

#ifndef HANTRO_H
#define HANTRO_H

#include <linux/ioctl.h>
#include <linux/dma-resv.h>
#include <linux/dma-mapping.h>
#include <drm/drmP.h>
#include <drm/drm_vma_manager.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem.h>
#include <linux/dma-buf.h>
#include <drm/drm.h>
#include <drm/drm_auth.h>
#include <linux/version.h>
#if KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE
#include <linux/dma-fence.h>
#endif


/*these domain definitions are identical to hantro_bufmgr.h*/
#define HANTRO_DOMAIN_NONE    0x00000
#define HANTRO_CPU_DOMAIN    0x00001
#define HANTRO_HEVC264_DOMAIN    0x00002
#define HANTRO_JPEG_DOMAIN    0x00004
#define HANTRO_DECODER0_DOMAIN    0x00008
#define HANTRO_DECODER1_DOMAIN    0x00010
#define HANTRO_DECODER2_DOMAIN    0x00020

#define HANTRO_GEM_FLAG_IMPORT    (1 << 0)
#define HANTRO_GEM_FLAG_EXPORT    (1 << 1)
#define HANTRO_GEM_FLAG_EXPORTUSED    (1 << 2)

/* dynamic ddr allocation defines */
#define DDR0_CHANNEL 0
#define DDR1_CHANNEL 1

struct hantro_mem {
	struct device *dev;     /* Child device managing the memory region. */
	void *vaddr;            /* The virtual address of the memory region. */
	dma_addr_t dma_handle;  /* The address of the memory region. */
	size_t size;            /* The size of the memory region. */
};

struct hantro_drm_fb {
	struct drm_framebuffer fb;
	struct drm_gem_object *obj[4];
};

struct drm_gem_hantro_object {
	struct drm_gem_object base;
	dma_addr_t paddr;
	struct sg_table *sgt;

	/* For objects with DMA memory allocated by GEM CMA */
	void *vaddr;
	struct page *pageaddr;
	struct page **pages;
	unsigned long num_pages;
	/*fence ref*/
	struct dma_resv kresv;
	unsigned int ctxno;
	int    handle;
	int    flag;
	int    ddr_channel;
};

struct hantro_fencecheck {
	unsigned int handle;
	int ready;
};

struct hantro_domainset {
	unsigned int handle;
	unsigned int writedomain;
	unsigned int readdomain;
};

struct hantro_addrmap {
	unsigned int handle;
	unsigned long vm_addr;
	unsigned long phy_addr;
};

struct hantro_regtransfer {
	unsigned long coreid;
	unsigned long offset;
	unsigned long size;
	const void *data;
	int benc;                        /*encoder core or decoder core*/
	int direction;            /*0=read, 1=write*/
};

struct hantro_corenum {
	unsigned int deccore;
	unsigned int enccore;
};

#define HANTRO_FENCE_WRITE  1
struct hantro_acquirebuf {
	unsigned long handle;
	unsigned long flags;
	unsigned long timeout;
	unsigned long fence_handle;
};

struct hantro_releasebuf {
	unsigned long fence_handle;
};

struct core_desc {
	__u32 id; /* id of the core */
	__u32 *regs; /* pointer to user registers */
	__u32 size; /* size of register space */
	__u32 reg_id;
};

/* Ioctl definitions */
/*hantro drm related */
#define HANTRO_IOCTL_START (DRM_COMMAND_BASE)
#define DRM_IOCTL_HANTRO_TESTCMD        DRM_IOWR(HANTRO_IOCTL_START, unsigned int)
#define DRM_IOCTL_HANTRO_GETPADDR       DRM_IOWR(HANTRO_IOCTL_START+1, struct hantro_addrmap)
//#define DRM_IOCTL_HANTRO_REGTRANS       DRM_IOWR(HANTRO_IOCTL_START+2, struct hantro_regtransfer)
#define DRM_IOCTL_HANTRO_TESTREADY      DRM_IOWR(HANTRO_IOCTL_START+3, struct hantro_fencecheck)
#define DRM_IOCTL_HANTRO_SETDOMAIN      DRM_IOWR(HANTRO_IOCTL_START+4, struct hantro_domainset)
#define DRM_IOCTL_HANTRO_ACQUIREBUF     DRM_IOWR(HANTRO_IOCTL_START+6, struct hantro_acquirebuf)
#define DRM_IOCTL_HANTRO_RELEASEBUF     DRM_IOWR(HANTRO_IOCTL_START+7, struct hantro_releasebuf)
#define DRM_IOCTL_HANTRO_GETPRIMEADDR     DRM_IOWR(HANTRO_IOCTL_START+8, unsigned long *)
#define DRM_IOCTL_HANTRO_PTR_PHYADDR     DRM_IOWR(HANTRO_IOCTL_START+9, unsigned long *)

/* hantro enc related */
#define HX280ENC_IOC_START              DRM_IO(HANTRO_IOCTL_START + 16)
#define HX280ENC_IOCGHWOFFSET           DRM_IOR(HANTRO_IOCTL_START +  17, unsigned long *)
#define HX280ENC_IOCGHWIOSIZE           DRM_IOWR(HANTRO_IOCTL_START + 18, unsigned long *)
#define HX280ENC_IOC_CLI                DRM_IO(HANTRO_IOCTL_START + 19)
#define HX280ENC_IOC_STI                DRM_IO(HANTRO_IOCTL_START + 20)
#define HX280ENC_IOCHARDRESET           DRM_IO(HANTRO_IOCTL_START + 21)   /* debugging tool */
#define HX280ENC_IOCGSRAMOFFSET         DRM_IOR(HANTRO_IOCTL_START +  22, unsigned long *)
#define HX280ENC_IOCGSRAMEIOSIZE        DRM_IOR(HANTRO_IOCTL_START +  23, unsigned int *)
#define HX280ENC_IOCH_ENC_RESERVE       DRM_IOR(HANTRO_IOCTL_START + 24, unsigned int *)
#define HX280ENC_IOCH_ENC_RELEASE       DRM_IOR(HANTRO_IOCTL_START + 25, unsigned int *)
#define HX280ENC_IOCG_CORE_NUM          DRM_IOR(HANTRO_IOCTL_START + 26, unsigned int *)
#define HX280ENC_IOCG_CORE_WAIT         DRM_IOR(HANTRO_IOCTL_START + 27, unsigned int *)
//#define HX280ENC_IOCG_WRITE_REG         DRM_IOW( HANTRO_IOCTL_START+ 28, struct core_desc *)
//#define HX280ENC_IOCG_READ_REG          DRM_IOWR(HANTRO_IOCTL_START+ 29, struct core_desc *)
//#define HX280ENC_IOCG_PUSH_REG          DRM_IOW( HANTRO_IOCTL_START+ 30, struct core_desc *)
//#define HX280ENC_IOCG_PULL_REG          DRM_IOW( HANTRO_IOCTL_START+ 31, struct core_desc *)

#define HX280ENC_IOC_END                DRM_IO(HANTRO_IOCTL_START +  39)

/* hantro dec related */
#define HANTRODEC_IOC_START             DRM_IO(HANTRO_IOCTL_START + 40)
#define HANTRODEC_PP_INSTANCE           DRM_IO(HANTRO_IOCTL_START + 41)
#define HANTRODEC_HW_PERFORMANCE        DRM_IO(HANTRO_IOCTL_START + 42)
#define HANTRODEC_IOCGHWOFFSET          DRM_IOR(HANTRO_IOCTL_START +  43, unsigned long *)
#define HANTRODEC_IOCGHWIOSIZE          DRM_IOR(HANTRO_IOCTL_START +  44, unsigned int *)
#define HANTRODEC_IOC_CLI               DRM_IO(HANTRO_IOCTL_START + 45)
#define HANTRODEC_IOC_STI               DRM_IO(HANTRO_IOCTL_START + 46)
#define HANTRODEC_IOC_MC_OFFSETS        DRM_IOR(HANTRO_IOCTL_START + 47, unsigned long *)
#define HANTRODEC_IOC_MC_CORES          DRM_IOR(HANTRO_IOCTL_START + 48, unsigned int *)
#define HANTRODEC_IOCS_DEC_PUSH_REG     DRM_IOW(HANTRO_IOCTL_START + 49, struct core_desc *)
#define HANTRODEC_IOCS_PP_PUSH_REG      DRM_IOW(HANTRO_IOCTL_START + 50, struct core_desc *)
#define HANTRODEC_IOCH_DEC_RESERVE      DRM_IO(HANTRO_IOCTL_START + 51)
#define HANTRODEC_IOCT_DEC_RELEASE      DRM_IO(HANTRO_IOCTL_START + 52)
#define HANTRODEC_IOCQ_PP_RESERVE       DRM_IO(HANTRO_IOCTL_START + 53)
#define HANTRODEC_IOCT_PP_RELEASE       DRM_IO(HANTRO_IOCTL_START + 54)
#define HANTRODEC_IOCX_DEC_WAIT         DRM_IOWR(HANTRO_IOCTL_START + 55, struct core_desc *)
#define HANTRODEC_IOCX_PP_WAIT          DRM_IOWR(HANTRO_IOCTL_START + 56, struct core_desc *)
#define HANTRODEC_IOCS_DEC_PULL_REG     DRM_IOWR(HANTRO_IOCTL_START + 57, struct core_desc *)
#define HANTRODEC_IOCS_PP_PULL_REG      DRM_IOWR(HANTRO_IOCTL_START + 58, struct core_desc *)
#define HANTRODEC_IOCG_CORE_WAIT        DRM_IOR(HANTRO_IOCTL_START + 59, int *)
#define HANTRODEC_IOX_ASIC_ID           DRM_IOR(HANTRO_IOCTL_START + 60, __u32 *)
#define HANTRODEC_IOCG_CORE_ID          DRM_IO(HANTRO_IOCTL_START + 61)
#define HANTRODEC_IOCS_DEC_WRITE_REG    DRM_IOW(HANTRO_IOCTL_START + 62, struct core_desc *)
#define HANTRODEC_IOCS_DEC_READ_REG     DRM_IOWR(HANTRO_IOCTL_START + 63, struct core_desc *)
#define HANTRODEC_DEBUG_STATUS          DRM_IO(HANTRO_IOCTL_START + 64)
#define HANTRODEC_IOX_ASIC_BUILD_ID  DRM_IOR(HANTRO_IOCTL_START + 65, __u32 *)

#define HANTRODEC_IOC_END               DRM_IO(HANTRO_IOCTL_START + 80)

#endif /* HANTRO_H */
