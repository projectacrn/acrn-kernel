/*
 *    Hantro encoder hardware driver.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/uaccess.h>
#include <linux/ioport.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/timer.h>
#include "hx280enc.h"
#include <linux/irq.h>
#include <linux/clk.h>

struct semaphore enc_core_sem;
static DECLARE_WAIT_QUEUE_HEAD(enc_hw_queue);
static DEFINE_SPINLOCK(enc_owner_lock);
static DECLARE_WAIT_QUEUE_HEAD(enc_wait_queue);

#define ENABLE_HANTRO_CLK
#define USE_IRQ
//#define USE_JUNO
/*------------------------------------------------------------------------
 *****************************PORTING LAYER********************************
 *-------------------------------------------------------------------------
 */
/*0:no resource sharing inter cores 1: existing resource sharing*/
#define RESOURCE_SHARED_INTER_CORES        0
/*customer specify according to own platform*/
#ifdef USE_JUNO
#define CORE_0_IO_ADDR                 0x60020000 // Video Encoder
#define CORE_1_IO_ADDR                 0x600a0000 // JPEG Encoder
#else //HTT FPGA Base Address
#define CORE_0_IO_ADDR                 0x20884000 // Video Encoder
#define CORE_1_IO_ADDR                 0x208a0000 // JPEG Encoder
#endif

#define CORE_0_IO_SIZE                 (250 * 4)    /* bytes */
#define CORE_1_IO_SIZE                 (250 * 4)    /* bytes */

#ifdef USE_IRQ
#ifdef USE_JUNO
#define INT_PIN_CORE_0                       8
#define INT_PIN_CORE_1                       -1
#else // HTT FPGA Interrupt
#define INT_PIN_CORE_0                       137
#define INT_PIN_CORE_1                       64
#endif
#else /* Polling Pins */
#define INT_PIN_CORE_0                       -1
#define INT_PIN_CORE_1                       -1
#endif

#define HANTRO_VC8KE_REG_BWREAD 216
#define HANTRO_VC8KE_REG_BWWRITE 220
#define VC8KE_BURSTWIDTH                 16

#define KMB_VC8000E_PAGE_LUT		0x20885000

/*for all cores, the core info should be listed here for subsequent use*/
/*base_addr, iosize, irq, resource_shared*/
CORE_CONFIG core_array[] = {
	{CORE_0_IO_ADDR, CORE_0_IO_SIZE, INT_PIN_CORE_0, RESOURCE_SHARED_INTER_CORES}, /* core_0, hevc and avc */
	{CORE_1_IO_ADDR, CORE_1_IO_SIZE, INT_PIN_CORE_1, RESOURCE_SHARED_INTER_CORES}  /* core_1, jpeg */
};

/* Interrupt Pin Name */
const char *core_irq_names[] = {
	"irq_hantro_videoencoder",   /* core_0, hevc and avc */
	"irq_hantro_jpegencoder"    /* core_1, jpeg */
};

/* KMB VC8000E page lookup table */
static unsigned long page_lut_read = KMB_VC8000E_PAGE_LUT;
static u8 *page_lut_regs_read;

/* Only for KMB ARM */
#ifdef ENABLE_HANTRO_CLK
static struct clk *hantro_clk_xin_venc;
static struct clk *hantro_clk_xin_jpeg;
static struct clk *hantro_clk_venc;
static struct clk *hantro_clk_jpeg;


static int hantro_clk_enable(void)
{
	clk_prepare_enable(hantro_clk_xin_venc);
	clk_prepare_enable(hantro_clk_xin_jpeg);
	clk_prepare_enable(hantro_clk_venc);
	clk_prepare_enable(hantro_clk_jpeg);
	return 0;
}

static int hantro_clk_disable(void)
{
	if (hantro_clk_xin_venc)
		clk_disable_unprepare(hantro_clk_xin_venc);

	if (hantro_clk_venc)
		clk_disable_unprepare(hantro_clk_venc);

	if (hantro_clk_xin_jpeg)
		clk_disable_unprepare(hantro_clk_xin_jpeg);

	if (hantro_clk_jpeg)
		clk_disable_unprepare(hantro_clk_jpeg);

	return 0;
}
#endif

/*------------------------------END-------------------------------------*/

/***************************TYPE AND FUNCTION DECLARATION****************/

/* here's all the must remember stuff */
struct hantroenc_t {
	CORE_CONFIG  core_cfg; //config of each core,such as base addr, irq,etc
	u32 hw_id; //hw id to indicate project
	u32 core_id; //core id for driver and sw internal use
	u32 is_valid; //indicate this core is hantro's core or not
	u32 is_reserved; //indicate this core is occupied by user or not
	int pid; //indicate which process is occupying the core
	u32 irq_received; //indicate this core receives irq
	u32 irq_status;
	char *buffer;
	unsigned int buffsize;
	u8 *hwregs;
	struct fasync_struct *async_queue;
};

static int ReserveIO(void);
static void ReleaseIO(void);
static void ResetAsic(struct hantroenc_t *dev);
static int CheckCoreOccupation(struct hantroenc_t *dev);
static void ReleaseEncoder(struct hantroenc_t *dev, u32 *core_info);

#ifdef hantroenc_DEBUG
static void dump_regs(unsigned long data);
#endif

/* IRQ handler */
#if KERNEL_VERSION(2, 6, 18) > LINUX_VERSION_CODE
static irqreturn_t hantroenc_isr(int irq, void *dev_id, struct pt_regs *regs);
#else
static irqreturn_t hantroenc_isr(int irq, void *dev_id);
#endif

/*********************local variable declaration*****************/
unsigned long sram_base;
unsigned int sram_size;
/* and this is our MAJOR; use 0 for dynamic allocation (recommended)*/
static int hantroenc_major;
static int total_core_num;
/* dynamic allocation*/
static struct hantroenc_t *hantroenc_data;

/******************************************************************************/
static int CheckEncIrq(struct hantroenc_t *dev, u32 *core_info, u32 *irq_status)
{
	unsigned long flags;
	int rdy = 0;
	u32 i = 0;
	u8 core_mapping = 0;

	core_mapping = (u8)(*core_info & 0xFF);

	while (core_mapping) {
		if (core_mapping & 0x1) {
			if (i > total_core_num - 1)
				break;

			spin_lock_irqsave(&enc_owner_lock, flags);

			if (dev[i].irq_received) {
				/* reset the wait condition(s) */
				PDEBUG("check %d irq ready\n", i);
				dev[i].irq_received = 0;
				rdy = 1;
				*core_info = i;
				*irq_status = dev[i].irq_status;
			}

			spin_unlock_irqrestore(&enc_owner_lock, flags);
			break;
		}
		core_mapping = core_mapping >> 1;
		i++;
	}
	return rdy;
}

static unsigned int WaitEncReady(struct hantroenc_t *dev, u32 *core_info, u32 *irq_status)
{
	PDEBUG("%s\n", __func__);

	if (wait_event_interruptible(
		enc_wait_queue,
		CheckEncIrq(dev, core_info, irq_status))) {
		PDEBUG("ENC wait_event_interruptible interrupted\n");
		ReleaseEncoder(dev, core_info);
		return -ERESTARTSYS;
	}

	return 0;
}

u32 hantroenc_readbandwidth(int isreadBW)
{
	int i;
	u32 bandwidth = 0;

	for (i = 0; i < total_core_num; i++) {
		if (hantroenc_data[i].is_valid == 0)
			continue;

		if (isreadBW)
			bandwidth += ioread32((void *)(hantroenc_data[i].hwregs + HANTRO_VC8KE_REG_BWREAD * 4));
		else
			bandwidth += ioread32((void *)(hantroenc_data[i].hwregs + HANTRO_VC8KE_REG_BWWRITE * 4));
	}
	return bandwidth * VC8KE_BURSTWIDTH;
}

static int CheckCoreOccupation(struct hantroenc_t *dev)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&enc_owner_lock, flags);
	if (!dev->is_reserved) {
		dev->is_reserved = 1;
		dev->pid = current->pid;
		ret = 1;
		PDEBUG("%s pid=%d\n", __func__, dev->pid);
	}

	spin_unlock_irqrestore(&enc_owner_lock, flags);

	return ret;
}

static int GetWorkableCore(struct hantroenc_t *dev, u32 *core_info, u32 *core_info_tmp)
{
	int ret = 0;
	u32 i = 0;
	u32 cores;
	u32 core_id = 0;
	u8 core_mapping = 0;
	u32 required_num = 0;

	cores = *core_info;
	required_num = ((cores >> CORE_INFO_AMOUNT_OFFSET) & 0x7) + 1;
	core_mapping = (u8)(cores & 0xFF);

	if (*core_info_tmp == 0)
		*core_info_tmp = required_num << 8;
	else
		required_num = ((*core_info_tmp & 0xF00) >> 8);

	PDEBUG("%s:required_num=%d,core_info=%x\n",
		__func__, required_num, *core_info);

	if (required_num) {
		/* a valid free Core that has specified core id */
		while (core_mapping) {
			if (core_mapping & 0x1) {
				if (i > total_core_num - 1)
					break;
				core_id = i;
				if (dev[core_id].is_valid &&
					CheckCoreOccupation(&dev[core_id])) {
					*core_info_tmp = ((((*core_info_tmp & 0xF00) >> 8) - 1) << 8) | (*core_info_tmp & 0x0FF);
					*core_info_tmp = *core_info_tmp | (1 << core_id);
					if (((*core_info_tmp & 0xF00) >> 8) == 0) {
						ret = 1;
						*core_info = (*core_info & 0xFFFFFF00) | (*core_info_tmp & 0xFF);
						*core_info_tmp = 0;
						required_num = 0;
						break;
					}
				}
			}
			core_mapping = core_mapping >> 1;
			i++;
		}
	} else
		ret = 1;

	PDEBUG("*core_info = %x\n", *core_info);
	return ret;
}

static long ReserveEncoder(struct hantroenc_t *dev, u32 *core_info)
{
	u32 core_info_tmp = 0;
	/*If HW resources are shared inter cores, just make sure only one is using the HW*/
	if (dev[0].core_cfg.resource_shared) {
		if (down_interruptible(&enc_core_sem))
			return -ERESTARTSYS;
	}

	/* lock a core that has specified core id*/
	if (wait_event_interruptible(enc_hw_queue,
		GetWorkableCore(dev, core_info, &core_info_tmp) != 0))
		return -ERESTARTSYS;

	return 0;
}

static void ReleaseEncoder(struct hantroenc_t *dev, u32 *core_info)
{
	unsigned long flags;
	u32 core_num = 0;
	u32 i = 0, core_id;
	u8 core_mapping = 0;

	core_num = ((*core_info >> CORE_INFO_AMOUNT_OFFSET) & 0x7) + 1;

	core_mapping = (u8)(*core_info & 0xFF);

	PDEBUG("%s:core_num=%d,core_mapping=%x\n", __func__, core_num, core_mapping);
	/* release specified core id */
	while (core_mapping) {
		if (core_mapping & 0x1) {
			core_id = i;
			spin_lock_irqsave(&enc_owner_lock, flags);
			PDEBUG("dev[core_id].pid=%d,current->pid=%d\n", dev[core_id].pid, current->pid);
			if (dev[core_id].is_reserved && dev[core_id].pid == current->pid) {
				dev[core_id].pid = -1;
				dev[core_id].is_reserved = 0;
			}

			dev[core_id].irq_received = 0;
			dev[core_id].irq_status = 0;
			spin_unlock_irqrestore(&enc_owner_lock, flags);

			//wake_up_interruptible_all(&enc_hw_queue);
		}
		core_mapping = core_mapping >> 1;
		i++;
	}

	wake_up_interruptible_all(&enc_hw_queue);

	if (dev->core_cfg.resource_shared)
		up(&enc_core_sem);
}

long hantroenc_ioctl(
	struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	unsigned int tmp;

	if (hantroenc_data == NULL)
		return -ENXIO;
	switch (cmd) {
	case HX280ENC_IOCGHWOFFSET: {
		u32 id;

		__get_user(id, (u32 *)arg);

		if (id >= total_core_num || hantroenc_data[id].is_valid == 0)
			return -EFAULT;

		__put_user(hantroenc_data[id].core_cfg.base_addr, (unsigned long *) arg);
		break;
	}

	case HX280ENC_IOCGHWIOSIZE: {
		u32 id;
		u32 io_size;

		__get_user(id, (u32 *)arg);

		if (id >= total_core_num || hantroenc_data[id].is_valid == 0)
			return -EFAULT;

		io_size = hantroenc_data[id].core_cfg.iosize;
		__put_user(io_size, (u32 *) arg);

		return 0;
	}
	case HX280ENC_IOCGSRAMOFFSET:
		__put_user(sram_base, (unsigned long *) arg);
		break;
	case HX280ENC_IOCGSRAMEIOSIZE:
		__put_user(sram_size, (unsigned int *) arg);
		break;
	case HX280ENC_IOCG_CORE_NUM:
		__put_user(total_core_num, (unsigned int *) arg);
		PDEBUG("enc core num = %d", total_core_num);
		break;
	case HX280ENC_IOCH_ENC_RESERVE: {
		u32 core_info;
		int ret;

		PDEBUG("Reserve ENC Cores\n");
		__get_user(core_info, (u32 *)arg);
		ret = ReserveEncoder(hantroenc_data, &core_info);
		if (ret == 0)
			__put_user(core_info, (u32 *) arg);
		return ret;
	}
	case HX280ENC_IOCH_ENC_RELEASE: {
		u32 core_info;

		__get_user(core_info, (u32 *)arg);

		PDEBUG("Release ENC Core\n");

		ReleaseEncoder(hantroenc_data, &core_info);

		break;
	}

	case HX280ENC_IOCG_CORE_WAIT: {
		u32 core_info;
		u32 irq_status;

		__get_user(core_info, (u32 *)arg);

		tmp = WaitEncReady(hantroenc_data, &core_info, &irq_status);
		if (tmp == 0) {
			__put_user(irq_status, (unsigned int *)arg);
			return core_info;//return core_id
		}
		__put_user(0, (unsigned int *)arg);
		return -1;

		break;
	}
	}
	return 0;
}

int hantroenc_init(struct platform_device *pdev)
{
	int result = 0;
	int i;

	sram_base = 0;
	sram_size = 0;
	hantroenc_major = 0;
	total_core_num = 0;
	hantroenc_data = NULL;

	total_core_num = sizeof(core_array) / sizeof(CORE_CONFIG);
	for (i = 0; i < total_core_num; i++) {
		pr_info("hantroenc: module init - core[%d] addr = %lx\n", i,
		       (size_t)core_array[i].base_addr);
	}

	hantroenc_data = vmalloc(sizeof(struct hantroenc_t) * total_core_num);
	if (hantroenc_data == NULL)
		goto err;
	memset(hantroenc_data, 0, sizeof(struct hantroenc_t)*total_core_num);

	for (i = 0; i < total_core_num; i++) {
		hantroenc_data[i].core_cfg = core_array[i];
		hantroenc_data[i].async_queue = NULL;
		hantroenc_data[i].hwregs = NULL;
		hantroenc_data[i].core_id = i;
	}

#ifdef USE_IRQ
	PDEBUG("hantroenc in IRQ mode!\n");
	hantroenc_data[0].core_cfg.irq = platform_get_irq_byname(pdev, "irq_hantro_videoencoder");
	hantroenc_data[1].core_cfg.irq = platform_get_irq_byname(pdev, "irq_hantro_jpgencoder");
	PDEBUG("platform_vc8000e_probe VENC irq_num = %d\n", hantroenc_data[0].core_cfg.irq);
	PDEBUG("platform_vc8000e_probe JPEG ENC irq_num = %d\n", hantroenc_data[1].core_cfg.irq);
#else /* Polling Mode */
	PDEBUG("hantroenc in polling mode!\n");
	hantroenc_data[0].core_cfg.irq = -1;
	hantroenc_data[1].core_cfg.irq = -1;
#endif

#ifdef ENABLE_HANTRO_CLK /* Enable and set the VENC and JPEG ENC clocks */
	hantro_clk_xin_venc = clk_get(&pdev->dev, "clk_xin_venc");
	hantro_clk_venc = clk_get(&pdev->dev, "clk_venc");
	hantro_clk_xin_jpeg = clk_get(&pdev->dev, "clk_xin_jpeg");
	hantro_clk_jpeg = clk_get(&pdev->dev, "clk_jpeg");
	hantro_clk_enable();

	/* Set KMB VENC CLK to max 700Mhz VENC */
	pr_info("hx280enc venc: Before setting any clocks: clk_xin_venc: %ld | clk_venc %ld\n",
		clk_get_rate(hantro_clk_xin_venc), clk_get_rate(hantro_clk_venc));
	clk_set_rate(hantro_clk_xin_venc, 700000000);
	pr_info("hx280enc venc: Trying to set 700Mhz: clk_xin_venc: %ld | clk_venc %ld\n",
		clk_get_rate(hantro_clk_xin_venc), clk_get_rate(hantro_clk_venc));
	if (clk_get_rate(hantro_clk_xin_venc) < 700000000) {
		pr_info("hx280enc venc: Failed to set 700Mhz, setting to 666Mhz\n");
		clk_set_rate(hantro_clk_xin_venc, 666666666);
		pr_info("hx280enc venc: clk_xin_venc: %ld | clk_venc %ld\n",
			clk_get_rate(hantro_clk_xin_venc), clk_get_rate(hantro_clk_venc));
	}

	/* Set KMB JPEGENC CLK to max 700Mhz JPEGENC */
	pr_info("hx280enc jpegenc: Before setting any clocks: clk_xin_jpeg: %ld | clk_jpeg %ld\n",
		clk_get_rate(hantro_clk_xin_jpeg), clk_get_rate(hantro_clk_jpeg));
	clk_set_rate(hantro_clk_xin_jpeg, 700000000);
	pr_info("hx280enc jpegenc: Trying to set 700Mhz: clk_xin_jpeg: %ld | clk_jpeg %ld\n",
		clk_get_rate(hantro_clk_xin_jpeg), clk_get_rate(hantro_clk_jpeg));
	if (clk_get_rate(hantro_clk_xin_jpeg) < 700000000) {
		pr_info("hx280enc jpegenc: Failed to set 700Mhz, setting to 500Mhz\n");
		clk_set_rate(hantro_clk_xin_jpeg, 500000000);
		pr_info("hx280enc jpegenc: clk_xin_jpeg: %ld | clk_jpeg %ld\n",
			clk_get_rate(hantro_clk_xin_jpeg), clk_get_rate(hantro_clk_jpeg));
	}
#endif

	result = ReserveIO();
	if (result < 0)
		goto err;

	ResetAsic(hantroenc_data);  /* reset hardware */

	sema_init(&enc_core_sem, 1);

	/* Dynamic AXI ID and Page LUT routines */
	/* Register and set the page lookup table for encoder */
	if (!request_mem_region(page_lut_read, hantroenc_data[0].core_cfg.iosize, "hantroenc_pagelut_read")) {
		pr_info("hantroenc: failed to reserve page lookup table regs\n");
		return -EBUSY;
	}
	page_lut_regs_read = (u8 *)ioremap_nocache(page_lut_read, hantroenc_data[0].core_cfg.iosize);
	if (page_lut_regs_read == NULL)
		pr_info("hantroenc: failed to ioremap page lookup table regs\n");

	/* Set write page LUT AXI ID 1-8 to 0x4 */
	iowrite32(0x04040400, (void *)page_lut_regs_read + 0x10);
	pr_info("hx280enc: Page LUT WR AXI ID 3:0 = %x\n", ioread32((void *) page_lut_regs_read + 0x10));
	iowrite32(0x04040404, (void *)page_lut_regs_read + 0x14);
	pr_info("hx280enc: Page LUT WR AXI ID 7:4 = %x\n", ioread32((void *) page_lut_regs_read + 0x14));
	iowrite32(0x4, (void *)page_lut_regs_read + 0x18);
	pr_info("hx280enc: Page LUT WR AXI ID 8 = %x\n", ioread32((void *) page_lut_regs_read + 0x18));
	/*
	 * iowrite32(0x4, (void *)page_lut_regs_read);
	 * pr_info("hx280enc: Page LUT RD AXI ID 0 = %x\n", ioread32((void *) page_lut_regs_read));
	*/

	/* Set VENC sw_enc_axi_rd_id_e = 1 */
	iowrite32(1<<16, (void *)hantroenc_data[0].hwregs + 0x8);
	pr_info("hx280enc: sw_enc_axi_rd_id_e  = %x\n", ioread32((void *) hantroenc_data[0].hwregs + 0x8));
	/* Set RD Page LUT AXI ID 0.1 to 0x0 and the rest AXI ID 2-8 to 0x4 */
	iowrite32(0x04040000, (void *)page_lut_regs_read);
	pr_info("hx280enc: RD AXI 3:0 = %x\n", ioread32((void *) page_lut_regs_read));
	iowrite32(0x04040404, (void *)page_lut_regs_read + 0x4);
	pr_info("hx280enc: RD AXI 7:4  = %x\n", ioread32((void *) page_lut_regs_read + 0x4));
	iowrite32(0x00000004, (void *)page_lut_regs_read + 0x8);
	pr_info("hx280enc: RD AXI 8 = %x\n", ioread32((void *) page_lut_regs_read + 0x8));

#ifdef USE_IRQ
	/* get the IRQ line */
	for (i = 0; i < total_core_num; i++) {
		if (hantroenc_data[i].is_valid == 0)
			continue;
		if (hantroenc_data[i].core_cfg.irq > 0) {
			PDEBUG("hx280enc: Trying to request_irq = %d\n", hantroenc_data[i].core_cfg.irq);

			result = request_irq(hantroenc_data[i].core_cfg.irq, hantroenc_isr,
					IRQF_SHARED, core_irq_names[i], (void *) &hantroenc_data[i]);

			if (result == -EINVAL) {
				PDEBUG("hx280enc: Bad irq number or handler\n");
				ReleaseIO();
				goto err;
			} else if (result == -EBUSY) {
				PDEBUG("hx280enc: IRQ <%d> busy, change your config\n",
					hantroenc_data[i].core_cfg.irq);
				ReleaseIO();
				goto err;
			}
		}
	}
#endif
	pr_info("hantroenc: module inserted.\n");

	return 0;

err:
	pr_info("hantroenc: module not inserted\n");
	return result;
}

void hantroenc_cleanup(void)
{
	int i = 0;

	for (i = 0; i < total_core_num; i++) {
		u32 hwId = hantroenc_data[i].hw_id;
		u32 majorId = (hwId & 0x0000FF00) >> 8;
		u32 wClr = (majorId >= 0x61) ? (0x1FD) : (0);

		if (hantroenc_data[i].is_valid == 0)
			continue;
		iowrite32(0, (void *)(hantroenc_data[i].hwregs + 0x14)); /* disable HW */
		iowrite32(wClr, (void *)(hantroenc_data[i].hwregs + 0x04)); /* clear enc IRQ */

		/* free the encoder IRQ */
		if (hantroenc_data[i].core_cfg.irq > 0)
			free_irq(hantroenc_data[i].core_cfg.irq, (void *)&hantroenc_data[i]);
	}

	ReleaseIO();
	vfree(hantroenc_data);
#ifdef ENABLE_HANTRO_CLK
	hantro_clk_disable();
#endif
	pr_info("hantroenc: module removed\n");
}

static int ReserveIO(void)
{
	u32 hwid;
	int i;
	u32 found_hw = 0;

	for (i = 0; i < total_core_num; i++) {
		if (!request_mem_region
			(hantroenc_data[i].core_cfg.base_addr, hantroenc_data[i].core_cfg.iosize, "hx280enc")) {
			PDEBUG("hantroenc: failed to reserve HW regs\n");
			continue;
		}

		hantroenc_data[i].hwregs =
			(u8 *) ioremap_nocache(hantroenc_data[i].core_cfg.base_addr,
			hantroenc_data[i].core_cfg.iosize);

		if (hantroenc_data[i].hwregs == NULL) {
			PDEBUG("hantroenc: failed to ioremap HW regs\n");
			ReleaseIO();
			continue;
		}

		/*read hwid and check validness and store it*/
		hwid = (u32)ioread32((void *)hantroenc_data[i].hwregs);
		PDEBUG("hwid=0x%08x\n", hwid);

		/* check for encoder HW ID */
		if (((((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID1 >> 16) & 0xFFFF))) &&
			((((hwid >> 16) & 0xFFFF) != ((ENC_HW_ID2 >> 16) & 0xFFFF)))) {
			PDEBUG("hantroenc: HW not found at %lx\n",
				hantroenc_data[i].core_cfg.base_addr);
#ifdef hantroenc_DEBUG
			dump_regs((unsigned long) &hantroenc_data);
#endif
			ReleaseIO();
			hantroenc_data[i].is_valid = 0;
			continue;
		}
		hantroenc_data[i].hw_id = hwid;
		hantroenc_data[i].is_valid = 1;
		found_hw = 1;

		PDEBUG("hantroenc: HW at base <%lx> with ID <0x%08x>\n",
		       hantroenc_data[i].core_cfg.base_addr, hwid);

	}

	if (found_hw == 0) {
		PDEBUG("hantroenc: NO ANY HW found!!\n");
		return -1;
	}

	return 0;
}

static void ReleaseIO(void)
{
	u32 i;

	for (i = 0; i < total_core_num; i++) {
		if (hantroenc_data[i].is_valid == 0)
			continue;
		if (hantroenc_data[i].hwregs)
			iounmap((void *) hantroenc_data[i].hwregs);
		release_mem_region(hantroenc_data[i].core_cfg.base_addr, hantroenc_data[i].core_cfg.iosize);
	}

	iounmap((void *) page_lut_regs_read);
	release_mem_region(page_lut_read, hantroenc_data[0].core_cfg.iosize);
}

static irqreturn_t hantroenc_isr(int irq, void *dev_id)
{
	unsigned int handled = 0;
	struct hantroenc_t *dev = (struct hantroenc_t *) dev_id;
	u32 irq_status;
	unsigned long flags;

	/*If core is not reserved by any user, but irq is received, just ignore it*/
	spin_lock_irqsave(&enc_owner_lock, flags);
	if (!dev->is_reserved) {
		PDEBUG("hantroenc_isr:received IRQ but core is not reserved!\n");
		irq_status = (u32)ioread32((void *)(dev->hwregs + 0x04));
		if (irq_status & 0x01) {
			/* clear all IRQ bits. (hwId >= 0x80006100) means IRQ is cleared by writing 1 */
			u32 hwId = ioread32((void *)dev->hwregs);
			u32 majorId = (hwId & 0x0000FF00) >> 8;
			u32 wClr = (majorId >= 0x61) ? irq_status : (irq_status & (~0x1FD));

			iowrite32(wClr, (void *)(dev->hwregs + 0x04));
		}
		spin_unlock_irqrestore(&enc_owner_lock, flags);
		return IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&enc_owner_lock, flags);

	PDEBUG("hantroenc_isr:received IRQ!\n");
	irq_status = (u32)ioread32((void *)(dev->hwregs + 0x04));
	PDEBUG("hx280enc: irq_status of %d is:%x\n", dev->core_id, irq_status);
	if (irq_status & 0x01) {
		/* clear all IRQ bits. (hwId >= 0x80006100) means IRQ is cleared by writing 1 */
		u32 hwId = ioread32((void *)dev->hwregs);
		u32 majorId = (hwId & 0x0000FF00) >> 8;
		u32 wClr = (majorId >= 0x61) ? irq_status : (irq_status & (~0x1FD));

		iowrite32(wClr, (void *)(dev->hwregs + 0x04));
		spin_lock_irqsave(&enc_owner_lock, flags);
		dev->irq_received = 1;
		dev->irq_status = irq_status & (~0x01);
		spin_unlock_irqrestore(&enc_owner_lock, flags);

		wake_up_interruptible_all(&enc_wait_queue);
		handled++;
	}
	if (!handled)
		PDEBUG("IRQ received, but not hantro's!\n");

	return IRQ_HANDLED;
}

static void ResetAsic(struct hantroenc_t *dev)
{
	int i, n;

	for (n = 0; n < total_core_num; n++) {
		if (dev[n].is_valid == 0)
			continue;
		iowrite32(0, (void *)(dev[n].hwregs + 0x14));
		for (i = 4; i < dev[n].core_cfg.iosize; i += 4)
			iowrite32(0, (void *)(dev[n].hwregs + i));
	}
}
