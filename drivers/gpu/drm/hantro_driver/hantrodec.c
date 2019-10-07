/*
 *    Hantro decoder hardware driver.
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

#include "hantrodec.h"
#include "dwl_defs.h"
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>

#define ENABLE_HANTRO_CLK
#define USE_IRQ
//#define USE_JUNO
/*hantro G1 regs config including dec and pp*/
#define HANTRO_DEC_ORG_REGS             60
#define HANTRO_PP_ORG_REGS              41

#define HANTRO_DEC_EXT_REGS             27
#define HANTRO_PP_EXT_REGS              9

#define HANTRO_G1_DEC_TOTAL_REGS   (HANTRO_DEC_ORG_REGS + HANTRO_DEC_EXT_REGS)
#define HANTRO_PP_TOTAL_REGS         (HANTRO_PP_ORG_REGS + HANTRO_PP_EXT_REGS)
#define HANTRO_G1_DEC_REGS            155 /*G1 total regs*/

#define HANTRO_DEC_ORG_FIRST_REG        0
#define HANTRO_DEC_ORG_LAST_REG         59
#define HANTRO_DEC_EXT_FIRST_REG        119
#define HANTRO_DEC_EXT_LAST_REG         145

#define HANTRO_PP_ORG_FIRST_REG         60
#define HANTRO_PP_ORG_LAST_REG          100
#define HANTRO_PP_EXT_FIRST_REG         146
#define HANTRO_PP_EXT_LAST_REG          154

/*hantro G2 reg config*/
#define HANTRO_G2_DEC_REGS                 265 /*G2 total regs*/

#define HANTRO_G2_DEC_FIRST_REG            0
#define HANTRO_G2_DEC_LAST_REG             (HANTRO_G2_DEC_REGS - 1)

/* hantro VC8000D reg config */
#define HANTRO_VC8000D_REGS             342 /*VC8000D total regs*/
#define HANTRO_VC8000D_FIRST_REG        0
#define HANTRO_VC8000D_LAST_REG         (HANTRO_VC8000D_REGS-1)
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define HANTRO_VC8KD_REG_BWREAD     300
#define HANTRO_VC8KD_REG_BWWRITE   301
#define VC8KD_BURSTWIDTH                         16

#define DEC_IO_SIZE_MAX                 \
	(MAX(MAX(HANTRO_G2_DEC_REGS, HANTRO_G1_DEC_REGS), \
		HANTRO_VC8000D_REGS)*4)

/********************************************************************
 *                                              PORTING SEGMENT
 * NOTES: customer should modify these configuration if do porting to own
 * platform. Please guarantee the base_addr, io_size,dec_irq belong to
 * same core.
 ********************************************************************/

#define HXDEC_MAX_CORES                 2

/* Logic module base address */
#ifdef USE_JUNO
#define SOCLE_LOGIC_0_BASE              0x60090000
#define SOCLE_LOGIC_1_BASE              0x60090000
#else // HTT FPGA Base Address
#define SOCLE_LOGIC_0_BASE              0x20888000
#define SOCLE_LOGIC_1_BASE              0x20888800
#endif

#define VEXPRESS_LOGIC_0_BASE           0xFC010000
#define VEXPRESS_LOGIC_1_BASE           0xFC020000

#define DEC_IO_SIZE_0                   DEC_IO_SIZE_MAX /* bytes */
#define DEC_IO_SIZE_1                   DEC_IO_SIZE_MAX /* bytes */

#ifdef USE_IRQ
#ifdef USE_JUNO
#define DEC_IRQ_0                       8
#define DEC_IRQ_1                       9
#else // HTT FPGA Interrupts
#define DEC_IRQ_0                       138
#define DEC_IRQ_1                       138
#endif
#else
#define DEC_IRQ_0                       -1
#define DEC_IRQ_1                       -1
#endif

#define KMB_VC8000D_PAGE_LUT		0x20889000

/***********************************************************************/

#define IS_G1(hw_id)                    (((hw_id) == 0x6731) ? 1 : 0)
#define IS_G2(hw_id)                    (((hw_id) == 0x6732) ? 1 : 0)
#define IS_VC8000D(hw_id)               (((hw_id) == 0x8001) ? 1 : 0)

static const int DecHwId[] = {
	0x6731, /* G1 */
	0x6732, /* G2 */
	0x8001 /* VDEC */
};

ulong multicorebase[HXDEC_MAX_CORES] = {
	SOCLE_LOGIC_0_BASE,
	SOCLE_LOGIC_1_BASE
};

int irq[HXDEC_MAX_CORES] = {
	DEC_IRQ_0,
	DEC_IRQ_1
};

unsigned int iosize[HXDEC_MAX_CORES] = {
	DEC_IO_SIZE_0,
	DEC_IO_SIZE_1
};

/* KMB page lookup table */
static unsigned long page_lut_read = KMB_VC8000D_PAGE_LUT;
static u8 *page_lut_regs_read;

/* Because one core may contain multi-pipeline,
 * so multicore base may be changed
 */
unsigned long multicorebase_actual[HXDEC_MAX_CORES];
int elements = 2;
static struct device *parent_dev;
static int hantro_dbg = -1;
#undef PDEBUG
#define PDEBUG(fmt, arg...)     \
	do {                                      \
		if (hantro_dbg > 0)\
			dev_info(parent_dev, fmt, ## arg); \
	} while (0)


/* here's all the must remember stuff */
struct hantrodec_t {
	char *buffer;

	unsigned int iosize[HXDEC_MAX_CORES];
	/*all access to hwregs are through readl/writel
	 * so volatile is removed according to doc "volatile is evil"
	 */
	u8 *hwregs[HXDEC_MAX_CORES];
	int irq[HXDEC_MAX_CORES];
	int hw_id[HXDEC_MAX_CORES];
	int cores;
	struct fasync_struct *async_queue_dec;
	struct fasync_struct *async_queue_pp;
};

struct core_cfg {
	/* indicate the supported format */
	u32 cfg[HXDEC_MAX_CORES];
	/* back up of cfg */
	u32 cfg_backup[HXDEC_MAX_CORES];
	/* indicate if main core exist */
	int its_main_core_id[HXDEC_MAX_CORES];
	/* indicate if aux core exist */
	int its_aux_core_id[HXDEC_MAX_CORES];
};

static struct hantrodec_t hantrodec_data; /* dynamic allocation? */

static int ReserveIO(void);
static void ReleaseIO(void);

static void ResetAsic(struct hantrodec_t *dev);

#ifdef HANTRODEC_DEBUG
static void dump_regs(struct hantrodec_t *dev);
#endif

/* IRQ handler */
static irqreturn_t hantrodec_isr(int irq, void *dev_id);

static u32 dec_regs[HXDEC_MAX_CORES][DEC_IO_SIZE_MAX / 4];
struct semaphore dec_core_sem;
struct semaphore pp_core_sem;

static int dec_irq;
static int pp_irq;

atomic_t irq_rx = ATOMIC_INIT(0);
atomic_t irq_tx = ATOMIC_INIT(0);

static struct file *dec_owner[HXDEC_MAX_CORES];
static struct file *pp_owner[HXDEC_MAX_CORES];
static int CoreHasFormat(const u32 *cfg, int core, u32 format);

/* spinlock_t owner_lock = SPIN_LOCK_UNLOCKED; */
static DEFINE_SPINLOCK(owner_lock);

static DECLARE_WAIT_QUEUE_HEAD(dec_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD(pp_wait_queue);

static DECLARE_WAIT_QUEUE_HEAD(hw_queue);

#define DWL_CLIENT_TYPE_H264_DEC         1U
#define DWL_CLIENT_TYPE_MPEG4_DEC        2U
#define DWL_CLIENT_TYPE_JPEG_DEC         3U
#define DWL_CLIENT_TYPE_PP               4U
#define DWL_CLIENT_TYPE_VC1_DEC          5U
#define DWL_CLIENT_TYPE_MPEG2_DEC        6U
#define DWL_CLIENT_TYPE_VP6_DEC          7U
#define DWL_CLIENT_TYPE_AVS_DEC          8U
#define DWL_CLIENT_TYPE_RV_DEC           9U
#define DWL_CLIENT_TYPE_VP8_DEC          10U
#define DWL_CLIENT_TYPE_VP9_DEC          11U
#define DWL_CLIENT_TYPE_HEVC_DEC         12U

static struct core_cfg config;
static u32 timeout;

#ifdef ENABLE_HANTRO_CLK
static struct clk *hantro_clk_xin_vdec;
static struct clk *hantro_clk_vdec;

static int hantro_clk_enable(void)
{
	clk_prepare_enable(hantro_clk_xin_vdec);
	clk_prepare_enable(hantro_clk_vdec);
	return 0;
}

static int hantro_clk_disable(void)
{
	if (hantro_clk_xin_vdec)
		clk_disable_unprepare(hantro_clk_xin_vdec);

	if (hantro_clk_vdec)
		clk_disable_unprepare(hantro_clk_vdec);

	return 0;
}
#endif

u32 hantrodec_readbandwidth(int isreadBW)
{
	int i;
	u32 bandwidth = 0;
	struct hantrodec_t *dev = &hantrodec_data;

	for (i = 0; i < hantrodec_data.cores; i++) {
		if (isreadBW)
			bandwidth += ioread32((void *)(dev->hwregs[i] + HANTRO_VC8KD_REG_BWREAD * 4));
		else
			bandwidth += ioread32((void *)(dev->hwregs[i] + HANTRO_VC8KD_REG_BWWRITE * 4));
	}
	return bandwidth * VC8KD_BURSTWIDTH;
}

static void ReadCoreConfig(struct hantrodec_t *dev)
{
	int c;
	u32 reg, tmp, mask;

	memset(config.cfg, 0, sizeof(config.cfg));

	for (c = 0; c < dev->cores; c++) {
		/* Decoder configuration */
		if (IS_G1(dev->hw_id[c])) {
			reg =
				ioread32((void *)
				(dev->hwregs[c] + HANTRODEC_SYNTH_CFG * 4));

			tmp = (reg >> DWL_H264_E) & 0x3U;
			if (tmp)
				pr_info("hantrodec: core[%d] has H264\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

			tmp = (reg >> DWL_JPEG_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has JPEG\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;

			tmp = (reg >> DWL_MPEG4_E) & 0x3U;
			if (tmp)
				pr_info("hantrodec: core[%d] has MPEG4\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_MPEG4_DEC : 0;

			tmp = (reg >> DWL_VC1_E) & 0x3U;
			if (tmp)
				pr_info("hantrodec: core[%d] has VC1\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_VC1_DEC : 0;

			tmp = (reg >> DWL_MPEG2_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has MPEG2\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_MPEG2_DEC : 0;

			tmp = (reg >> DWL_VP6_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has VP6\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_VP6_DEC : 0;

			reg =
				ioread32((void *)
				(dev->hwregs[c] + HANTRODEC_SYNTH_CFG_2 * 4));

			/* VP7 and WEBP is part of VP8 */
			mask = (1 << DWL_VP8_E) |
					(1 << DWL_VP7_E) |
					(1 << DWL_WEBP_E);
			tmp = (reg & mask);
			if (tmp & (1 << DWL_VP8_E))
				pr_info("hantrodec: core[%d] has VP8\n", c);
			if (tmp & (1 << DWL_VP7_E))
				pr_info("hantrodec: core[%d] has VP7\n", c);
			if (tmp & (1 << DWL_WEBP_E))
				pr_info("hantrodec: core[%d] has WebP\n", c);
			config.cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_VP8_DEC : 0;

			tmp = (reg >> DWL_AVS_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has AVS\n", c);
			config.cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_AVS_DEC : 0;

			tmp = (reg >> DWL_RV_E) & 0x03U;
			if (tmp)
				pr_info("hantrodec: core[%d] has RV\n", c);
			config.cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_RV_DEC : 0;

			/* Post-processor configuration */
			reg =
				ioread32((void *)(dev->hwregs[c] +
					HANTROPP_SYNTH_CFG * 4));

			tmp = (reg >> DWL_G1_PP_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has PP\n", c);
			config.cfg[c] |= tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;
		} else if ((IS_G2(dev->hw_id[c]))) {
			reg = ioread32((void *)(dev->hwregs[c] +
				HANTRODEC_CFG_STAT * 4));

			tmp = (reg >> DWL_G2_HEVC_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has HEVC\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_HEVC_DEC : 0;

			tmp = (reg >> DWL_G2_VP9_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has VP9\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_VP9_DEC : 0;

			/* Post-processor configuration */
			reg = ioread32((void *)(dev->hwregs[c] +
				HANTRODECPP_SYNTH_CFG * 4));

			tmp = (reg >> DWL_G2_PP_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has PP\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;
		} else if ((IS_VC8000D(dev->hw_id[c])) &&
			config.its_main_core_id[c] < 0) {
			reg = ioread32((void *)(dev->hwregs[c] +
					HANTRODEC_SYNTH_CFG * 4));

			tmp = (reg >> DWL_H264_E) & 0x3U;
			if (tmp)
				pr_info("hantrodec: core[%d] has H264\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

			tmp = (reg >> DWL_JPEG_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has JPEG\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;

			tmp = (reg >> DWL_MPEG4_E) & 0x3U;
			if (tmp)
				pr_info("hantrodec: core[%d] has MPEG4\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_MPEG4_DEC : 0;

			tmp = (reg >> DWL_VC1_E) & 0x3U;
			if (tmp)
				pr_info("hantrodec: core[%d] has VC1\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_VC1_DEC : 0;

			tmp = (reg >> DWL_MPEG2_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has MPEG2\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_MPEG2_DEC : 0;

			tmp = (reg >> DWL_VP6_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has VP6\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_VP6_DEC : 0;

			reg = ioread32((void *)(dev->hwregs[c] +
					HANTRODEC_SYNTH_CFG_2 * 4));

			/* VP7 and WEBP is part of VP8 */
			mask = (1 << DWL_VP8_E) |
					(1 << DWL_VP7_E) |
					(1 << DWL_WEBP_E);
			tmp = (reg & mask);
			if (tmp & (1 << DWL_VP8_E))
				pr_info("hantrodec: core[%d] has VP8\n", c);
			if (tmp & (1 << DWL_VP7_E))
				pr_info("hantrodec: core[%d] has VP7\n", c);
			if (tmp & (1 << DWL_WEBP_E))
				pr_info("hantrodec: core[%d] has WebP\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_VP8_DEC : 0;

			tmp = (reg >> DWL_AVS_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has AVS\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_AVS_DEC : 0;

			tmp = (reg >> DWL_RV_E) & 0x03U;
			if (tmp)
				pr_info("hantrodec: core[%d] has RV\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_RV_DEC : 0;

			reg = ioread32((void *)(dev->hwregs[c] +
					HANTRODEC_SYNTH_CFG_3 * 4));

			tmp = (reg >> DWL_HEVC_E) & 0x07U;
			if (tmp)
				pr_info("hantrodec: core[%d] has HEVC\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_HEVC_DEC : 0;

			tmp = (reg >> DWL_VP9_E) & 0x07U;
			if (tmp)
				pr_info("hantrodec: core[%d] has VP9\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_VP9_DEC : 0;

			/* Post-processor configuration */
			reg = ioread32((void *)(dev->hwregs[c] +
					HANTRODECPP_CFG_STAT * 4));

			tmp = (reg >> DWL_PP_E) & 0x01U;
			if (tmp)
				pr_info("hantrodec: core[%d] has PP\n", c);
			config.cfg[c] |=
				tmp ? 1 << DWL_CLIENT_TYPE_PP : 0;

			if (config.its_aux_core_id[c] >= 0) {
				/* set main_core_id and aux_core_id */
				reg = ioread32((void *)(dev->hwregs[c] +
						HANTRODEC_SYNTH_CFG_2 * 4));

				tmp = (reg >> DWL_H264_PIPELINE_E) & 0x01U;
				if (tmp)
					pr_info("hantrodec: core[%d] has pipeline H264\n", c);
				config.cfg[config.its_aux_core_id[c]] |=
					tmp ? 1 << DWL_CLIENT_TYPE_H264_DEC : 0;

				tmp = (reg >> DWL_JPEG_PIPELINE_E) & 0x01U;
				if (tmp)
					pr_info("hantrodec: core[%d] has pipeline JPEG\n", c);
				config.cfg[config.its_aux_core_id[c]] |=
					tmp ? 1 << DWL_CLIENT_TYPE_JPEG_DEC : 0;
			}
		}
	}
	memcpy(config.cfg_backup, config.cfg, sizeof(config.cfg));
}

static int CoreHasFormat(const u32 *cfg, int core, u32 format)
{
	return (cfg[core] & (1 << format)) ? 1 : 0;
}

static int GetDecCore(
	long core,
	struct hantrodec_t *dev,
	struct file *filp,
	unsigned long format)
{
	int success = 0;
	unsigned long flags;

	spin_lock_irqsave(&owner_lock, flags);
	if (CoreHasFormat(config.cfg, core, format) &&
		/*&& config.its_main_core_id[core] >= 0*/
		dec_owner[core] == NULL) {
		dec_owner[core] = filp;
		success = 1;

		/* If one main core takes one format which doesn't supported
		 * by aux core, set aux core's cfg to none video format support
		 */
		if (config.its_aux_core_id[core] >= 0) {
			if (!CoreHasFormat(config.cfg, config.its_aux_core_id[core], format)) {
				config.cfg[config.its_aux_core_id[core]] = 0;
			} else {
				config.cfg[config.its_aux_core_id[core]] = (1 << format);
			}
		}
		/* If one aux core takes one format,
		 *set main core's cfg to aux core supported video format
		 */
		else if (config.its_main_core_id[core] >= 0) {
			config.cfg[config.its_main_core_id[core]] = (1 << format);
		}
	}

	spin_unlock_irqrestore(&owner_lock, flags);

	return success;
}

static int GetDecCoreAny(
	long *core,
	struct hantrodec_t *dev,
	struct file *filp,
	unsigned long format)
{
	int success = 0;
	long c;

	*core = -1;

	for (c = 0; c < dev->cores; c++) {
		/* a free core that has format */
		if (GetDecCore(c, dev, filp, format)) {
			success = 1;
			*core = c;
			break;
		}
	}

	return success;
}

static int GetDecCoreID(
	struct hantrodec_t *dev,
	struct file *filp,
	unsigned long format)
{
	long c;
	unsigned long flags;

	int core_id = -1;

	for (c = 0; c < dev->cores; c++) {
		/* a core that has format */
		spin_lock_irqsave(&owner_lock, flags);
		if (CoreHasFormat(config.cfg_backup, c, format)) {
			core_id = c;
			spin_unlock_irqrestore(&owner_lock, flags);
			break;
		}
		spin_unlock_irqrestore(&owner_lock, flags);
	}
	return core_id;
}


static long ReserveDecoder(struct hantrodec_t *dev, struct file *filp, unsigned long format)
{
	long core = -1;

	/* reserve a core */
	if (down_interruptible(&dec_core_sem))
		return -ERESTARTSYS;

	/* lock a core that has specific format*/
	if (wait_event_interruptible(hw_queue,
		GetDecCoreAny(&core, dev, filp, format) != 0))
		return -ERESTARTSYS;
	PDEBUG("reserve core %ld:%lx", core, (unsigned long)filp);

	return core;
}

static void ReleaseDecoder(struct hantrodec_t *dev, long core)
{
	u32 status;
	unsigned long flags;

	status = ioread32((void *)(dev->hwregs[core] +
				HANTRODEC_IRQ_STAT_DEC_OFF));

	/* make sure HW is disabled */
	if (status & HANTRODEC_DEC_E) {
		PDEBUG("hantrodec: DEC[%li] still enabled -> reset\n", core);

		/* abort decoder */
		status |= HANTRODEC_DEC_ABORT | HANTRODEC_DEC_IRQ_DISABLE;
		iowrite32(status, (void *)(dev->hwregs[core] +
				HANTRODEC_IRQ_STAT_DEC_OFF));
	}

	spin_lock_irqsave(&owner_lock, flags);

	/* If aux core released, revert main core's config back */
	if (config.its_main_core_id[core] >= 0)
		config.cfg[config.its_main_core_id[core]] =
			config.cfg_backup[config.its_main_core_id[core]];

	/* If main core released, revert aux core's config back */
	if (config.its_aux_core_id[core] >= 0)
		config.cfg[config.its_aux_core_id[core]] =
			config.cfg_backup[config.its_aux_core_id[core]];

	PDEBUG("release core %ld", core);
	dec_owner[core] = NULL;
	spin_unlock_irqrestore(&owner_lock, flags);
	up(&dec_core_sem);
	wake_up_interruptible_all(&hw_queue);
}

static long ReservePostProcessor(struct hantrodec_t *dev, struct file *filp)
{
	unsigned long flags;

	long core = 0;

	/* single core PP only */
	if (down_interruptible(&pp_core_sem))
		return -ERESTARTSYS;

	spin_lock_irqsave(&owner_lock, flags);

	pp_owner[core] = filp;

	spin_unlock_irqrestore(&owner_lock, flags);

	return core;
}

static void ReleasePostProcessor(struct hantrodec_t *dev, long core)
{
	unsigned long flags;

	u32 status = ioread32((void *)(dev->hwregs[core] +
			HANTRO_IRQ_STAT_PP_OFF));

	/* make sure HW is disabled */
	if (status & HANTRO_PP_E) {
		PDEBUG("hantrodec: PP[%li] still enabled -> reset\n", core);

		/* disable IRQ */
		status |= HANTRO_PP_IRQ_DISABLE;

		/* disable postprocessor */
		status &= (~HANTRO_PP_E);
		iowrite32(0x10, (void *)(dev->hwregs[core] +
				HANTRO_IRQ_STAT_PP_OFF));
	}

	spin_lock_irqsave(&owner_lock, flags);

	pp_owner[core] = NULL;

	spin_unlock_irqrestore(&owner_lock, flags);

	up(&pp_core_sem);
}

long ReserveDecPp(struct hantrodec_t *dev, struct file *filp, unsigned long format)
{
	/* reserve core 0, DEC+PP for pipeline */
	unsigned long flags;

	long core = 0;

	/* check that core has the requested dec format */
	if (!CoreHasFormat(config.cfg, core, format))
		return -EFAULT;

	/* check that core has PP */
	if (!CoreHasFormat(config.cfg, core, DWL_CLIENT_TYPE_PP))
		return -EFAULT;

	/* reserve a core */
	if (down_interruptible(&dec_core_sem))
		return -ERESTARTSYS;

	/* wait until the core is available */
	if (wait_event_interruptible(hw_queue,
		GetDecCore(core, dev, filp, format) != 0)) {
		up(&dec_core_sem);
		return -ERESTARTSYS;
	}

	if (down_interruptible(&pp_core_sem)) {
		ReleaseDecoder(dev, core);
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&owner_lock, flags);
	pp_owner[core] = filp;
	spin_unlock_irqrestore(&owner_lock, flags);

	return core;
}

static long DecFlushRegs(struct hantrodec_t *dev, struct core_desc *core)
{
	long ret = 0, i;

	u32 id = core->id;

	ret = copy_from_user(dec_regs[id], core->regs, HANTRO_VC8000D_REGS * 4);
	if (ret) {
		PDEBUG("copy_from_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	/* write all regs but the status reg[1] to hardware */
	//iowrite32(0x0, (void *)(dev->hwregs[id] + 4));
	//for(i = 2; i <= HANTRO_G2_DEC_LAST_REG; i++) {
	for (i = 2; i <= HANTRO_VC8000D_LAST_REG; i++)
		iowrite32(dec_regs[id][i], (void *)(dev->hwregs[id] + i * 4));

	/* write the status register, which may start the decoder */
	iowrite32(dec_regs[id][1], (void *)(dev->hwregs[id] + 4));

	PDEBUG("flushed registers on core %d\n", id);

	return 0;
}

static long DecRefreshRegs(struct hantrodec_t *dev, struct core_desc *core)
{
	long ret, i;
	u32 id = core->id;

	for (i = 0; i <= HANTRO_VC8000D_LAST_REG; i++)
		dec_regs[id][i] = ioread32((void *)(dev->hwregs[id] + i * 4));

	ret = copy_to_user(core->regs, dec_regs[id],
			HANTRO_VC8000D_LAST_REG * 4);
	if (ret) {
		PDEBUG("copy_to_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	return 0;
}

static int CheckDecIrq(struct hantrodec_t *dev, int id)
{
	unsigned long flags;
	int rdy = 0;

	const u32 irq_mask = (1 << id);

	spin_lock_irqsave(&owner_lock, flags);

	if (dec_irq & irq_mask) {
		/* reset the wait condition(s) */
		dec_irq &= ~irq_mask;
		rdy = 1;
	}

	spin_unlock_irqrestore(&owner_lock, flags);

	return rdy;
}

static long WaitDecReadyAndRefreshRegs(struct hantrodec_t *dev, struct core_desc *Core)
{
	u32 id = Core->id;
	long ret;

	PDEBUG("wait_event_interruptible DEC[%d]\n", id);

	ret = wait_event_interruptible_timeout(dec_wait_queue, CheckDecIrq(dev, id), msecs_to_jiffies(10));
	if (ret == -ERESTARTSYS) {
		pr_err("DEC[%d]  failed to wait_event_interruptible interrupted\n", id);
		return -ERESTARTSYS;
	} else if (ret == 0) {
		pr_err("DEC[%d]  wait_event_interruptible timeout\n", id);
		timeout = 1;
		return -EBUSY;
	}
	atomic_inc(&irq_tx);

	/* refresh registers */
	return DecRefreshRegs(dev, Core);
}

static long DecWriteRegs(struct hantrodec_t *dev, struct core_desc *core)
{
	long ret = 0, i;
	u32 id = core->id;

	i = core->reg_id;
	ret = copy_from_user(dec_regs[id] + core->reg_id,
			core->regs + core->reg_id, 4);
	if (ret) {
		PDEBUG("copy_from_user failed, returned %li\n", ret);
		return -EFAULT;
	}
	iowrite32(dec_regs[id][i], (void *)dev->hwregs[id] + i * 4);
	return 0;
}

u32 *hantrodec_getRegAddr(u32 coreid, u32 regid)
{
	if (coreid >= hantrodec_data.cores)
		return NULL;
	if (regid * 4 >= hantrodec_data.iosize[coreid])
		return NULL;
	return (u32 *)(hantrodec_data.hwregs[coreid] + regid * 4);
}

static long DecReadRegs(struct hantrodec_t *dev, struct core_desc *core)
{
	long ret, i;
	u32 id = core->id;

	i = core->reg_id;

	/* user has to know exactly what they are asking for */
	//if(core->size != (HANTRO_VC8000D_REGS * 4))
	//  return -EFAULT;

	/* read specific registers from hardware */
	i = core->reg_id;
	dec_regs[id][i] = ioread32((void *)dev->hwregs[id] + i * 4);

	/* put registers to user space*/
	ret = copy_to_user(core->regs + core->reg_id,
			dec_regs[id] + core->reg_id, 4);
	if (ret) {
		PDEBUG("copy_to_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	return 0;
}

static long PPFlushRegs(struct hantrodec_t *dev, struct core_desc *Core)
{
	long ret = 0;
	u32 id = Core->id;
	u32 i;

	/* copy original dec regs to kernal space*/
	ret = copy_from_user(dec_regs[id] + HANTRO_PP_ORG_FIRST_REG,
			Core->regs + HANTRO_PP_ORG_FIRST_REG,
			HANTRO_PP_ORG_REGS * 4);
#ifdef USE_64BIT_ENV
	/* copy extended dec regs to kernal space*/
	ret = copy_from_user(dec_regs[id] + HANTRO_PP_EXT_FIRST_REG,
			Core->regs + HANTRO_PP_EXT_FIRST_REG,
			HANTRO_PP_EXT_REGS * 4);
#endif
	if (ret) {
		pr_err("copy_from_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	/* write all regs but the status reg[1] to hardware */
	/* both original and extended regs need to be written */
	for (i = HANTRO_PP_ORG_FIRST_REG + 1;
		i <= HANTRO_PP_ORG_LAST_REG;
		i++)
		iowrite32(dec_regs[id][i], (void *)dev->hwregs[id] + i * 4);
#ifdef USE_64BIT_ENV
	for (i = HANTRO_PP_EXT_FIRST_REG;
		i <= HANTRO_PP_EXT_LAST_REG;
		i++)
		iowrite32(dec_regs[id][i], (void *)dev->hwregs[id] + i * 4);
#endif
	/* write the stat reg, which may start the PP */
	iowrite32(dec_regs[id][HANTRO_PP_ORG_FIRST_REG],
		(void *)dev->hwregs[id] +
		HANTRO_PP_ORG_FIRST_REG * 4);

	return 0;
}

static long PPRefreshRegs(struct hantrodec_t *dev, struct core_desc *Core)
{
	long i, ret;
	u32 id = Core->id;
#ifdef USE_64BIT_ENV
	/* user has to know exactly what they are asking for */
	if (Core->size != (HANTRO_PP_TOTAL_REGS * 4))
		return -EFAULT;
#else
	/* user has to know exactly what they are asking for */
	if (Core->size != (HANTRO_PP_ORG_REGS * 4))
		return -EFAULT;
#endif

	/* read all registers from hardware */
	/* both original and extended regs need to be read */
	for (i = HANTRO_PP_ORG_FIRST_REG; i <= HANTRO_PP_ORG_LAST_REG; i++)
		dec_regs[id][i] = ioread32((void *)dev->hwregs[id] + i * 4);
#ifdef USE_64BIT_ENV
	for (i = HANTRO_PP_EXT_FIRST_REG; i <= HANTRO_PP_EXT_LAST_REG; i++)
		dec_regs[id][i] = ioread32((void *)dev->hwregs[id] + i * 4);
#endif
	/* put registers to user space*/
	/* put original registers to user space*/
	ret = copy_to_user(Core->regs + HANTRO_PP_ORG_FIRST_REG,
		dec_regs[id] + HANTRO_PP_ORG_FIRST_REG, HANTRO_PP_ORG_REGS * 4);
#ifdef USE_64BIT_ENV
	/* put extended registers to user space*/
	ret = copy_to_user(Core->regs + HANTRO_PP_EXT_FIRST_REG,
			dec_regs[id] + HANTRO_PP_EXT_FIRST_REG,
			HANTRO_PP_EXT_REGS * 4);
#endif
	if (ret) {
		pr_err("copy_to_user failed, returned %li\n", ret);
		return -EFAULT;
	}

	return 0;
}

static int CheckPPIrq(struct hantrodec_t *dev, int id)
{
	unsigned long flags;
	int rdy = 0;

	const u32 irq_mask = (1 << id);

	spin_lock_irqsave(&owner_lock, flags);

	if (pp_irq & irq_mask) {
		/* reset the wait condition(s) */
		pp_irq &= ~irq_mask;
		rdy = 1;
	}

	spin_unlock_irqrestore(&owner_lock, flags);

	return rdy;
}

static long WaitPPReadyAndRefreshRegs(struct hantrodec_t *dev, struct core_desc *Core)
{
	u32 id = Core->id;

	PDEBUG("wait_event_interruptible PP[%d]\n", id);

	if (wait_event_interruptible(pp_wait_queue, CheckPPIrq(dev, id))) {
		pr_err("PP[%d]  failed to wait_event_interruptible interrupted\n", id);
		return -ERESTARTSYS;
	}

	atomic_inc(&irq_tx);

	/* refresh registers */
	return PPRefreshRegs(dev, Core);
}

static int CheckCoreIrq(struct hantrodec_t *dev, const struct file *filp, int *id)
{
	unsigned long flags;
	int rdy = 0, n = 0;

	do {
		u32 irq_mask = (1 << n);

		spin_lock_irqsave(&owner_lock, flags);

		if (dec_irq & irq_mask) {
			PDEBUG("%s get irq for core %d:%lx", __func__, n, (unsigned long)filp);

			if (*id == n) {	//if(dec_owner[n] == filp)
				/* we have an IRQ for our client */

				/* reset the wait condition(s) */
				dec_irq &= ~irq_mask;

				/* signal ready Core no. for our client */
				*id = n;

				rdy = 1;
				spin_unlock_irqrestore(&owner_lock, flags);
				break;
			} else if (dec_owner[n] == NULL) {
				/* zombie IRQ */
				PDEBUG("IRQ on Core[%d], but no owner!!!\n", n);

				/* reset the wait condition(s) */
				dec_irq &= ~irq_mask;
			}
		}

		spin_unlock_irqrestore(&owner_lock, flags);

		n++; /* next Core */
	} while (n < dev->cores);

	return rdy;
}

static long WaitCoreReady(struct hantrodec_t *dev, const struct file *filp, int *id)
{
	PDEBUG("wait_event_interruptible CORE\n");

	if (wait_event_interruptible(dec_wait_queue,
		CheckCoreIrq(dev, filp, id))) {
		pr_err("CORE  failed to wait_event_interruptible interrupted\n");
		return -ERESTARTSYS;
	}

	atomic_inc(&irq_tx);

	return 0;
}

/*-------------------------------------------------------------------------
 *Function name   : hantrodec_ioctl
 *Description     : communication method to/from the user space
 *
 *Return type     : long
 *-------------------------------------------------------------------------
 */

long hantrodec_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	u32 id;
	long tmp;
	struct core_desc core;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(HANTRODEC_IOC_CLI): {
		id = arg;
		if (id >= hantrodec_data.cores)
			return -EFAULT;
		disable_irq(hantrodec_data.irq[id]);
		break;
	}
	case _IOC_NR(HANTRODEC_IOC_STI): {
		id = arg;
		if (id >= hantrodec_data.cores)
			return -EFAULT;
		enable_irq(hantrodec_data.irq[id]);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCGHWOFFSET): {
		__get_user(id, (__u32 *)arg);
		if (id >= hantrodec_data.cores)
			return -EFAULT;

		__put_user(multicorebase_actual[id], (unsigned long *) arg);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCGHWIOSIZE): {
		__u32 io_size;

		__get_user(id, (__u32 *)arg);
		if (id >= hantrodec_data.cores)
			return -EFAULT;
		io_size = hantrodec_data.iosize[id];
		__put_user(io_size, (u32 *) arg);

		return 0;
	}
	case _IOC_NR(HANTRODEC_IOC_MC_OFFSETS): {
		tmp = copy_to_user((unsigned long *) arg,
				multicorebase_actual,
				sizeof(multicorebase_actual));
		if (err) {
			pr_err("copy_to_user failed, returned %li\n", tmp);
			return -EFAULT;
		}
		break;
	}
	case _IOC_NR(HANTRODEC_IOC_MC_CORES):
		__put_user(hantrodec_data.cores, (unsigned int *) arg);
		PDEBUG("hantrodec_data.cores=%d\n", hantrodec_data.cores);
		break;
	case _IOC_NR(HANTRODEC_IOCS_DEC_PUSH_REG): {
		struct core_desc Core;

		/* get registers from user space*/
		tmp = copy_from_user(&Core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		DecFlushRegs(&hantrodec_data, &Core);
		break;
	}

	case _IOC_NR(HANTRODEC_IOCS_DEC_WRITE_REG): {
		/* get registers from user space*/
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			PDEBUG("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		DecWriteRegs(&hantrodec_data, &core);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCS_PP_PUSH_REG): {
		/* get registers from user space*/
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		PPFlushRegs(&hantrodec_data, &core);
		break;
	}
	case _IOC_NR(HANTRODEC_IOCS_DEC_PULL_REG): {
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		return DecRefreshRegs(&hantrodec_data, &core);
	}
	case _IOC_NR(HANTRODEC_IOCS_DEC_READ_REG): {
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			PDEBUG("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		return DecReadRegs(&hantrodec_data, &core);
	}
	case _IOC_NR(HANTRODEC_IOCS_PP_PULL_REG): {
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		return PPRefreshRegs(&hantrodec_data, &core);
	}
	case _IOC_NR(HANTRODEC_IOCH_DEC_RESERVE): {
		PDEBUG("Reserve DEC core, format = %li\n", arg);
		return ReserveDecoder(&hantrodec_data, filp, arg);
	}
	case _IOC_NR(HANTRODEC_IOCT_DEC_RELEASE): {
		if (arg >= hantrodec_data.cores ||
			dec_owner[arg] != filp) {
			pr_err("bogus DEC release, Core = %li\n", arg);
			return -EFAULT;
		}

		PDEBUG("Release DEC, core = %li\n", arg);

		ReleaseDecoder(&hantrodec_data, arg);

		break;
	}
	case _IOC_NR(HANTRODEC_IOCQ_PP_RESERVE):
		return ReservePostProcessor(&hantrodec_data, filp);
	case _IOC_NR(HANTRODEC_IOCT_PP_RELEASE): {
		if (arg != 0 || pp_owner[arg] != filp) {
			pr_err("bogus PP release %li\n", arg);
			return -EFAULT;
		}

		ReleasePostProcessor(&hantrodec_data, arg);

		break;
	}
	case _IOC_NR(HANTRODEC_IOCX_DEC_WAIT): {
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		return WaitDecReadyAndRefreshRegs(&hantrodec_data, &core);
	}
	case _IOC_NR(HANTRODEC_IOCX_PP_WAIT): {
		tmp = copy_from_user(&core,
				(void *)arg, sizeof(struct core_desc));
		if (tmp) {
			pr_err("copy_from_user failed, returned %li\n", tmp);
			return -EFAULT;
		}

		return WaitPPReadyAndRefreshRegs(
				&hantrodec_data,
				&core);
	}
	case _IOC_NR(HANTRODEC_IOCG_CORE_WAIT): {
		int id;

		__get_user(id, (int *)arg);
		tmp = WaitCoreReady(&hantrodec_data, filp, &id);
		__put_user(id, (int *) arg);
		return tmp;
	}
	case _IOC_NR(HANTRODEC_IOX_ASIC_ID): {
		__get_user(id, (u32 *)arg);
		if (id >= hantrodec_data.cores)
			return -EFAULT;
		id = ioread32((void *)hantrodec_data.hwregs[id]);
		__put_user(id, (u32 *) arg);
		return 0;
	}
	case _IOC_NR(HANTRODEC_IOCG_CORE_ID): {
		PDEBUG("Get DEC Core_id, format = %li\n", arg);
		tmp = GetDecCoreID(&hantrodec_data, filp, arg);
		return tmp;
	}
	case _IOC_NR(HANTRODEC_DEBUG_STATUS): {
		PDEBUG("hantrodec: dec_irq     = 0x%08x\n", dec_irq);
		PDEBUG("hantrodec: pp_irq      = 0x%08x\n", pp_irq);

		PDEBUG("hantrodec: IRQs received/sent2user = %d / %d\n",
		       atomic_read(&irq_rx), atomic_read(&irq_tx));

		for (tmp = 0; tmp < hantrodec_data.cores; tmp++) {
			PDEBUG("hantrodec: dec_core[%li] %s\n",
			       tmp, dec_owner[tmp] == NULL ?
			       "FREE" : "RESERVED");
			PDEBUG("hantrodec: pp_core[%li]  %s\n",
			       tmp, pp_owner[tmp] == NULL ?
			       "FREE" : "RESERVED");
		}
		break;
	}
	default:
		return -ENOTTY;
	}

	return 0;
}

/*---------------------------------------------------------------------------
 *Function name   : hantrodec_release
 *Description     : Release driver
 *
 *Return type     : int
 *----------------------------------------------------------------------------
 */
int hantrodec_release(struct file *filp)
{
	int n;
	struct hantrodec_t *dev = &hantrodec_data;

	for (n = 0; n < dev->cores; n++) {
		if (dec_owner[n] == filp) {
			PDEBUG("releasing dec Core %i lock\n", n);
			ReleaseDecoder(dev, n);
		}
	}

	for (n = 0; n < 1; n++) {
		if (pp_owner[n] == filp) {
			PDEBUG("releasing pp Core %i lock\n", n);
			ReleasePostProcessor(dev, n);
		}
	}

	PDEBUG("closed\n");
	return 0;
}

/*---------------------------------------------------------------------------
 *Function name   : hantrodec_init
 *Description     : Initialize the driver
 *
 *Return type     : int
 *---------------------------------------------------------------------------
 */

int hantrodec_open(struct inode *inode, struct file *filp)
{
	//hantro_clk_enable();
	return 0;
}

int hantrodec_init(struct platform_device *pdev)
{
	int result = 0;
	int irq_0;
#ifdef USE_JUNO
	int irq_1;
#endif
	int i;

	dec_irq = 0;
	pp_irq = 0;
	parent_dev = &pdev->dev;
	pr_info("hantrodec: Init multi Core[0] at 0x%16lx\n"
		"Core[1] at 0x%16lx\n", multicorebase[0], multicorebase[1]);

	hantrodec_data.cores = 0;
	hantrodec_data.iosize[0] = DEC_IO_SIZE_0;
	hantrodec_data.irq[0] = -1;
	hantrodec_data.iosize[1] = DEC_IO_SIZE_1;
	hantrodec_data.irq[1] = -1;

	for (i = 0; i < HXDEC_MAX_CORES; i++) {
		hantrodec_data.hwregs[i] = 0;
		/* If user gave less core bases that we have by default,
		 * invalidate default bases
		 */
		if (elements && i >= elements)
			multicorebase[i] = -1;
	}

	hantrodec_data.async_queue_dec = NULL;
	hantrodec_data.async_queue_pp = NULL;

#ifdef ENABLE_HANTRO_CLK /* Enable and set the VDEC clks */
	hantro_clk_xin_vdec = clk_get(&pdev->dev, "clk_xin_vdec");
	hantro_clk_vdec = clk_get(&pdev->dev, "clk_vdec");
	hantro_clk_enable();

	/* Set KMB CLK to 700 Mhz VDEC */
	pr_info("hantrodec: Before setting any clocks: clk_xin_vdec: %ld | clk_vdec %ld\n",
		clk_get_rate(hantro_clk_xin_vdec), clk_get_rate(hantro_clk_vdec));
	clk_set_rate(hantro_clk_xin_vdec, 700000000);
	pr_info("hantrodec: Trying to set 700Mhz: clk_xin_vdec: %ld | clk_vdec %ld\n",
		clk_get_rate(hantro_clk_xin_vdec), clk_get_rate(hantro_clk_vdec));
	if (clk_get_rate(hantro_clk_xin_vdec) < 700000000) {
		pr_info("hantrodec: Set to 700 Mhz failed, setting to 666Mhz\n");
		clk_set_rate(hantro_clk_xin_vdec, 666666666);
		pr_info("hantrodec: clk_xin_vdec: %ld | clk_vdec %ld\n", clk_get_rate(hantro_clk_xin_vdec), clk_get_rate(hantro_clk_vdec));
	}
#endif

	result = ReserveIO();
	if (result < 0)
		goto err;
	PDEBUG("reserveIO success\n");

	memset(dec_owner, 0, sizeof(dec_owner));
	memset(pp_owner, 0, sizeof(pp_owner));

	//FIXME: should be cores-1 for multi-core
	sema_init(&dec_core_sem, hantrodec_data.cores);
	sema_init(&pp_core_sem, 1);

	/* read configuration fo all cores */
	ReadCoreConfig(&hantrodec_data);

	/* reset hardware */
	ResetAsic(&hantrodec_data);

	/* Dynamic AXI ID and Page LUT routines */
	/* Register and set the page lookup table for read */
	if (!request_mem_region(page_lut_read, hantrodec_data.iosize[0], "hantrodec_pagelut_read")) {
		pr_info("hantrodec: failed to reserve page lookup table registers\n");
		return -EBUSY;
	}
	page_lut_regs_read = (u8 *)ioremap(page_lut_read, hantrodec_data.iosize[0]);
	if (page_lut_regs_read == NULL)
		pr_info("hantrodec: failed to ioremap page lookup table registers\n");

	/* Set VDEC RD Page LUT AXI ID 0-15 to 0x4 */
	iowrite32(0x04040404, (void *)page_lut_regs_read);
	pr_info("hantrodec: RD AXI ID 3:0 = %x\n", ioread32((void *) page_lut_regs_read));
	iowrite32(0x04040404, (void *)page_lut_regs_read + 0x4);
	pr_info("hantrodec: RD AXI ID 7:4 = %x\n", ioread32((void *) page_lut_regs_read + 0x4));
	iowrite32(0x04040404, (void *)page_lut_regs_read + 0x8);
	pr_info("hantrodec: RD AXI ID 11:8 = %x\n", ioread32((void *) page_lut_regs_read + 0x8));
	iowrite32(0x04040404, (void *)page_lut_regs_read + 0xc);
	pr_info("hantrodec: RD AXI ID 15:12 = %x\n", ioread32((void *) page_lut_regs_read + 0xc));

#ifdef STATIC_AXI_WR
	iowrite32(0x04, (void *)page_lut_regs_read + 0x10);
	pr_info("hantrodec: WR AXI ID 0 = %x\n", ioread32((void *) page_lut_regs_read + 0x10));
#else	/* dynamic WR AXI ID */
	/* Set sw_dec_axi_wr_id_e to 1 */
	iowrite32(1<<13, (void *) hantrodec_data.hwregs[0] + 0xE8);
	pr_info("hantrodec: sw_dec_axi_wr_id_e  = %x\n", ioread32((void *) hantrodec_data.hwregs[0] + 0xE8));
	/* Set WR Page LUT AXI ID 0-3, 6-15 to 0x4 and WR Page LUT AXI ID 4,5 to 0x0 */
	iowrite32(0x04040404, (void *)page_lut_regs_read + 0x10);
	pr_info("hantrodec: page_lut_regs WR AXI ID 3:0= %x\n", ioread32((void *) page_lut_regs_read + 0x10));
	iowrite32(0x04040000, (void *)page_lut_regs_read + 0x14);
	pr_info("hantrodec: page_lut_regs WR AXI ID 7:4= %x\n", ioread32((void *) page_lut_regs_read + 0x14));
	iowrite32(0x04040404, (void *)page_lut_regs_read + 0x18);
	pr_info("hantrodec: page_lut_regs WR AXI ID 11:8= %x\n", ioread32((void *) page_lut_regs_read + 0x18));
	iowrite32(0x04040404, (void *)page_lut_regs_read + 0x1c);
	pr_info("hantrodec: page_lut_regs WR AXI ID 15:12= %x\n", ioread32((void *) page_lut_regs_read + 0x1c));
#endif
	/* register irq for each core*/
	irq_0 = irq[0];
#ifdef USE_IRQ
	if (irq_0 > 0) {
		PDEBUG("irq_0 platform_get_irq\n");
		irq_0 = platform_get_irq_byname(pdev, "irq_hantro_decoder");
		result = request_irq(irq_0, hantrodec_isr, IRQF_SHARED,
#ifdef USE_JUNO
		"irq_hantro_decoder0", (void *) &hantrodec_data);
#else
		"irq_hantro_decoder", (void *) &hantrodec_data);
#endif
		if (result != 0) {
			PDEBUG("can't reserve irq0\n");
			goto err0;
		}
		PDEBUG("reserve irq0 success with irq0 = %d\n", irq_0);
		hantrodec_data.irq[0] = irq_0;
	} else {
		PDEBUG("can't get irq0 and irq0 value = %d\n", irq_0);
		result = -EINVAL;
		goto err0;
	}
#endif

#ifdef USE_JUNO
	irq_1 = irq[1];
#ifdef USE_IRQ
	if (irq_1 > 0) {
		PDEBUG("irq_1 platform_get_irq\n");
		irq_1 = platform_get_irq(pdev, 0);
		PDEBUG("irq1 platform_get_irq = %d\n", irq_1);
		result = request_irq(irq_1, hantrodec_isr, IRQF_SHARED,
				"irq_hantro_decoder1", (void *) &hantrodec_data);

		if (result != 0) {
			PDEBUG("can't reserve irq1, result = %d\n", result);
			goto err1;
		}
		PDEBUG("reserve irq1 success, irq1 = %d\n", irq_1);
		hantrodec_data.irq[1] = irq_1;
	} else {
		PDEBUG("can't get irq1 and irq1 value = %d\n", irq_1);
		result = -EINVAL;
		goto err1;
	}
#endif
#endif
	pr_info("hantrodec: module inserted.\n");
	return 0;

#ifdef USE_JUNO
err1:
	free_irq(irq_0, (void *)&hantrodec_data);
#endif
err0:
	ReleaseIO();

err:
	return result;
}

/*---------------------------------------------------------------------------
 *Function name   : hantrodec_cleanup
 *Description     : clean up
 *
 *Return type     : int
 *---------------------------------------------------------------------------
 */
void hantrodec_cleanup(void)
{
	struct hantrodec_t *dev = &hantrodec_data;
	int n = 0;
	/* reset hardware */
	ResetAsic(dev);

	/* free the IRQ */
	for (n = 0; n < dev->cores; n++) {
		if (dev->irq[n] != -1)
			free_irq(dev->irq[n], (void *) dev);
	}

	ReleaseIO();
#ifdef ENABLE_HANTRO_CLK
	hantro_clk_disable();
#endif
	pr_info("hantrodec: module removed\n");
}

/*---------------------------------------------------------------------------
 *Function name   : CheckHwId
 *Return type     : int
 *---------------------------------------------------------------------------
 */
static int CheckHwId(struct hantrodec_t *dev)
{
	long hwid;
	int i;
	size_t num_hw = sizeof(DecHwId) / sizeof(*DecHwId);

	int found = 0;

	for (i = 0; i < dev->cores; i++) {
		if (dev->hwregs[i] != NULL) {
			hwid = readl(dev->hwregs[i]);
			PDEBUG("hantrodec: Core %d HW ID=0x%16lx\n", i, hwid);
			hwid = (hwid >> 16) & 0xFFFF; /* product version only */

			while (num_hw--) {
				if (hwid == DecHwId[num_hw]) {
					PDEBUG("hantrodec: Supported HW found at 0x%16lx\n",
					       multicorebase_actual[i]);
					found++;
					dev->hw_id[i] = hwid;
					break;
				}
			}
			if (!found) {
				PDEBUG("hantrodec: Unknown HW found at 0x%16lx\n",
				       multicorebase_actual[i]);
				return 0;
			}
			found = 0;
			num_hw = sizeof(DecHwId) / sizeof(*DecHwId);
		}
	}

	return 1;
}

/*---------------------------------------------------------------------------
 *Function name   : ReserveIO
 *Description     : IO reserve
 *
 *Return type     : int
 *---------------------------------------------------------------------------
 */
static int ReserveIO(void)
{
	int i;
	long hwid;
	u32 reg;

	memcpy(multicorebase_actual, multicorebase,
			HXDEC_MAX_CORES * sizeof(unsigned long));
	memcpy((unsigned int *)(hantrodec_data.iosize), iosize,
			HXDEC_MAX_CORES * sizeof(unsigned int));
	memcpy((unsigned int *)hantrodec_data.irq, irq,
			HXDEC_MAX_CORES * sizeof(int));

	for (i = 0; i < HXDEC_MAX_CORES; i++) {
		if (multicorebase_actual[i] != -1) {
			if (!request_mem_region(multicorebase_actual[i],
					hantrodec_data.iosize[i],
					"hantrodec0")) {
				PDEBUG("hantrodec: failed to reserve HW regs\n");
				return -EBUSY;
			}

			hantrodec_data.hwregs[i] =
				(u8 *) ioremap_nocache(
						multicorebase_actual[i],
						hantrodec_data.iosize[i]);

			if (hantrodec_data.hwregs[i] == NULL) {
				PDEBUG("hantrodec: failed to ioremap HW regs\n");
				ReleaseIO();
				return -EBUSY;
			}
			hantrodec_data.cores++;
			config.its_main_core_id[i] = -1;
			config.its_aux_core_id[i] = -1;

			/* product version only */
			hwid = ((readl(hantrodec_data.hwregs[i])) >> 16) & 0xFFFF;

			if (IS_VC8000D(hwid)) {
				reg = readl(hantrodec_data.hwregs[i] +
						HANTRODEC_SYNTH_CFG_2_OFF);
				if (((reg >> DWL_H264_PIPELINE_E) & 0x01U) ||
					((reg >> DWL_JPEG_PIPELINE_E) & 0x01U)) {
					i++;
					config.its_aux_core_id[i - 1] = i;
					config.its_main_core_id[i] = i - 1;
					config.its_aux_core_id[i] = -1;
					multicorebase_actual[i] = multicorebase_actual[i - 1] + 0x800;
					hantrodec_data.iosize[i] = hantrodec_data.iosize[i - 1];
					memcpy(multicorebase_actual + i + 1,
							multicorebase + i,
							(HXDEC_MAX_CORES - i - 1) * sizeof(unsigned long));
					memcpy((unsigned int *)(hantrodec_data.iosize + i + 1),
							iosize + i,
							(HXDEC_MAX_CORES - i - 1) * sizeof(unsigned int));
					if (!request_mem_region(
							multicorebase_actual[i],
							hantrodec_data.iosize[i],
							"hantrodec0")) {
						PDEBUG("hantrodec: failed to reserve HW regs\n");
						return -EBUSY;
					}

					hantrodec_data.hwregs[i] = (u8 *) ioremap_nocache(multicorebase_actual[i],
						hantrodec_data.iosize[i]);

					if (hantrodec_data.hwregs[i] == NULL) {
						PDEBUG("hantrodec: failed to ioremap HW regs\n");
						ReleaseIO();
						return -EBUSY;
					}
					hantrodec_data.cores++;
				}
			}
		}
	}

	/* check for correct HW */
	if (!CheckHwId(&hantrodec_data)) {
		ReleaseIO();
		return -EBUSY;
	}

	return 0;
}

/*---------------------------------------------------------------------------
 *Function name   : releaseIO
 *Description     : release
 *
 *Return type     : void
 *---------------------------------------------------------------------------
 */
static void ReleaseIO(void)
{
	int i;

	for (i = 0; i < hantrodec_data.cores; i++) {
		if (hantrodec_data.hwregs[i])
			iounmap((void *) hantrodec_data.hwregs[i]);
		release_mem_region(multicorebase_actual[i],
				hantrodec_data.iosize[i]);
	}
	iounmap((void *) page_lut_regs_read);
	release_mem_region(page_lut_read, hantrodec_data.iosize[0]);
}

/*---------------------------------------------------------------------------
 *Function name   : hantrodec_isr
 *Description     : interrupt handler
 *
 *Return type     : irqreturn_t
 *---------------------------------------------------------------------------
 */
static irqreturn_t hantrodec_isr(int irq, void *dev_id)
{
	unsigned long flags;
	unsigned int handled = 0;
	int i;

	u8 *hwregs;
	struct hantrodec_t *dev;
	u32 irq_status_dec;

	dev = (struct hantrodec_t *) dev_id;
	spin_lock_irqsave(&owner_lock, flags);

	for (i = 0; i < dev->cores; i++) {
		u8 *hwregs = dev->hwregs[i];

		/* interrupt status register read */
		irq_status_dec = ioread32((void *)hwregs +
			HANTRODEC_IRQ_STAT_DEC_OFF);
		PDEBUG("%d core irq = %x\n", i, irq_status_dec);
		if (irq_status_dec & HANTRODEC_DEC_IRQ) {
			/* clear dec IRQ */
			irq_status_dec &= (~HANTRODEC_DEC_IRQ);
			iowrite32(irq_status_dec, (void *)hwregs +
					HANTRODEC_IRQ_STAT_DEC_OFF);

			PDEBUG("decoder IRQ received! Core %d\n", i);

			atomic_inc(&irq_rx);

			dec_irq |= (1 << i);

			wake_up_interruptible_all(&dec_wait_queue);
			handled++;
		}
	}

	spin_unlock_irqrestore(&owner_lock, flags);

	if (!handled)
		PDEBUG("IRQ received, but not hantrodec's!\n");

	(void)hwregs;
	return IRQ_RETVAL(handled);
}

/*---------------------------------------------------------------------------
 *Function name   : ResetAsic
 *Description     : reset asic
 *
 *Return type     :
 *---------------------------------------------------------------------------
 */
static void ResetAsic(struct hantrodec_t *dev)
{
	int i, j;
	u32 status;

	for (j = 0; j < dev->cores; j++) {
		status = ioread32((void *)dev->hwregs[j] +
				HANTRODEC_IRQ_STAT_DEC_OFF);

		if (status & HANTRODEC_DEC_E) {
			/* abort with IRQ disabled */
			status = HANTRODEC_DEC_ABORT |
					HANTRODEC_DEC_IRQ_DISABLE;
			iowrite32(status, (void *)dev->hwregs[j] +
					HANTRODEC_IRQ_STAT_DEC_OFF);
		}

		for (i = 4; i < dev->iosize[j]; i += 4)
			iowrite32(0, (void *)dev->hwregs[j] + i);
	}
}
