/*
 *	kmb_platform.h - ASoC CPU DAI driver for KMB
 *
 *  Copyright (C) 2018 Intel Corp
 *  Authors:	Sia Jee Heng <jee.heng.sia@intel.com>
 *  Authors:	Sit, Michael Wei Hong <michael.wei.hong.sit@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#ifndef KMB_PLATFORM_H_
#define KMB_PLATFORMP_H_

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/info.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/dmaengine_pcm.h>

#define I2S_MIN_RATE		8000
#define I2S_MAX_RATE		48000
#define I2S_MIN_PERIODS		10
#define I2S_MAX_PERIODS		50
#define I2S_FIFO_SIZE		0

#define NUMBER_OF_I2S_PORT      4
/* I2S support */
#define I2S_MONO_CHANNEL 1
#define I2S_STEREO_CHANNEL 2

#define I2S_MAX_BUFFER		96000
#define I2S_MIN_BUFFER		96000
#define I2S_MIN_PERIOD_BYTES	640
#define I2S_MAX_PERIOD_BYTES	48000

/* TDM 8 support */
#define I2S_TDM8_CHANNEL 8

#define I2S_MAX_BUFFER_TDM		768000 /*500ms@48,4bytes,8ch - BYT*/
#define I2S_MIN_PERIOD_BYTES_TDM	2560  /*10ms@8kHz,4bytes,8ch - BYT*/
#define I2S_MAX_PERIOD_BYTES_TDM	76800 /*Use DMA transfer limit or 50ms@48,4bytes,8ch - BYT*/

#define TRISTATE_BIT			0
#define FRAME_SYNC_RELATIVE_TIMING_BIT	1
#define DUMMY_START_ONE_PERIOD_OFFSET	2
#define DUMMY_START_ONE_PERIOD_MASK     0x3

#define IS_TRISTATE_ENABLED(x) (x & BIT(TRISTATE_BIT))
#define IS_NEXT_FRMS_ASSERTED_WITH_LSB_PREVIOUS_FRM(x) \
			((x & BIT(FRAME_SYNC_RELATIVE_TIMING_BIT)) \
					>> FRAME_SYNC_RELATIVE_TIMING_BIT)
#define IS_DUMMY_START_ONE_PERIOD_OFFSET(x) \
			((x >> DUMMY_START_ONE_PERIOD_OFFSET) \
					& DUMMY_START_ONE_PERIOD_MASK)

#define I2S_RX_FIFO_THRESHOLD 8
#define I2S_TX_FIFO_THRESHOLD 8

// Register values with reference to KMB databook v0.8 //

/* common register for all channel */
#define IER		0x000
#define IRER		0x004
#define ITER		0x008
#define CER		0x00C
#define CCR		0x010
#define RXFFR		0x014
#define TXFFR		0x018

/* Interrupt status register fields */
#define ISR_TXFO	BIT(5)
#define ISR_TXFE	BIT(4)
#define ISR_RXFO	BIT(1)
#define ISR_RXDA	BIT(0)

/* I2STxRxRegisters for all channels */
#define LRBR_LTHR(x)	(0x40 * x + 0x020)
#define RRBR_RTHR(x)	(0x40 * x + 0x024)
#define RER(x)		(0x40 * x + 0x028)
#define TER(x)		(0x40 * x + 0x02C)
#define RCR(x)		(0x40 * x + 0x030)
#define TCR(x)		(0x40 * x + 0x034)
#define ISR(x)		(0x40 * x + 0x038)
#define IMR(x)		(0x40 * x + 0x03C)
#define ROR(x)		(0x40 * x + 0x040)
#define TOR(x)		(0x40 * x + 0x044)
#define RFCR(x)		(0x40 * x + 0x048)
#define TFCR(x)		(0x40 * x + 0x04C)
#define RFF(x)		(0x40 * x + 0x050)
#define TFF(x)		(0x40 * x + 0x054)

/* I2SCOMPRegisters */
#define I2S_COMP_PARAM_2	0x01F0
#define I2S_COMP_PARAM_1	0x01F4
#define I2S_COMP_VERSION	0x01F8
#define I2S_COMP_TYPE		0x01FC

/*
 * Component parameter register fields - define the I2S block's
 * configuration.
 */
#define	COMP1_TX_WORDSIZE_3(r)	(((r) & GENMASK(27, 25)) >> 25)
#define	COMP1_TX_WORDSIZE_2(r)	(((r) & GENMASK(24, 22)) >> 22)
#define	COMP1_TX_WORDSIZE_1(r)	(((r) & GENMASK(21, 19)) >> 19)
#define	COMP1_TX_WORDSIZE_0(r)	(((r) & GENMASK(18, 16)) >> 16)
#define	COMP1_TX_CHANNELS(r)	(((r) & GENMASK(10, 9)) >> 9)
#define	COMP1_RX_CHANNELS(r)	(((r) & GENMASK(8, 7)) >> 7)
#define	COMP1_RX_ENABLED(r)	(((r) & BIT(6)) >> 6)
#define	COMP1_TX_ENABLED(r)	(((r) & BIT(5)) >> 5)
#define	COMP1_MODE_EN(r)	(((r) & BIT(4)) >> 4)
#define	COMP1_FIFO_DEPTH_GLOBAL(r)	(((r) & GENMASK(3, 2)) >> 2)
#define	COMP1_APB_DATA_WIDTH(r)	(((r) & GENMASK(1, 0)) >> 0)

#define	COMP2_RX_WORDSIZE_3(r)	(((r) & GENMASK(12, 10)) >> 10)
#define	COMP2_RX_WORDSIZE_2(r)	(((r) & GENMASK(9, 7)) >> 7)
#define	COMP2_RX_WORDSIZE_1(r)	(((r) & GENMASK(5, 3)) >> 3)
#define	COMP2_RX_WORDSIZE_0(r)	(((r) & GENMASK(2, 0)) >> 0)

/* Number of entries in WORDSIZE and DATA_WIDTH parameter registers */
#define	COMP_MAX_WORDSIZE	(1 << 3)
#define	COMP_MAX_DATA_WIDTH	(1 << 2)

#define MAX_CHANNEL_NUM		8
#define MIN_CHANNEL_NUM		2

#define DW_I2S_MASTER	(1 << 3)
#define TWO_CHANNEL_SUPPORT	2	/* up to 2.0 */
#define FOUR_CHANNEL_SUPPORT	4	/* up to 3.1 */
#define SIX_CHANNEL_SUPPORT	6	/* up to 5.1 */
#define EIGHT_CHANNEL_SUPPORT	8	/* up to 7.1 */

	#define DWC_I2S_PLAY	(1 << 0)
	#define DWC_I2S_RECORD	(1 << 1)
	#define DW_I2S_SLAVE	(1 << 2)
	#define DW_I2S_MASTER	(1 << 3)

#define I2S_RXDMA		0x01C0
#define I2S_TXDMA		0x01C8

	#define DW_I2S_QUIRK_COMP_REG_OFFSET	(1 << 0)
	#define DW_I2S_QUIRK_COMP_PARAM1	(1 << 1)
	#define DW_I2S_QUIRK_16BIT_IDX_OVERRIDE (1 << 2)

/*
 * Structures Definition
 */

/*
 * struct i2s_clk_config_data - represent i2s clk configuration data
 * @chan_nr: number of channel
 * @data_width: number of bits per sample (8/16/24/32 bit)
 * @sample_rate: sampling frequency (8Khz, 16Khz, 32Khz, 44Khz, 48Khz)
 */
struct i2s_clk_config_data {
	int chan_nr;
	u32 data_width;
	u32 sample_rate;
};

union kmb_i2s_snd_dma_data {
	struct snd_dmaengine_dai_dma_data dt;
};

struct  kmb_i2s_config {
	bool kmb_mid_dma_alloc;
};

struct  kmb_i2s_info {
	void __iomem *i2s_base;
	struct workqueue_struct *i2s_dai_wq;
	struct clk *clk_i2s;
	int active;
	bool i2s_dai_tx_allocated;
	bool i2s_dai_rx_allocated;
	unsigned int capability;
	unsigned int quirks;
	unsigned int i2s_reg_comp1;
	unsigned int i2s_reg_comp2;
	struct device *dev;
	u32 ccr;
	u32 xfer_resolution;
	u32 fifo_th;

	/* data related to DMA transfers b/w i2s and DMAC */
	union kmb_i2s_snd_dma_data play_dma_data;
	union kmb_i2s_snd_dma_data capture_dma_data;

	struct i2s_clk_config_data config;
	int (*i2s_clk_cfg)(struct i2s_clk_config_data *config);

	/* data related to PIO transfers */
	bool use_pio;
	struct snd_pcm_substream __rcu *tx_substream;
	struct snd_pcm_substream __rcu *rx_substream;
	unsigned int (*tx_fn)(struct kmb_i2s_info *dev,
			struct snd_pcm_runtime *runtime, unsigned int tx_ptr,
			bool *period_elapsed);
	unsigned int (*rx_fn)(struct kmb_i2s_info *dev,
			struct snd_pcm_runtime *runtime, unsigned int rx_ptr,
			bool *period_elapsed);
	unsigned int tx_ptr;
	unsigned int rx_ptr;
};

struct  kmb_alsa_i2s_stream_info {
	struct snd_pcm_substream *substream;
	struct work_struct i2s_ws;
	struct kmb_i2s_config *i2s_config;
	unsigned long stream_status;
	u32 period_req_index;
	s32 period_cb_index;
	u8 *addr;
	int length;
};


/*
 * Enum Definition
 */

enum kmb_alsa_i2s_stream_status {
	KMB_ALSA_i2s_STREAM_INIT = 0,
	KMB_ALSA_i2s_STREAM_STARTED,
	KMB_ALSA_i2s_STREAM_RUNNING,
	KMB_ALSA_i2s_STREAM_PAUSED,
	KMB_ALSA_i2s_STREAM_DROPPED,
};


#endif /* KMB_PLATFORM_H_ */
