// SPDX-License-Identifier: GPL-2.0+
/*
 * ASPEED Video Engine Driver
 *
 * Copyright 2018 IBM Corp.
 *
 * Eddie James <eajames@linux.vnet.ibm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/eventpoll.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/videodev2.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#include "aspeed-video-jpeg.h"

#define DEVICE_NAME			"aspeed-video"

#define MAX_NUM_FRAMES			32
#define NUM_RES_DETECT_ATTEMPTS		5
#define NUM_POLARITY_CHECKS		10
#define NUM_SPURIOUS_COMP_READY		64
#define RESOLUTION_CHANGE_DELAY		msecs_to_jiffies(500)
#define MODE_DETECT_TIMEOUT		msecs_to_jiffies(500)
#define DIRECT_FETCH_THRESHOLD		0xc0000
#define PACKET_SIZE			0x20000

#define VE_SRC_BUFFER_SIZE		0x900000 /* 1920 * 1200, 32bpp */
#define VE_COMP_BUFFER_SIZE		0x100000 /* 128K packet * 8 packets */
#define VE_JPEG_BUFFER_SIZE		0x006000 /* 512 * 12 * 4 */

#define VE_PROTECTION_KEY		0x000
#define  VE_PROTECTION_KEY_UNLOCK	0x1A038AA8

#define VE_SEQ_CTRL			0x004
#define  VE_SEQ_CTRL_TRIG_MODE_DET	BIT(0)
#define  VE_SEQ_CTRL_TRIG_CAPTURE	BIT(1)
#define  VE_SEQ_CTRL_FORCE_IDLE		BIT(2)
#define  VE_SEQ_CTRL_MULT_FRAME		BIT(3)
#define  VE_SEQ_CTRL_TRIG_COMP		BIT(4)
#define  VE_SEQ_CTRL_AUTO_COMP		BIT(5)
#define  VE_SEQ_CTRL_EN_WATCHDOG	BIT(7)
#define  VE_SEQ_CTRL_YUV420		BIT(10)
#define  VE_SEQ_CTRL_COMP_FMT		GENMASK(11, 10)
#define  VE_SEQ_CTRL_HALT		BIT(12)
#define  VE_SEQ_CTRL_EN_WATCHDOG_COMP	BIT(14)
#define  VE_SEQ_CTRL_TRIG_JPG		BIT(15)
#define  VE_SEQ_CTRL_CAP_BUSY		BIT(16)
#define  VE_SEQ_CTRL_COMP_BUSY		BIT(18)

#ifdef CONFIG_MACH_ASPEED_G5
#define  VE_SEQ_CTRL_JPEG_MODE		BIT(13)	/* AST2500 */
#else
#define  VE_SEQ_CTRL_JPEG_MODE		BIT(8)	/* AST2400 */
#endif /* CONFIG_MACH_ASPEED_G5 */

#define VE_CTRL				0x008
#define  VE_CTRL_HSYNC_POL		BIT(0)
#define  VE_CTRL_VSYNC_POL		BIT(1)
#define  VE_CTRL_SOURCE			BIT(2)
#define  VE_CTRL_INT_DE			BIT(4)
#define  VE_CTRL_DIRECT_FETCH		BIT(5)
#define  VE_CTRL_YUV			BIT(6)
#define  VE_CTRL_RGB			BIT(7)
#define  VE_CTRL_CAPTURE_FMT		GENMASK(7, 6)
#define  VE_CTRL_AUTO_OR_CURSOR		BIT(8)
#define  VE_CTRL_CLK_INVERSE		BIT(11)
#define  VE_CTRL_CLK_DELAY		GENMASK(11, 9)
#define  VE_CTRL_INTERLACE		BIT(14)
#define  VE_CTRL_HSYNC_POL_CTRL		BIT(15)
#define  VE_CTRL_FRC			GENMASK(23, 16)

#define VE_TGS_0			0x00c
#define VE_TGS_1			0x010
#define  VE_TGS_FIRST			GENMASK(28, 16)
#define  VE_TGS_LAST			GENMASK(12, 0)

#define VE_SCALING_FACTOR		0x014
#define VE_SCALING_FILTER0		0x018
#define VE_SCALING_FILTER1		0x01c
#define VE_SCALING_FILTER2		0x020
#define VE_SCALING_FILTER3		0x024

#define VE_BCD_CTRL			0x02c
#define  VE_BCD_CTRL_ENABLE		BIT(0)
#define  VE_BCD_CTRL_ABCD		BIT(1)
#define  VE_BCD_CTRL_EN_COPY		BIT(2)
#define  VE_BCD_CTRL_HQ_DELAY		GENMASK(4, 3)
#define  VE_BCD_CTRL_BQ_DELAY		GENMASK(7, 5)

#define VE_CAP_WINDOW			0x030
#define VE_COMP_WINDOW			0x034
#define VE_COMP_PROC_OFFSET		0x038
#define VE_COMP_OFFSET			0x03c
#define VE_JPEG_ADDR			0x040
#define VE_SRC0_ADDR			0x044
#define VE_SRC_SCANLINE_OFFSET		0x048
#define VE_SRC1_ADDR			0x04c
#define VE_BCD_ADDR			0x050
#define VE_COMP_ADDR			0x054

#define VE_STREAM_BUF_SIZE		0x058
#define  VE_STREAM_BUF_SIZE_N_PACKETS	GENMASK(5, 3)
#define  VE_STREAM_BUF_SIZE_P_SIZE	GENMASK(2, 0)

#define VE_STREAM_WRITE_OFFSET		0x05c

#define VE_COMP_CTRL			0x060
#define  VE_COMP_CTRL_VQ_DCT_ONLY	BIT(0)
#define  VE_COMP_CTRL_VQ_4COLOR		BIT(1)
#define  VE_COMP_CTRL_QUANTIZE		BIT(2)
#define  VE_COMP_CTRL_EN_BQ		BIT(4)
#define  VE_COMP_CTRL_EN_CRYPTO		BIT(5)
#define  VE_COMP_CTRL_DCT_CHR		GENMASK(10, 6)
#define  VE_COMP_CTRL_DCT_LUM		GENMASK(15, 11)
#define  VE_COMP_CTRL_EN_HQ		BIT(16)
#define  VE_COMP_CTRL_RSVD		BIT(19)
#define  VE_COMP_CTRL_ENCODE		GENMASK(21, 20)
#define  VE_COMP_CTRL_HQ_DCT_CHR	GENMASK(26, 22)
#define  VE_COMP_CTRL_HQ_DCT_LUM	GENMASK(31, 27)

#define VE_BCD2_ADDR			0x06c
#define VE_SIZE_COMP_STREAM		0x070
#define VE_NUM_COMP_BLOCKS		0x074
#define VE_OFFSET_COMP_STREAM		0x078
#define VE_COMP_FRAME_COUNTER		0x07c

#define VE_SRC_LR_EDGE_DET		0x090
#define  VE_SRC_LR_EDGE_DET_LEFT	GENMASK(11, 0)
#define  VE_SRC_LR_EDGE_DET_NO_V	BIT(12)
#define  VE_SRC_LR_EDGE_DET_NO_H	BIT(13)
#define  VE_SRC_LR_EDGE_DET_NO_DISP	BIT(14)
#define  VE_SRC_LR_EDGE_DET_NO_CLK	BIT(15)
#define  VE_SRC_LR_EDGE_DET_RT_SHF	16
#define  VE_SRC_LR_EDGE_DET_RT		GENMASK(27, VE_SRC_LR_EDGE_DET_RT_SHF)
#define  VE_SRC_LR_EDGE_DET_INTERLACE	BIT(31)

#define VE_SRC_TB_EDGE_DET		0x094
#define  VE_SRC_TB_EDGE_DET_TOP		GENMASK(12, 0)
#define  VE_SRC_TB_EDGE_DET_BOT_SHF	16
#define  VE_SRC_TB_EDGE_DET_BOT		GENMASK(28, VE_SRC_TB_EDGE_DET_BOT_SHF)

#define VE_MODE_DETECT_STATUS		0x098
#define  VE_MODE_DETECT_STATUS_VSYNC	BIT(28)
#define  VE_MODE_DETECT_STATUS_HSYNC	BIT(29)

#define VE_MGMT_CTRL			0x300
#define  VE_MGMT_CTRL_RC4_RESET		BIT(8)
#define  VE_MGMT_CTRL_COMP_24BPP	BIT(10)
#define  VE_MGMT_CTRL_COMP_16BPP	GENMASK(11, 10)
#define  VE_MGMT_CTRL_COMP_MODE		GENMASK(11, 10)
#define  VE_MGMT_CTRL_LIN_RGB		BIT(29)
#define  VE_MGMT_CTRL_CAPTURE_FMT	GENMASK(29, 28)

#define VE_INTERRUPT_CTRL		0x304
#define VE_INTERRUPT_STATUS		0x308
#define  VE_INTERRUPT_MODE_DETECT_WD	BIT(0)
#define  VE_INTERRUPT_CAPTURE_COMPLETE	BIT(1)
#define  VE_INTERRUPT_COMP_READY	BIT(2)
#define  VE_INTERRUPT_COMP_COMPLETE	BIT(3)
#define  VE_INTERRUPT_MODE_DETECT	BIT(4)
#define  VE_INTERRUPT_FRAME_COMPLETE	BIT(5)
#define  VE_INTERRUPT_DECODE_ERR	BIT(6)
#define  VE_INTERRUPT_HALT_READY	BIT(8)
#define  VE_INTERRUPT_HANG_WD		BIT(9)
#define  VE_INTERRUPT_STREAM_DESC	BIT(10)
#define  VE_INTERRUPT_VSYNC_DESC	BIT(11)

#define VE_MODE_DETECT			0x30c
#define VE_MEM_RESTRICT_START		0x310
#define VE_MEM_RESTRICT_END		0x314

#define SCU_VGA1			0x40
#define  SCU_VGA1_FW_INIT_DRAM		BIT(6)
#define  SCU_VGA1_INIT_DRAM		BIT(7)

struct aspeed_video_frame {
	u32 offset;
	u32 size;
	struct timeval timestamp;
};

enum {
	VIDEO_MODE_DETECT_DONE,
	VIDEO_STREAMING,
	VIDEO_RES_CHANGE,
};

struct aspeed_video_addr {
	dma_addr_t dma;
	void *virt;
};

struct aspeed_video {
	void __iomem *base;
	struct clk *eclk;
	struct clk *vclk;
	struct reset_control *rst;
	struct regmap *scu;

	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct mutex video_lock;

	atomic_t clients;
	wait_queue_head_t wait;
	struct delayed_work res_work;
	unsigned long flags;

	u32 previous_comp_offset;
	u32 previous_comp_proc_offset;
	unsigned int frame_counter;
	unsigned int frame_done_idx;
	unsigned int frame_idx;
	unsigned int frame_read_count;
	struct aspeed_video_frame frames[MAX_NUM_FRAMES];
	spinlock_t frame_lock;

	unsigned int comp_ready_count;

	dma_addr_t max;
	dma_addr_t min;
	struct aspeed_video_addr srcs[2];
	struct aspeed_video_addr comp;
	struct aspeed_video_addr jpeg;

	int frame_rate;
	int jpeg_quality;
	struct v4l2_pix_format fmt;
};

#define to_aspeed_video(x) container_of((x), struct aspeed_video, v4l2_dev)

static void aspeed_video_init_jpeg_table(u32 *table, bool yuv420)
{
	int i;
	unsigned int base;

	for (i = 0; i < ASPEED_VIDEO_JPEG_NUM_QUALITIES; i++) {
		int j;

		base = 256 * i;
		for (j = 0; j < ASPEED_VIDEO_JPEG_HEADER_SIZE; j++)
			table[base + j] =
				le32_to_cpu(aspeed_video_jpeg_header[j]);

		base += ASPEED_VIDEO_JPEG_HEADER_SIZE;
		for (j = 0; j < ASPEED_VIDEO_JPEG_DCT_SIZE; j++)
			table[base + j] =
				le32_to_cpu(aspeed_video_jpeg_dct[i][j]);

		base += ASPEED_VIDEO_JPEG_DCT_SIZE;
		for (j = 0; j < ASPEED_VIDEO_JPEG_QUANT_SIZE; j++)
			table[base + j] =
				le32_to_cpu(aspeed_video_jpeg_quant[j]);

		if (yuv420)
			table[base + 2] = le32_to_cpu(0x00220103);
	}
}

static void aspeed_video_update(struct aspeed_video *video, u32 reg,
				unsigned long mask, u32 bits)
{
	u32 t = readl(video->base + reg);
	u32 before = t;

	t &= mask;
	t |= bits;
	writel(t, video->base + reg);
	dev_dbg(video->dev, "update %03x[%08x -> %08x]\n", reg, before,
		readl(video->base + reg));
}

static u32 aspeed_video_read(struct aspeed_video *video, u32 reg)
{
	u32 t = readl(video->base + reg);

	dev_dbg(video->dev, "read %03x[%08x]\n", reg, t);
	return t;
}

static void aspeed_video_write(struct aspeed_video *video, u32 reg, u32 val)
{
	writel(val, video->base + reg);
	dev_dbg(video->dev, "write %03x[%08x]\n", reg,
		readl(video->base + reg));
}

static bool aspeed_video_engine_busy(struct aspeed_video *video)
{
	u32 seq_ctrl = aspeed_video_read(video, VE_SEQ_CTRL);

	if (!(seq_ctrl & VE_SEQ_CTRL_COMP_BUSY) ||
	    !(seq_ctrl & VE_SEQ_CTRL_CAP_BUSY)) {
		dev_info(video->dev, "video engine busy\n");
		return true;
	}

	return false;
}

static int aspeed_video_start_frame(struct aspeed_video *video)
{
	if (aspeed_video_engine_busy(video))
		return -EBUSY;

	aspeed_video_write(video, VE_COMP_OFFSET, 0);
	aspeed_video_write(video, VE_COMP_PROC_OFFSET, 0);

	aspeed_video_update(video, VE_INTERRUPT_CTRL, 0xFFFFFFFF,
			    VE_INTERRUPT_COMP_READY |
			    VE_INTERRUPT_FRAME_COMPLETE |
			    VE_INTERRUPT_HANG_WD);

	aspeed_video_update(video, VE_SEQ_CTRL, ~VE_SEQ_CTRL_FORCE_IDLE,
			    VE_SEQ_CTRL_TRIG_CAPTURE | VE_SEQ_CTRL_TRIG_COMP);

	return 0;
}

static void aspeed_video_start_mode_detect(struct aspeed_video *video)
{
	/* Enable mode detect interrupts */
	aspeed_video_update(video, VE_INTERRUPT_CTRL, 0xFFFFFFFF,
			    VE_INTERRUPT_MODE_DETECT);

	/* Trigger mode detect */
	aspeed_video_update(video, VE_SEQ_CTRL, 0xFFFFFFFF,
			    VE_SEQ_CTRL_TRIG_MODE_DET);
}

static void aspeed_video_disable_mode_detect(struct aspeed_video *video)
{
	/* Disable mode detect interrupts */
	aspeed_video_update(video, VE_INTERRUPT_CTRL,
			    ~VE_INTERRUPT_MODE_DETECT, 0);

	/* Disable mode detect */
	aspeed_video_update(video, VE_SEQ_CTRL, ~VE_SEQ_CTRL_TRIG_MODE_DET, 0);
}

static void aspeed_video_off(struct aspeed_video *video)
{
	/* Reset the engine */
	reset_control_assert(video->rst);
	udelay(100);
	reset_control_deassert(video->rst);

	/* Turn off the relevant clocks */
	clk_disable_unprepare(video->vclk);
	clk_disable_unprepare(video->eclk);
}

static void aspeed_video_on(struct aspeed_video *video)
{
	/* Turn on the relevant clocks */
	clk_prepare_enable(video->eclk);
	clk_prepare_enable(video->vclk);

	/* Reset the engine */
	reset_control_assert(video->rst);
	udelay(100);
	reset_control_deassert(video->rst);
}

static irqreturn_t aspeed_video_irq(int irq, void *arg)
{
	struct aspeed_video *video = arg;
	u32 sts = aspeed_video_read(video, VE_INTERRUPT_STATUS);

	if (atomic_read(&video->clients) == 0) {
		dev_info(video->dev, "irq with no client; disabling irqs\n");
		aspeed_video_write(video, VE_INTERRUPT_CTRL, 0);
		aspeed_video_write(video, VE_INTERRUPT_STATUS, 0xFFFFFFFF);
		return IRQ_HANDLED;
	}

	/* Resolution changed; reset entire engine and reinitialize */
	if (sts & VE_INTERRUPT_MODE_DETECT_WD) {
		dev_info(video->dev, "resolution changed; resetting\n");
		set_bit(VIDEO_RES_CHANGE, &video->flags);
		video->frame_counter = 0;	/* prevent further reads */

		aspeed_video_off(video);

		schedule_delayed_work(&video->res_work,
				      RESOLUTION_CHANGE_DELAY);
		return IRQ_HANDLED;
	}

	if (sts & VE_INTERRUPT_MODE_DETECT) {
		set_bit(VIDEO_MODE_DETECT_DONE, &video->flags);
		aspeed_video_update(video, VE_INTERRUPT_CTRL,
				    ~VE_INTERRUPT_MODE_DETECT, 0);
		wake_up_interruptible_all(&video->wait);
	}

	if (sts & VE_INTERRUPT_FRAME_COMPLETE) {
		unsigned long flags;
		unsigned int pidx = (video->frame_idx ?: MAX_NUM_FRAMES) - 1;
		u32 offs = aspeed_video_read(video, VE_OFFSET_COMP_STREAM);

		video->comp_ready_count = 0;

		spin_lock_irqsave(&video->frame_lock, flags);
		if (offs < video->frames[pidx].offset)
			video->frames[pidx].size = (VE_COMP_BUFFER_SIZE -
				video->frames[pidx].offset) + offs;
		else
			video->frames[pidx].size = offs -
				video->frames[pidx].offset;
		v4l2_get_timestamp(&video->frames[pidx].timestamp);
		video->frames[video->frame_idx].offset = offs;

		video->frame_idx = (video->frame_idx + 1) % MAX_NUM_FRAMES;
		video->frame_done_idx = pidx;
		video->frame_counter++;
		spin_unlock_irqrestore(&video->frame_lock, flags);

		dev_dbg(video->dev, "frame[%d] offs[%08x] size[%08x]\n", pidx,
			video->frames[pidx].offset, video->frames[pidx].size);

		wake_up_interruptible_all(&video->wait);
	}

	if (sts & VE_INTERRUPT_COMP_READY) {
		unsigned int pidx = (video->frame_idx ?: MAX_NUM_FRAMES) - 1;
		u32 comp_offset = video->frames[video->frame_done_idx].offset;
		u32 comp_proc_offset = video->frames[pidx].offset;

		if (comp_proc_offset != video->previous_comp_proc_offset) {
			aspeed_video_write(video, VE_COMP_PROC_OFFSET,
					   comp_proc_offset);
			video->previous_comp_proc_offset = comp_proc_offset;
		} else if (video->previous_comp_proc_offset + PACKET_SIZE >
			   VE_COMP_BUFFER_SIZE) {
			aspeed_video_write(video, VE_COMP_PROC_OFFSET,
					   VE_COMP_BUFFER_SIZE);
			video->previous_comp_proc_offset = VE_COMP_BUFFER_SIZE;
		}

		if (comp_offset != video->previous_comp_offset) {
			aspeed_video_write(video, VE_COMP_OFFSET,
					   comp_offset);
			video->previous_comp_offset = comp_offset;
		}

		if (video->comp_ready_count++ > NUM_SPURIOUS_COMP_READY) {
			/*
			 * Disable and re-enable compression ready irq, which
			 * can kickstart compression engine again?
			 */
			aspeed_video_update(video, VE_INTERRUPT_CTRL,
					    ~VE_INTERRUPT_COMP_READY, 0);
			udelay(1);
			aspeed_video_update(video, VE_INTERRUPT_CTRL,
					    0xFFFFFFFF,
					    VE_INTERRUPT_COMP_READY);
			video->comp_ready_count = 0;
		}
	}

	if (sts & VE_INTERRUPT_HANG_WD) {
		u32 seq_ctrl = aspeed_video_read(video, VE_SEQ_CTRL);

		if ((seq_ctrl & VE_SEQ_CTRL_CAP_BUSY) &&
		    !(seq_ctrl & VE_SEQ_CTRL_COMP_BUSY)) {
			aspeed_video_update(video, VE_SEQ_CTRL, 0xFFFFFFFF,
					    VE_SEQ_CTRL_FORCE_IDLE);
			aspeed_video_update(video, VE_INTERRUPT_CTRL,
					    ~VE_INTERRUPT_HANG_WD, 0);
		} else {
			dev_info(video->dev, "irq hang wd but cap busy!\n");
		}
	}

	aspeed_video_write(video, VE_INTERRUPT_STATUS, sts);

	return IRQ_HANDLED;
}

static void aspeed_video_check_polarity(struct aspeed_video *video)
{
	int i;
	int hsync_counter = 0;
	int vsync_counter = 0;
	u32 sts;

	for (i = 0; i < NUM_POLARITY_CHECKS; ++i) {
		sts = aspeed_video_read(video, VE_MODE_DETECT_STATUS);
		if (sts & VE_MODE_DETECT_STATUS_VSYNC)
			vsync_counter--;
		else
			vsync_counter++;

		if (sts & VE_MODE_DETECT_STATUS_HSYNC)
			hsync_counter--;
		else
			hsync_counter++;
	}

	if (hsync_counter < 0 || vsync_counter < 0) {
		u32 ctrl;

		if (hsync_counter < 0)
			ctrl = VE_CTRL_HSYNC_POL;

		if (vsync_counter < 0)
			ctrl = VE_CTRL_VSYNC_POL;

		aspeed_video_update(video, VE_CTRL, 0xFFFFFFFF, ctrl);
	}
}

#define res_check(v) test_and_clear_bit(VIDEO_MODE_DETECT_DONE, &(v)->flags)

static int aspeed_video_get_resolution(struct aspeed_video *video)
{
	int rc;
	unsigned int bottom;
	unsigned int left;
	unsigned int right;
	unsigned int top;
	u32 src_lr_edge;
	u32 src_tb_edge;

	video->fmt.width = 0;
	video->fmt.height = 0;

	aspeed_video_start_mode_detect(video);

	rc = wait_event_interruptible_timeout(video->wait, res_check(video),
					      MODE_DETECT_TIMEOUT);
	if (!rc) {
		dev_err(video->dev, "timed out on 1st mode detect\n");
		aspeed_video_disable_mode_detect(video);
		return -ETIME;
	}

	/* Disable mode detect in order to re-trigger */
	aspeed_video_update(video, VE_SEQ_CTRL, ~VE_SEQ_CTRL_TRIG_MODE_DET, 0);

	aspeed_video_check_polarity(video);

	aspeed_video_start_mode_detect(video);

	rc = wait_event_interruptible_timeout(video->wait, res_check(video),
					      MODE_DETECT_TIMEOUT);
	if (!rc) {
		dev_err(video->dev, "timed out on 2nd mode detect\n");
		aspeed_video_disable_mode_detect(video);
		return -ETIME;
	}

	src_lr_edge = aspeed_video_read(video, VE_SRC_LR_EDGE_DET);
	src_tb_edge = aspeed_video_read(video, VE_SRC_TB_EDGE_DET);

	bottom = (src_tb_edge & VE_SRC_TB_EDGE_DET_BOT) >>
		VE_SRC_TB_EDGE_DET_BOT_SHF;
	top = src_tb_edge & VE_SRC_TB_EDGE_DET_TOP;
	if (top > bottom) {
		dev_err(video->dev, "invalid resolution detected\n");
		return -EMSGSIZE;
	}

	right = (src_lr_edge & VE_SRC_LR_EDGE_DET_RT) >>
		VE_SRC_LR_EDGE_DET_RT_SHF;
	left = src_lr_edge & VE_SRC_LR_EDGE_DET_LEFT;
	if (left > right) {
		dev_err(video->dev, "invalid resolution detected\n");
		return -EMSGSIZE;
	}

	video->fmt.height = (bottom - top) + 1;
	video->fmt.width = (right - left) + 1;

	/* Don't use direct mode below 1024 x 768 (irqs don't fire) */
	if (video->fmt.height * video->fmt.width < DIRECT_FETCH_THRESHOLD) {
		aspeed_video_write(video, VE_TGS_0,
				   FIELD_PREP(VE_TGS_FIRST, left - 1) |
				   FIELD_PREP(VE_TGS_LAST, right));
		aspeed_video_write(video, VE_TGS_1,
				   FIELD_PREP(VE_TGS_FIRST, top) |
				   FIELD_PREP(VE_TGS_LAST, bottom + 1));
		aspeed_video_update(video, VE_CTRL,
				    ~VE_CTRL_DIRECT_FETCH, VE_CTRL_INT_DE);
	}

	aspeed_video_write(video, VE_CAP_WINDOW,
			   video->fmt.width << 16 | video->fmt.height);
	aspeed_video_write(video, VE_COMP_WINDOW,
			   video->fmt.width << 16 | video->fmt.height);
	aspeed_video_write(video, VE_SRC_SCANLINE_OFFSET,
			   video->fmt.width * 4);

	aspeed_video_update(video, VE_INTERRUPT_CTRL, 0xFFFFFFFF,
			    VE_INTERRUPT_MODE_DETECT_WD);
	aspeed_video_update(video, VE_SEQ_CTRL, 0xFFFFFFFF,
			    VE_SEQ_CTRL_EN_WATCHDOG);

	dev_dbg(video->dev, "got resolution[%dx%d]\n", video->fmt.width,
		video->fmt.height);

	return 0;
}

static void aspeed_video_init_regs(struct aspeed_video *video)
{
	u32 comp_ctrl = VE_COMP_CTRL_RSVD |
		FIELD_PREP(VE_COMP_CTRL_DCT_LUM, video->jpeg_quality) |
		FIELD_PREP(VE_COMP_CTRL_DCT_CHR, video->jpeg_quality | 0x10);
	u32 ctrl = VE_CTRL_DIRECT_FETCH | VE_CTRL_AUTO_OR_CURSOR;
	u32 seq_ctrl = VE_SEQ_CTRL_AUTO_COMP | VE_SEQ_CTRL_MULT_FRAME |
		VE_SEQ_CTRL_JPEG_MODE;

	if (video->frame_rate)
		ctrl |= FIELD_PREP(VE_CTRL_FRC, video->frame_rate);

	if (video->fmt.pixelformat == V4L2_PIX_FMT_YUV420)
		seq_ctrl |= VE_SEQ_CTRL_YUV420;

	/* Unlock VE registers */
	aspeed_video_write(video, VE_PROTECTION_KEY, VE_PROTECTION_KEY_UNLOCK);

	/* Disable interrupts */
	aspeed_video_write(video, VE_INTERRUPT_CTRL, 0);
	aspeed_video_write(video, VE_INTERRUPT_STATUS, 0xFFFFFFFF);

	/* Clear the offset */
	aspeed_video_write(video, VE_COMP_PROC_OFFSET, 0);
	aspeed_video_write(video, VE_COMP_OFFSET, 0);

	/* Set memory restrictions */
	aspeed_video_write(video, VE_MEM_RESTRICT_START, video->min);
	aspeed_video_write(video, VE_MEM_RESTRICT_END, video->max);

	/* Set buffer addresses */
	aspeed_video_write(video, VE_SRC0_ADDR, video->srcs[0].dma);
	aspeed_video_write(video, VE_SRC1_ADDR, video->srcs[1].dma);
	aspeed_video_write(video, VE_COMP_ADDR, video->comp.dma);
	aspeed_video_write(video, VE_JPEG_ADDR, video->jpeg.dma);

	/* Set control registers */
	aspeed_video_write(video, VE_SEQ_CTRL, seq_ctrl);
	aspeed_video_write(video, VE_CTRL, ctrl);
	aspeed_video_write(video, VE_COMP_CTRL, comp_ctrl);

	/* Compression buffer size */
	aspeed_video_write(video, VE_STREAM_BUF_SIZE, 0xf);

	/* Don't downscale */
	aspeed_video_write(video, VE_SCALING_FACTOR, 0x10001000);
	aspeed_video_write(video, VE_SCALING_FILTER0, 0x00200000);
	aspeed_video_write(video, VE_SCALING_FILTER1, 0x00200000);
	aspeed_video_write(video, VE_SCALING_FILTER2, 0x00200000);
	aspeed_video_write(video, VE_SCALING_FILTER3, 0x00200000);

	/* Set mode detection defaults */
	aspeed_video_write(video, VE_MODE_DETECT, 0x22666500);
}

static int aspeed_video_allocate_cma(struct aspeed_video *video)
{
	video->srcs[0].virt = dma_alloc_coherent(video->dev,
						 VE_SRC_BUFFER_SIZE,
						 &video->srcs[0].dma,
						 GFP_KERNEL);
	if (!video->srcs[0].virt) {
		dev_err(video->dev,
			"Failed to allocate source buffer 0, size[%x]\n",
			VE_SRC_BUFFER_SIZE);
		goto err;
	}

	video->srcs[1].virt = dma_alloc_coherent(video->dev,
						 VE_SRC_BUFFER_SIZE,
						 &video->srcs[1].dma,
						 GFP_KERNEL);
	if (!video->srcs[1].virt) {
		dev_err(video->dev,
			"Failed to allocate source buffer 1, size[%x]\n",
			VE_SRC_BUFFER_SIZE);
		goto free_src0;
	}

	video->comp.virt = dma_alloc_coherent(video->dev, VE_COMP_BUFFER_SIZE,
					      &video->comp.dma, GFP_KERNEL);
	if (!video->comp.virt) {
		dev_err(video->dev,
			"Failed to allocate compression buffer, size[%x]\n",
			VE_COMP_BUFFER_SIZE);
		goto free_src1;
	}

	video->jpeg.virt = dma_alloc_coherent(video->dev, VE_JPEG_BUFFER_SIZE,
					      &video->jpeg.dma, GFP_KERNEL);
	if (!video->jpeg.virt) {
		dev_err(video->dev,
			"Failed to allocate JPEG buffer, size[%x]\n",
			VE_JPEG_BUFFER_SIZE);
		goto free_comp;
	}

	if (video->fmt.pixelformat == V4L2_PIX_FMT_YUV420)
		aspeed_video_init_jpeg_table(video->jpeg.virt, true);
	else
		aspeed_video_init_jpeg_table(video->jpeg.virt, false);

	/*
	 * Calculate the memory restrictions. Don't consider the JPEG header
	 * buffer since HW doesn't need to write to it.
	 */
	video->max = max(video->srcs[0].dma + VE_SRC_BUFFER_SIZE,
			 video->srcs[1].dma + VE_SRC_BUFFER_SIZE);
	video->max = max(video->max, video->comp.dma + VE_COMP_BUFFER_SIZE);

	video->min = min(video->srcs[0].dma, video->srcs[1].dma);
	video->min = min(video->min, video->comp.dma);

	return 0;

free_comp:
	dma_free_coherent(video->dev, VE_COMP_BUFFER_SIZE, video->comp.virt,
			  video->comp.dma);
free_src1:
	dma_free_coherent(video->dev, VE_SRC_BUFFER_SIZE, video->srcs[1].virt,
			  video->srcs[1].dma);
free_src0:
	dma_free_coherent(video->dev, VE_SRC_BUFFER_SIZE, video->srcs[0].virt,
			  video->srcs[0].dma);
err:
	return -ENOMEM;
}

static void aspeed_video_free_cma(struct aspeed_video *video)
{
	dma_free_coherent(video->dev, VE_JPEG_BUFFER_SIZE, video->jpeg.virt,
			  video->jpeg.dma);
	dma_free_coherent(video->dev, VE_COMP_BUFFER_SIZE, video->comp.virt,
			  video->comp.dma);
	dma_free_coherent(video->dev, VE_SRC_BUFFER_SIZE, video->srcs[1].virt,
			  video->srcs[1].dma);
	dma_free_coherent(video->dev, VE_SRC_BUFFER_SIZE, video->srcs[0].virt,
			  video->srcs[0].dma);

	video->srcs[0].dma = 0ULL;
	video->srcs[0].virt = NULL;
	video->srcs[1].dma = 0ULL;
	video->srcs[1].virt = NULL;
	video->comp.dma = 0ULL;
	video->comp.virt = NULL;
	video->jpeg.dma = 0ULL;
	video->jpeg.virt = NULL;
}

static int aspeed_video_start(struct aspeed_video *video)
{
	int rc = aspeed_video_allocate_cma(video);

	if (rc)
		return rc;

	aspeed_video_on(video);

	aspeed_video_init_regs(video);

	rc = aspeed_video_get_resolution(video);
	if (rc)
		aspeed_video_free_cma(video);

	return rc;
}

static void aspeed_video_stop(struct aspeed_video *video)
{
	cancel_delayed_work_sync(&video->res_work);

	aspeed_video_off(video);

	aspeed_video_free_cma(video);

	clear_bit(VIDEO_STREAMING, &video->flags);
}

static int aspeed_video_querycap(struct file *file, void *fh,
				 struct v4l2_capability *cap)
{
	struct aspeed_video *video = video_drvdata(file);

	strncpy(cap->driver, DEVICE_NAME, sizeof(cap->driver));
	cap->capabilities = video->vdev.device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int aspeed_video_get_format(struct file *file, void *fh,
				   struct v4l2_format *f)
{
	int rc;
	struct aspeed_video *video = video_drvdata(file);

	if (test_bit(VIDEO_RES_CHANGE, &video->flags)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		rc = wait_event_interruptible(video->wait,
					      !test_bit(VIDEO_RES_CHANGE,
							&video->flags));
		if (rc)
			return -EINTR;
	}

	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f->fmt.pix = video->fmt;

	return 0;
}

static int aspeed_video_set_format(struct file *file, void *fh,
				   struct v4l2_format *f)
{
	struct aspeed_video *video = video_drvdata(file);

	if (f->fmt.pix.pixelformat == video->fmt.pixelformat)
		return 0;

	if (f->fmt.pix.pixelformat == V4L2_PIX_FMT_YUV444) {
		video->fmt.pixelformat = V4L2_PIX_FMT_YUV444;
		aspeed_video_init_jpeg_table(video->jpeg.virt, false);
		aspeed_video_update(video, VE_SEQ_CTRL, ~VE_SEQ_CTRL_YUV420,
				    0);
	} else if (f->fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420) {
		video->fmt.pixelformat = V4L2_PIX_FMT_YUV420;
		aspeed_video_init_jpeg_table(video->jpeg.virt, true);
		aspeed_video_update(video, VE_SEQ_CTRL, 0xFFFFFFFF,
				    VE_SEQ_CTRL_YUV420);
	} else {
		return -EINVAL;
	}

	return 0;
}

static int aspeed_video_get_jpegcomp(struct file *file, void *fh,
				     struct v4l2_jpegcompression *a)
{
	struct aspeed_video *video = video_drvdata(file);

	a->quality = video->jpeg_quality;

	return 0;
}

static int aspeed_video_set_jpegcomp(struct file *file, void *fh,
				     const struct v4l2_jpegcompression *a)
{
	u32 comp_ctrl;
	struct aspeed_video *video = video_drvdata(file);

	if (a->quality < 0 || a->quality > 11)
		return -EINVAL;

	video->jpeg_quality = a->quality;
	comp_ctrl = FIELD_PREP(VE_COMP_CTRL_DCT_LUM, video->jpeg_quality) |
		FIELD_PREP(VE_COMP_CTRL_DCT_CHR, video->jpeg_quality | 0x10);

	aspeed_video_update(video, VE_COMP_CTRL,
			    ~(VE_COMP_CTRL_DCT_LUM | VE_COMP_CTRL_DCT_CHR),
			    comp_ctrl);

	return 0;
}

static int aspeed_video_get_parm(struct file *file, void *fh,
				 struct v4l2_streamparm *a)
{
	struct aspeed_video *video = video_drvdata(file);

	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->parm.capture.timeperframe.numerator = 1;
	a->parm.capture.timeperframe.denominator = video->frame_rate;

	return 0;
}

static int aspeed_video_set_parm(struct file *file, void *fh,
				 struct v4l2_streamparm *a)
{
	int frame_rate;
	struct aspeed_video *video = video_drvdata(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	frame_rate = a->parm.capture.timeperframe.denominator /
		a->parm.capture.timeperframe.numerator;

	if (frame_rate < 0 || frame_rate > 60)
		return -EINVAL;

	if (video->frame_rate != frame_rate) {
		video->frame_rate = frame_rate;
		aspeed_video_update(video, VE_CTRL, ~VE_CTRL_FRC,
				    FIELD_PREP(VE_CTRL_FRC, frame_rate));
	}

	return 0;
}

static const struct v4l2_ioctl_ops aspeed_video_ioctls = {
	.vidioc_querycap = aspeed_video_querycap,
	.vidioc_g_fmt_vid_cap = aspeed_video_get_format,
	.vidioc_s_fmt_vid_cap = aspeed_video_set_format,
	.vidioc_g_jpegcomp = aspeed_video_get_jpegcomp,
	.vidioc_s_jpegcomp = aspeed_video_set_jpegcomp,
	.vidioc_g_parm = aspeed_video_get_parm,
	.vidioc_s_parm = aspeed_video_set_parm,
};

static bool aspeed_video_frame_available(struct aspeed_video *video)
{
	return video->frame_counter > video->frame_read_count;
}

static void aspeed_video_reinit_counters(struct aspeed_video *video)
{
	video->previous_comp_offset = 0;
	video->previous_comp_proc_offset = 0;
	video->frame_counter = 0;
	video->frame_done_idx = 0;
	video->frame_idx = 1;
	video->frame_read_count = 0;
	memset(video->frames, 0,
	       sizeof(struct aspeed_video_frame) * MAX_NUM_FRAMES);
	video->comp_ready_count = 0;
}

static void aspeed_video_resolution_work(struct work_struct *work)
{
	int rc;
	struct delayed_work *dwork = to_delayed_work(work);
	struct aspeed_video *video = container_of(dwork, struct aspeed_video,
						  res_work);

	/* No clients remaining after delay */
	if (atomic_read(&video->clients) == 0)
		goto done;

	aspeed_video_on(video);

	aspeed_video_init_regs(video);

	rc = aspeed_video_get_resolution(video);
	if (rc) {
		dev_err(video->dev,
			"resolution changed; couldn't get new resolution\n");
	} else if (test_bit(VIDEO_STREAMING, &video->flags)) {
		aspeed_video_reinit_counters(video);
		aspeed_video_start_frame(video);
	}

done:
	clear_bit(VIDEO_RES_CHANGE, &video->flags);
	wake_up_interruptible_all(&video->wait);
}

static ssize_t aspeed_video_file_read(struct file *file, char __user *buf,
				      size_t count, loff_t *ppos)
{
	int rc;
	unsigned long flags;
	size_t size;
	struct aspeed_video *video = video_drvdata(file);
	struct aspeed_video_frame *latest;

	if (mutex_lock_interruptible(&video->video_lock))
		return -EINTR;

	if (!test_bit(VIDEO_STREAMING, &video->flags)) {
		aspeed_video_reinit_counters(video);
		rc = aspeed_video_start_frame(video);
		if (rc)
			goto unlock;

		set_bit(VIDEO_STREAMING, &video->flags);
	}

	if (file->f_flags & O_NONBLOCK) {
		if (!aspeed_video_frame_available(video)) {
			rc = -EAGAIN;
			goto unlock;
		} else {
			goto ready;
		}
	}

	rc = wait_event_interruptible(video->wait,
				      aspeed_video_frame_available(video));
	if (rc) {
		rc = -EINTR;
		goto unlock;
	}

ready:
	spin_lock_irqsave(&video->frame_lock, flags);
	latest = &video->frames[video->frame_done_idx];
	video->frame_read_count = video->frame_counter;
	spin_unlock_irqrestore(&video->frame_lock, flags);

	size = min_t(size_t, count, latest->size);
	if (latest->offset + size > VE_COMP_BUFFER_SIZE) {
		size_t remainder = VE_COMP_BUFFER_SIZE - latest->offset;

		if (copy_to_user(buf, video->comp.virt + latest->offset,
				 remainder)) {
			rc = -EFAULT;
			goto unlock;
		}
		if (copy_to_user(&buf[remainder], video->comp.virt,
				 size - remainder)) {
			rc = -EFAULT;
			goto unlock;
		}
	} else {
		if (copy_to_user(buf, video->comp.virt + latest->offset,
				 size)) {
			rc = -EFAULT;
			goto unlock;
		}
	}

	rc = size;

unlock:
	mutex_unlock(&video->video_lock);
	return rc;
}

static int aspeed_video_open(struct file *file)
{
	int rc;
	struct aspeed_video *video = video_drvdata(file);

	if (atomic_inc_return(&video->clients) == 1) {
		rc = aspeed_video_start(video);
		if (rc) {
			dev_err(video->dev, "Failed to start video engine\n");
			atomic_dec(&video->clients);
			return rc;
		}
	}

	return v4l2_fh_open(file);
}

static int aspeed_video_release(struct file *file)
{
	int rc;
	struct aspeed_video *video = video_drvdata(file);

	rc = v4l2_fh_release(file);

	if (atomic_dec_return(&video->clients) == 0)
		aspeed_video_stop(video);

	return rc;
}

static const struct v4l2_file_operations aspeed_video_v4l2_fops = {
	.owner = THIS_MODULE,
	.read = aspeed_video_file_read,
	.open = aspeed_video_open,
	.release = aspeed_video_release,
	.unlocked_ioctl = video_ioctl2,
};

static void aspeed_video_device_release(struct video_device *vdev)
{
}

static int aspeed_video_setup_video(struct aspeed_video *video)
{
	int rc;
	struct v4l2_device *v4l2_dev = &video->v4l2_dev;
	struct video_device *vdev = &video->vdev;

	rc = v4l2_device_register(video->dev, v4l2_dev);
	if (rc) {
		dev_err(video->dev, "Failed to register v4l2 device\n");
		return rc;
	}

	vdev->fops = &aspeed_video_v4l2_fops;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE;
	vdev->v4l2_dev = v4l2_dev;
	strncpy(vdev->name, DEVICE_NAME, sizeof(vdev->name));
	vdev->vfl_type = VFL_TYPE_GRABBER;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->release = aspeed_video_device_release;
	vdev->ioctl_ops = &aspeed_video_ioctls;
	vdev->lock = &video->video_lock;

	video_set_drvdata(vdev, video);
	rc = video_register_device(vdev, VFL_TYPE_GRABBER, 0);
	if (rc) {
		v4l2_device_unregister(v4l2_dev);
		dev_err(video->dev, "Failed to register video device\n");
		return rc;
	}

	/* set pixel format defaults */
	video->fmt.pixelformat = V4L2_PIX_FMT_YUV444;
	video->fmt.field = V4L2_FIELD_NONE;

	return 0;
}

static void aspeed_video_scu_init(struct aspeed_video *video)
{
	unsigned int scu_vga1;

	regmap_read(video->scu, SCU_VGA1, &scu_vga1);
	dev_info(video->dev, "scu@0x40[%08x]\n", scu_vga1);
	if (scu_vga1 & SCU_VGA1_INIT_DRAM) {
		dev_info(video->dev, "set fw init dram\n");
		regmap_update_bits(video->scu, SCU_VGA1, SCU_VGA1_FW_INIT_DRAM,
				   SCU_VGA1_FW_INIT_DRAM);
	}
}

static int aspeed_video_init(struct aspeed_video *video)
{
	int irq;
	int rc;
	struct device *dev = video->dev;

	irq = irq_of_parse_and_map(dev->of_node, 0);
	if (!irq) {
		dev_err(dev, "Unable to find IRQ\n");
		return -ENODEV;
	}

	rc = devm_request_irq(dev, irq, aspeed_video_irq, IRQF_SHARED,
			      DEVICE_NAME, video);
	if (rc < 0) {
		dev_err(dev, "Unable to request IRQ %d\n", irq);
		return rc;
	}

	video->scu = syscon_regmap_lookup_by_phandle(dev->of_node, "reg-scu");
	if (IS_ERR(video->scu)) {
		dev_err(dev, "Unable to get SCU regs\n");
		return PTR_ERR(video->scu);
	}

	video->eclk = devm_clk_get(dev, "eclk-gate");
	if (IS_ERR(video->eclk)) {
		dev_err(dev, "Unable to get ECLK\n");
		return PTR_ERR(video->eclk);
	}

	video->vclk = devm_clk_get(dev, "vclk-gate");
	if (IS_ERR(video->vclk)) {
		dev_err(dev, "Unable to get VCLK\n");
		return PTR_ERR(video->vclk);
	}

	video->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(video->rst)) {
		dev_err(dev, "Unable to get VE reset\n");
		return PTR_ERR(video->rst);
	}

	rc = of_reserved_mem_device_init(dev);
	if (rc) {
		dev_err(dev, "Unable to reserve memory\n");
		return rc;
	}

	rc = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (rc) {
		dev_err(dev, "Failed to set DMA mask\n");
		of_reserved_mem_device_release(dev);
		return rc;
	}

	aspeed_video_scu_init(video);

	return 0;
}

static int aspeed_video_probe(struct platform_device *pdev)
{
	int rc;
	struct resource *res;
	struct aspeed_video *video = kzalloc(sizeof(*video), GFP_KERNEL);

	if (!video)
		return -ENOMEM;

	video->frame_rate = 30;
	video->dev = &pdev->dev;
	mutex_init(&video->video_lock);
	init_waitqueue_head(&video->wait);
	INIT_DELAYED_WORK(&video->res_work, aspeed_video_resolution_work);
	spin_lock_init(&video->frame_lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	video->base = devm_ioremap_resource(video->dev, res);

	if (IS_ERR(video->base))
		return PTR_ERR(video->base);

	rc = aspeed_video_init(video);
	if (rc)
		return rc;

	rc = aspeed_video_setup_video(video);
	if (rc)
		return rc;

	return 0;
}

static int aspeed_video_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_device *v4l2_dev = dev_get_drvdata(dev);
	struct aspeed_video *video = to_aspeed_video(v4l2_dev);

	video_unregister_device(&video->vdev);

	v4l2_device_unregister(v4l2_dev);

	of_reserved_mem_device_release(dev);

	return 0;
}

static const struct of_device_id aspeed_video_of_match[] = {
	{ .compatible = "aspeed,ast2400-video" },
	{ .compatible = "aspeed,ast2500-video" },
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_video_of_match);

static struct platform_driver aspeed_video_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = aspeed_video_of_match,
	},
	.probe = aspeed_video_probe,
	.remove = aspeed_video_remove,
};

module_platform_driver(aspeed_video_driver);

MODULE_DESCRIPTION("ASPEED Video Engine Driver");
MODULE_AUTHOR("Eddie James");
MODULE_LICENSE("GPL v2");
