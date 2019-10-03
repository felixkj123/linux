// SPDX-License-Identifier: GPL-2.0+
// Copyright IBM Corp 2019

#include <linux/aspeed-xdma.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/genalloc.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define DEVICE_NAME				"aspeed-xdma"

#define SCU_AST2500_STRAP			0x070
#define  SCU_AST2500_STRAP_VGA_MEM		 GENMASK(3, 2)
#define SCU_AST2600_STRAP			0x500
#define  SCU_AST2600_STRAP_VGA_MEM		 GENMASK(14, 13)

#define SCU_AST2500_PCIE_CONF			0x180
#define SCU_AST2600_PCIE_CONF			0xc20
#define  SCU_PCIE_CONF_VGA_EN			 BIT(0)
#define  SCU_PCIE_CONF_VGA_EN_MMIO		 BIT(1)
#define  SCU_PCIE_CONF_VGA_EN_LPC		 BIT(2)
#define  SCU_PCIE_CONF_VGA_EN_MSI		 BIT(3)
#define  SCU_PCIE_CONF_VGA_EN_MCTP		 BIT(4)
#define  SCU_PCIE_CONF_VGA_EN_IRQ		 BIT(5)
#define  SCU_PCIE_CONF_VGA_EN_DMA		 BIT(6)
#define  SCU_PCIE_CONF_BMC_EN			 BIT(8)
#define  SCU_PCIE_CONF_BMC_EN_MMIO		 BIT(9)
#define  SCU_PCIE_CONF_BMC_EN_MSI		 BIT(11)
#define  SCU_PCIE_CONF_BMC_EN_MCTP		 BIT(12)
#define  SCU_PCIE_CONF_BMC_EN_IRQ		 BIT(13)
#define  SCU_PCIE_CONF_BMC_EN_DMA		 BIT(14)

#define SCU_AST2500_BMC_CLASS_REV		0x19c
#define SCU_AST2600_BMC_CLASS_REV		0xc4c
#define  SCU_BMC_CLASS_REV_XDMA			 0xff000001

#define SDMC_BASE				0x1e6e0000
#define SDMC_CONF				0x004
#define  SDMC_CONF_MEM				 GENMASK(1, 0)
#define SDMC_REMAP				0x008
#define  SDMC_AST2500_REMAP_MAGIC		 (BIT(16) | BIT(17))
#define  SDMC_AST2600_REMAP_MAGIC		 BIT(18)

#define XDMA_CMDQ_SIZE				PAGE_SIZE
#define XDMA_NUM_CMDS				\
	(XDMA_CMDQ_SIZE / sizeof(struct aspeed_xdma_cmd))
#define XDMA_OP_SIZE_MAX			sizeof(struct aspeed_xdma_op)
#define XDMA_OP_SIZE_MIN			\
	(sizeof(struct aspeed_xdma_op) - sizeof(u64))

/* Aspeed specification requires 10ms after switching the reset line */
#define XDMA_RESET_TIME_MS			10

#define XDMA_DS_PCIE_REQ_SIZE_128		0
#define XDMA_DS_PCIE_REQ_SIZE_256		1
#define XDMA_DS_PCIE_REQ_SIZE_512		2
#define XDMA_DS_PCIE_REQ_SIZE_1K		3
#define XDMA_DS_PCIE_REQ_SIZE_2K		4
#define XDMA_DS_PCIE_REQ_SIZE_4K		5

#define XDMA_CMD_AST2500_PITCH_SHIFT		3
#define XDMA_CMD_AST2500_PITCH_BMC		GENMASK_ULL(62, 51)
#define XDMA_CMD_AST2500_PITCH_HOST		GENMASK_ULL(46, 35)
#define XDMA_CMD_AST2500_PITCH_UPSTREAM		BIT_ULL(31)
#define XDMA_CMD_AST2500_PITCH_ADDR		GENMASK_ULL(29, 4)
#define XDMA_CMD_AST2500_PITCH_ID		BIT_ULL(0)
#define XDMA_CMD_AST2500_CMD_IRQ_EN		BIT_ULL(31)
#define XDMA_CMD_AST2500_CMD_LINE_NO		GENMASK_ULL(27, 16)
#define XDMA_CMD_AST2500_CMD_IRQ_BMC		BIT_ULL(15)
#define XDMA_CMD_AST2500_CMD_LINE_SIZE_SHIFT	4
#define XDMA_CMD_AST2500_CMD_LINE_SIZE		\
	GENMASK_ULL(14, XDMA_CMD_AST2500_CMD_LINE_SIZE_SHIFT)
#define XDMA_CMD_AST2500_CMD_ID			BIT_ULL(1)

#define XDMA_CMD_AST2600_PITCH_BMC		GENMASK_ULL(62, 48)
#define XDMA_CMD_AST2600_PITCH_HOST		GENMASK_ULL(46, 32)
#define XDMA_CMD_AST2600_PITCH_ADDR		GENMASK_ULL(30, 0)
#define XDMA_CMD_AST2600_CMD_64_EN		BIT_ULL(40)
#define XDMA_CMD_AST2600_CMD_IRQ_BMC		BIT_ULL(37)
#define XDMA_CMD_AST2600_CMD_IRQ_HOST		BIT_ULL(36)
#define XDMA_CMD_AST2600_CMD_UPSTREAM		BIT_ULL(32)
#define XDMA_CMD_AST2600_CMD_LINE_NO		GENMASK_ULL(27, 16)
#define XDMA_CMD_AST2600_CMD_LINE_SIZE		GENMASK_ULL(14, 0)

#define XDMA_AST2500_QUEUE_ENTRY_SIZE		4
#define XDMA_AST2500_HOST_CMDQ_ADDR0		0x00
#define XDMA_AST2500_HOST_CMDQ_ENDP		0x04
#define XDMA_AST2500_HOST_CMDQ_WRITEP		0x08
#define XDMA_AST2500_HOST_CMDQ_READP		0x0c
#define XDMA_AST2500_BMC_CMDQ_ADDR		0x10
#define XDMA_AST2500_BMC_CMDQ_ENDP		0x14
#define XDMA_AST2500_BMC_CMDQ_WRITEP		0x18
#define XDMA_AST2500_BMC_CMDQ_READP		0x1c
#define  XDMA_BMC_CMDQ_READP_MAGIC		 0xee882266
#define XDMA_AST2500_CTRL			0x20
#define  XDMA_AST2500_CTRL_US_COMP		 BIT(4)
#define  XDMA_AST2500_CTRL_DS_COMP		 BIT(5)
#define  XDMA_AST2500_CTRL_DS_DIRTY		 BIT(6)
#define  XDMA_AST2500_CTRL_DS_SIZE		 GENMASK(19, 17)
#define  XDMA_AST2500_CTRL_DS_TIMEOUT		 BIT(28)
#define  XDMA_AST2500_CTRL_DS_CHECK_ID		 BIT(29)
#define XDMA_AST2500_STATUS			0x24
#define  XDMA_AST2500_STATUS_US_COMP		 BIT(4)
#define  XDMA_AST2500_STATUS_DS_COMP		 BIT(5)
#define  XDMA_AST2500_STATUS_DS_DIRTY		 BIT(6)
#define XDMA_AST2500_INPRG_DS_CMD1		0x38
#define XDMA_AST2500_INPRG_DS_CMD2		0x3c
#define XDMA_AST2500_INPRG_US_CMD00		0x40
#define XDMA_AST2500_INPRG_US_CMD01		0x44
#define XDMA_AST2500_INPRG_US_CMD10		0x48
#define XDMA_AST2500_INPRG_US_CMD11		0x4c
#define XDMA_AST2500_INPRG_US_CMD20		0x50
#define XDMA_AST2500_INPRG_US_CMD21		0x54
#define XDMA_AST2500_HOST_CMDQ_ADDR1		0x60
#define XDMA_AST2500_VGA_CMDQ_ADDR0		0x64
#define XDMA_AST2500_VGA_CMDQ_ENDP		0x68
#define XDMA_AST2500_VGA_CMDQ_WRITEP		0x6c
#define XDMA_AST2500_VGA_CMDQ_READP		0x70
#define XDMA_AST2500_VGA_CMD_STATUS		0x74
#define XDMA_AST2500_VGA_CMDQ_ADDR1		0x78

#define XDMA_AST2600_QUEUE_ENTRY_SIZE		2
#define XDMA_AST2600_HOST_CMDQ_ADDR0		0x00
#define XDMA_AST2600_HOST_CMDQ_ADDR1		0x04
#define XDMA_AST2600_HOST_CMDQ_ENDP		0x08
#define XDMA_AST2600_HOST_CMDQ_WRITEP		0x0c
#define XDMA_AST2600_HOST_CMDQ_READP		0x10
#define XDMA_AST2600_BMC_CMDQ_ADDR		0x14
#define XDMA_AST2600_BMC_CMDQ_ENDP		0x18
#define XDMA_AST2600_BMC_CMDQ_WRITEP		0x1c
#define XDMA_AST2600_BMC_CMDQ_READP		0x20
#define XDMA_AST2600_VGA_CMDQ_ADDR0		0x24
#define XDMA_AST2600_VGA_CMDQ_ADDR1		0x28
#define XDMA_AST2600_VGA_CMDQ_ENDP		0x2c
#define XDMA_AST2600_VGA_CMDQ_WRITEP		0x30
#define XDMA_AST2600_VGA_CMDQ_READP		0x34
#define XDMA_AST2600_CTRL			0x38
#define  XDMA_AST2600_CTRL_US_COMP		 BIT(16)
#define  XDMA_AST2600_CTRL_DS_COMP		 BIT(17)
#define  XDMA_AST2600_CTRL_DS_DIRTY		 BIT(18)
#define  XDMA_AST2600_CTRL_DS_SIZE		 GENMASK(22, 20)
#define XDMA_AST2600_STATUS			0x3c
#define  XDMA_AST2600_STATUS_US_COMP		 BIT(16)
#define  XDMA_AST2600_STATUS_DS_COMP		 BIT(17)
#define  XDMA_AST2600_STATUS_DS_DIRTY		 BIT(18)
#define XDMA_AST2600_INPRG_DS_CMD00		0x40
#define XDMA_AST2600_INPRG_DS_CMD01		0x44
#define XDMA_AST2600_INPRG_DS_CMD10		0x48
#define XDMA_AST2600_INPRG_DS_CMD11		0x4c
#define XDMA_AST2600_INPRG_DS_CMD20		0x50
#define XDMA_AST2600_INPRG_DS_CMD21		0x54
#define XDMA_AST2600_INPRG_US_CMD00		0x60
#define XDMA_AST2600_INPRG_US_CMD01		0x64
#define XDMA_AST2600_INPRG_US_CMD10		0x68
#define XDMA_AST2600_INPRG_US_CMD11		0x6c
#define XDMA_AST2600_INPRG_US_CMD20		0x70
#define XDMA_AST2600_INPRG_US_CMD21		0x74

enum versions { xdma_ast2500, xdma_ast2600 };

struct aspeed_xdma_cmd {
	u64 host_addr;
	u64 pitch;
	u64 cmd;
	u64 reserved;
};

struct aspeed_xdma_regs {
	u8 bmc_cmdq_addr;
	u8 bmc_cmdq_endp;
	u8 bmc_cmdq_writep;
	u8 bmc_cmdq_readp;
	u8 control;
	u8 status;
};

struct aspeed_xdma_status_bits {
	u32 us_comp;
	u32 ds_comp;
	u32 ds_dirty;
};

struct aspeed_xdma_client;

struct aspeed_xdma {
	enum versions version;
	u32 control;
	unsigned int queue_entry_size;
	struct aspeed_xdma_regs regs;
	struct aspeed_xdma_status_bits status_bits;

	struct device *dev;
	void __iomem *base;
	struct reset_control *reset;

	bool in_progress;
	bool in_reset;
	bool upstream;
	unsigned int cmd_idx;
	struct mutex file_lock;
	struct mutex start_lock;
	struct delayed_work reset_work;
	spinlock_t client_lock;
	spinlock_t reset_lock;
	wait_queue_head_t wait;
	struct aspeed_xdma_client *current_client;

	u32 vga_phys;
	u32 vga_size;
	void *cmdq;
	void __iomem *vga_virt;
	dma_addr_t cmdq_vga_phys;
	void *cmdq_vga_virt;
	struct gen_pool *vga_pool;

	struct miscdevice misc;
};

struct aspeed_xdma_client {
	struct aspeed_xdma *ctx;

	bool error;
	bool in_progress;
	void *virt;
	dma_addr_t phys;
	u32 size;
};

static u32 aspeed_xdma_readl(struct aspeed_xdma *ctx, u8 reg)
{
	u32 v = readl(ctx->base + reg);

	dev_dbg(ctx->dev, "read %02x[%08x]\n", reg, v);
	return v;
}

static void aspeed_xdma_writel(struct aspeed_xdma *ctx, u8 reg, u32 val)
{
	writel(val, ctx->base + reg);
	dev_dbg(ctx->dev, "write %02x[%08x]\n", reg, readl(ctx->base + reg));
}

static void aspeed_xdma_init_eng(struct aspeed_xdma *ctx)
{
	aspeed_xdma_writel(ctx, ctx->regs.bmc_cmdq_endp,
			   ctx->queue_entry_size * XDMA_NUM_CMDS);
	aspeed_xdma_writel(ctx, ctx->regs.bmc_cmdq_readp,
			   XDMA_BMC_CMDQ_READP_MAGIC);
	aspeed_xdma_writel(ctx, ctx->regs.bmc_cmdq_writep, 0);
	aspeed_xdma_writel(ctx, ctx->regs.control, ctx->control);
	aspeed_xdma_writel(ctx, ctx->regs.bmc_cmdq_addr, ctx->cmdq_vga_phys);

	ctx->cmd_idx = 0;
	ctx->in_progress = false;
}

static unsigned int aspeed_xdma_ast2500_set_cmd(struct aspeed_xdma *ctx,
						struct aspeed_xdma_op *op,
						u32 bmc_addr)
{
	u64 cmd = XDMA_CMD_AST2500_CMD_IRQ_EN | XDMA_CMD_AST2500_CMD_IRQ_BMC |
		XDMA_CMD_AST2500_CMD_ID;
	u64 cmd_pitch = (op->direction ? XDMA_CMD_AST2500_PITCH_UPSTREAM : 0) |
		XDMA_CMD_AST2500_PITCH_ID;
	unsigned int line_size;
	unsigned int nidx = (ctx->cmd_idx + 1) % XDMA_NUM_CMDS;
	unsigned int line_no = 1;
	unsigned int pitch = 1;
	struct aspeed_xdma_cmd *ncmd =
		&(((struct aspeed_xdma_cmd *)ctx->cmdq)[ctx->cmd_idx]);

	dev_dbg(ctx->dev, "xdma %s ast2500: bmc[%08x] len[%08x] host[%08x]\n",
		op->direction ? "upstream" : "downstream", bmc_addr, op->len,
		(u32)op->host_addr);

	if (op->len > XDMA_CMD_AST2500_CMD_LINE_SIZE) {
		unsigned int rem;
		unsigned int total;

		line_no = op->len / XDMA_CMD_AST2500_CMD_LINE_SIZE;
		total = XDMA_CMD_AST2500_CMD_LINE_SIZE * line_no;
		rem = (op->len - total) >>
			XDMA_CMD_AST2500_CMD_LINE_SIZE_SHIFT;
		line_size = XDMA_CMD_AST2500_CMD_LINE_SIZE;
		pitch = line_size >> XDMA_CMD_AST2500_PITCH_SHIFT;
		line_size >>= XDMA_CMD_AST2500_CMD_LINE_SIZE_SHIFT;

		if (rem) {
			u32 rbmc = bmc_addr + total;
			struct aspeed_xdma_cmd *rcmd =
				&(((struct aspeed_xdma_cmd *)ctx->cmdq)[nidx]);

			rcmd->host_addr = op->host_addr + (u64)total;
			rcmd->pitch = cmd_pitch |
				((u64)rbmc & XDMA_CMD_AST2500_PITCH_ADDR) |
				FIELD_PREP(XDMA_CMD_AST2500_PITCH_HOST, 1) |
				FIELD_PREP(XDMA_CMD_AST2500_PITCH_BMC, 1);
			rcmd->cmd = cmd |
				FIELD_PREP(XDMA_CMD_AST2500_CMD_LINE_NO, 1) |
				FIELD_PREP(XDMA_CMD_AST2500_CMD_LINE_SIZE,
					   rem);

			print_hex_dump_debug("xdma rem", DUMP_PREFIX_OFFSET,
					     16, 1, rcmd, sizeof(*rcmd), true);

			cmd &= ~(XDMA_CMD_AST2500_CMD_IRQ_EN |
				 XDMA_CMD_AST2500_CMD_IRQ_BMC);

			nidx = (nidx + 1) % XDMA_NUM_CMDS;
		}
	} else {
		line_size = op->len >> XDMA_CMD_AST2500_CMD_LINE_SIZE_SHIFT;
	}

	ncmd->host_addr = op->host_addr;
	ncmd->pitch = cmd_pitch |
		((u64)bmc_addr & XDMA_CMD_AST2500_PITCH_ADDR) |
		FIELD_PREP(XDMA_CMD_AST2500_PITCH_HOST, pitch) |
		FIELD_PREP(XDMA_CMD_AST2500_PITCH_BMC, pitch);
	ncmd->cmd = cmd | FIELD_PREP(XDMA_CMD_AST2500_CMD_LINE_NO, line_no) |
		FIELD_PREP(XDMA_CMD_AST2500_CMD_LINE_SIZE, line_size);

	print_hex_dump_debug("xdma cmd", DUMP_PREFIX_OFFSET, 16, 1, ncmd,
			     sizeof(*ncmd), true);

	return nidx;
}

static unsigned int aspeed_xdma_ast2600_set_cmd(struct aspeed_xdma *ctx,
						struct aspeed_xdma_op *op,
						u32 bmc_addr)
{
	u64 cmd = XDMA_CMD_AST2600_CMD_IRQ_BMC |
		(op->direction ? XDMA_CMD_AST2600_CMD_UPSTREAM : 0);
	unsigned int line_size;
	unsigned int nidx = (ctx->cmd_idx + 1) % XDMA_NUM_CMDS;
	unsigned int line_no = 1;
	unsigned int pitch = 1;
	struct aspeed_xdma_cmd *ncmd =
		&(((struct aspeed_xdma_cmd *)ctx->cmdq)[ctx->cmd_idx]);

	if ((op->host_addr + op->len) & 0xffffffff00000000ULL)
		cmd |= XDMA_CMD_AST2600_CMD_64_EN;

	dev_dbg(ctx->dev, "xdma %s ast2600: bmc[%08x] len[%08x] "
		"host[%016llx]\n", op->direction ? "upstream" : "downstream",
		bmc_addr, op->len, op->host_addr);

	if (op->len > XDMA_CMD_AST2600_CMD_LINE_SIZE) {
		unsigned int rem;
		unsigned int total;

		line_no = op->len / XDMA_CMD_AST2600_CMD_LINE_SIZE;
		total = XDMA_CMD_AST2600_CMD_LINE_SIZE * line_no;
		rem = op->len - total;
		line_size = XDMA_CMD_AST2600_CMD_LINE_SIZE;
		pitch = line_size;

		if (rem) {
			u32 rbmc = bmc_addr + total;
			struct aspeed_xdma_cmd *rcmd =
				&(((struct aspeed_xdma_cmd *)ctx->cmdq)[nidx]);

			rcmd->host_addr = op->host_addr + (u64)total;
			rcmd->pitch =
				((u64)rbmc & XDMA_CMD_AST2600_PITCH_ADDR) |
				FIELD_PREP(XDMA_CMD_AST2600_PITCH_HOST, 1) |
				FIELD_PREP(XDMA_CMD_AST2600_PITCH_BMC, 1);
			rcmd->cmd = cmd |
				FIELD_PREP(XDMA_CMD_AST2600_CMD_LINE_NO, 1) |
				FIELD_PREP(XDMA_CMD_AST2600_CMD_LINE_SIZE,
					   rem);

			print_hex_dump_debug("xdma rem", DUMP_PREFIX_OFFSET,
					     16, 1, rcmd, sizeof(*rcmd), true);

			cmd &= ~XDMA_CMD_AST2600_CMD_IRQ_BMC;

			nidx = (nidx + 1) % XDMA_NUM_CMDS;
		}
	} else {
		line_size = op->len;
	}

	ncmd->host_addr = op->host_addr;
	ncmd->pitch = ((u64)bmc_addr & XDMA_CMD_AST2600_PITCH_ADDR) |
		FIELD_PREP(XDMA_CMD_AST2600_PITCH_HOST, pitch) |
		FIELD_PREP(XDMA_CMD_AST2600_PITCH_BMC, pitch);
	ncmd->cmd = cmd | FIELD_PREP(XDMA_CMD_AST2600_CMD_LINE_NO, line_no) |
		FIELD_PREP(XDMA_CMD_AST2600_CMD_LINE_SIZE, line_size);

	print_hex_dump_debug("xdma cmd", DUMP_PREFIX_OFFSET, 16, 1, ncmd,
			     sizeof(*ncmd), true);

	return nidx;
}

static void aspeed_xdma_start(struct aspeed_xdma *ctx,
			      struct aspeed_xdma_op *op, u32 bmc_addr,
			      struct aspeed_xdma_client *client)
{
	unsigned int nidx;

	mutex_lock(&ctx->start_lock);

	switch (ctx->version) {
	default:
	case xdma_ast2500:
		nidx = aspeed_xdma_ast2500_set_cmd(ctx, op, bmc_addr);
		break;
	case xdma_ast2600:
		nidx = aspeed_xdma_ast2600_set_cmd(ctx, op, bmc_addr);
		break;
	}

	memcpy(ctx->cmdq_vga_virt, ctx->cmdq, XDMA_CMDQ_SIZE);

	client->in_progress = true;
	ctx->current_client = client;

	ctx->in_progress = true;
	ctx->upstream = op->direction ? true : false;

	aspeed_xdma_writel(ctx, ctx->regs.bmc_cmdq_writep,
			   nidx * ctx->queue_entry_size);

	ctx->cmd_idx = nidx;

	mutex_unlock(&ctx->start_lock);
}

static void aspeed_xdma_done(struct aspeed_xdma *ctx, bool error)
{
	unsigned long flags;

	/*
	 * Lock to make sure simultaneous reset and transfer complete don't
	 * leave the client with the wrong error state.
	 */
	spin_lock_irqsave(&ctx->client_lock, flags);

	if (ctx->current_client) {
		ctx->current_client->error = error;
		ctx->current_client->in_progress = false;
		ctx->current_client = NULL;
	}

	spin_unlock_irqrestore(&ctx->client_lock, flags);

	ctx->in_progress = false;
	wake_up_interruptible_all(&ctx->wait);
}

static irqreturn_t aspeed_xdma_irq(int irq, void *arg)
{
	struct aspeed_xdma *ctx = arg;
	u32 status = aspeed_xdma_readl(ctx, ctx->regs.status);

	if (status & ctx->status_bits.ds_dirty) {
		aspeed_xdma_done(ctx, true);
	} else {
		if (status & ctx->status_bits.us_comp) {
			if (ctx->upstream)
				aspeed_xdma_done(ctx, false);
		}

		if (status & ctx->status_bits.ds_comp) {
			if (!ctx->upstream)
				aspeed_xdma_done(ctx, false);
		}
	}

	aspeed_xdma_writel(ctx, ctx->regs.status, status);

	return IRQ_HANDLED;
}

static void aspeed_xdma_reset_finish(struct aspeed_xdma *ctx)
{
	unsigned long flags;

	spin_lock_irqsave(&ctx->reset_lock, flags);

	ctx->in_reset = false;
	reset_control_deassert(ctx->reset);

	spin_unlock_irqrestore(&ctx->reset_lock, flags);

	msleep(XDMA_RESET_TIME_MS);

	aspeed_xdma_init_eng(ctx);
	aspeed_xdma_done(ctx, true);
}

static bool aspeed_xdma_reset_start(struct aspeed_xdma *ctx)
{
	bool rc = true;
	unsigned long flags;

	spin_lock_irqsave(&ctx->reset_lock, flags);

	if (ctx->in_reset) {
		rc = false;
	} else {
		ctx->in_reset = true;
		reset_control_assert(ctx->reset);
	}

	spin_unlock_irqrestore(&ctx->reset_lock, flags);

	return rc;
}

static void aspeed_xdma_reset_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct aspeed_xdma *ctx = container_of(dwork, struct aspeed_xdma,
					       reset_work);

	/*
	 * Lock to make sure operations aren't started while the engine is
	 * in an undefined state coming out of reset and waiting to init.
	 */
	mutex_lock(&ctx->start_lock);

	aspeed_xdma_reset_finish(ctx);

	mutex_unlock(&ctx->start_lock);
}

static irqreturn_t aspeed_xdma_pcie_irq(int irq, void *arg)
{
	struct aspeed_xdma *ctx = arg;

	dev_dbg(ctx->dev, "pcie reset\n");

	if (aspeed_xdma_reset_start(ctx))
		schedule_delayed_work(&ctx->reset_work,
				      msecs_to_jiffies(XDMA_RESET_TIME_MS));

	return IRQ_HANDLED;
}

static ssize_t aspeed_xdma_write(struct file *file, const char __user *buf,
				 size_t len, loff_t *offset)
{
	int rc;
	struct aspeed_xdma_op op;
	struct aspeed_xdma_client *client = file->private_data;
	struct aspeed_xdma *ctx = client->ctx;
	u32 offs = client->phys ? (client->phys - ctx->vga_phys) :
		XDMA_CMDQ_SIZE;

	if (len < XDMA_OP_SIZE_MIN)
		return -EINVAL;

	if (len > XDMA_OP_SIZE_MAX)
		len = XDMA_OP_SIZE_MAX;

	rc = copy_from_user(&op, buf, len);
	if (rc)
		return rc;

	if (op.direction == ASPEED_XDMA_DIRECTION_RESET) {
		mutex_lock(&ctx->start_lock);

		if (aspeed_xdma_reset_start(ctx)) {
			msleep(XDMA_RESET_TIME_MS);

			aspeed_xdma_reset_finish(ctx);
		}

		mutex_unlock(&ctx->start_lock);

		return len;
	} else if (op.direction > ASPEED_XDMA_DIRECTION_RESET) {
		return -EINVAL;
	}

	if (op.len > ctx->vga_size - offs)
		return -EINVAL;

	if (file->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&ctx->file_lock))
			return -EAGAIN;

		if (ctx->in_progress || ctx->in_reset) {
			mutex_unlock(&ctx->file_lock);
			return -EAGAIN;
		}
	} else {
		mutex_lock(&ctx->file_lock);

		rc = wait_event_interruptible(ctx->wait, !ctx->in_progress &&
					      !ctx->in_reset);
		if (rc) {
			mutex_unlock(&ctx->file_lock);
			return -EINTR;
		}
	}

	aspeed_xdma_start(ctx, &op, ctx->vga_phys + offs, client);

	mutex_unlock(&ctx->file_lock);

	if (!(file->f_flags & O_NONBLOCK)) {
		rc = wait_event_interruptible(ctx->wait, !ctx->in_progress);
		if (rc)
			return -EINTR;

		if (client->error)
			return -EIO;
	}

	return len;
}

static __poll_t aspeed_xdma_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	__poll_t mask = 0;
	__poll_t req = poll_requested_events(wait);
	struct aspeed_xdma_client *client = file->private_data;
	struct aspeed_xdma *ctx = client->ctx;

	if (req & (EPOLLIN | EPOLLRDNORM)) {
		if (client->in_progress)
			poll_wait(file, &ctx->wait, wait);

		if (!client->in_progress) {
			if (client->error)
				mask |= EPOLLERR;
			else
				mask |= EPOLLIN | EPOLLRDNORM;
		}
	}

	if (req & (EPOLLOUT | EPOLLWRNORM)) {
		if (ctx->in_progress)
			poll_wait(file, &ctx->wait, wait);

		if (!ctx->in_progress)
			mask |= EPOLLOUT | EPOLLWRNORM;
	}

	return mask;
}

static void aspeed_xdma_vma_close(struct vm_area_struct *vma)
{
	struct aspeed_xdma_client *client = vma->vm_private_data;

	gen_pool_free(client->ctx->vga_pool, (unsigned long)client->virt,
		      client->size);

	client->virt = NULL;
	client->phys = 0;
	client->size = 0;
}

static const struct vm_operations_struct aspeed_xdma_vm_ops = {
	.close =	aspeed_xdma_vma_close,
};

static int aspeed_xdma_mmap(struct file *file, struct vm_area_struct *vma)
{
	int rc;
	struct aspeed_xdma_client *client = file->private_data;
	struct aspeed_xdma *ctx = client->ctx;

	/* restrict file to one mapping */
	if (client->size)
		return -ENOMEM;

	client->size = vma->vm_end - vma->vm_start;
	client->virt = gen_pool_dma_alloc(ctx->vga_pool, client->size,
					  &client->phys);
	if (!client->virt) {
		client->phys = 0;
		client->size = 0;
		return -ENOMEM;
	}

	vma->vm_pgoff = (client->phys - ctx->vga_phys) >> PAGE_SHIFT;
	vma->vm_ops = &aspeed_xdma_vm_ops;
	vma->vm_private_data = client;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	rc = io_remap_pfn_range(vma, vma->vm_start, client->phys >> PAGE_SHIFT,
				client->size, vma->vm_page_prot);
	if (rc) {
		gen_pool_free(ctx->vga_pool, (unsigned long)client->virt,
			      client->size);

		client->virt = NULL;
		client->phys = 0;
		client->size = 0;
		return rc;
	}

	dev_dbg(ctx->dev, "mmap: v[%08lx] to p[%08x], s[%08x]\n",
		vma->vm_start, (u32)client->phys, client->size);

	return 0;
}

static int aspeed_xdma_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct aspeed_xdma *ctx = container_of(misc, struct aspeed_xdma, misc);
	struct aspeed_xdma_client *client = kzalloc(sizeof(*client),
						    GFP_KERNEL);

	if (!client)
		return -ENOMEM;

	client->ctx = ctx;
	file->private_data = client;
	return 0;
}

static int aspeed_xdma_release(struct inode *inode, struct file *file)
{
	struct aspeed_xdma_client *client = file->private_data;

	if (client->ctx->current_client == client)
		client->ctx->current_client = NULL;

	kfree(client);
	return 0;
}

static const struct file_operations aspeed_xdma_fops = {
	.owner			= THIS_MODULE,
	.write			= aspeed_xdma_write,
	.poll			= aspeed_xdma_poll,
	.mmap			= aspeed_xdma_mmap,
	.open			= aspeed_xdma_open,
	.release		= aspeed_xdma_release,
};

static int aspeed_xdma_init(struct aspeed_xdma *ctx)
{
	int rc;
	struct regmap *scu;
	u32 conf;
	u32 mem_size;
	u32 remap;
	u32 scu_bmc_class;
	u32 scu_pcie_conf;
	u32 scu_strap;
	u32 sdmc_remap_magic;
	u32 strap = 0;
	const u32 bmc = SCU_PCIE_CONF_BMC_EN | SCU_PCIE_CONF_BMC_EN_MSI |
		SCU_PCIE_CONF_BMC_EN_MCTP | SCU_PCIE_CONF_BMC_EN_IRQ |
		SCU_PCIE_CONF_BMC_EN_DMA;
	const u32 vga = SCU_PCIE_CONF_VGA_EN | SCU_PCIE_CONF_VGA_EN_MSI |
		SCU_PCIE_CONF_VGA_EN_MCTP | SCU_PCIE_CONF_VGA_EN_IRQ |
		SCU_PCIE_CONF_VGA_EN_DMA;
	u32 mem_sizes[4] = { 0x8000000, 0x10000000, 0x20000000, 0x40000000 };
	const u32 vga_sizes[4] = { 0x800000, 0x1000000, 0x2000000, 0x4000000 };
	void __iomem *sdmc_base = ioremap(SDMC_BASE, 0x100);

	if (!sdmc_base) {
		dev_err(ctx->dev, "Failed to ioremap mem controller regs.\n");
		return -ENOMEM;
	}

	switch (ctx->version) {
	default:
	case xdma_ast2500:
		scu_bmc_class = SCU_AST2500_BMC_CLASS_REV;
		scu_pcie_conf = SCU_AST2500_PCIE_CONF;
		scu_strap = SCU_AST2500_STRAP;
		sdmc_remap_magic = SDMC_AST2500_REMAP_MAGIC;

		scu = syscon_regmap_lookup_by_compatible("aspeed,ast2500-scu");
		break;
	case xdma_ast2600:
		scu_bmc_class = SCU_AST2600_BMC_CLASS_REV;
		scu_pcie_conf = SCU_AST2600_PCIE_CONF;
		scu_strap = SCU_AST2600_STRAP;
		sdmc_remap_magic = SDMC_AST2600_REMAP_MAGIC;

		mem_sizes[0] *= 2;
		mem_sizes[1] *= 2;
		mem_sizes[2] *= 2;
		mem_sizes[3] *= 2;

		scu = syscon_regmap_lookup_by_compatible("aspeed,ast2600-scu");
		break;
	};

	if (!scu) {
		dev_err(ctx->dev, "Failed to grab SCU regs.\n");
		return -ENOMEM;
	}

	/* Set SOC to use the BMC PCIe device and set the device class code */
	regmap_update_bits(scu, scu_pcie_conf, bmc | vga, bmc);
	regmap_write(scu, scu_bmc_class, SCU_BMC_CLASS_REV_XDMA);

	/*
	 * Calculate the VGA memory size and physical address from the SCU and
	 * memory controller registers.
	 */
	regmap_read(scu, scu_strap, &strap);

	switch (ctx->version) {
	case xdma_ast2500:
		ctx->vga_size = vga_sizes[FIELD_GET(SCU_AST2500_STRAP_VGA_MEM,
						    strap)];
		break;
	case xdma_ast2600:
		ctx->vga_size = vga_sizes[FIELD_GET(SCU_AST2600_STRAP_VGA_MEM,
						    strap)];
		break;
	}

	conf = readl(sdmc_base + SDMC_CONF);
	remap = readl(sdmc_base + SDMC_REMAP);
	remap |= sdmc_remap_magic;
	writel(remap, sdmc_base + SDMC_REMAP);
	mem_size = mem_sizes[conf & SDMC_CONF_MEM];

	iounmap(sdmc_base);

	ctx->vga_phys = (mem_size - ctx->vga_size) + 0x80000000;

	ctx->cmdq = devm_kzalloc(ctx->dev, XDMA_CMDQ_SIZE, GFP_KERNEL);
	if (!ctx->cmdq) {
		dev_err(ctx->dev, "Failed to allocate command queue.\n");
		return -ENOMEM;
	}

	ctx->vga_virt = ioremap(ctx->vga_phys, ctx->vga_size);
	if (!ctx->vga_virt) {
		dev_err(ctx->dev, "Failed to ioremap VGA memory.\n");
		return -ENOMEM;
	}

	rc = gen_pool_add_virt(ctx->vga_pool, (unsigned long)ctx->vga_virt,
			       ctx->vga_phys, ctx->vga_size, -1);
	if (rc) {
		dev_err(ctx->dev, "Failed to add memory to genalloc pool.\n");
		iounmap(ctx->vga_virt);
		return rc;
	}

	ctx->cmdq_vga_virt = gen_pool_dma_alloc(ctx->vga_pool, XDMA_CMDQ_SIZE,
						&ctx->cmdq_vga_phys);
	if (!ctx->cmdq_vga_virt) {
		dev_err(ctx->dev, "Failed to genalloc cmdq.\n");
		iounmap(ctx->vga_virt);
		return -ENOMEM;
	}

	dev_dbg(ctx->dev, "VGA mapped at phys[%08x], size[%08x].\n",
		ctx->vga_phys, ctx->vga_size);

	return 0;
}

static int aspeed_xdma_probe(struct platform_device *pdev)
{
	int irq;
	int pcie_irq;
	int rc;
	enum versions vs = xdma_ast2500;
	struct device *dev = &pdev->dev;
	struct aspeed_xdma *ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	const void *md = of_device_get_match_data(dev);

	if (!ctx)
		return -ENOMEM;

	if (md)
		vs = (enum versions)md;

	switch (vs) {
	default:
	case xdma_ast2500:
		ctx->version = xdma_ast2500;
		ctx->control = XDMA_AST2500_CTRL_US_COMP |
			XDMA_AST2500_CTRL_DS_COMP |
			XDMA_AST2500_CTRL_DS_DIRTY |
			FIELD_PREP(XDMA_AST2500_CTRL_DS_SIZE,
				   XDMA_DS_PCIE_REQ_SIZE_256) |
			XDMA_AST2500_CTRL_DS_TIMEOUT |
			XDMA_AST2500_CTRL_DS_CHECK_ID;
		ctx->queue_entry_size = XDMA_AST2500_QUEUE_ENTRY_SIZE;
		ctx->regs.bmc_cmdq_addr = XDMA_AST2500_BMC_CMDQ_ADDR;
		ctx->regs.bmc_cmdq_endp = XDMA_AST2500_BMC_CMDQ_ENDP;
		ctx->regs.bmc_cmdq_writep = XDMA_AST2500_BMC_CMDQ_WRITEP;
		ctx->regs.bmc_cmdq_readp = XDMA_AST2500_BMC_CMDQ_READP;
		ctx->regs.control = XDMA_AST2500_CTRL;
		ctx->regs.status = XDMA_AST2500_STATUS;
		ctx->status_bits.us_comp = XDMA_AST2500_STATUS_US_COMP;
		ctx->status_bits.ds_comp = XDMA_AST2500_STATUS_DS_COMP;
		ctx->status_bits.ds_dirty = XDMA_AST2500_STATUS_DS_DIRTY;
		break;
	case xdma_ast2600:
		ctx->version = xdma_ast2600;
		ctx->control = XDMA_AST2600_CTRL_US_COMP |
			XDMA_AST2600_CTRL_DS_COMP |
			XDMA_AST2600_CTRL_DS_DIRTY |
			FIELD_PREP(XDMA_AST2600_CTRL_DS_SIZE,
				   XDMA_DS_PCIE_REQ_SIZE_256);
		ctx->queue_entry_size = XDMA_AST2600_QUEUE_ENTRY_SIZE;
		ctx->regs.bmc_cmdq_addr = XDMA_AST2600_BMC_CMDQ_ADDR;
		ctx->regs.bmc_cmdq_endp = XDMA_AST2600_BMC_CMDQ_ENDP;
		ctx->regs.bmc_cmdq_writep = XDMA_AST2600_BMC_CMDQ_WRITEP;
		ctx->regs.bmc_cmdq_readp = XDMA_AST2600_BMC_CMDQ_READP;
		ctx->regs.control = XDMA_AST2600_CTRL;
		ctx->regs.status = XDMA_AST2600_STATUS;
		ctx->status_bits.us_comp = XDMA_AST2600_STATUS_US_COMP;
		ctx->status_bits.ds_comp = XDMA_AST2600_STATUS_DS_COMP;
		ctx->status_bits.ds_dirty = XDMA_AST2600_STATUS_DS_DIRTY;
		break;
	};

	ctx->dev = dev;
	platform_set_drvdata(pdev, ctx);
	mutex_init(&ctx->file_lock);
	mutex_init(&ctx->start_lock);
	INIT_DELAYED_WORK(&ctx->reset_work, aspeed_xdma_reset_work);
	spin_lock_init(&ctx->client_lock);
	spin_lock_init(&ctx->reset_lock);
	init_waitqueue_head(&ctx->wait);

	ctx->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ctx->base)) {
		dev_err(dev, "Unable to ioremap registers.\n");
		return PTR_ERR(ctx->base);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "Unable to find IRQ.\n");
		return -ENODEV;
	}

	rc = devm_request_irq(dev, irq, aspeed_xdma_irq, IRQF_SHARED,
			      DEVICE_NAME, ctx);
	if (rc < 0) {
		dev_err(dev, "Unable to request IRQ %d.\n", irq);
		return rc;
	}

	ctx->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(ctx->reset)) {
		dev_err(dev, "Unable to request reset control.\n");
		return PTR_ERR(ctx->reset);
	}

	ctx->vga_pool = devm_gen_pool_create(dev, ilog2(PAGE_SIZE), -1, NULL);
	if (!ctx->vga_pool) {
		dev_err(dev, "Unable to setup genalloc pool.\n");
		return -ENOMEM;
	}

	reset_control_deassert(ctx->reset);

	msleep(XDMA_RESET_TIME_MS);

	rc = aspeed_xdma_init(ctx);
	if (rc) {
		reset_control_assert(ctx->reset);
		return rc;
	}

	aspeed_xdma_init_eng(ctx);

	ctx->misc.minor = MISC_DYNAMIC_MINOR;
	ctx->misc.fops = &aspeed_xdma_fops;
	ctx->misc.name = "aspeed-xdma";
	ctx->misc.parent = dev;
	rc = misc_register(&ctx->misc);
	if (rc) {
		dev_err(dev, "Unable to register xdma miscdevice.\n");

		gen_pool_free(ctx->vga_pool, (unsigned long)ctx->cmdq_vga_virt,
			      XDMA_CMDQ_SIZE);
		iounmap(ctx->vga_virt);
		reset_control_assert(ctx->reset);
		return rc;
	}

	/*
	 * This interrupt could fire immediately so only request it once the
	 * engine and driver are initialized.
	 */
	pcie_irq = platform_get_irq(pdev, 1);
	if (pcie_irq < 0) {
		dev_warn(dev, "Unable to find PCI-E IRQ.\n");
	} else {
		rc = devm_request_irq(dev, pcie_irq, aspeed_xdma_pcie_irq,
				      IRQF_SHARED, DEVICE_NAME, ctx);
		if (rc < 0)
			dev_warn(dev, "Unable to request PCI-E IRQ %d.\n", rc);
	}

	return 0;
}

static int aspeed_xdma_remove(struct platform_device *pdev)
{
	struct aspeed_xdma *ctx = platform_get_drvdata(pdev);

	misc_deregister(&ctx->misc);
	gen_pool_free(ctx->vga_pool, (unsigned long)ctx->cmdq_vga_virt,
		      XDMA_CMDQ_SIZE);
	iounmap(ctx->vga_virt);
	reset_control_assert(ctx->reset);

	return 0;
}

static const struct of_device_id aspeed_xdma_match[] = {
	{
		.compatible = "aspeed,ast2500-xdma",
		.data = (void *)xdma_ast2500,
	},
	{
		.compatible = "aspeed,ast2600-xdma",
		.data = (void *)xdma_ast2600,
	},
	{ },
};

static struct platform_driver aspeed_xdma_driver = {
	.probe = aspeed_xdma_probe,
	.remove = aspeed_xdma_remove,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = aspeed_xdma_match,
	},
};

module_platform_driver(aspeed_xdma_driver);

MODULE_AUTHOR("Eddie James");
MODULE_DESCRIPTION("Aspeed XDMA Engine Driver");
MODULE_LICENSE("GPL v2");
