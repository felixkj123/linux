// SPDX-License-Identifier: GPL-2.0+
// Copyright IBM Corp 2019

#include <linux/aspeed-xdma.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/genalloc.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define DEVICE_NAME			"aspeed-xdma"

#define SCU_STRAP			0x070
#define  SCU_STRAP_VGA_MEM		GENMASK(3, 2)

#define SCU_PCIE_CONF			0x180
#define  SCU_PCIE_CONF_VGA_EN		BIT(0)
#define  SCU_PCIE_CONF_VGA_EN_MMIO	BIT(1)
#define  SCU_PCIE_CONF_VGA_EN_LPC	BIT(2)
#define  SCU_PCIE_CONF_VGA_EN_MSI	BIT(3)
#define  SCU_PCIE_CONF_VGA_EN_MCTP	BIT(4)
#define  SCU_PCIE_CONF_VGA_EN_IRQ	BIT(5)
#define  SCU_PCIE_CONF_VGA_EN_DMA	BIT(6)
#define  SCU_PCIE_CONF_BMC_EN		BIT(8)
#define  SCU_PCIE_CONF_BMC_EN_MMIO	BIT(9)
#define  SCU_PCIE_CONF_BMC_EN_MSI	BIT(11)
#define  SCU_PCIE_CONF_BMC_EN_MCTP	BIT(12)
#define  SCU_PCIE_CONF_BMC_EN_IRQ	BIT(13)
#define  SCU_PCIE_CONF_BMC_EN_DMA	BIT(14)
#define  SCU_PCIE_CONF_RSVD		GENMASK(19, 18)

#define SDMC_CONF			0x004
#define  SDMC_CONF_MEM			GENMASK(1, 0)
#define SDMC_REMAP			0x008
#define  SDMC_REMAP_MAGIC		GENMASK(17, 16)

#define XDMA_CMD_SIZE			4
#define XDMA_CMDQ_SIZE			PAGE_SIZE
#define XDMA_BYTE_ALIGN			16
#define XDMA_MAX_LINE_SIZE		BIT(10)
#define XDMA_NUM_CMDS			\
	(XDMA_CMDQ_SIZE / sizeof(struct aspeed_xdma_cmd))
#define XDMA_NUM_DEBUGFS_REGS		6

#define XDMA_CMD_BMC_CHECK		BIT(0)
#define XDMA_CMD_BMC_ADDR		GENMASK(29, 4)
#define XDMA_CMD_BMC_DIR_US		BIT(31)

#define XDMA_CMD_COMM1_HI_HOST_PITCH	GENMASK(14, 3)
#define XDMA_CMD_COMM1_HI_BMC_PITCH	GENMASK(30, 19)

#define XDMA_CMD_CONF_CHECK		BIT(1)
#define XDMA_CMD_CONF_LINE_SIZE		GENMASK(14, 4)
#define XDMA_CMD_CONF_IRQ_BMC		BIT(15)
#define XDMA_CMD_CONF_NUM_LINES		GENMASK(27, 16)
#define XDMA_CMD_CONF_IRQ		BIT(31)

#define XDMA_CMD_ID_UPDIR		GENMASK(17, 16)
#define  XDMA_CMD_ID_UPDIR_BMC		0
#define  XDMA_CMD_ID_UPDIR_HOST		1
#define  XDMA_CMD_ID_UPDIR_VGA		2

#define XDMA_DS_PCIE_REQ_SIZE_128	0
#define XDMA_DS_PCIE_REQ_SIZE_256	1
#define XDMA_DS_PCIE_REQ_SIZE_512	2
#define XDMA_DS_PCIE_REQ_SIZE_1K	3
#define XDMA_DS_PCIE_REQ_SIZE_2K	4
#define XDMA_DS_PCIE_REQ_SIZE_4K	5

#define XDMA_BMC_CMD_QUEUE_ADDR		0x10
#define XDMA_BMC_CMD_QUEUE_ENDP		0x14
#define XDMA_BMC_CMD_QUEUE_WRITEP	0x18
#define XDMA_BMC_CMD_QUEUE_READP	0x1c
#define  XDMA_BMC_CMD_QUEUE_READP_MAGIC	0xee882266
#define XDMA_CTRL			0x20
#define  XDMA_CTRL_US_COMP		BIT(4)
#define  XDMA_CTRL_DS_COMP		BIT(5)
#define  XDMA_CTRL_DS_DIRTY		BIT(6)
#define  XDMA_CTRL_DS_PCIE_REQ_SIZE	GENMASK(19, 17)
#define  XDMA_CTRL_DS_DATA_TIMEOUT	BIT(28)
#define  XDMA_CTRL_DS_CHECK_ID		BIT(29)
#define XDMA_STATUS			0x24
#define  XDMA_STATUS_US_COMP		BIT(4)
#define  XDMA_STATUS_DS_COMP		BIT(5)

enum {
	XDMA_IN_PRG,
	XDMA_UPSTREAM,
};

struct aspeed_xdma_cmd {
	u32 host_addr_lo;
	u32 host_addr_hi;
	u32 bmc_addr;
	u32 comm1_hi;
	u32 conf;
	u32 id;
	u32 resv0;
	u32 resv1;
};

struct aspeed_xdma_client;

struct aspeed_xdma {
	struct device *dev;
	void __iomem *base;
	struct regmap *scu;
	struct reset_control *reset;

	unsigned long flags;
	unsigned int cmd_idx;
	struct mutex list_lock;
	struct mutex start_lock;
	wait_queue_head_t wait;
	struct aspeed_xdma_client *current_client;

	u32 vga_phys;
	u32 vga_size;
	dma_addr_t vga_dma;
	void *cmdq;
	void *vga_virt;
	dma_addr_t cmdq_vga_phys;
	void *cmdq_vga_virt;
	struct gen_pool *vga_pool;

	char pcidev[4];
	struct miscdevice misc;
};

struct aspeed_xdma_client {
	struct aspeed_xdma *ctx;

	unsigned long flags;
	void *virt;
	dma_addr_t phys;
	u32 size;
};

static const u32 aspeed_xdma_bmc_pcie_conf = SCU_PCIE_CONF_BMC_EN |
	SCU_PCIE_CONF_BMC_EN_MSI | SCU_PCIE_CONF_BMC_EN_MCTP |
	SCU_PCIE_CONF_BMC_EN_IRQ | SCU_PCIE_CONF_BMC_EN_DMA |
	SCU_PCIE_CONF_RSVD;

static const u32 aspeed_xdma_vga_pcie_conf = SCU_PCIE_CONF_VGA_EN |
	SCU_PCIE_CONF_VGA_EN_MSI | SCU_PCIE_CONF_VGA_EN_MCTP |
	SCU_PCIE_CONF_VGA_EN_IRQ | SCU_PCIE_CONF_VGA_EN_DMA |
	SCU_PCIE_CONF_RSVD;

static char *_pcidev = "vga";
module_param_named(pcidev, _pcidev, charp, 0600);
MODULE_PARM_DESC(pcidev, "Default PCI device used by XDMA engine for DMA ops");

static void aspeed_scu_pcie_write(struct aspeed_xdma *ctx, u32 conf)
{
	u32 v = 0;

	regmap_write(ctx->scu, SCU_PCIE_CONF, conf);
	regmap_read(ctx->scu, SCU_PCIE_CONF, &v);

	dev_dbg(ctx->dev, "write scu pcie_conf[%08x]\n", v);
}

static u32 aspeed_xdma_reg_read(struct aspeed_xdma *ctx, u32 reg)
{
	u32 v = readl(ctx->base + reg);

	dev_dbg(ctx->dev, "read %02x[%08x]\n", reg, v);
	return v;
}

static void aspeed_xdma_reg_write(struct aspeed_xdma *ctx, u32 reg, u32 val)
{
	writel(val, ctx->base + reg);
	dev_dbg(ctx->dev, "write %02x[%08x]\n", reg, readl(ctx->base + reg));
}

static void aspeed_xdma_init_eng(struct aspeed_xdma *ctx)
{
	const u32 ctrl = XDMA_CTRL_US_COMP | XDMA_CTRL_DS_COMP |
		XDMA_CTRL_DS_DIRTY | FIELD_PREP(XDMA_CTRL_DS_PCIE_REQ_SIZE,
						XDMA_DS_PCIE_REQ_SIZE_256) |
		XDMA_CTRL_DS_DATA_TIMEOUT | XDMA_CTRL_DS_CHECK_ID;

	aspeed_xdma_reg_write(ctx, XDMA_BMC_CMD_QUEUE_ENDP,
			      XDMA_CMD_SIZE * XDMA_NUM_CMDS);
	aspeed_xdma_reg_write(ctx, XDMA_BMC_CMD_QUEUE_READP,
			      XDMA_BMC_CMD_QUEUE_READP_MAGIC);
	aspeed_xdma_reg_write(ctx, XDMA_BMC_CMD_QUEUE_WRITEP, 0);
	aspeed_xdma_reg_write(ctx, XDMA_CTRL, ctrl);

	aspeed_xdma_reg_write(ctx, XDMA_BMC_CMD_QUEUE_ADDR,
			      ctx->cmdq_vga_phys);

	ctx->cmd_idx = 0;
	ctx->flags = 0;
}

static void aspeed_xdma_reset(struct aspeed_xdma *ctx)
{
	reset_control_assert(ctx->reset);

	msleep(10);

	reset_control_deassert(ctx->reset);

	msleep(10);

	aspeed_xdma_init_eng(ctx);
}

static void aspeed_xdma_start(struct aspeed_xdma *ctx,
			      struct aspeed_xdma_op *op, u32 bmc_addr)
{
	u32 conf = XDMA_CMD_CONF_CHECK | XDMA_CMD_CONF_IRQ_BMC |
		XDMA_CMD_CONF_IRQ;
	unsigned int line_size = op->len / XDMA_BYTE_ALIGN;
	unsigned int num_lines = 1;
	unsigned int nidx = (ctx->cmd_idx + 1) % XDMA_NUM_CMDS;
	unsigned int pitch = 1;
	struct aspeed_xdma_cmd *cmd =
		&(((struct aspeed_xdma_cmd *)ctx->cmdq)[ctx->cmd_idx]);

	if (line_size > XDMA_MAX_LINE_SIZE) {
		unsigned int rem;
		unsigned int total;

		num_lines = line_size / XDMA_MAX_LINE_SIZE;
		total = XDMA_MAX_LINE_SIZE * num_lines;
		rem = line_size - total;
		line_size = XDMA_MAX_LINE_SIZE;
		pitch = line_size;

		if (rem) {
			unsigned int offs = total * XDMA_BYTE_ALIGN;
			u32 r_bmc_addr = bmc_addr + offs;
			u64 r_host_addr = op->host_addr + (u64)offs;
			struct aspeed_xdma_cmd *r_cmd =
				&(((struct aspeed_xdma_cmd *)ctx->cmdq)[nidx]);

			r_cmd->host_addr_lo =
				(u32)(r_host_addr & 0xFFFFFFFFULL);
			r_cmd->host_addr_hi = (u32)(r_host_addr >> 32ULL);
			r_cmd->bmc_addr = (r_bmc_addr & XDMA_CMD_BMC_ADDR) |
				XDMA_CMD_BMC_CHECK |
				(op->upstream ? XDMA_CMD_BMC_DIR_US : 0);
			r_cmd->conf = conf |
				FIELD_PREP(XDMA_CMD_CONF_LINE_SIZE, rem) |
				FIELD_PREP(XDMA_CMD_CONF_NUM_LINES, 1);
			r_cmd->comm1_hi =
				FIELD_PREP(XDMA_CMD_COMM1_HI_HOST_PITCH, 1) |
				FIELD_PREP(XDMA_CMD_COMM1_HI_BMC_PITCH, 1);

			/* do not trigger IRQ for first command */
			conf = XDMA_CMD_CONF_CHECK;

			nidx = (nidx + 1) % XDMA_NUM_CMDS;
		}

		/* undocumented formula to get required number of lines */
		num_lines = (num_lines * 2) - 1;
	}

	/* ctrl == 0 indicates engine hasn't started properly; restart it */
	if (!aspeed_xdma_reg_read(ctx, XDMA_CTRL))
		aspeed_xdma_reset(ctx);

	cmd->host_addr_lo = (u32)(op->host_addr & 0xFFFFFFFFULL);
	cmd->host_addr_hi = (u32)(op->host_addr >> 32ULL);
	cmd->bmc_addr = (bmc_addr & XDMA_CMD_BMC_ADDR) | XDMA_CMD_BMC_CHECK |
		(op->upstream ? XDMA_CMD_BMC_DIR_US : 0);
	cmd->conf = conf |
		FIELD_PREP(XDMA_CMD_CONF_LINE_SIZE, line_size) |
		FIELD_PREP(XDMA_CMD_CONF_NUM_LINES, num_lines);
	cmd->comm1_hi = FIELD_PREP(XDMA_CMD_COMM1_HI_HOST_PITCH, pitch) |
			FIELD_PREP(XDMA_CMD_COMM1_HI_BMC_PITCH, pitch);

	memcpy(ctx->cmdq_vga_virt, ctx->cmdq, XDMA_CMDQ_SIZE);

	if (op->upstream)
		set_bit(XDMA_UPSTREAM, &ctx->flags);
	else
		clear_bit(XDMA_UPSTREAM, &ctx->flags);

	set_bit(XDMA_IN_PRG, &ctx->flags);

	aspeed_xdma_reg_write(ctx, XDMA_BMC_CMD_QUEUE_WRITEP,
			      nidx * XDMA_CMD_SIZE);
	ctx->cmd_idx = nidx;
}

static void aspeed_xdma_done(struct aspeed_xdma *ctx)
{
	if (ctx->current_client) {
		clear_bit(XDMA_IN_PRG, &ctx->current_client->flags);

		ctx->current_client = NULL;
	}

	clear_bit(XDMA_IN_PRG, &ctx->flags);
	wake_up_interruptible_all(&ctx->wait);
}

static irqreturn_t aspeed_xdma_irq(int irq, void *arg)
{
	struct aspeed_xdma *ctx = arg;
	u32 status = aspeed_xdma_reg_read(ctx, XDMA_STATUS);

	if (status & XDMA_STATUS_US_COMP) {
		if (test_bit(XDMA_UPSTREAM, &ctx->flags))
			aspeed_xdma_done(ctx);
	}

	if (status & XDMA_STATUS_DS_COMP) {
		if (!test_bit(XDMA_UPSTREAM, &ctx->flags))
			aspeed_xdma_done(ctx);
	}

	aspeed_xdma_reg_write(ctx, XDMA_STATUS, status);

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

	if (len != sizeof(struct aspeed_xdma_op))
		return -EINVAL;

	rc = copy_from_user(&op, buf, len);
	if (rc)
		return rc;

	if (op.len > (ctx->vga_size - offs) || op.len < XDMA_BYTE_ALIGN)
		return -EINVAL;

	if (file->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&ctx->start_lock))
			return -EAGAIN;

		if (test_bit(XDMA_IN_PRG, &ctx->flags)) {
			mutex_unlock(&ctx->start_lock);
			return -EAGAIN;
		}
	} else {
		mutex_lock(&ctx->start_lock);

		rc = wait_event_interruptible(ctx->wait,
					      !test_bit(XDMA_IN_PRG,
							&ctx->flags));
		if (rc) {
			mutex_unlock(&ctx->start_lock);
			return -EINTR;
		}
	}

	ctx->current_client = client;
	set_bit(XDMA_IN_PRG, &client->flags);

	aspeed_xdma_start(ctx, &op, ctx->vga_phys + offs);

	mutex_unlock(&ctx->start_lock);

	if (!(file->f_flags & O_NONBLOCK)) {
		rc = wait_event_interruptible(ctx->wait,
					      !test_bit(XDMA_IN_PRG,
							&ctx->flags));
		if (rc)
			return -EINTR;
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
		if (test_bit(XDMA_IN_PRG, &client->flags))
			poll_wait(file, &ctx->wait, wait);

		if (!test_bit(XDMA_IN_PRG, &client->flags))
			mask |= EPOLLIN | EPOLLRDNORM;
	}

	if (req & (EPOLLOUT | EPOLLWRNORM)) {
		if (test_bit(XDMA_IN_PRG, &ctx->flags))
			poll_wait(file, &ctx->wait, wait);

		if (!test_bit(XDMA_IN_PRG, &ctx->flags))
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

	rc = dma_mmap_coherent(ctx->dev, vma, ctx->vga_virt, ctx->vga_dma,
			       ctx->vga_size);
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

static int aspeed_xdma_init_mem(struct aspeed_xdma *ctx, u32 conf)
{
	int rc;
	u32 scu_conf = 0;
	u32 mem_size = 0x20000000;
	const u32 mem_sizes[4] = { 0x8000000, 0x10000000, 0x20000000,
				   0x40000000 };
	const u32 vga_sizes[4] = { 0x800000, 0x1000000, 0x2000000, 0x4000000 };
	void __iomem *sdmc_base = ioremap(0x1e6e0000, 0x100);

	aspeed_scu_pcie_write(ctx, conf);

	regmap_read(ctx->scu, SCU_STRAP, &scu_conf);
	ctx->vga_size = vga_sizes[FIELD_GET(SCU_STRAP_VGA_MEM, scu_conf)];

	if (sdmc_base) {
		u32 sdmc = readl(sdmc_base + SDMC_CONF);
		u32 remap = readl(sdmc_base + SDMC_REMAP);

		remap |= SDMC_REMAP_MAGIC;
		writel(remap, sdmc_base + SDMC_REMAP);
		remap = readl(sdmc_base + SDMC_REMAP);

		mem_size = mem_sizes[sdmc & SDMC_CONF_MEM];
		iounmap(sdmc_base);
	}

	ctx->vga_phys = (mem_size - ctx->vga_size) + 0x80000000;

	ctx->cmdq = devm_kzalloc(ctx->dev, XDMA_CMDQ_SIZE, GFP_KERNEL);
	if (!ctx->cmdq) {
		dev_err(ctx->dev, "Failed to allocate command queue.\n");
		return -ENOMEM;
	}

	rc = dma_set_mask_and_coherent(ctx->dev, DMA_BIT_MASK(32));
	if (rc) {
		dev_err(ctx->dev, "Failed to set DMA mask: %d.\n", rc);
		return rc;
	}

	rc = dma_declare_coherent_memory(ctx->dev, ctx->vga_phys,
					 ctx->vga_phys, ctx->vga_size);
	if (rc) {
		dev_err(ctx->dev, "Failed to declare coherent memory: %d.\n",
			rc);
		return rc;
	}

	ctx->vga_virt = dma_alloc_coherent(ctx->dev, ctx->vga_size,
					   &ctx->vga_dma, GFP_KERNEL);
	if (!ctx->vga_virt) {
		dev_err(ctx->dev, "Failed to allocate DMA.\n");
		rc = -ENOMEM;
		goto err_dma;
	}

	rc = gen_pool_add_virt(ctx->vga_pool, (unsigned long)ctx->vga_virt,
			       ctx->vga_phys, ctx->vga_size, -1);
	if (rc) {
		dev_err(ctx->dev, "Failed to add memory to genalloc pool.\n");
		goto err_genalloc;
	}

	ctx->cmdq_vga_virt = gen_pool_dma_alloc(ctx->vga_pool, XDMA_CMDQ_SIZE,
						&ctx->cmdq_vga_phys);
	if (!ctx->cmdq_vga_virt) {
		dev_err(ctx->dev, "Failed to genalloc cmdq.\n");
		rc = -ENOMEM;
		goto err_genalloc;
	}

	dev_dbg(ctx->dev, "VGA mapped at phys[%08x], size[%08x].\n",
		ctx->vga_phys, ctx->vga_size);

	return 0;

err_dma:
	dma_release_declared_memory(ctx->dev);

err_genalloc:
	dma_free_coherent(ctx->dev, ctx->vga_size, ctx->vga_virt,
			  ctx->vga_dma);
	return rc;
}

static int aspeed_xdma_change_pcie_conf(struct aspeed_xdma *ctx, u32 conf)
{
	int rc;

	mutex_lock(&ctx->start_lock);
	rc = wait_event_interruptible_timeout(ctx->wait,
					      !test_bit(XDMA_IN_PRG,
							&ctx->flags),
					      msecs_to_jiffies(1000));
	if (rc < 0) {
		mutex_unlock(&ctx->start_lock);
		return -EINTR;
	}

	/* previous op didn't complete, wake up waiters anyway */
	if (!rc)
		wake_up_interruptible_all(&ctx->wait);

	reset_control_assert(ctx->reset);
	msleep(10);

	aspeed_scu_pcie_write(ctx, conf);
	msleep(10);

	reset_control_deassert(ctx->reset);
	msleep(10);

	aspeed_xdma_init_eng(ctx);

	mutex_unlock(&ctx->start_lock);

	return 0;
}

static int aspeed_xdma_pcidev_to_conf(struct aspeed_xdma *ctx,
				      const char *pcidev, u32 *conf)
{
	if (!strcasecmp(pcidev, "vga")) {
		*conf = aspeed_xdma_vga_pcie_conf;
		return 0;
	}

	if (!strcasecmp(pcidev, "bmc")) {
		*conf = aspeed_xdma_bmc_pcie_conf;
		return 0;
	}

	return -EINVAL;
}

static ssize_t aspeed_xdma_show_pcidev(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct aspeed_xdma *ctx = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%s", ctx->pcidev);
}

static ssize_t aspeed_xdma_store_pcidev(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	u32 conf;
	struct aspeed_xdma *ctx = dev_get_drvdata(dev);
	int rc = aspeed_xdma_pcidev_to_conf(ctx, buf, &conf);

	if (rc)
		return rc;

	rc = aspeed_xdma_change_pcie_conf(ctx, conf);
	if (rc)
		return rc;

	strcpy(ctx->pcidev, buf);
	return count;
}
static DEVICE_ATTR(pcidev, 0644, aspeed_xdma_show_pcidev,
		   aspeed_xdma_store_pcidev);

static int aspeed_xdma_probe(struct platform_device *pdev)
{
	int irq;
	int rc;
	u32 conf;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct aspeed_xdma *ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);

	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	platform_set_drvdata(pdev, ctx);
	init_waitqueue_head(&ctx->wait);
	mutex_init(&ctx->list_lock);
	mutex_init(&ctx->start_lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctx->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ctx->base)) {
		dev_err(dev, "Unable to ioremap registers.\n");
		return PTR_ERR(ctx->base);
	}

	irq = irq_of_parse_and_map(dev->of_node, 0);
	if (!irq) {
		dev_err(dev, "Unable to find IRQ.\n");
		return -ENODEV;
	}

	rc = devm_request_irq(dev, irq, aspeed_xdma_irq, IRQF_SHARED,
			      DEVICE_NAME, ctx);
	if (rc < 0) {
		dev_err(dev, "Unable to request IRQ %d.\n", irq);
		return rc;
	}

	ctx->scu = syscon_regmap_lookup_by_compatible("aspeed,ast2500-scu");
	if (IS_ERR(ctx->scu)) {
		dev_err(ctx->dev, "Unable to grab SCU regs.\n");
		return PTR_ERR(ctx->scu);
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

	msleep(10);

	if (aspeed_xdma_pcidev_to_conf(ctx, _pcidev, &conf)) {
		conf = aspeed_xdma_vga_pcie_conf;
		strcpy(ctx->pcidev, "vga");
	} else {
		strcpy(ctx->pcidev, _pcidev);
	}

	rc = aspeed_xdma_init_mem(ctx, conf);
	if (rc) {
		reset_control_assert(ctx->reset);
		return rc;
	}

	aspeed_xdma_init_eng(ctx);

	ctx->misc.minor = MISC_DYNAMIC_MINOR;
	ctx->misc.fops = &aspeed_xdma_fops;
	ctx->misc.name = "xdma";
	ctx->misc.parent = dev;
	rc = misc_register(&ctx->misc);
	if (rc) {
		dev_err(dev, "Unable to register xdma miscdevice.\n");

		gen_pool_free(ctx->vga_pool, (unsigned long)ctx->cmdq_vga_virt,
			      XDMA_CMDQ_SIZE);
		dma_free_coherent(dev, ctx->vga_size, ctx->vga_virt,
				  ctx->vga_dma);
		dma_release_declared_memory(dev);
		reset_control_assert(ctx->reset);
		return rc;
	}

	device_create_file(dev, &dev_attr_pcidev);

	return 0;
}

static int aspeed_xdma_remove(struct platform_device *pdev)
{
	struct aspeed_xdma *ctx = platform_get_drvdata(pdev);

	device_remove_file(ctx->dev, &dev_attr_pcidev);

	misc_deregister(&ctx->misc);
	gen_pool_free(ctx->vga_pool, (unsigned long)ctx->cmdq_vga_virt,
		      XDMA_CMDQ_SIZE);
	dma_free_coherent(ctx->dev, ctx->vga_size, ctx->vga_virt,
			  ctx->vga_dma);
	dma_release_declared_memory(ctx->dev);
	reset_control_assert(ctx->reset);

	return 0;
}

static const struct of_device_id aspeed_xdma_match[] = {
	{ .compatible = "aspeed,ast2500-xdma" },
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
