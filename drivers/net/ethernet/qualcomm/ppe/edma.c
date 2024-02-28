// SPDX-License-Identifier: GPL-2.0-only
 /* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
  */

 /* Qualcomm Ethernet DMA driver setup, HW configuration, clocks and
  * interrupt initializations.
  */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "edma.h"
#include "ppe_regs.h"

#define EDMA_IRQ_NAME_SIZE		32

/* Global EDMA context. */
struct edma_context *edma_ctx;

/* Priority to multi-queue mapping. */
static u8 edma_pri_map[PPE_QUEUE_INTER_PRI_NUM] = {
	0, 1, 2, 3, 4, 5, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7};

enum edma_clk_id {
	EDMA_CLK,
	EDMA_CFG_CLK,
	EDMA_CLK_MAX
};

static const char * const clock_name[EDMA_CLK_MAX] = {
	[EDMA_CLK] = "edma",
	[EDMA_CFG_CLK] = "edma-cfg",
};

/* Rx Fill ring info for IPQ9574. */
static struct edma_ring_info ipq9574_rxfill_ring_info = {
	.max_rings = 8,
	.ring_start = 4,
	.num_rings = 4,
};

/* Rx ring info for IPQ9574. */
static struct edma_ring_info ipq9574_rx_ring_info = {
	.max_rings = 24,
	.ring_start = 20,
	.num_rings = 4,
};

/* Tx ring info for IPQ9574. */
static struct edma_ring_info ipq9574_tx_ring_info = {
	.max_rings = 32,
	.ring_start = 8,
	.num_rings = 24,
};

/* Tx complete ring info for IPQ9574. */
static struct edma_ring_info ipq9574_txcmpl_ring_info = {
	.max_rings = 32,
	.ring_start = 8,
	.num_rings = 24,
};

/* HW info for IPQ9574. */
static struct edma_hw_info ipq9574_hw_info = {
	.rxfill = &ipq9574_rxfill_ring_info,
	.rx = &ipq9574_rx_ring_info,
	.tx = &ipq9574_tx_ring_info,
	.txcmpl = &ipq9574_txcmpl_ring_info,
	.max_ports = 6,
	.napi_budget_rx = 128,
	.napi_budget_tx = 512,
};

static int edma_clock_set_and_enable(struct device *dev,
				     const char *id, unsigned long rate)
{
	struct device_node *edma_np;
	struct clk *clk = NULL;
	int ret;

	edma_np = of_get_child_by_name(dev->of_node, "edma");

	clk = devm_get_clk_from_child(dev, edma_np, id);
	if (IS_ERR(clk)) {
		dev_err(dev, "clk %s get failed\n", id);
		of_node_put(edma_np);
		return PTR_ERR(clk);
	}

	ret = clk_set_rate(clk, rate);
	if (ret) {
		dev_err(dev, "set %lu rate for %s failed\n", rate, id);
		of_node_put(edma_np);
		return ret;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(dev, "clk %s enable failed\n", id);
		of_node_put(edma_np);
		return ret;
	}

	of_node_put(edma_np);

	dev_dbg(dev, "set %lu rate for %s\n", rate, id);

	return 0;
}

static int edma_clock_init(void)
{
	struct ppe_device *ppe_dev = edma_ctx->ppe_dev;
	struct device *dev = ppe_dev->dev;
	unsigned long ppe_rate;
	int ret;

	ppe_rate = ppe_dev->clk_rate;

	ret = edma_clock_set_and_enable(dev, clock_name[EDMA_CLK],
					ppe_rate);
	if (ret)
		return ret;

	ret = edma_clock_set_and_enable(dev, clock_name[EDMA_CFG_CLK],
					ppe_rate);
	if (ret)
		return ret;

	return 0;
}

/**
 * edma_configure_ucast_prio_map_tbl - Configure unicast priority map table.
 *
 * Map int_priority values to priority class and initialize
 * unicast priority map table for default profile_id.
 */
static int edma_configure_ucast_prio_map_tbl(void)
{
	u8 pri_class, int_pri;
	int ret = 0;

	/* Set the priority class value for every possible priority. */
	for (int_pri = 0; int_pri < PPE_QUEUE_INTER_PRI_NUM; int_pri++) {
		pri_class = edma_pri_map[int_pri];

		/* Priority offset should be less than maximum supported
		 * queue priority.
		 */
		if (pri_class > EDMA_PRI_MAX_PER_CORE - 1) {
			pr_err("Configured incorrect priority offset: %d\n",
			       pri_class);
			return -EINVAL;
		}

		ret = ppe_edma_queue_offset_config(edma_ctx->ppe_dev,
						   PPE_QUEUE_CLASS_PRIORITY, int_pri, pri_class);

		if (ret) {
			pr_err("Failed with error: %d to set queue priority class for int_pri: %d for profile_id: %d\n",
			       ret, int_pri, 0);
			return ret;
		}

		pr_debug("profile_id: %d, int_priority: %d, pri_class: %d\n",
			 0, int_pri, pri_class);
	}

	return ret;
}

static int edma_irq_init(void)
{
	struct edma_hw_info *hw_info = edma_ctx->hw_info;
	struct edma_ring_info *txcmpl = hw_info->txcmpl;
	struct ppe_device *ppe_dev = edma_ctx->ppe_dev;
	struct edma_ring_info *rx = hw_info->rx;
	char edma_irq_name[EDMA_IRQ_NAME_SIZE];
	struct device *dev = ppe_dev->dev;
	struct platform_device *pdev;
	struct device_node *edma_np;
	u32 i;

	pdev = to_platform_device(dev);
	edma_np = of_get_child_by_name(dev->of_node, "edma");
	edma_ctx->intr_info.intr_txcmpl = kzalloc((sizeof(*edma_ctx->intr_info.intr_txcmpl) *
						  txcmpl->num_rings), GFP_KERNEL);
	if (!edma_ctx->intr_info.intr_txcmpl) {
		of_node_put(edma_np);
		return -ENOMEM;
	}

	/* Get TXCMPL rings IRQ numbers. */
	for (i = 0; i < txcmpl->num_rings; i++) {
		snprintf(edma_irq_name, sizeof(edma_irq_name), "edma_txcmpl_%d",
			 txcmpl->ring_start + i);
		edma_ctx->intr_info.intr_txcmpl[i] = of_irq_get_byname(edma_np, edma_irq_name);
		if (edma_ctx->intr_info.intr_txcmpl[i] < 0) {
			dev_err(dev, "%s: txcmpl_info.intr[%u] irq get failed\n",
				edma_np->name, i);
			of_node_put(edma_np);
			kfree(edma_ctx->intr_info.intr_txcmpl);
			return edma_ctx->intr_info.intr_txcmpl[i];
		}

		dev_dbg(dev, "%s: intr_info.intr_txcmpl[%u] = %u\n",
			edma_np->name, i, edma_ctx->intr_info.intr_txcmpl[i]);
	}

	edma_ctx->intr_info.intr_rx = kzalloc((sizeof(*edma_ctx->intr_info.intr_rx) *
					      rx->num_rings), GFP_KERNEL);
	if (!edma_ctx->intr_info.intr_rx) {
		of_node_put(edma_np);
		kfree(edma_ctx->intr_info.intr_txcmpl);
		return -ENOMEM;
	}

	/* Get RXDESC rings IRQ numbers. */
	for (i = 0; i < rx->num_rings; i++) {
		snprintf(edma_irq_name, sizeof(edma_irq_name), "edma_rxdesc_%d",
			 rx->ring_start + i);
		edma_ctx->intr_info.intr_rx[i] = of_irq_get_byname(edma_np, edma_irq_name);
		if (edma_ctx->intr_info.intr_rx[i] < 0) {
			dev_err(dev, "%s: rx_queue_map_info.intr[%u] irq get failed\n",
				edma_np->name, i);
			of_node_put(edma_np);
			kfree(edma_ctx->intr_info.intr_rx);
			kfree(edma_ctx->intr_info.intr_txcmpl);
			return edma_ctx->intr_info.intr_rx[i];
		}

		dev_dbg(dev, "%s: intr_info.intr_rx[%u] = %u\n",
			edma_np->name, i, edma_ctx->intr_info.intr_rx[i]);
	}

	/* Get misc IRQ number. */
	edma_ctx->intr_info.intr_misc = of_irq_get_byname(edma_np, "edma_misc");
	if (edma_ctx->intr_info.intr_misc < 0) {
		dev_err(dev, "%s: misc_intr irq get failed\n", edma_np->name);
		of_node_put(edma_np);
		kfree(edma_ctx->intr_info.intr_rx);
		kfree(edma_ctx->intr_info.intr_txcmpl);
		return edma_ctx->intr_info.intr_misc;
	}

	of_node_put(edma_np);

	dev_dbg(dev, "%s: misc IRQ:%u\n", edma_np->name,
		edma_ctx->intr_info.intr_misc);

	return 0;
}

static int edma_hw_reset(void)
{
	struct ppe_device *ppe_dev = edma_ctx->ppe_dev;
	struct device *dev = ppe_dev->dev;
	struct reset_control *edma_hw_rst;
	struct device_node *edma_np;
	const char *reset_string;
	u32 count, i;
	int ret;

	/* Count and parse reset names from DTSI. */
	edma_np = of_get_child_by_name(dev->of_node, "edma");
	count = of_property_count_strings(edma_np, "reset-names");
	if (count < 0) {
		dev_err(dev, "EDMA reset entry not found\n");
		of_node_put(edma_np);
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		ret = of_property_read_string_index(edma_np, "reset-names",
						    i, &reset_string);
		if (ret) {
			dev_err(dev, "Error reading reset-names");
			of_node_put(edma_np);
			return -EINVAL;
		}

		edma_hw_rst = of_reset_control_get_exclusive(edma_np, reset_string);
		if (IS_ERR(edma_hw_rst)) {
			of_node_put(edma_np);
			return PTR_ERR(edma_hw_rst);
		}

		/* 100ms delay is required by hardware to reset EDMA. */
		reset_control_assert(edma_hw_rst);
		fsleep(100);

		reset_control_deassert(edma_hw_rst);
		fsleep(100);

		reset_control_put(edma_hw_rst);
		dev_dbg(dev, "EDMA HW reset, i:%d reset_string:%s\n", i, reset_string);
	}

	of_node_put(edma_np);

	return 0;
}

static int edma_hw_configure(void)
{
	struct edma_hw_info *hw_info = edma_ctx->hw_info;
	struct ppe_device *ppe_dev = edma_ctx->ppe_dev;
	struct regmap *regmap = ppe_dev->regmap;
	u32 data, reg;
	int ret;

	reg = EDMA_BASE_OFFSET + EDMA_REG_MAS_CTRL_ADDR;
	ret = regmap_read(regmap, reg, &data);
	if (ret)
		return ret;

	pr_debug("EDMA ver %d hw init\n", data);

	/* Setup private data structure. */
	edma_ctx->intr_info.intr_mask_rx = EDMA_RXDESC_INT_MASK_PKT_INT;
	edma_ctx->intr_info.intr_mask_txcmpl = EDMA_TX_INT_MASK_PKT_INT;

	/* Reset EDMA. */
	ret = edma_hw_reset();
	if (ret) {
		pr_err("Error in resetting the hardware. ret: %d\n", ret);
		return ret;
	}

	/* Allocate memory for netdevices. */
	edma_ctx->netdev_arr = kzalloc((sizeof(**edma_ctx->netdev_arr) *
					 hw_info->max_ports),
					 GFP_KERNEL);
	if (!edma_ctx->netdev_arr)
		return -ENOMEM;

	/* Configure DMA request priority, DMA read burst length,
	 * and AXI write size.
	 */
	data = FIELD_PREP(EDMA_DMAR_BURST_LEN_MASK, EDMA_BURST_LEN_ENABLE);
	data |= FIELD_PREP(EDMA_DMAR_REQ_PRI_MASK, 0);
	data |= FIELD_PREP(EDMA_DMAR_TXDATA_OUTSTANDING_NUM_MASK, 31);
	data |= FIELD_PREP(EDMA_DMAR_TXDESC_OUTSTANDING_NUM_MASK, 7);
	data |= FIELD_PREP(EDMA_DMAR_RXFILL_OUTSTANDING_NUM_MASK, 7);

	reg = EDMA_BASE_OFFSET + EDMA_REG_DMAR_CTRL_ADDR;
	ret = regmap_write(regmap, reg, data);
	if (ret)
		return ret;

	/* Configure Tx Timeout Threshold. */
	data = EDMA_TX_TIMEOUT_THRESH_VAL;

	reg = EDMA_BASE_OFFSET + EDMA_REG_TX_TIMEOUT_THRESH_ADDR;
	ret = regmap_write(regmap, reg, data);
	if (ret)
		return ret;

	/* Set Miscellaneous error mask. */
	data = EDMA_MISC_AXI_RD_ERR_MASK |
		EDMA_MISC_AXI_WR_ERR_MASK |
		EDMA_MISC_RX_DESC_FIFO_FULL_MASK |
		EDMA_MISC_RX_ERR_BUF_SIZE_MASK |
		EDMA_MISC_TX_SRAM_FULL_MASK |
		EDMA_MISC_TX_CMPL_BUF_FULL_MASK |
		EDMA_MISC_DATA_LEN_ERR_MASK;
	data |= EDMA_MISC_TX_TIMEOUT_MASK;
	edma_ctx->intr_info.intr_mask_misc = data;

	/* Global EDMA enable and padding enable. */
	data = EDMA_PORT_PAD_EN | EDMA_PORT_EDMA_EN;

	reg = EDMA_BASE_OFFSET + EDMA_REG_PORT_CTRL_ADDR;
	ret = regmap_write(regmap, reg, data);
	if (ret)
		return ret;

	/* Initialize unicast priority map table. */
	ret = (int)edma_configure_ucast_prio_map_tbl();
	if (ret) {
		pr_err("Failed to initialize unicast priority map table: %d\n",
		       ret);
		kfree(edma_ctx->netdev_arr);
		return ret;
	}

	return 0;
}

/**
 * edma_destroy - EDMA Destroy.
 * @ppe_dev: PPE device
 *
 * Free the memory allocated during setup.
 */
void edma_destroy(struct ppe_device *ppe_dev)
{
	kfree(edma_ctx->intr_info.intr_rx);
	kfree(edma_ctx->intr_info.intr_txcmpl);
	kfree(edma_ctx->netdev_arr);
}

/**
 * edma_setup - EDMA Setup.
 * @ppe_dev: PPE device
 *
 * Configure Ethernet global ctx, clocks, hardware and interrupts.
 *
 * Return 0 on success, negative error code on failure.
 */
int edma_setup(struct ppe_device *ppe_dev)
{
	struct device *dev = ppe_dev->dev;
	int ret;

	edma_ctx = devm_kzalloc(dev, sizeof(*edma_ctx), GFP_KERNEL);
	if (!edma_ctx)
		return -ENOMEM;

	edma_ctx->hw_info = &ipq9574_hw_info;
	edma_ctx->ppe_dev = ppe_dev;

	/* Configure the EDMA common clocks. */
	ret = edma_clock_init();
	if (ret) {
		dev_err(dev, "Error in configuring the EDMA clocks\n");
		return ret;
	}

	dev_dbg(dev, "QCOM EDMA common clocks are configured\n");

	ret = edma_hw_configure();
	if (ret) {
		dev_err(dev, "Error in edma configuration\n");
		return ret;
	}

	ret = edma_irq_init();
	if (ret) {
		dev_err(dev, "Error in irq initialization\n");
		return ret;
	}

	dev_info(dev, "EDMA configuration successful\n");

	return 0;
}

/**
 * ppe_edma_queue_offset_config - Configure queue offset for EDMA interface
 * @ppe_dev: PPE device
 * @class: The class to configure queue offset
 * @index: Class index, internal priority or hash value
 * @queue_offset: Queue offset value
 *
 * PPE EDMA queue offset is configured based on the PPE internal priority or
 * RSS hash value, the profile ID is fixed to 0 for EDMA interface.
 *
 * Return 0 on success, negative error code on failure.
 */
int ppe_edma_queue_offset_config(struct ppe_device *ppe_dev,
				 enum ppe_queue_class_type class,
				 int index, int queue_offset)
{
	if (class == PPE_QUEUE_CLASS_PRIORITY)
		return ppe_queue_ucast_offset_pri_set(ppe_dev, 0,
						      index, queue_offset);

	return ppe_queue_ucast_offset_hash_set(ppe_dev, 0,
					       index, queue_offset);
}
