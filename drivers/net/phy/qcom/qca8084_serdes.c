// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/dev_printk.h>
#include <linux/mdio.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/property.h>
#include <linux/reset.h>

#include "qca8084_serdes.h"

/* XPCS includes 4 channels, each channel has the different MMD ID for
 * configuring auto-negotiation complete interrupt, mii-4bit, auto-
 * negotiation capabilities and TX configuration for the connected PHY.
 *
 * MMD31 is for channel 0;
 * MMD26 is for channel 1;
 * MMD27 is for channel 2;
 * MMD28 is for channel 3;
 */
#define QCA8084_CHANNEL_MAX			4

enum pcs_clk_id {
	PCS_CLK,
	PCS_RX_ROOT_CLK,
	PCS_TX_ROOT_CLK,
	PCS_CLK_MAX
};

enum xpcs_clk_id {
	XPCS_XGMII_RX_CLK,
	XPCS_XGMII_TX_CLK,
	XPCS_RX_CLK,
	XPCS_TX_CLK,
	XPCS_PORT_RX_CLK,
	XPCS_PORT_TX_CLK,
	XPCS_RX_SRC_CLK,
	XPCS_TX_SRC_CLK,
	XPCS_CLK_MAX
};

struct qca8084_xpcs_channel_priv {
	int ch_id;
	struct reset_control *rstcs;
	struct clk *clks[XPCS_CLK_MAX];
};

struct qca8084_pcs_data {
	struct reset_control *rstc;
	struct clk *clks[PCS_CLK_MAX];
};

struct qca8084_xpcs_data {
	struct reset_control *rstc;
	struct qca8084_xpcs_channel_priv xpcs_ch[QCA8084_CHANNEL_MAX];
};

static const char *const xpcs_clock_names[XPCS_CLK_MAX] = {
	[XPCS_XGMII_RX_CLK] =	"xgmii_rx",
	[XPCS_XGMII_TX_CLK] =	"xgmii_tx",
	[XPCS_RX_CLK] =		"xpcs_rx",
	[XPCS_TX_CLK] =		"xpcs_tx",
	[XPCS_PORT_RX_CLK] =	"port_rx",
	[XPCS_PORT_TX_CLK] =	"port_tx",
	[XPCS_RX_SRC_CLK] =	"rx_src",
	[XPCS_TX_SRC_CLK] =	"tx_src",
};

static const char *const pcs_clock_names[PCS_CLK_MAX] = {
	[PCS_CLK] =		"pcs",
	[PCS_RX_ROOT_CLK] =	"pcs_rx_root",
	[PCS_TX_ROOT_CLK] =	"pcs_tx_root",
};

struct mdio_device *qca8084_package_pcs_probe(struct device_node *pcs_np)
{
	struct qca8084_pcs_data *pcs_data;
	struct mdio_device *mdiodev;
	struct reset_control *rstc;
	struct device *dev;
	struct clk *clk;
	int i;

	mdiodev = fwnode_mdio_find_device(of_fwnode_handle(pcs_np));
	if (!mdiodev)
		return ERR_PTR(-EPROBE_DEFER);

	dev = &mdiodev->dev;
	pcs_data = devm_kzalloc(dev, sizeof(*pcs_data), GFP_KERNEL);
	if (!pcs_data) {
		dev_err(dev, "Allocate PCS data failed\n");
		return ERR_PTR(-ENOMEM);
	}

	rstc = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(rstc)) {
		dev_err(dev, "Get PCS reset failed\n");
		return ERR_CAST(rstc);
	}

	pcs_data->rstc = rstc;

	for (i = 0; i < ARRAY_SIZE(pcs_clock_names); i++) {
		clk = devm_clk_get(dev, pcs_clock_names[i]);
		if (IS_ERR(clk)) {
			dev_err(dev, "Failed to get the PCS clock ID %s\n",
				pcs_clock_names[i]);
			return ERR_CAST(clk);
		}
		pcs_data->clks[i] = clk;
	}

	mdiodev_set_drvdata(mdiodev, pcs_data);

	return mdiodev;
}

struct mdio_device *qca8084_package_xpcs_probe(struct device_node *xpcs_np)
{
	struct qca8084_xpcs_data *xpcs_data;
	struct mdio_device *mdiodev;
	struct reset_control *rstc;
	struct device_node *child;
	struct device *dev;
	struct clk *clk;
	int i, j, node;

	mdiodev = fwnode_mdio_find_device(of_fwnode_handle(xpcs_np));
	if (!mdiodev)
		return ERR_PTR(-EPROBE_DEFER);

	dev = &mdiodev->dev;

	xpcs_data = devm_kzalloc(dev, sizeof(*xpcs_data), GFP_KERNEL);
	if (!xpcs_data) {
		dev_err(dev, "Allocate XPCS data failed\n");
		return ERR_PTR(-ENOMEM);
	}

	rstc = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(rstc)) {
		dev_err(dev, "Get XPCS reset failed\n");
		return ERR_CAST(rstc);
	}

	xpcs_data->rstc = rstc;

	/* Sanity check the number of channel sub nodes */
	node = of_get_available_child_count(xpcs_np);
	if (node != QCA8084_CHANNEL_MAX)
		return ERR_PTR(-EINVAL);

	node = 0;
	for_each_available_child_of_node(xpcs_np, child) {
		struct qca8084_xpcs_channel_priv *ch_data;
		u32 channel;

		/* The subnode name must be 'channel'. */
		if (!(of_node_name_eq(child, "channel")))
			continue;

		if (of_property_read_u32(child, "reg", &channel)) {
			dev_err(dev, "%s: Failed to get reg\n",
				child->full_name);

			mdiodev = ERR_PTR(-EINVAL);
			goto put_ch_clk_rst;
		}

		if (channel >= QCA8084_CHANNEL_MAX) {
			dev_err(dev, "%s: Invalid reg %d\n",
				child->full_name, channel);

			mdiodev = ERR_PTR(-EINVAL);
			goto put_ch_clk_rst;
		}

		ch_data = &xpcs_data->xpcs_ch[node];
		ch_data->ch_id = channel;

		ch_data->rstcs = of_reset_control_array_get_exclusive(child);
		if (IS_ERR(ch_data->rstcs)) {
			dev_err(dev, "%s: Failed to get reset\n",
				child->full_name);

			mdiodev = ERR_CAST(ch_data->rstcs);
			goto put_ch_clk_rst;
		}

		for (j = 0; j < ARRAY_SIZE(xpcs_clock_names); j++) {
			clk = of_clk_get_by_name(child, xpcs_clock_names[j]);
			if (IS_ERR(clk)) {
				dev_err(dev, "Failed to get the clock ID %s\n",
					xpcs_clock_names[j]);
				mdiodev = ERR_CAST(clk);
				goto put_ch_child;
			}
			ch_data->clks[j] = clk;
		}

		node++;
	}

	mdiodev_set_drvdata(mdiodev, xpcs_data);

	return mdiodev;

put_ch_child:
	node++;

put_ch_clk_rst:
	for (i = 0; i < node; i++) {
		j--;
		while (j >= 0) {
			clk_put(xpcs_data->xpcs_ch[i].clks[j]);
			j--;
		}

		j = ARRAY_SIZE(xpcs_clock_names);
	}

	for (i = 0; i < node; i++)
		reset_control_put(xpcs_data->xpcs_ch[i].rstcs);

	of_node_put(child);

	return mdiodev;
}

void qca8084_package_xpcs_and_pcs_remove(struct mdio_device *xpcs_mdiodev,
					 struct mdio_device *pcs_mdiodev)
{
	struct qca8084_xpcs_data *xpcs_data = mdiodev_get_drvdata(xpcs_mdiodev);
	int i, j;

	for (i = 0; i < ARRAY_SIZE(xpcs_data->xpcs_ch); i++) {
		reset_control_put(xpcs_data->xpcs_ch[i].rstcs);

		for (j = 0; j < ARRAY_SIZE(xpcs_data->xpcs_ch[i].clks); j++)
			clk_put(xpcs_data->xpcs_ch[i].clks[j]);
	}

	mdio_device_put(xpcs_mdiodev);
	mdio_device_put(pcs_mdiodev);
}
