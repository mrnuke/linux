// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* PPE Port MAC initialization and PPE port MAC functions. */

#include <linux/clk.h>
#include <linux/of_net.h>
#include <linux/pcs/pcs-qcom-ipq-uniphy.h>
#include <linux/phylink.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/rtnetlink.h>

#include "ppe.h"
#include "ppe_port.h"
#include "ppe_regs.h"

/* PPE MAC max frame size which including 4bytes FCS */
#define PPE_PORT_MAC_MAX_FRAME_SIZE		0x3000

/* PPE BM port start for PPE MAC ports */
#define PPE_BM_PORT_MAC_START			7

/* PPE port clock and reset name */
static const char * const ppe_port_clk_rst_name[] = {
	[PPE_PORT_CLK_RST_MAC] = "port_mac",
	[PPE_PORT_CLK_RST_RX] = "port_rx",
	[PPE_PORT_CLK_RST_TX] = "port_tx",
};

/* PPE port and MAC reset */
static int ppe_port_mac_reset(struct ppe_port *ppe_port)
{
	struct ppe_device *ppe_dev = ppe_port->ppe_dev;
	int ret;

	ret = reset_control_assert(ppe_port->rstcs[PPE_PORT_CLK_RST_MAC]);
	if (ret)
		goto error;

	ret = reset_control_assert(ppe_port->rstcs[PPE_PORT_CLK_RST_RX]);
	if (ret)
		goto error;

	ret = reset_control_assert(ppe_port->rstcs[PPE_PORT_CLK_RST_TX]);
	if (ret)
		goto error;

	/* 150ms delay is required by hardware to reset PPE port and MAC */
	msleep(150);

	ret = reset_control_deassert(ppe_port->rstcs[PPE_PORT_CLK_RST_MAC]);
	if (ret)
		goto error;

	ret = reset_control_deassert(ppe_port->rstcs[PPE_PORT_CLK_RST_RX]);
	if (ret)
		goto error;

	ret = reset_control_deassert(ppe_port->rstcs[PPE_PORT_CLK_RST_TX]);
	if (ret)
		goto error;

	return ret;

error:
	dev_err(ppe_dev->dev, "%s: port %d reset fail %d\n",
		__func__, ppe_port->port_id, ret);
	return ret;
}

/* PPE port MAC configuration for phylink */
static void ppe_port_mac_config(struct phylink_config *config,
				unsigned int mode,
				const struct phylink_link_state *state)
{
	struct ppe_port *ppe_port = container_of(config, struct ppe_port,
						 phylink_config);
	struct ppe_device *ppe_dev = ppe_port->ppe_dev;
	int port = ppe_port->port_id;
	enum ppe_mac_type mac_type;
	u32 val, mask;
	int ret;

	switch (state->interface) {
	case PHY_INTERFACE_MODE_2500BASEX:
	case PHY_INTERFACE_MODE_USXGMII:
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_10G_QXGMII:
		mac_type = PPE_MAC_TYPE_XGMAC;
		break;
	case PHY_INTERFACE_MODE_QSGMII:
	case PHY_INTERFACE_MODE_PSGMII:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
		mac_type = PPE_MAC_TYPE_GMAC;
		break;
	default:
		dev_err(ppe_dev->dev, "%s: Unsupport interface %s\n",
			__func__, phy_modes(state->interface));
		return;
	}

	/* Reset Port MAC for GMAC */
	if (mac_type == PPE_MAC_TYPE_GMAC) {
		ret = ppe_port_mac_reset(ppe_port);
		if (ret)
			goto err_mac_config;
	}

	/* Port mux to select GMAC or XGMAC */
	mask = PPE_PORT_SEL_XGMAC(port);
	val = mac_type == PPE_MAC_TYPE_GMAC ? 0 : mask;
	ret = regmap_update_bits(ppe_dev->regmap,
				 PPE_PORT_MUX_CTRL_ADDR,
				 mask, val);
	if (ret)
		goto err_mac_config;

	ppe_port->mac_type = mac_type;

	return;

err_mac_config:
	dev_err(ppe_dev->dev, "%s: port %d MAC config fail %d\n",
		__func__, port, ret);
}

/* PPE port GMAC link up configuration */
static int ppe_port_gmac_link_up(struct ppe_port *ppe_port, int speed,
				 int duplex, bool tx_pause, bool rx_pause)
{
	struct ppe_device *ppe_dev = ppe_port->ppe_dev;
	int ret, port = ppe_port->port_id;
	u32 reg, val;

	/* Set GMAC speed */
	switch (speed) {
	case SPEED_1000:
		val = GMAC_SPEED_1000;
		break;
	case SPEED_100:
		val = GMAC_SPEED_100;
		break;
	case SPEED_10:
		val = GMAC_SPEED_10;
		break;
	default:
		dev_err(ppe_dev->dev, "%s: Invalid GMAC speed %s\n",
			__func__, phy_speed_to_str(speed));
		return -EINVAL;
	}

	reg = PPE_PORT_GMAC_ADDR(port);
	ret = regmap_update_bits(ppe_dev->regmap, reg + GMAC_SPEED_ADDR,
				 GMAC_SPEED_M, val);
	if (ret)
		return ret;

	/* Set duplex, flow control and enable GMAC */
	val = GMAC_TRXEN;
	if (duplex == DUPLEX_FULL)
		val |= GMAC_DUPLEX_FULL;
	if (tx_pause)
		val |= GMAC_TXFCEN;
	if (rx_pause)
		val |= GMAC_RXFCEN;

	ret = regmap_update_bits(ppe_dev->regmap, reg + GMAC_ENABLE_ADDR,
				 GMAC_ENABLE_ALL, val);

	return ret;
}

/* PPE port XGMAC link up configuration */
static int ppe_port_xgmac_link_up(struct ppe_port *ppe_port,
				  phy_interface_t interface,
				  int speed, int duplex,
				  bool tx_pause, bool rx_pause)
{
	struct ppe_device *ppe_dev = ppe_port->ppe_dev;
	int ret, port = ppe_port->port_id;
	u32 reg, val;

	/* Set XGMAC TX speed and enable TX */
	switch (speed) {
	case SPEED_10000:
		if (interface == PHY_INTERFACE_MODE_USXGMII)
			val = XGMAC_SPEED_10000_USXGMII;
		else
			val = XGMAC_SPEED_10000;
		break;
	case SPEED_5000:
		val = XGMAC_SPEED_5000;
		break;
	case SPEED_2500:
		if (interface == PHY_INTERFACE_MODE_USXGMII ||
		    interface == PHY_INTERFACE_MODE_10G_QXGMII)
			val = XGMAC_SPEED_2500_USXGMII;
		else
			val = XGMAC_SPEED_2500;
		break;
	case SPEED_1000:
		val = XGMAC_SPEED_1000;
		break;
	case SPEED_100:
		val = XGMAC_SPEED_100;
		break;
	case SPEED_10:
		val = XGMAC_SPEED_10;
		break;
	default:
		dev_err(ppe_dev->dev, "%s: Invalid XGMAC speed %s\n",
			__func__, phy_speed_to_str(speed));
		return -EINVAL;
	}

	reg = PPE_PORT_XGMAC_ADDR(port);
	val |= XGMAC_TXEN;
	ret = regmap_update_bits(ppe_dev->regmap, reg + XGMAC_TX_CONFIG_ADDR,
				 XGMAC_SPEED_M | XGMAC_TXEN, val);
	if (ret)
		return ret;

	/* Set XGMAC TX flow control */
	val = FIELD_PREP(XGMAC_PAUSE_TIME_M, FIELD_MAX(XGMAC_PAUSE_TIME_M));
	val |= tx_pause ? XGMAC_TXFCEN : 0;
	ret = regmap_update_bits(ppe_dev->regmap, reg + XGMAC_TX_FLOW_CTRL_ADDR,
				 XGMAC_PAUSE_TIME_M | XGMAC_TXFCEN, val);
	if (ret)
		return ret;

	/* Set XGMAC RX flow control */
	val = rx_pause ? XGMAC_RXFCEN : 0;
	ret = regmap_update_bits(ppe_dev->regmap, reg + XGMAC_RX_FLOW_CTRL_ADDR,
				 XGMAC_RXFCEN, val);
	if (ret)
		return ret;

	/* Enable XGMAC RX*/
	ret = regmap_update_bits(ppe_dev->regmap, reg + XGMAC_RX_CONFIG_ADDR,
				 XGMAC_RXEN, XGMAC_RXEN);

	return ret;
}

/* PPE port MAC link up configuration for phylink */
static void ppe_port_mac_link_up(struct phylink_config *config,
				 struct phy_device *phy,
				 unsigned int mode,
				 phy_interface_t interface,
				 int speed, int duplex,
				 bool tx_pause, bool rx_pause)
{
	struct ppe_port *ppe_port = container_of(config, struct ppe_port,
						 phylink_config);
	enum ppe_mac_type mac_type = ppe_port->mac_type;
	struct ppe_device *ppe_dev = ppe_port->ppe_dev;
	int ret, port = ppe_port->port_id;
	u32 reg, val;

	if (mac_type == PPE_MAC_TYPE_GMAC)
		ret = ppe_port_gmac_link_up(ppe_port,
					    speed, duplex, tx_pause, rx_pause);
	else
		ret = ppe_port_xgmac_link_up(ppe_port, interface,
					     speed, duplex, tx_pause, rx_pause);
	if (ret)
		goto err_port_mac_link_up;

	/* Set PPE port BM flow control */
	reg = PPE_BM_PORT_FC_MODE_ADDR +
		PPE_BM_PORT_FC_MODE_INC * (port + PPE_BM_PORT_MAC_START);
	val = tx_pause ? PPE_BM_PORT_FC_MODE_EN : 0;
	ret = regmap_update_bits(ppe_dev->regmap, reg,
				 PPE_BM_PORT_FC_MODE_EN, val);
	if (ret)
		goto err_port_mac_link_up;

	/* Enable PPE port TX */
	reg = PPE_PORT_BRIDGE_CTRL_ADDR + PPE_PORT_BRIDGE_CTRL_INC * port;
	ret = regmap_update_bits(ppe_dev->regmap, reg,
				 PPE_PORT_BRIDGE_TXMAC_EN,
				 PPE_PORT_BRIDGE_TXMAC_EN);
	if (ret)
		goto err_port_mac_link_up;

	return;

err_port_mac_link_up:
	dev_err(ppe_dev->dev, "%s: port %d link up fail %d\n",
		__func__, port, ret);
}

/* PPE port MAC link down configuration for phylink */
static void ppe_port_mac_link_down(struct phylink_config *config,
				   unsigned int mode,
				   phy_interface_t interface)
{
	struct ppe_port *ppe_port = container_of(config, struct ppe_port,
						 phylink_config);
	enum ppe_mac_type mac_type = ppe_port->mac_type;
	struct ppe_device *ppe_dev = ppe_port->ppe_dev;
	int ret, port = ppe_port->port_id;
	u32 reg;

	/* Disable PPE port TX */
	reg = PPE_PORT_BRIDGE_CTRL_ADDR + PPE_PORT_BRIDGE_CTRL_INC * port;
	ret = regmap_update_bits(ppe_dev->regmap, reg,
				 PPE_PORT_BRIDGE_TXMAC_EN, 0);
	if (ret)
		goto err_port_mac_link_down;

	/* Disable PPE MAC */
	if (mac_type == PPE_MAC_TYPE_GMAC) {
		reg = PPE_PORT_GMAC_ADDR(port) + GMAC_ENABLE_ADDR;
		ret = regmap_update_bits(ppe_dev->regmap, reg, GMAC_TRXEN, 0);
		if (ret)
			goto err_port_mac_link_down;
	} else {
		reg = PPE_PORT_XGMAC_ADDR(port);
		ret = regmap_update_bits(ppe_dev->regmap,
					 reg + XGMAC_RX_CONFIG_ADDR,
					 XGMAC_RXEN, 0);
		if (ret)
			goto err_port_mac_link_down;

		ret = regmap_update_bits(ppe_dev->regmap,
					 reg + XGMAC_TX_CONFIG_ADDR,
					 XGMAC_TXEN, 0);
		if (ret)
			goto err_port_mac_link_down;
	}

	return;

err_port_mac_link_down:
	dev_err(ppe_dev->dev, "%s: port %d link down fail %d\n",
		__func__, port, ret);
}

/* PPE port MAC PCS selection for phylink */
static
struct phylink_pcs *ppe_port_mac_select_pcs(struct phylink_config *config,
					    phy_interface_t interface)
{
	struct ppe_port *ppe_port = container_of(config, struct ppe_port,
						 phylink_config);
	struct ppe_device *ppe_dev = ppe_port->ppe_dev;
	int ret, port = ppe_port->port_id;
	u32 val;

	/* PPE port5 can connects with PCS0 or PCS1. In PSGMII
	 * mode, it selects PCS0; otherwise, it selects PCS1.
	 */
	if (port == 5) {
		val = interface == PHY_INTERFACE_MODE_PSGMII ?
			0 : PPE_PORT5_SEL_PCS1;
		ret = regmap_update_bits(ppe_dev->regmap,
					 PPE_PORT_MUX_CTRL_ADDR,
					 PPE_PORT5_SEL_PCS1, val);
		if (ret) {
			dev_err(ppe_dev->dev, "%s: port5 select PCS fail %d\n",
				__func__, ret);
			return NULL;
		}
	}

	return ppe_port->pcs;
}

static const struct phylink_mac_ops ppe_phylink_ops = {
	.mac_config = ppe_port_mac_config,
	.mac_link_up = ppe_port_mac_link_up,
	.mac_link_down = ppe_port_mac_link_down,
	.mac_select_pcs = ppe_port_mac_select_pcs,
};

/**
 * ppe_port_phylink_setup() - Set phylink instance for the given PPE port
 * @ppe_port: PPE port
 * @netdev: Netdevice
 *
 * Description: Wrapper function to help setup phylink for the PPE port
 * specified by @ppe_port and associated with the net device @netdev.
 *
 * Return: 0 upon success or a negative error upon failure.
 */
int ppe_port_phylink_setup(struct ppe_port *ppe_port, struct net_device *netdev)
{
	struct ppe_device *ppe_dev = ppe_port->ppe_dev;
	struct device_node *pcs_node;
	int ret;

	/* Create PCS */
	pcs_node = of_parse_phandle(ppe_port->np, "pcs-handle", 0);
	if (!pcs_node)
		return -ENODEV;

	ppe_port->pcs = ipq_unipcs_create(pcs_node);
	of_node_put(pcs_node);
	if (IS_ERR(ppe_port->pcs)) {
		dev_err(ppe_dev->dev, "%s: port %d failed to create PCS\n",
			__func__, ppe_port->port_id);
		return PTR_ERR(ppe_port->pcs);
	}

	/* Port phylink capability */
	ppe_port->phylink_config.dev = &netdev->dev;
	ppe_port->phylink_config.type = PHYLINK_NETDEV;
	ppe_port->phylink_config.mac_capabilities = MAC_ASYM_PAUSE |
		MAC_SYM_PAUSE | MAC_10 | MAC_100 | MAC_1000 |
		MAC_2500FD | MAC_5000FD | MAC_10000FD;
	__set_bit(PHY_INTERFACE_MODE_QSGMII,
		  ppe_port->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_PSGMII,
		  ppe_port->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_SGMII,
		  ppe_port->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_1000BASEX,
		  ppe_port->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_2500BASEX,
		  ppe_port->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_USXGMII,
		  ppe_port->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_10GBASER,
		  ppe_port->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_10G_QXGMII,
		  ppe_port->phylink_config.supported_interfaces);

	/* Create phylink */
	ppe_port->phylink = phylink_create(&ppe_port->phylink_config,
					   of_fwnode_handle(ppe_port->np),
					   ppe_port->interface,
					   &ppe_phylink_ops);
	if (IS_ERR(ppe_port->phylink)) {
		dev_err(ppe_dev->dev, "%s: port %d failed to create phylink\n",
			__func__, ppe_port->port_id);
		ret = PTR_ERR(ppe_port->phylink);
		goto err_free_pcs;
	}

	/* Connect phylink */
	ret = phylink_of_phy_connect(ppe_port->phylink, ppe_port->np, 0);
	if (ret) {
		dev_err(ppe_dev->dev, "%s: port %d failed to connect phylink\n",
			__func__, ppe_port->port_id);
		goto err_free_phylink;
	}

	return 0;

err_free_phylink:
	phylink_destroy(ppe_port->phylink);
	ppe_port->phylink = NULL;
err_free_pcs:
	ipq_unipcs_destroy(ppe_port->pcs);
	ppe_port->pcs = NULL;
	return ret;
}

/**
 * ppe_port_phylink_destroy() - Destroy phylink instance for the given PPE port
 * @ppe_port: PPE port
 *
 * Description: Wrapper function to help destroy phylink for the PPE port
 * specified by @ppe_port.
 */
void ppe_port_phylink_destroy(struct ppe_port *ppe_port)
{
	/* Destroy phylink */
	if (ppe_port->phylink) {
		rtnl_lock();
		phylink_disconnect_phy(ppe_port->phylink);
		rtnl_unlock();
		phylink_destroy(ppe_port->phylink);
		ppe_port->phylink = NULL;
	}

	/* Destroy PCS */
	if (ppe_port->pcs) {
		ipq_unipcs_destroy(ppe_port->pcs);
		ppe_port->pcs = NULL;
	}
}

/* PPE port clock initialization */
static int ppe_port_clock_init(struct ppe_port *ppe_port)
{
	struct device_node *port_node = ppe_port->np;
	struct reset_control *rstc;
	struct clk *clk;
	int i, j, ret;

	for (i = 0; i < PPE_PORT_CLK_RST_MAX; i++) {
		/* Get PPE port resets which will be used to reset PPE
		 * port and MAC.
		 */
		rstc = of_reset_control_get_exclusive(port_node,
						      ppe_port_clk_rst_name[i]);
		if (IS_ERR(rstc)) {
			ret =  PTR_ERR(rstc);
			goto err_rst;
		}

		clk = of_clk_get_by_name(port_node, ppe_port_clk_rst_name[i]);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			goto err_clk_get;
		}

		ret = clk_prepare_enable(clk);
		if (ret)
			goto err_clk_en;

		ppe_port->clks[i] = clk;
		ppe_port->rstcs[i] = rstc;
	}

	return 0;

err_clk_en:
	clk_put(clk);
err_clk_get:
	reset_control_put(rstc);
err_rst:
	for (j = 0; j < i; j++) {
		clk_disable_unprepare(ppe_port->clks[j]);
		clk_put(ppe_port->clks[j]);
		reset_control_put(ppe_port->rstcs[j]);
	}

	return ret;
}

/* PPE port clock deinitialization */
static void ppe_port_clock_deinit(struct ppe_port *ppe_port)
{
	int i;

	for (i = 0; i < PPE_PORT_CLK_RST_MAX; i++) {
		clk_disable_unprepare(ppe_port->clks[i]);
		clk_put(ppe_port->clks[i]);
		reset_control_put(ppe_port->rstcs[i]);
	}
}

/* PPE port MAC hardware init configuration */
static int ppe_port_mac_hw_init(struct ppe_port *ppe_port)
{
	struct ppe_device *ppe_dev = ppe_port->ppe_dev;
	int ret, port = ppe_port->port_id;
	u32 reg, val;

	/* GMAC RX and TX are initialized as disabled */
	reg = PPE_PORT_GMAC_ADDR(port);
	ret = regmap_update_bits(ppe_dev->regmap,
				 reg + GMAC_ENABLE_ADDR, GMAC_TRXEN, 0);
	if (ret)
		return ret;

	/* GMAC max frame size configuration */
	val = FIELD_PREP(GMAC_JUMBO_SIZE_M, PPE_PORT_MAC_MAX_FRAME_SIZE);
	ret = regmap_update_bits(ppe_dev->regmap, reg + GMAC_JUMBO_SIZE_ADDR,
				 GMAC_JUMBO_SIZE_M, val);
	if (ret)
		return ret;

	val = FIELD_PREP(GMAC_MAXFRAME_SIZE_M, PPE_PORT_MAC_MAX_FRAME_SIZE);
	val |= FIELD_PREP(GMAC_TX_THD_M, 0x1);
	ret = regmap_update_bits(ppe_dev->regmap, reg + GMAC_CTRL_ADDR,
				 GMAC_CTRL_MASK, val);
	if (ret)
		return ret;

	val = FIELD_PREP(GMAC_HIGH_IPG_M, 0xc);
	ret = regmap_update_bits(ppe_dev->regmap, reg + GMAC_DBG_CTRL_ADDR,
				 GMAC_HIGH_IPG_M, val);
	if (ret)
		return ret;

	/* Enable and reset GMAC MIB counters and set as read clear
	 * mode, the GMAC MIB counters will be cleared after reading.
	 */
	ret = regmap_update_bits(ppe_dev->regmap, reg + GMAC_MIB_CTRL_ADDR,
				 GMAC_MIB_CTRL_MASK, GMAC_MIB_CTRL_MASK);
	if (ret)
		return ret;

	ret = regmap_update_bits(ppe_dev->regmap, reg + GMAC_MIB_CTRL_ADDR,
				 GMAC_MIB_RST, 0);
	if (ret)
		return ret;

	/* XGMAC RX and TX disabled and max frame size configuration */
	reg = PPE_PORT_XGMAC_ADDR(port);
	ret = regmap_update_bits(ppe_dev->regmap, reg + XGMAC_TX_CONFIG_ADDR,
				 XGMAC_TXEN | XGMAC_JD, XGMAC_JD);
	if (ret)
		return ret;

	val = FIELD_PREP(XGMAC_GPSL_M, PPE_PORT_MAC_MAX_FRAME_SIZE);
	val |= XGMAC_GPSLEN;
	val |= XGMAC_CST;
	val |= XGMAC_ACS;
	ret = regmap_update_bits(ppe_dev->regmap, reg + XGMAC_RX_CONFIG_ADDR,
				 XGMAC_RX_CONFIG_MASK, val);
	if (ret)
		return ret;

	ret = regmap_update_bits(ppe_dev->regmap, reg + XGMAC_WD_TIMEOUT_ADDR,
				 XGMAC_WD_TIMEOUT_MASK, XGMAC_WD_TIMEOUT_VAL);
	if (ret)
		return ret;

	ret = regmap_update_bits(ppe_dev->regmap, reg + XGMAC_PKT_FILTER_ADDR,
				 XGMAC_PKT_FILTER_MASK, XGMAC_PKT_FILTER_VAL);
	if (ret)
		return ret;

	/* Enable and reset XGMAC MIB counters */
	ret = regmap_update_bits(ppe_dev->regmap, reg + XGMAC_MMC_CTRL_ADDR,
				 XGMAC_MCF | XGMAC_CNTRST, XGMAC_CNTRST);

	return ret;
}

/**
 * ppe_port_mac_init() - Initialization of PPE ports for the PPE device
 * @ppe_dev: PPE device
 *
 * Description: Initialize the PPE MAC ports on the PPE device specified
 * by @ppe_dev.
 *
 * Return: 0 upon success or a negative error upon failure.
 */
int ppe_port_mac_init(struct ppe_device *ppe_dev)
{
	struct device_node *ports_node, *port_node;
	int port, num, ret, j, i = 0;
	struct ppe_ports *ppe_ports;
	phy_interface_t phy_mode;

	ports_node = of_get_child_by_name(ppe_dev->dev->of_node,
					  "ethernet-ports");
	if (!ports_node) {
		dev_err(ppe_dev->dev, "Failed to get ports node\n");
		return -ENODEV;
	}

	num = of_get_available_child_count(ports_node);

	ppe_ports = devm_kzalloc(ppe_dev->dev,
				 struct_size(ppe_ports, port, num),
				 GFP_KERNEL);
	if (!ppe_ports) {
		ret = -ENOMEM;
		goto err_ports_node;
	}

	ppe_dev->ports = ppe_ports;
	ppe_ports->num = num;

	for_each_available_child_of_node(ports_node, port_node) {
		ret = of_property_read_u32(port_node, "reg", &port);
		if (ret) {
			dev_err(ppe_dev->dev, "Failed to get port id\n");
			goto err_port_node;
		}

		ret = of_get_phy_mode(port_node, &phy_mode);
		if (ret) {
			dev_err(ppe_dev->dev, "Failed to get phy mode\n");
			goto err_port_node;
		}

		ppe_ports->port[i].ppe_dev = ppe_dev;
		ppe_ports->port[i].port_id = port;
		ppe_ports->port[i].np = port_node;
		ppe_ports->port[i].interface = phy_mode;

		ret = ppe_port_clock_init(&ppe_ports->port[i]);
		if (ret) {
			dev_err(ppe_dev->dev, "Failed to initialize port clocks\n");
			goto err_port_clk;
		}

		ret = ppe_port_mac_hw_init(&ppe_ports->port[i]);
		if (ret) {
			dev_err(ppe_dev->dev, "Failed to initialize MAC hardware\n");
			goto err_port_node;
		}

		i++;
	}

	of_node_put(ports_node);
	return 0;

err_port_clk:
	for (j = 0; j < i; j++)
		ppe_port_clock_deinit(&ppe_ports->port[j]);
err_port_node:
	of_node_put(port_node);
err_ports_node:
	of_node_put(ports_node);
	return ret;
}

/**
 * ppe_port_mac_deinit() - Deinitialization of PPE ports for the PPE device
 * @ppe_dev: PPE device
 *
 * Description: Deinitialize the PPE MAC ports on the PPE device specified
 * by @ppe_dev.
 */
void ppe_port_mac_deinit(struct ppe_device *ppe_dev)
{
	struct ppe_port *ppe_port;
	int i;

	for (i = 0; i < ppe_dev->ports->num; i++) {
		ppe_port = &ppe_dev->ports->port[i];
		ppe_port_clock_deinit(ppe_port);
	}
}
