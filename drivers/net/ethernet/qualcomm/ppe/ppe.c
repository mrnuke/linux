// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* PPE platform device probe, DTSI read and basic HW initialization functions
 * such as BM, QM, TDM and scheduler configs.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/if_ether.h>
#include <linux/of_net.h>
#include <linux/rtnetlink.h>
#include <linux/soc/qcom/ppe.h>
#include "ppe.h"
#include "ppe_regs.h"
#include "ppe_ops.h"
#include "ppe_debugfs.h"
#include "ppe_uniphy.h"

#define PPE_SCHEDULER_PORT_NUM		8
#define MPPE_SCHEDULER_PORT_NUM		3
#define PPE_SCHEDULER_L0_NUM		300
#define PPE_SCHEDULER_L1_NUM		64
#define PPE_SP_PRIORITY_NUM		8

#define PPE_ETHTOOL_XGMIB_STAT(x) { #x, \
				    offsetof(struct ppe_xgmib_hw_stats, x) / sizeof(u64) }
#define PPE_ETHTOOL_GMIB_STAT(x) { #x, \
				   offsetof(struct ppe_gmib_hw_stats, x) / sizeof(u64) }

/* Poll interval time to poll GMAC MIBs for overflow protection */
#define PPE_GMIB_STATS_POLL_INTERVAL	120000

/* XGMAC strings used by ethtool */
static const struct ppe_ethtool_gstrings_xgmib_stats {
	char name[ETH_GSTRING_LEN];
	u32 offset;
} ppe_ethtool_gstrings_xgmib_stats[] = {
	PPE_ETHTOOL_XGMIB_STAT(rx_frames),
	PPE_ETHTOOL_XGMIB_STAT(rx_bytes),
	PPE_ETHTOOL_XGMIB_STAT(rx_bytes_g),
	PPE_ETHTOOL_XGMIB_STAT(rx_broadcast_g),
	PPE_ETHTOOL_XGMIB_STAT(rx_multicast_g),
	PPE_ETHTOOL_XGMIB_STAT(rx_unicast_g),
	PPE_ETHTOOL_XGMIB_STAT(rx_crc_err),
	PPE_ETHTOOL_XGMIB_STAT(rx_runt_err),
	PPE_ETHTOOL_XGMIB_STAT(rx_jabber_err),
	PPE_ETHTOOL_XGMIB_STAT(rx_undersize_g),
	PPE_ETHTOOL_XGMIB_STAT(rx_oversize_g),
	PPE_ETHTOOL_XGMIB_STAT(rx_pkt64),
	PPE_ETHTOOL_XGMIB_STAT(rx_pkt65to127),
	PPE_ETHTOOL_XGMIB_STAT(rx_pkt128to255),
	PPE_ETHTOOL_XGMIB_STAT(rx_pkt256to511),
	PPE_ETHTOOL_XGMIB_STAT(rx_pkt512to1023),
	PPE_ETHTOOL_XGMIB_STAT(rx_pkt1024tomax),
	PPE_ETHTOOL_XGMIB_STAT(rx_len_err),
	PPE_ETHTOOL_XGMIB_STAT(rx_outofrange_err),
	PPE_ETHTOOL_XGMIB_STAT(rx_pause),
	PPE_ETHTOOL_XGMIB_STAT(rx_fifo_overflow),
	PPE_ETHTOOL_XGMIB_STAT(rx_vlan),
	PPE_ETHTOOL_XGMIB_STAT(rx_wdog_err),
	PPE_ETHTOOL_XGMIB_STAT(rx_lpi_usec),
	PPE_ETHTOOL_XGMIB_STAT(rx_lpi_tran),
	PPE_ETHTOOL_XGMIB_STAT(rx_drop_frames),
	PPE_ETHTOOL_XGMIB_STAT(rx_drop_bytes),
	PPE_ETHTOOL_XGMIB_STAT(tx_bytes),
	PPE_ETHTOOL_XGMIB_STAT(tx_bytes_g),
	PPE_ETHTOOL_XGMIB_STAT(tx_frames),
	PPE_ETHTOOL_XGMIB_STAT(tx_frame_g),
	PPE_ETHTOOL_XGMIB_STAT(tx_broadcast),
	PPE_ETHTOOL_XGMIB_STAT(tx_broadcast_g),
	PPE_ETHTOOL_XGMIB_STAT(tx_multicast),
	PPE_ETHTOOL_XGMIB_STAT(tx_multicast_g),
	PPE_ETHTOOL_XGMIB_STAT(tx_unicast),
	PPE_ETHTOOL_XGMIB_STAT(tx_pkt64),
	PPE_ETHTOOL_XGMIB_STAT(tx_pkt65to127),
	PPE_ETHTOOL_XGMIB_STAT(tx_pkt128to255),
	PPE_ETHTOOL_XGMIB_STAT(tx_pkt256to511),
	PPE_ETHTOOL_XGMIB_STAT(tx_pkt512to1023),
	PPE_ETHTOOL_XGMIB_STAT(tx_pkt1024tomax),
	PPE_ETHTOOL_XGMIB_STAT(tx_underflow_err),
	PPE_ETHTOOL_XGMIB_STAT(tx_pause),
	PPE_ETHTOOL_XGMIB_STAT(tx_vlan_g),
	PPE_ETHTOOL_XGMIB_STAT(tx_lpi_usec),
	PPE_ETHTOOL_XGMIB_STAT(tx_lpi_tran),
};

/* GMAC strings used by ethtool */
static const struct ppe_ethtool_gstrings_gmib_stats {
	char name[ETH_GSTRING_LEN];
	u32 offset;
} ppe_ethtool_gstrings_gmib_stats[] = {
	PPE_ETHTOOL_GMIB_STAT(rx_broadcast),
	PPE_ETHTOOL_GMIB_STAT(rx_pause),
	PPE_ETHTOOL_GMIB_STAT(rx_unicast),
	PPE_ETHTOOL_GMIB_STAT(rx_multicast),
	PPE_ETHTOOL_GMIB_STAT(rx_fcserr),
	PPE_ETHTOOL_GMIB_STAT(rx_alignerr),
	PPE_ETHTOOL_GMIB_STAT(rx_runt),
	PPE_ETHTOOL_GMIB_STAT(rx_frag),
	PPE_ETHTOOL_GMIB_STAT(rx_jmbfcserr),
	PPE_ETHTOOL_GMIB_STAT(rx_jmbalignerr),
	PPE_ETHTOOL_GMIB_STAT(rx_pkt64),
	PPE_ETHTOOL_GMIB_STAT(rx_pkt65to127),
	PPE_ETHTOOL_GMIB_STAT(rx_pkt128to255),
	PPE_ETHTOOL_GMIB_STAT(rx_pkt256to511),
	PPE_ETHTOOL_GMIB_STAT(rx_pkt512to1023),
	PPE_ETHTOOL_GMIB_STAT(rx_pkt1024to1518),
	PPE_ETHTOOL_GMIB_STAT(rx_pkt1519tomax),
	PPE_ETHTOOL_GMIB_STAT(rx_toolong),
	PPE_ETHTOOL_GMIB_STAT(rx_pktgoodbyte),
	PPE_ETHTOOL_GMIB_STAT(rx_pktbadbyte),
	PPE_ETHTOOL_GMIB_STAT(tx_broadcast),
	PPE_ETHTOOL_GMIB_STAT(tx_pause),
	PPE_ETHTOOL_GMIB_STAT(tx_multicast),
	PPE_ETHTOOL_GMIB_STAT(tx_underrun),
	PPE_ETHTOOL_GMIB_STAT(tx_pkt64),
	PPE_ETHTOOL_GMIB_STAT(tx_pkt65to127),
	PPE_ETHTOOL_GMIB_STAT(tx_pkt128to255),
	PPE_ETHTOOL_GMIB_STAT(tx_pkt256to511),
	PPE_ETHTOOL_GMIB_STAT(tx_pkt512to1023),
	PPE_ETHTOOL_GMIB_STAT(tx_pkt1024to1518),
	PPE_ETHTOOL_GMIB_STAT(tx_pkt1519tomax),
	PPE_ETHTOOL_GMIB_STAT(tx_pktbyte),
	PPE_ETHTOOL_GMIB_STAT(tx_collisions),
	PPE_ETHTOOL_GMIB_STAT(tx_abortcol),
	PPE_ETHTOOL_GMIB_STAT(tx_multicol),
	PPE_ETHTOOL_GMIB_STAT(tx_singlecol),
	PPE_ETHTOOL_GMIB_STAT(tx_exesdeffer),
	PPE_ETHTOOL_GMIB_STAT(tx_deffer),
	PPE_ETHTOOL_GMIB_STAT(tx_latecol),
	PPE_ETHTOOL_GMIB_STAT(tx_unicast),
};

static const char * const ppe_clock_name[PPE_CLK_MAX] = {
	"cmn_ahb",
	"cmn_sys",
	"uniphy0_sys",
	"uniphy1_sys",
	"uniphy2_sys",
	"uniphy0_ahb",
	"uniphy1_ahb",
	"uniphy2_ahb",
	"gcc_nsscc",
	"gcc_nssnoc_nsscc",
	"gcc_nssnoc_snoc",
	"gcc_nssnoc_snoc_1",
	"gcc_im_sleep",
	"nss_ppe",
	"nss_ppe_cfg",
	"nssnoc_ppe",
	"nssnoc_ppe_cfg",
	"nss_edma",
	"nss_edma_cfg",
	"nss_ppe_ipe",
	"nss_ppe_btq",
	"port1_mac",
	"port2_mac",
	"port3_mac",
	"port4_mac",
	"port5_mac",
	"port6_mac",
	"nss_port1_rx",
	"nss_port1_tx",
	"nss_port2_rx",
	"nss_port2_tx",
	"nss_port3_rx",
	"nss_port3_tx",
	"nss_port4_rx",
	"nss_port4_tx",
	"nss_port5_rx",
	"nss_port5_tx",
	"nss_port6_rx",
	"nss_port6_tx",
	"uniphy_port1_rx",
	"uniphy_port1_tx",
	"uniphy_port2_rx",
	"uniphy_port2_tx",
	"uniphy_port3_rx",
	"uniphy_port3_tx",
	"uniphy_port4_rx",
	"uniphy_port4_tx",
	"uniphy_port5_rx",
	"uniphy_port5_tx",
	"uniphy_port6_rx",
	"uniphy_port6_tx",
	"nss_port5_rx_clk_src",
	"nss_port5_tx_clk_src",
};

static const char * const ppe_reset_name[PPE_RST_MAX] = {
	"ppe",
	"uniphy0_sys",
	"uniphy1_sys",
	"uniphy2_sys",
	"uniphy0_ahb",
	"uniphy1_ahb",
	"uniphy2_ahb",
	"uniphy0_xpcs",
	"uniphy1_xpcs",
	"uniphy2_xpcs",
	"uniphy0_soft",
	"uniphy1_soft",
	"uniphy2_soft",
	"uniphy_port1_dis",
	"uniphy_port2_dis",
	"uniphy_port3_dis",
	"uniphy_port4_dis",
	"uniphy_port1_rx",
	"uniphy_port1_tx",
	"uniphy_port2_rx",
	"uniphy_port2_tx",
	"nss_port1_rx",
	"nss_port1_tx",
	"nss_port2_rx",
	"nss_port2_tx",
	"nss_port1",
	"nss_port2",
	"nss_port3",
	"nss_port4",
	"nss_port5",
	"nss_port6",
	"nss_port1_mac",
	"nss_port2_mac",
	"nss_port3_mac",
	"nss_port4_mac",
	"nss_port5_mac",
	"nss_port6_mac",
};

static struct ppe_scheduler_port_resource ppe_scheduler_res[PPE_SCHEDULER_PORT_NUM];

int ppe_write(struct ppe_device *ppe_dev, u32 reg, unsigned int val)
{
	return regmap_write(ppe_dev->regmap, reg, val);
}

int ppe_read(struct ppe_device *ppe_dev, u32 reg, unsigned int *val)
{
	return regmap_read(ppe_dev->regmap, reg, val);
}

int ppe_mask(struct ppe_device *ppe_dev, u32 reg, u32 mask, unsigned int set)
{
	return regmap_update_bits(ppe_dev->regmap, reg, mask, set);
}

int ppe_write_tbl(struct ppe_device *ppe_dev, u32 reg,
		  const unsigned int *val, int cnt)
{
	int i, ret;

	for (i = 0; i < cnt / 4; i++) {
		ret = ppe_write(ppe_dev, reg + i * 4, val[i]);
		if (ret)
			return ret;
	}

	return ret;
}

int ppe_read_tbl(struct ppe_device *ppe_dev, u32 reg,
		 unsigned int *val, int cnt)
{
	int i, ret;

	for (i = 0; i < cnt / 4; i++) {
		ret = ppe_read(ppe_dev, reg + i * 4, &val[i]);
		if (ret)
			return ret;
	}

	return ret;
}

int ppe_type_get(struct ppe_device *ppe_dev)
{
	struct ppe_data *ppe_dev_priv = ppe_dev->ppe_priv;

	if (!ppe_dev_priv)
		return PPE_TYPE_MAX;

	return ppe_dev_priv->ppe_type;
}

struct clk **ppe_clock_get(struct ppe_device *ppe_dev)
{
	struct ppe_data *ppe_dev_priv = ppe_dev->ppe_priv;

	if (!ppe_dev_priv)
		return NULL;

	return ppe_dev_priv->clk;
}

struct reset_control **ppe_reset_get(struct ppe_device *ppe_dev)
{
	struct ppe_data *ppe_dev_priv = ppe_dev->ppe_priv;

	if (!ppe_dev_priv)
		return NULL;

	return ppe_dev_priv->rst;
}

static struct ppe_port *ppe_port_get(struct ppe_device *ppe_dev, int port)
{
	struct ppe_ports *ppe_ports = (struct ppe_ports *)ppe_dev->ports;
	int i = 0;

	for (i = 0; i < ppe_ports->num; i++) {
		if (ppe_ports->port[i].port_id == port)
			return &ppe_ports->port[i];
	}

	return NULL;
}

static int ppe_clock_set_enable(struct ppe_device *ppe_dev,
				enum ppe_clk_id clk_id, unsigned long rate)
{
	struct ppe_data *ppe_dev_priv = ppe_dev->ppe_priv;

	if (clk_id >= PPE_CLK_MAX)
		return -EINVAL;

	if (rate != 0)
		clk_set_rate(ppe_dev_priv->clk[clk_id], rate);

	return clk_prepare_enable(ppe_dev_priv->clk[clk_id]);
}

static int ppe_fix_clock_init(struct ppe_device *ppe_dev)
{
	unsigned long noc_rate, ppe_rate;
	enum ppe_clk_id clk_id;
	int ppe_type = ppe_type_get(ppe_dev);

	switch (ppe_type) {
	case PPE_TYPE_APPE:
		noc_rate = 342857143;
		ppe_rate = 353000000;
		break;
	case PPE_TYPE_MPPE:
		noc_rate = 266660000;
		ppe_rate = 200000000;
		ppe_clock_set_enable(ppe_dev, PPE_IM_SLEEP_CLK, 0);
		break;
	default:
		return -EINVAL;
	}

	ppe_clock_set_enable(ppe_dev, PPE_CMN_AHB_CLK, 0);
	ppe_clock_set_enable(ppe_dev, PPE_CMN_SYS_CLK, 0);
	ppe_clock_set_enable(ppe_dev, PPE_NSSCC_CLK, 100000000);
	ppe_clock_set_enable(ppe_dev, PPE_NSSNOC_NSSCC_CLK, 100000000);

	ppe_clock_set_enable(ppe_dev, PPE_NSSNOC_SNOC_CLK, noc_rate);
	ppe_clock_set_enable(ppe_dev, PPE_NSSNOC_SNOC_1_CLK, noc_rate);

	ppe_clock_set_enable(ppe_dev, PPE_UNIPHY0_SYS_CLK, 24000000);
	ppe_clock_set_enable(ppe_dev, PPE_UNIPHY1_SYS_CLK, 24000000);
	ppe_clock_set_enable(ppe_dev, PPE_UNIPHY0_AHB_CLK, 100000000);
	ppe_clock_set_enable(ppe_dev, PPE_UNIPHY1_AHB_CLK, 100000000);

	if (ppe_type == PPE_TYPE_APPE) {
		ppe_clock_set_enable(ppe_dev, PPE_UNIPHY2_SYS_CLK, 24000000);
		ppe_clock_set_enable(ppe_dev, PPE_UNIPHY2_AHB_CLK, 100000000);
	}

	ppe_clock_set_enable(ppe_dev, PPE_PORT1_MAC_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_PORT2_MAC_CLK, ppe_rate);

	if (ppe_type == PPE_TYPE_APPE) {
		ppe_clock_set_enable(ppe_dev, PPE_PORT3_MAC_CLK, ppe_rate);
		ppe_clock_set_enable(ppe_dev, PPE_PORT4_MAC_CLK, ppe_rate);
		ppe_clock_set_enable(ppe_dev, PPE_PORT5_MAC_CLK, ppe_rate);
		ppe_clock_set_enable(ppe_dev, PPE_PORT6_MAC_CLK, ppe_rate);
	}

	ppe_clock_set_enable(ppe_dev, PPE_PPE_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_PPE_CFG_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_NSSNOC_PPE_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_NSSNOC_PPE_CFG_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_EDMA_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_EDMA_CFG_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_PPE_IPE_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_PPE_BTQ_CLK, ppe_rate);

	/* Enable uniphy port clocks */
	for (clk_id = PPE_NSS_PORT1_RX_CLK; clk_id <= PPE_UNIPHY_PORT6_TX_CLK; clk_id++)
		ppe_clock_set_enable(ppe_dev, clk_id, 0);

	return 0;
}

static int ppe_clock_config(struct platform_device *pdev)
{
	struct ppe_device *ppe_dev = platform_get_drvdata(pdev);
	struct ppe_data *ppe_dev_priv = ppe_dev->ppe_priv;
	int ret;

	ret = ppe_fix_clock_init(ppe_dev);
	if (ret)
		return ret;

	/* Reset PPE */
	reset_control_assert(ppe_dev_priv->rst[PPE_RST_PPE_RST]);
	fsleep(100000);
	reset_control_deassert(ppe_dev_priv->rst[PPE_RST_PPE_RST]);
	fsleep(100000);

	/* Reset the ahb uniphy connected with the PHY chip */
	if (ppe_type_get(ppe_dev) == PPE_TYPE_MPPE) {
		reset_control_assert(ppe_dev_priv->rst[PPE_UNIPHY1_AHB_RST]);
		fsleep(100000);
		reset_control_deassert(ppe_dev_priv->rst[PPE_UNIPHY1_AHB_RST]);
		fsleep(100000);
	}

	return 0;
}

static int ppe_port_mac_reset(struct ppe_device *ppe_dev, int port)
{
	struct ppe_data *ppe_dev_priv = ppe_dev->ppe_priv;

	reset_control_assert(ppe_dev_priv->rst[PPE_NSS_PORT1_MAC_RST + port - 1]);
	if (ppe_dev_priv->ppe_type == PPE_TYPE_APPE) {
		reset_control_assert(ppe_dev_priv->rst[PPE_NSS_PORT1_RST + port]);
	} else if (ppe_dev_priv->ppe_type == PPE_TYPE_MPPE) {
		reset_control_assert(ppe_dev_priv->rst[PPE_NSS_PORT1_RX_RST + ((port - 1) << 1)]);
		reset_control_assert(ppe_dev_priv->rst[PPE_NSS_PORT1_TX_RST + ((port - 1) << 1)]);
	}
	fsleep(150000);

	reset_control_deassert(ppe_dev_priv->rst[PPE_NSS_PORT1_MAC_RST + port - 1]);
	if (ppe_dev_priv->ppe_type == PPE_TYPE_APPE) {
		reset_control_deassert(ppe_dev_priv->rst[PPE_NSS_PORT1_RST + port]);
	} else if (ppe_dev_priv->ppe_type == PPE_TYPE_MPPE) {
		reset_control_deassert(ppe_dev_priv->rst[PPE_NSS_PORT1_RX_RST + ((port - 1) << 1)]);
		reset_control_deassert(ppe_dev_priv->rst[PPE_NSS_PORT1_TX_RST + ((port - 1) << 1)]);
	}
	fsleep(150000);

	return 0;
}

static int ppe_gcc_port_speed_clk_set(struct ppe_device *ppe_dev,
				      int port, int speed, phy_interface_t interface)
{
	struct ppe_data *ppe_dev_priv = ppe_dev->ppe_priv;
	enum ppe_clk_id rx_id, tx_id;
	unsigned long rate = 0;
	int err = 0;

	rx_id = PPE_NSS_PORT1_RX_CLK + ((port - 1) << 1);
	tx_id = PPE_NSS_PORT1_TX_CLK + ((port - 1) << 1);

	switch (interface) {
	case PHY_INTERFACE_MODE_USXGMII:
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_QUSGMII:
	case PHY_INTERFACE_MODE_10GBASER:
		if (speed == SPEED_10)
			rate = 1250000;
		else if (speed == SPEED_100)
			rate = 12500000;
		else if (speed == SPEED_1000)
			rate = 125000000;
		else if (speed == SPEED_2500)
			rate = 78125000;
		else if (speed == SPEED_5000)
			rate = 156250000;
		else if (speed == SPEED_10000)
			rate = 312500000;
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
		if (speed == SPEED_2500)
			rate = 312500000;
		break;
	case PHY_INTERFACE_MODE_QSGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
	case PHY_INTERFACE_MODE_SGMII:
		if (speed == SPEED_10)
			rate = 2500000;
		else if (speed == SPEED_100)
			rate = 25000000;
		else if (speed == SPEED_1000)
			rate = 125000000;
		break;
	default:
		break;
	}

	if (!IS_ERR(ppe_dev_priv->clk[rx_id])) {
		err = clk_set_rate(ppe_dev_priv->clk[rx_id], rate);
		if (err) {
			dev_err(ppe_dev->dev,
				"Failed to set ppe port %d speed rx clk(%d)\n",
				port, rx_id);
			return err;
		}
	}

	if (!IS_ERR(ppe_dev_priv->clk[tx_id])) {
		err = clk_set_rate(ppe_dev_priv->clk[tx_id], rate);
		if (err) {
			dev_err(ppe_dev->dev,
				"Failed to set ppe port %d speed rx clk(%d)\n",
				port, rx_id);
			return err;
		}
	}

	return 0;
}

static int ppe_mac_speed_set(struct ppe_device *ppe_dev,
			     int port, int speed, phy_interface_t interface)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);
	u32 val;

	if (!ppe_port) {
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);
		return -ENOENT;
	}

	if (ppe_port->mac_type == PPE_MAC_TYPE_GMAC) {
		ppe_read(ppe_dev,
			 PPE_PORT_GMAC_ADDR(port) + GMAC_SPEED,
			 &val);
		val &= ~GMAC_SPEED_MASK;
		switch (speed) {
		case SPEED_10:
			val |= GMAC_SPEED_10;
			break;
		case SPEED_100:
			val |= GMAC_SPEED_100;
			break;
		case SPEED_1000:
			val |= GMAC_SPEED_1000;
			break;
		default:
			break;
		}
		ppe_write(ppe_dev,
			  PPE_PORT_GMAC_ADDR(port) + GMAC_SPEED,
			  val);
	} else if (ppe_port->mac_type == PPE_MAC_TYPE_XGMAC) {
		ppe_read(ppe_dev,
			 PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_CONFIGURATION,
			 &val);
		val &= ~XGMAC_SPEED_MASK;
		switch (speed) {
		case SPEED_10000:
			if (interface == PHY_INTERFACE_MODE_USXGMII ||
			    interface == PHY_INTERFACE_MODE_QUSGMII)
				val |= XGMAC_SPEED_10000_USXGMII;
			else
				val |= XGMAC_SPEED_10000;
			break;
		case SPEED_5000:
			val |= XGMAC_SPEED_5000;
			break;
		case SPEED_2500:
			if (interface == PHY_INTERFACE_MODE_USXGMII ||
			    interface == PHY_INTERFACE_MODE_QUSGMII)
				val |= XGMAC_SPEED_2500_USXGMII;
			else
				val |= XGMAC_SPEED_2500;
			break;
		case SPEED_1000:
			val |= XGMAC_SPEED_1000;
			break;
		case SPEED_100:
			val |= XGMAC_SPEED_100;
			break;
		case SPEED_10:
			val |= XGMAC_SPEED_10;
			break;
		default:
			break;
		}
		ppe_write(ppe_dev,
			  PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_CONFIGURATION,
			  val);
	}

	return 0;
}

static int ppe_mac_duplex_set(struct ppe_device *ppe_dev, int port, int duplex)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);
	u32 val;

	if (!ppe_port) {
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);
		return -ENOENT;
	}

	if (ppe_port->mac_type == PPE_MAC_TYPE_GMAC) {
		ppe_read(ppe_dev,
			 PPE_PORT_GMAC_ADDR(port) + GMAC_ENABLE,
			 &val);
		if (duplex == DUPLEX_FULL)
			val |= GMAC_DUPLEX_FULL;
		else
			val &= ~GMAC_DUPLEX_FULL;
		ppe_write(ppe_dev,
			  PPE_PORT_GMAC_ADDR(port) + GMAC_ENABLE,
			  val);
	}

	return 0;
}

static int ppe_mac_txfc_status_set(struct ppe_device *ppe_dev, int port, bool enable)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);
	u32 val;

	if (!ppe_port) {
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);
		return -ENOENT;
	}

	if (ppe_port->mac_type == PPE_MAC_TYPE_GMAC) {
		ppe_read(ppe_dev,
			 PPE_PORT_GMAC_ADDR(port) + GMAC_ENABLE,
			 &val);
		if (enable)
			val |= GMAC_TX_FLOW_EN;
		else
			val &= ~GMAC_TX_FLOW_EN;
		ppe_write(ppe_dev,
			  PPE_PORT_GMAC_ADDR(port) + GMAC_ENABLE,
			  val);
	} else if (ppe_port->mac_type == PPE_MAC_TYPE_XGMAC) {
		ppe_read(ppe_dev,
			 PPE_PORT_XGMAC_ADDR(port) + XGMAC_Q0_TX_FLOW_CTRL,
			 &val);
		if (enable) {
			val &= ~XGMAC_PT_MASK;
			val |= (XGMAC_PAUSE_TIME | XGMAC_TFE);
		} else {
			val &= ~XGMAC_TFE;
		}
		ppe_write(ppe_dev,
			  PPE_PORT_XGMAC_ADDR(port) + XGMAC_Q0_TX_FLOW_CTRL,
			  val);
	}

	ppe_read(ppe_dev,
		 PPE_BM_PORT_FC_MODE + PPE_BM_PORT_FC_MODE_INC * (port + 7),
		 &val);
	if (enable)
		val |= PPE_BM_PORT_FC_MODE_EN;
	else
		val &= ~PPE_BM_PORT_FC_MODE_EN;
	ppe_write(ppe_dev,
		  PPE_BM_PORT_FC_MODE + PPE_BM_PORT_FC_MODE_INC * (port + 7),
		  val);

	return 0;
}

static int ppe_mac_rxfc_status_set(struct ppe_device *ppe_dev, int port, bool enable)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);
	u32 val;

	if (!ppe_port) {
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);
		return -ENOENT;
	}

	if (ppe_port->mac_type == PPE_MAC_TYPE_GMAC) {
		ppe_read(ppe_dev,
			 PPE_PORT_GMAC_ADDR(port) + GMAC_ENABLE,
			 &val);
		if (enable)
			val |= GMAC_RX_FLOW_EN;
		else
			val &= ~GMAC_RX_FLOW_EN;
		ppe_write(ppe_dev,
			  PPE_PORT_GMAC_ADDR(port) + GMAC_ENABLE,
			  val);
	} else if (ppe_port->mac_type == PPE_MAC_TYPE_XGMAC) {
		ppe_read(ppe_dev,
			 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_FLOW_CTRL,
			 &val);
		if (enable)
			val |= XGMAC_RFE;
		else
			val &= ~XGMAC_RFE;
		ppe_write(ppe_dev,
			  PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_FLOW_CTRL,
			  val);
	}

	return 0;
}

static int ppe_mac_txmac_en_set(struct ppe_device *ppe_dev, int port, bool enable)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);
	u32 val;

	if (!ppe_port) {
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);
		return -ENOENT;
	}

	if (ppe_port->mac_type == PPE_MAC_TYPE_GMAC) {
		ppe_read(ppe_dev,
			 PPE_PORT_GMAC_ADDR(port) + GMAC_ENABLE,
			 &val);
		if (enable)
			val |= GMAC_TXMAC_EN;
		else
			val &= ~GMAC_TXMAC_EN;
		ppe_write(ppe_dev,
			  PPE_PORT_GMAC_ADDR(port) + GMAC_ENABLE,
			  val);
	} else if (ppe_port->mac_type == PPE_MAC_TYPE_XGMAC) {
		ppe_read(ppe_dev,
			 PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_CONFIGURATION,
			 &val);
		if (enable)
			val |= XGMAC_TE;
		else
			val &= ~XGMAC_TE;
		ppe_write(ppe_dev,
			  PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_CONFIGURATION,
			  val);
	}

	return 0;
}

static int ppe_mac_rxmac_en_set(struct ppe_device *ppe_dev, int port, bool enable)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);
	u32 val;

	if (!ppe_port) {
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);
		return -ENOENT;
	}

	if (ppe_port->mac_type == PPE_MAC_TYPE_GMAC) {
		ppe_read(ppe_dev,
			 PPE_PORT_GMAC_ADDR(port) + GMAC_ENABLE,
			 &val);
		if (enable)
			val |= GMAC_RXMAC_EN;
		else
			val &= ~GMAC_RXMAC_EN;
		ppe_write(ppe_dev,
			  PPE_PORT_GMAC_ADDR(port) + GMAC_ENABLE,
			  val);
	} else if (ppe_port->mac_type == PPE_MAC_TYPE_XGMAC) {
		ppe_read(ppe_dev,
			 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_CONFIGURATION,
			 &val);
		if (enable)
			val |= XGMAC_RE;
		else
			val &= ~XGMAC_RE;
		ppe_write(ppe_dev,
			  PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_CONFIGURATION,
			  val);
	}

	return 0;
}

static int ppe_port_bridge_txmac_en_set(struct ppe_device *ppe_dev, int port, bool enable)
{
	u32 val;

	ppe_read(ppe_dev,
		 PPE_PORT_BRIDGE_CTRL + PPE_PORT_BRIDGE_CTRL_INC * port,
		 &val);

	if (enable)
		val |= PPE_PORT_BRIDGE_CTRL_TXMAC_EN;
	else
		val &= ~PPE_PORT_BRIDGE_CTRL_TXMAC_EN;

	ppe_write(ppe_dev,
		  PPE_PORT_BRIDGE_CTRL + PPE_PORT_BRIDGE_CTRL_INC * port,
		  val);

	return 0;
}

/* Get GMAC MIBs from GMAC registers and update to PPE port gmib stats */
static void ppe_gmib_stats_update(struct ppe_port *ppe_port)
{
	u32 val, hi;
	struct ppe_device *ppe_dev = ppe_port->ppe_dev;
	int port = ppe_port->port_id;

	spin_lock(&ppe_port->stats_lock);

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXBROAD, &val);
	ppe_port->gmib_stats->rx_broadcast += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXPAUSE, &val);
	ppe_port->gmib_stats->rx_pause += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXMULTI, &val);
	ppe_port->gmib_stats->rx_multicast += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXFCSERR, &val);
	ppe_port->gmib_stats->rx_fcserr += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXALIGNERR, &val);
	ppe_port->gmib_stats->rx_alignerr += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXRUNT, &val);
	ppe_port->gmib_stats->rx_runt += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXFRAG, &val);
	ppe_port->gmib_stats->rx_frag += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXJUMBOFCSERR, &val);
	ppe_port->gmib_stats->rx_jmbfcserr += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXJUMBOALIGNERR, &val);
	ppe_port->gmib_stats->rx_jmbalignerr += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXPKT64, &val);
	ppe_port->gmib_stats->rx_pkt64 += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXPKT65TO127, &val);
	ppe_port->gmib_stats->rx_pkt65to127 += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXPKT128TO255, &val);
	ppe_port->gmib_stats->rx_pkt128to255 += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXPKT256TO511, &val);
	ppe_port->gmib_stats->rx_pkt256to511 += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXPKT512TO1023, &val);
	ppe_port->gmib_stats->rx_pkt512to1023 += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXPKT1024TO1518, &val);
	ppe_port->gmib_stats->rx_pkt1024to1518 += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXPKT1519TOX, &val);
	ppe_port->gmib_stats->rx_pkt1519tomax += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXTOOLONG, &val);
	ppe_port->gmib_stats->rx_toolong += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXGOODBYTE_L, &val);
	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXGOODBYTE_H, &hi);
	ppe_port->gmib_stats->rx_pktgoodbyte += (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXBADBYTE_L, &val);
	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXBADBYTE_H, &hi);
	ppe_port->gmib_stats->rx_pktbadbyte += (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_RXUNI, &val);
	ppe_port->gmib_stats->rx_unicast += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXBROAD, &val);
	ppe_port->gmib_stats->tx_broadcast += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXPAUSE, &val);
	ppe_port->gmib_stats->tx_pause += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXMULTI, &val);
	ppe_port->gmib_stats->tx_multicast += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXUNDERRUN, &val);
	ppe_port->gmib_stats->tx_underrun += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXPKT64, &val);
	ppe_port->gmib_stats->tx_pkt64 += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXPKT65TO127, &val);
	ppe_port->gmib_stats->tx_pkt65to127 += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXPKT128TO255, &val);
	ppe_port->gmib_stats->tx_pkt128to255 += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXPKT256TO511, &val);
	ppe_port->gmib_stats->tx_pkt256to511 += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXPKT512TO1023, &val);
	ppe_port->gmib_stats->tx_pkt512to1023 += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXPKT1024TO1518, &val);
	ppe_port->gmib_stats->tx_pkt1024to1518 += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXPKT1519TOX, &val);
	ppe_port->gmib_stats->tx_pkt1519tomax += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXBYTE_L, &val);
	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXBYTE_H, &hi);
	ppe_port->gmib_stats->tx_pktbyte += (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXCOLLISIONS, &val);
	ppe_port->gmib_stats->tx_collisions += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXABORTCOL, &val);
	ppe_port->gmib_stats->tx_abortcol += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXMULTICOL, &val);
	ppe_port->gmib_stats->tx_multicol += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXSINGLECOL, &val);
	ppe_port->gmib_stats->tx_singlecol += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXEXCESSIVEDEFER, &val);
	ppe_port->gmib_stats->tx_exesdeffer += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXDEFER, &val);
	ppe_port->gmib_stats->tx_deffer += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXLATECOL, &val);
	ppe_port->gmib_stats->tx_latecol += (u64)val;

	ppe_read(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_TXUNI, &val);
	ppe_port->gmib_stats->tx_unicast += (u64)val;

	spin_unlock(&ppe_port->stats_lock);
}

/* Get XGMAC MIBs from XGMAC registers */
static void ppe_xgmib_stats_update(struct ppe_device *ppe_dev, int port,
				   struct ppe_xgmib_hw_stats *xgmib_hw_stats)
{
	u32 val, hi;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_OCTET_COUNT_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_OCTET_COUNT_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->tx_bytes = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_FRAME_COUNT_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_FRAME_COUNT_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->tx_frames = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_BROADCAST_FRAMES_GOOD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_BROADCAST_FRAMES_GOOD_HIGH, &hi);
	xgmib_hw_stats->tx_broadcast_g = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_MULTICAST_FRAMES_GOOD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_MULTICAST_FRAMES_GOOD_HIGH, &hi);
	xgmib_hw_stats->tx_multicast_g = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_64OCTETS_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_64OCTETS_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->tx_pkt64 = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_65TO127OCTETS_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_65TO127OCTETS_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->tx_pkt65to127 = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_128TO255OCTETS_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_128TO255OCTETS_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->tx_pkt128to255 = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_256TO511OCTETS_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_256TO511OCTETS_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->tx_pkt256to511 = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_512TO1023OCTETS_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_512TO1023OCTETS_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->tx_pkt512to1023 = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_1024TOMAXOCTETS_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_1024TOMAXOCTETS_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->tx_pkt1024tomax = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_UNICAST_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_UNICAST_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->tx_unicast = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_MULTICAST_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_MULTICAST_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->tx_multicast = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_BROADCAST_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_BROADCAST_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->tx_broadcast = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_UNDERFLOW_ERROR_FRAMES_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_UNDERFLOW_ERROR_FRAMES_HIGH, &hi);
	xgmib_hw_stats->tx_underflow_err = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_OCTET_COUNT_GOOD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_OCTET_COUNT_GOOD_HIGH, &hi);
	xgmib_hw_stats->tx_bytes_g = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_FRAME_COUNT_GOOD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_FRAME_COUNT_GOOD_HIGH, &hi);
	xgmib_hw_stats->tx_frame_g = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_PAUSE_FRAMES_LOW, &val);
	xgmib_hw_stats->tx_pause = (u64)val;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_VLAN_FRAMES_GOOD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_VLAN_FRAMES_GOOD_HIGH, &hi);
	xgmib_hw_stats->tx_vlan_g = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_LPI_USEC_CNTR, &val);
	xgmib_hw_stats->tx_lpi_usec = (u64)val;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_LPI_TRAN_CNTR, &val);
	xgmib_hw_stats->tx_lpi_tran = (u64)val;

	/* rx mib stats */
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_FRAME_COUNT_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_FRAME_COUNT_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->rx_frames = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_OCTET_COUNT_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_OCTET_COUNT_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->rx_bytes = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_OCTET_COUNT_GOOD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_OCTET_COUNT_GOOD_HIGH, &hi);
	xgmib_hw_stats->rx_bytes_g = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_BROADCAST_FRAMES_GOOD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_BROADCAST_FRAMES_GOOD_HIGH, &hi);
	xgmib_hw_stats->rx_broadcast_g = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_MULTICAST_FRAMES_GOOD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_MULTICAST_FRAMES_GOOD_HIGH, &hi);
	xgmib_hw_stats->rx_multicast_g = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_CRC_ERROR_FRAMES_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_CRC_ERROR_FRAMES_HIGH, &hi);
	xgmib_hw_stats->rx_crc_err = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_FRAG_ERROR_FRAMES, &val);
	xgmib_hw_stats->rx_runt_err = (u64)val;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_JABBER_ERROR_FRAMES, &val);
	xgmib_hw_stats->rx_jabber_err = (u64)val;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_UNDERSIZE_FRAMES_GOOD, &val);
	xgmib_hw_stats->rx_undersize_g = (u64)val;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_OVERSIZE_FRAMES_GOOD, &val);
	xgmib_hw_stats->rx_oversize_g = (u64)val;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_64OCTETS_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_64OCTETS_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->rx_pkt64 = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_65TO127OCTETS_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_65TO127OCTETS_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->rx_pkt65to127 = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_128TO255OCTETS_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_128TO255OCTETS_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->rx_pkt128to255 = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_256TO511OCTETS_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_256TO511OCTETS_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->rx_pkt256to511 = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_512TO1023OCTETS_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_512TO1023OCTETS_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->rx_pkt512to1023 = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_1024TOMAXOCTETS_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_1024TOMAXOCTETS_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->rx_pkt1024tomax = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_UNICAST_FRAMES_GOOD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_UNICAST_FRAMES_GOOD_HIGH, &hi);
	xgmib_hw_stats->rx_unicast_g = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_LENGTH_ERROR_FRAMES_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_LENGTH_ERROR_FRAMES_HIGH, &hi);
	xgmib_hw_stats->rx_len_err = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_OUTOFRANGE_FRAMES_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_OUTOFRANGE_FRAMES_HIGH, &hi);
	xgmib_hw_stats->rx_outofrange_err = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_PAUSE_FRAMES_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_PAUSE_FRAMES_HIGH, &hi);
	xgmib_hw_stats->rx_pause = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_FIFOOVERFLOW_FRAMES_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_FIFOOVERFLOW_FRAMES_HIGH, &hi);
	xgmib_hw_stats->rx_fifo_overflow = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_VLAN_FRAMES_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_VLAN_FRAMES_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->rx_vlan = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_WATCHDOG_ERROR_FRAMES, &val);
	xgmib_hw_stats->rx_wdog_err = (u64)val;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_LPI_USEC_CNTR, &val);
	xgmib_hw_stats->rx_lpi_usec = (u64)val;

	ppe_read(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_LPI_TRAN_CNTR, &val);
	xgmib_hw_stats->rx_lpi_tran = (u64)val;

	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_DISCARD_FRAME_COUNT_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_DISCARD_FRAME_COUNT_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->rx_drop_frames = (u64)val | (u64)hi << 32;

	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_DISCARD_OCTET_COUNT_GOOD_BAD_LOW, &val);
	ppe_read(ppe_dev,
		 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_DISCARD_OCTET_COUNT_GOOD_BAD_HIGH, &hi);
	xgmib_hw_stats->rx_drop_bytes = (u64)val | (u64)hi << 32;
}

static void ppe_gmib_stats_poll(struct work_struct *work)
{
	struct ppe_port *ppe_port = container_of(work, struct ppe_port,
						 gmib_read.work);

	ppe_gmib_stats_update(ppe_port);

	schedule_delayed_work(&ppe_port->gmib_read,
			      msecs_to_jiffies(PPE_GMIB_STATS_POLL_INTERVAL));
}

static void ppe_phylink_mac_config(struct ppe_device *ppe_dev, int port,
				   unsigned int mode, const struct phylink_link_state *state)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);
	int mac_type;
	u32 val;

	if (!ppe_port) {
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);
		return;
	}

	switch (state->interface) {
	case PHY_INTERFACE_MODE_USXGMII:
	case PHY_INTERFACE_MODE_2500BASEX:
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_QUSGMII:
		mac_type = PPE_MAC_TYPE_XGMAC;
		break;
	default:
		mac_type = PPE_MAC_TYPE_GMAC;
		break;
	}

	if (ppe_port->mac_type != mac_type) {
		/* Reset port mac for gmac */
		if (mac_type == PPE_MAC_TYPE_GMAC)
			ppe_port_mac_reset(ppe_dev, port);

		/* Port mux to select gmac or xgmac */
		mutex_lock(&ppe_dev->reg_mutex);
		ppe_read(ppe_dev, PPE_PORT_MUX_CTRL, &val);
		if (mac_type == PPE_MAC_TYPE_GMAC)
			val &= ~PPE_PORT_MAC_SEL(port);
		else
			val |= PPE_PORT_MAC_SEL(port);
		if (port == PPE_PORT5)
			val |= PPE_PORT5_PCS_SEL;

		ppe_write(ppe_dev, PPE_PORT_MUX_CTRL, val);
		mutex_unlock(&ppe_dev->reg_mutex);
		ppe_port->mac_type = mac_type;
	}

	/* Reset ppe port link status when interface changes,
	 * this allows PPE MAC and UNIPHY to be configured
	 * according to the port link up status in ppe phylink
	 * mac link up.
	 */
	if (state->interface != ppe_port->interface) {
		ppe_port->speed = SPEED_UNKNOWN;
		ppe_port->duplex = DUPLEX_UNKNOWN;
		ppe_port->pause = MLO_PAUSE_NONE;
		ppe_port->interface = state->interface;
	}

	dev_info(ppe_dev->dev, "PPE port %d mac config: interface %s, mac_type %d\n",
		 port, phy_modes(state->interface), mac_type);
}

static struct phylink_pcs *ppe_phylink_mac_select_pcs(struct ppe_device *ppe_dev,
						      int port, phy_interface_t interface)
{
	struct ppe_uniphy *uniphy = (struct ppe_uniphy *)ppe_dev->uniphy;
	int ppe_type = ppe_type_get(ppe_dev);
	int index;

	switch (port) {
	case PPE_PORT6:
		index = 2;
		break;
	case PPE_PORT5:
		index = 1;
		break;
	case PPE_PORT4:
	case PPE_PORT3:
		index = 0;
		break;
	case PPE_PORT2:
		if (ppe_type == PPE_TYPE_MPPE)
			index = 1;
		else if (ppe_type == PPE_TYPE_APPE)
			index = 0;
		break;
	case PPE_PORT1:
		index = 0;
		break;
	default:
		index = -1;
		break;
	}

	if (index >= 0)
		return &uniphy[index].pcs;
	else
		return NULL;
}

static void ppe_phylink_mac_link_up(struct ppe_device *ppe_dev, int port,
				    struct phy_device *phy,
				    unsigned int mode, phy_interface_t interface,
				    int speed, int duplex, bool tx_pause, bool rx_pause)
{
	struct phylink_pcs *pcs = ppe_phylink_mac_select_pcs(ppe_dev, port, interface);
	struct ppe_uniphy *uniphy = pcs_to_ppe_uniphy(pcs);
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);

	/* Wait uniphy auto-negotiation completion */
	ppe_uniphy_autoneg_complete_check(uniphy, port);

	if (speed != ppe_port->speed ||
	    duplex != ppe_port->duplex ||
		tx_pause != !!(ppe_port->pause & MLO_PAUSE_TX) ||
		rx_pause != !!(ppe_port->pause & MLO_PAUSE_RX)) {
		/* Disable gcc uniphy port clk */
		ppe_uniphy_port_gcc_clock_en_set(uniphy, port, false);

		if (speed != ppe_port->speed) {
			/* Set gcc port speed clock */
			ppe_gcc_port_speed_clk_set(ppe_dev, port, speed, interface);
			fsleep(10000);
			/* Set uniphy channel speed */
			ppe_uniphy_speed_set(uniphy, port, speed);
			/* Set mac speed */
			ppe_mac_speed_set(ppe_dev, port, speed, interface);
			ppe_port->speed = speed;
		}

		if (duplex != ppe_port->duplex) {
			/* Set uniphy channel duplex */
			ppe_uniphy_duplex_set(uniphy, port, duplex);
			/* Set mac duplex */
			ppe_mac_duplex_set(ppe_dev, port, duplex);
			ppe_port->duplex = duplex;
		}

		if (tx_pause != !!(ppe_port->pause & MLO_PAUSE_TX)) {
			/* Set mac tx flow ctrl */
			ppe_mac_txfc_status_set(ppe_dev, port, tx_pause);
			if (tx_pause)
				ppe_port->pause |= MLO_PAUSE_TX;
			else
				ppe_port->pause &= ~MLO_PAUSE_TX;
		}

		if (rx_pause != !!(ppe_port->pause & MLO_PAUSE_RX)) {
			/* Set mac rx flow ctrl */
			ppe_mac_rxfc_status_set(ppe_dev, port, rx_pause);
			if (rx_pause)
				ppe_port->pause |= MLO_PAUSE_RX;
			else
				ppe_port->pause &= ~MLO_PAUSE_RX;
		}

		/* Enable gcc uniphy port clk */
		ppe_uniphy_port_gcc_clock_en_set(uniphy, port, true);

		/* Reset uniphy channel adapter */
		ppe_uniphy_adapter_reset(uniphy, port);
	}

	/* Enable ppe mac tx and rx */
	ppe_mac_txmac_en_set(ppe_dev, port, true);
	ppe_mac_rxmac_en_set(ppe_dev, port, true);

	/* Enable ppe bridge port tx mac */
	ppe_port_bridge_txmac_en_set(ppe_dev, port, true);

	/* Start gmib statistics polling */
	schedule_delayed_work(&ppe_port->gmib_read, 0);

	dev_info(ppe_dev->dev,
		 "PPE port %d interface %s link up - %s%s - pause tx %d rx %d\n",
		 port, phy_modes(interface), phy_speed_to_str(speed),
		 phy_duplex_to_str(duplex), tx_pause, rx_pause);
}

static void ppe_phylink_mac_link_down(struct ppe_device *ppe_dev, int port,
				      unsigned int mode, phy_interface_t interface)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);

	if (!ppe_port)
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);

	/* Disable ppe port bridge tx mac */
	ppe_port_bridge_txmac_en_set(ppe_dev, port, false);

	/* Disable ppe mac rx */
	ppe_mac_rxmac_en_set(ppe_dev, port, false);
	fsleep(10000);

	/* Disable ppe mac tx */
	ppe_mac_txmac_en_set(ppe_dev, port, false);

	/* Stop gmib statistics polling */
	cancel_delayed_work_sync(&ppe_port->gmib_read);

	dev_info(ppe_dev->dev, "PPE port %d interface %s link down\n",
		 port, phy_modes(interface));
}

static int ppe_mac_init(struct platform_device *pdev)
{
	struct device_node *ports_node, *port_node;
	struct ppe_device *ppe_dev = platform_get_drvdata(pdev);
	struct ppe_ports *ppe_ports = NULL;
	phy_interface_t phy_mode = PHY_INTERFACE_MODE_NA;
	int i = 0, port = 0, err = 0, port_num = 0;

	ports_node = of_get_child_by_name(pdev->dev.of_node, "qcom,port_phyinfo");
	if (!ports_node) {
		dev_err(&pdev->dev, "Failed to get qcom port phy info node\n");
		return -ENODEV;
	}

	port_num = of_get_child_count(ports_node);

	ppe_ports = devm_kzalloc(&pdev->dev,
				 struct_size(ppe_ports, port, port_num),
				 GFP_KERNEL);
	if (!ppe_ports) {
		err = -ENOMEM;
		goto err_ports_node_put;
	}

	ppe_dev->ports = ppe_ports;
	ppe_ports->num = port_num;

	for_each_available_child_of_node(ports_node, port_node) {
		err = of_property_read_u32(port_node, "port_id", &port);
		if (err) {
			dev_err(&pdev->dev, "Failed to get port id\n");
			goto err_port_node_put;
		}

		err = of_get_phy_mode(port_node, &phy_mode);
		if (err) {
			dev_err(&pdev->dev, "Failed to get phy mode\n");
			goto err_port_node_put;
		}

		ppe_ports->port[i].ppe_dev = ppe_dev;
		ppe_ports->port[i].port_id = port;
		ppe_ports->port[i].np = port_node;
		ppe_ports->port[i].interface = phy_mode;
		ppe_ports->port[i].mac_type = PPE_MAC_TYPE_NA;
		ppe_ports->port[i].speed = SPEED_UNKNOWN;
		ppe_ports->port[i].duplex = DUPLEX_UNKNOWN;
		ppe_ports->port[i].pause = MLO_PAUSE_NONE;
		ppe_ports->port[i].gmib_stats = devm_kzalloc(&pdev->dev,
							     sizeof(*ppe_ports->port[i].gmib_stats),
							     GFP_KERNEL);
		spin_lock_init(&ppe_ports->port[i].stats_lock);
		INIT_DELAYED_WORK(&ppe_ports->port[i].gmib_read, ppe_gmib_stats_poll);
		i++;

		/* Port gmac HW initialization */
		ppe_mask(ppe_dev,
			 PPE_PORT_GMAC_ADDR(port) + GMAC_ENABLE,
			 GMAC_MAC_EN, 0);

		ppe_mask(ppe_dev,
			 PPE_PORT_GMAC_ADDR(port) + GMAC_MAC_JUMBO_SIZE,
			 GMAC_JUMBO_SIZE_MASK,
			 FIELD_PREP(GMAC_JUMBO_SIZE_MASK, MAC_MAX_FRAME_SIZE));

		ppe_mask(ppe_dev,
			 PPE_PORT_GMAC_ADDR(port) + GMAC_MAC_CTRL2,
			 GMAC_INIT_CTRL2_FIELD, GMAC_INIT_CTRL2);

		ppe_mask(ppe_dev,
			 PPE_PORT_GMAC_ADDR(port) + GMAC_MAC_DBG_CTRL,
			 GMAC_HIGH_IPG_MASK,
			 FIELD_PREP(GMAC_HIGH_IPG_MASK, GMAC_IPG_CHECK));

		ppe_mask(ppe_dev,
			 PPE_PORT_GMAC_ADDR(port) + GMAC_MAC_MIB_CTRL,
			 MAC_MIB_EN | MAC_MIB_RD_CLR | MAC_MIB_RESET,
			 MAC_MIB_EN | MAC_MIB_RD_CLR | MAC_MIB_RESET);

		ppe_mask(ppe_dev,
			 PPE_PORT_GMAC_ADDR(port) + GMAC_MAC_MIB_CTRL,
			 MAC_MIB_RESET, 0);

		/* Port xgmac HW initialization */
		ppe_mask(ppe_dev,
			 PPE_PORT_XGMAC_ADDR(port) + XGMAC_TX_CONFIGURATION,
			 XGMAC_INIT_TX_CONFIG_FIELD, XGMAC_INIT_TX_CONFIG);

		ppe_mask(ppe_dev,
			 PPE_PORT_XGMAC_ADDR(port) + XGMAC_RX_CONFIGURATION,
			 XGMAC_INIT_RX_CONFIG_FIELD, XGMAC_INIT_RX_CONFIG);

		ppe_mask(ppe_dev,
			 PPE_PORT_XGMAC_ADDR(port) + XGMAC_WATCHDOG_TIMEOUT,
			 XGMAC_INIT_WATCHDOG_FIELD, XGMAC_INIT_WATCHDOG);

		ppe_mask(ppe_dev,
			 PPE_PORT_XGMAC_ADDR(port) + XGMAC_PACKET_FILTER,
			 XGMAC_INIT_FILTER_FIELD, XGMAC_INIT_FILTER);

		ppe_mask(ppe_dev,
			 PPE_PORT_XGMAC_ADDR(port) + XGMAC_MMC_CONTROL,
			 XGMAC_MCF | XGMAC_CNTRST, XGMAC_CNTRST);
	}

	of_node_put(ports_node);
	dev_info(ppe_dev->dev, "QCOM PPE MAC init success\n");
	return 0;

err_port_node_put:
	of_node_put(port_node);
err_ports_node_put:
	of_node_put(ports_node);
	return err;
}

static void ppe_mac_config(struct phylink_config *config, unsigned int mode,
			   const struct phylink_link_state *state)
{
	struct ppe_device *ppe_dev = NULL;
	struct ppe_port *ppe_port = container_of(config,
						 struct ppe_port,
						 phylink_config);

	if (!ppe_port)
		dev_err(ppe_dev->dev, "Failed to find ppe port\n");

	ppe_dev = ppe_port->ppe_dev;

	if (ppe_dev && ppe_dev->ppe_ops &&
	    ppe_dev->ppe_ops->phylink_mac_config) {
		ppe_dev->ppe_ops->phylink_mac_config(ppe_dev,
						     ppe_port->port_id,
						     mode, state);
	} else {
		dev_err(ppe_dev->dev,
			"Failed to find ppe device mac config operation\n");
	}
}

static void ppe_mac_link_down(struct phylink_config *config, unsigned int mode,
			      phy_interface_t interface)
{
	struct ppe_device *ppe_dev = NULL;
	struct ppe_port *ppe_port = container_of(config,
						 struct ppe_port,
						 phylink_config);

	if (!ppe_port)
		dev_err(ppe_dev->dev, "Failed to find ppe port\n");

	ppe_dev = ppe_port->ppe_dev;

	if (ppe_dev && ppe_dev->ppe_ops &&
	    ppe_dev->ppe_ops->phylink_mac_link_down) {
		ppe_dev->ppe_ops->phylink_mac_link_down(ppe_dev,
							ppe_port->port_id,
							mode, interface);
	} else {
		dev_err(ppe_dev->dev,
			"Failed to find ppe device link down operation\n");
	}
}

static void ppe_mac_link_up(struct phylink_config *config,
			    struct phy_device *phy,
			    unsigned int mode, phy_interface_t interface,
			    int speed, int duplex, bool tx_pause, bool rx_pause)
{
	struct ppe_device *ppe_dev = NULL;
	struct ppe_port *ppe_port = container_of(config,
						 struct ppe_port,
						 phylink_config);

	if (!ppe_port)
		dev_err(ppe_dev->dev, "Failed to find ppe port\n");

	ppe_dev = ppe_port->ppe_dev;

	if (ppe_dev && ppe_dev->ppe_ops &&
	    ppe_dev->ppe_ops->phylink_mac_link_up) {
		ppe_dev->ppe_ops->phylink_mac_link_up(ppe_dev,
						      ppe_port->port_id,
						      phy, mode, interface,
						      speed, duplex,
						      tx_pause, rx_pause);
	} else {
		dev_err(ppe_dev->dev,
			"Failed to find ppe device link up operation\n");
	}
}

static struct phylink_pcs *ppe_mac_select_pcs(struct phylink_config *config,
					      phy_interface_t interface)
{
	struct ppe_device *ppe_dev = NULL;
	struct ppe_port *ppe_port = container_of(config,
						 struct ppe_port,
						 phylink_config);

	if (!ppe_port) {
		dev_err(ppe_dev->dev, "Failed to find ppe port");
		return NULL;
	}

	ppe_dev = ppe_port->ppe_dev;

	if (ppe_dev && ppe_dev->ppe_ops &&
	    ppe_dev->ppe_ops->phylink_mac_select_pcs) {
		return ppe_dev->ppe_ops->phylink_mac_select_pcs(ppe_dev,
								ppe_port->port_id,
								interface);
	} else {
		dev_err(ppe_dev->dev,
			"Failed to find ppe device pcs select operation\n");
		return NULL;
	}
}

static const struct phylink_mac_ops ppe_phylink_ops = {
	.mac_config = ppe_mac_config,
	.mac_link_down = ppe_mac_link_down,
	.mac_link_up = ppe_mac_link_up,
	.mac_select_pcs = ppe_mac_select_pcs,
};

static struct phylink *ppe_phylink_setup(struct ppe_device *ppe_dev,
					 struct net_device *netdev,
					 int port)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);
	int err;

	if (!ppe_port) {
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);
		return NULL;
	}

	/* per port phylink capability */
	ppe_port->phylink_config.dev = &netdev->dev;
	ppe_port->phylink_config.type = PHYLINK_NETDEV;
	ppe_port->phylink_config.mac_capabilities = MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
		MAC_10 | MAC_100 | MAC_1000 | MAC_2500FD | MAC_5000FD | MAC_10000FD;
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
	__set_bit(PHY_INTERFACE_MODE_QSGMII,
		  ppe_port->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_QUSGMII,
		  ppe_port->phylink_config.supported_interfaces);

	/* create phylink */
	ppe_port->phylink = phylink_create(&ppe_port->phylink_config,
					   of_fwnode_handle(ppe_port->np),
					   ppe_port->interface, &ppe_phylink_ops);
	if (IS_ERR(ppe_port->phylink)) {
		dev_err(ppe_dev->dev, "Failed to create phylink for port %d\n", port);
		return NULL;
	}

	/* connect phylink */
	err = phylink_of_phy_connect(ppe_port->phylink, ppe_port->np, 0);
	if (err) {
		dev_err(ppe_dev->dev, "Failed to connect phylink for port %d\n", port);
		phylink_destroy(ppe_port->phylink);
		ppe_port->phylink = NULL;
		return NULL;
	}

	return ppe_port->phylink;
}

static void ppe_phylink_destroy(struct ppe_device *ppe_dev, int port)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);

	if (!ppe_port)
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);

	if (ppe_port->phylink) {
		rtnl_lock();
		phylink_disconnect_phy(ppe_port->phylink);
		rtnl_unlock();
		phylink_destroy(ppe_port->phylink);
		ppe_port->phylink = NULL;
	}
}

static int ppe_get_sset_count(struct ppe_device *ppe_dev, int port, int sset)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);

	if (!ppe_port) {
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);
		return -ENODEV;
	}

	if (sset != ETH_SS_STATS)
		return 0;

	if (ppe_port->mac_type == PPE_MAC_TYPE_XGMAC)
		return ARRAY_SIZE(ppe_ethtool_gstrings_xgmib_stats);
	else
		return ARRAY_SIZE(ppe_ethtool_gstrings_gmib_stats);
}

static void ppe_get_strings(struct ppe_device *ppe_dev, int port, u32 stringset, u8 *data)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);
	int i;

	if (!ppe_port)
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);

	if (stringset != ETH_SS_STATS)
		return;

	if (ppe_port->mac_type == PPE_MAC_TYPE_XGMAC) {
		for (i = 0; i < ARRAY_SIZE(ppe_ethtool_gstrings_xgmib_stats); i++)
			memcpy(data + i * ETH_GSTRING_LEN,
			       ppe_ethtool_gstrings_xgmib_stats[i].name, ETH_GSTRING_LEN);
	} else {
		for (i = 0; i < ARRAY_SIZE(ppe_ethtool_gstrings_gmib_stats); i++)
			memcpy(data + i * ETH_GSTRING_LEN,
			       ppe_ethtool_gstrings_gmib_stats[i].name, ETH_GSTRING_LEN);
	}
}

static void ppe_get_ethtool_stats(struct ppe_device *ppe_dev, int port, u64 *data)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);
	u64 *data_src;
	int i;

	if (!ppe_port)
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);

	if (ppe_port->mac_type == PPE_MAC_TYPE_XGMAC) {
		struct ppe_xgmib_hw_stats xgmib_hw_stats;

		ppe_xgmib_stats_update(ppe_dev, port, &xgmib_hw_stats);
		data_src = (u64 *)(&xgmib_hw_stats);
		for (i = 0; i < ARRAY_SIZE(ppe_ethtool_gstrings_xgmib_stats); i++)
			data[i] = *(data_src + ppe_ethtool_gstrings_xgmib_stats[i].offset);
	} else {
		ppe_gmib_stats_update(ppe_port);
		data_src = (u64 *)(ppe_port->gmib_stats);
		for (i = 0; i < ARRAY_SIZE(ppe_ethtool_gstrings_gmib_stats); i++)
			data[i] = *(data_src + ppe_ethtool_gstrings_gmib_stats[i].offset);
	}
}

static void ppe_get_stats64(struct ppe_device *ppe_dev, int port, struct rtnl_link_stats64 *s)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);

	if (!ppe_port)
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);

	if (ppe_port->mac_type == PPE_MAC_TYPE_XGMAC) {
		struct ppe_xgmib_hw_stats xgmib_hw_stats;

		ppe_xgmib_stats_update(ppe_dev, port, &xgmib_hw_stats);
		s->rx_packets = xgmib_hw_stats.rx_unicast_g +
			xgmib_hw_stats.rx_broadcast_g + xgmib_hw_stats.rx_multicast_g;
		s->tx_packets = xgmib_hw_stats.tx_unicast +
			xgmib_hw_stats.tx_broadcast_g + xgmib_hw_stats.tx_multicast_g;
		s->rx_bytes = xgmib_hw_stats.rx_bytes;
		s->tx_bytes = xgmib_hw_stats.tx_bytes;
		s->multicast = xgmib_hw_stats.rx_multicast_g;

		s->rx_crc_errors = xgmib_hw_stats.rx_crc_err;
		s->rx_frame_errors = xgmib_hw_stats.rx_runt_err;
		s->rx_fifo_errors = xgmib_hw_stats.rx_fifo_overflow;
		s->rx_length_errors = xgmib_hw_stats.rx_len_err;
		s->rx_errors = s->rx_crc_errors + s->rx_frame_errors +
			s->rx_fifo_errors + s->rx_length_errors;
		s->rx_dropped = xgmib_hw_stats.rx_drop_frames + s->rx_errors;

		s->tx_fifo_errors = xgmib_hw_stats.tx_underflow_err;
		s->tx_errors = s->tx_fifo_errors;
	} else {
		ppe_gmib_stats_update(ppe_port);
		s->rx_packets = ppe_port->gmib_stats->rx_unicast +
			ppe_port->gmib_stats->rx_broadcast + ppe_port->gmib_stats->rx_multicast;
		s->tx_packets = ppe_port->gmib_stats->tx_unicast +
			ppe_port->gmib_stats->tx_broadcast + ppe_port->gmib_stats->tx_multicast;
		s->rx_bytes = ppe_port->gmib_stats->rx_pktgoodbyte;
		s->tx_bytes = ppe_port->gmib_stats->tx_pktbyte;

		s->rx_crc_errors = ppe_port->gmib_stats->rx_fcserr +
			ppe_port->gmib_stats->rx_jmbfcserr;
		s->rx_frame_errors = ppe_port->gmib_stats->rx_alignerr +
			ppe_port->gmib_stats->rx_jmbalignerr;
		s->rx_fifo_errors = ppe_port->gmib_stats->rx_runt;
		s->rx_errors = s->rx_crc_errors + s->rx_frame_errors + s->rx_fifo_errors;
		s->rx_dropped = ppe_port->gmib_stats->rx_toolong + s->rx_errors;

		s->tx_fifo_errors = ppe_port->gmib_stats->tx_underrun;
		s->tx_aborted_errors = ppe_port->gmib_stats->tx_abortcol;
		s->tx_errors = s->tx_fifo_errors + s->tx_aborted_errors;
		s->collisions = ppe_port->gmib_stats->tx_collisions;
		s->multicast = ppe_port->gmib_stats->rx_multicast;
	}
}

static int ppe_set_mac_address(struct ppe_device *ppe_dev, int port, u8 *macaddr)
{
	u32 reg_val;
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);

	if (!ppe_port) {
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);
		return -ENODEV;
	}

	if (ppe_port->mac_type == PPE_MAC_TYPE_XGMAC) {
		reg_val = (macaddr[5] << 8) | macaddr[4] | XGMAC_ADDR_EN;
		ppe_write(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_MAC_ADDR0_HIGH, reg_val);
		reg_val = (macaddr[3] << 24) | (macaddr[2] << 16) | (macaddr[1] << 8) | macaddr[0];
		ppe_write(ppe_dev, PPE_PORT_XGMAC_ADDR(port) + XGMAC_MAC_ADDR0_LOW, reg_val);
	} else {
		reg_val = (macaddr[5] << 8) | macaddr[4];
		ppe_write(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_GOL_MAC_ADDR0, reg_val);
		reg_val = (macaddr[0] << 24) | (macaddr[1] << 16) | (macaddr[2] << 8) | macaddr[3];
		ppe_write(ppe_dev, PPE_PORT_GMAC_ADDR(port) + GMAC_GOL_MAC_ADDR1, reg_val);
	}

	return 0;
}

static int ppe_set_mac_eee(struct ppe_device *ppe_dev, int port, struct ethtool_eee *eee)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);
	u32 reg_val;

	if (!ppe_port) {
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);
		return -ENODEV;
	}

	if (ppe_port->mac_type == PPE_MAC_TYPE_XGMAC) {
		ppe_read(ppe_dev,
			 PPE_PORT_XGMAC_ADDR(port) + XGMAC_LPI_CONTROL_STATUS,
			 &reg_val);
		reg_val |= (XGMAC_LPI_PLS | XGMAC_LPI_TXA | XGMAC_LPI_TE);
		if (eee->tx_lpi_enabled)
			reg_val |= XGMAC_LPI_TXEN;
		else
			reg_val &= ~XGMAC_LPI_TXEN;
		ppe_write(ppe_dev,
			  PPE_PORT_XGMAC_ADDR(port) + XGMAC_LPI_CONTROL_STATUS,
			  reg_val);
		ppe_mask(ppe_dev,
			 PPE_PORT_XGMAC_ADDR(port) + XGMAC_1US_TIC_COUNTER,
			 XGMAC_1US_TIC_CNTR, FIELD_PREP(XGMAC_1US_TIC_CNTR, 0x15f));
		ppe_mask(ppe_dev,
			 PPE_PORT_XGMAC_ADDR(port) + XGMAC_LPI_AUTO_ENTRY_TIMER,
			 XGMAC_LPI_ET, FIELD_PREP(XGMAC_LPI_ET, 0x2c));
	} else {
		ppe_read(ppe_dev, PPE_LPI_LPI_EN, &reg_val);
		if (eee->tx_lpi_enabled)
			reg_val |= PPE_LPI_PORT_EN(port);
		else
			reg_val &= ~PPE_LPI_PORT_EN(port);
		ppe_write(ppe_dev, PPE_LPI_LPI_EN, reg_val);
	}

	return 0;
}

static int ppe_get_mac_eee(struct ppe_device *ppe_dev, int port, struct ethtool_eee *eee)
{
	struct ppe_port *ppe_port = ppe_port_get(ppe_dev, port);
	u32 reg_val;

	if (!ppe_port) {
		dev_err(ppe_dev->dev, "Failed to find ppe port %d\n", port);
		return -ENODEV;
	}

	if (ppe_port->mac_type == PPE_MAC_TYPE_XGMAC) {
		ppe_read(ppe_dev,
			 PPE_PORT_XGMAC_ADDR(port) + XGMAC_LPI_CONTROL_STATUS,
			 &reg_val);
		if (reg_val & XGMAC_LPI_TXEN)
			eee->tx_lpi_enabled = 1;
		else
			eee->tx_lpi_enabled = 0;
	} else {
		ppe_read(ppe_dev, PPE_LPI_LPI_EN, &reg_val);
		if (reg_val & PPE_LPI_PORT_EN(port))
			eee->tx_lpi_enabled = 1;
		else
			eee->tx_lpi_enabled = 0;
	}

	return 0;
}

bool ppe_is_probed(struct platform_device *pdev)
{
	struct ppe_device *ppe_dev = platform_get_drvdata(pdev);

	return ppe_dev && ppe_dev->is_ppe_probed;
}
EXPORT_SYMBOL_GPL(ppe_is_probed);

struct ppe_device *ppe_dev_get(struct platform_device *pdev)
{
	return platform_get_drvdata(pdev);
}
EXPORT_SYMBOL_GPL(ppe_dev_get);

struct ppe_device_ops *ppe_ops_get(struct platform_device *pdev)
{
	struct ppe_device *ppe_dev = platform_get_drvdata(pdev);

	if (!ppe_dev)
		return NULL;

	return ppe_dev->ppe_ops;
}
EXPORT_SYMBOL_GPL(ppe_ops_get);

static int ppe_port_maxframe_set(struct ppe_device *ppe_dev,
				 int port, int maxframe_size)
{
	union ppe_mru_mtu_ctrl_cfg_u mru_mtu_cfg;

	/* The max frame size should be MTU added by ETH_HLEN in PPE */
	maxframe_size += ETH_HLEN;

	if (port < PPE_MC_MTU_CTRL_TBL_NUM)
		ppe_mask(ppe_dev, PPE_MC_MTU_CTRL_TBL + PPE_MC_MTU_CTRL_TBL_INC * port,
			 PPE_MC_MTU_CTRL_TBL_MTU,
			 FIELD_PREP(PPE_MC_MTU_CTRL_TBL_MTU, maxframe_size));

	memset(&mru_mtu_cfg, 0, sizeof(mru_mtu_cfg));
	ppe_read_tbl(ppe_dev, PPE_MRU_MTU_CTRL_TBL + PPE_MRU_MTU_CTRL_TBL_INC * port,
		     mru_mtu_cfg.val, sizeof(mru_mtu_cfg.val));

	mru_mtu_cfg.bf.mru = maxframe_size;
	mru_mtu_cfg.bf.mtu = maxframe_size;

	return ppe_write_tbl(ppe_dev, PPE_MRU_MTU_CTRL_TBL + PPE_MRU_MTU_CTRL_TBL_INC * port,
			     mru_mtu_cfg.val, sizeof(mru_mtu_cfg.val));
}

static struct ppe_device_ops qcom_ppe_ops = {
	.phylink_setup = ppe_phylink_setup,
	.phylink_destroy = ppe_phylink_destroy,
	.phylink_mac_config = ppe_phylink_mac_config,
	.phylink_mac_link_up = ppe_phylink_mac_link_up,
	.phylink_mac_link_down = ppe_phylink_mac_link_down,
	.phylink_mac_select_pcs = ppe_phylink_mac_select_pcs,
	.get_sset_count = ppe_get_sset_count,
	.get_strings = ppe_get_strings,
	.get_ethtool_stats = ppe_get_ethtool_stats,
	.get_stats64 = ppe_get_stats64,
	.set_mac_address = ppe_set_mac_address,
	.set_mac_eee = ppe_set_mac_eee,
	.get_mac_eee = ppe_get_mac_eee,
	.set_maxframe = ppe_port_maxframe_set,
};

static const struct regmap_range ppe_readable_ranges[] = {
	regmap_reg_range(0x0, 0x1FF), /* GLB */
	regmap_reg_range(0x400, 0x5FF), /* LPI CSR */
	regmap_reg_range(0x1000, 0x11FF), /* GMAC0 */
	regmap_reg_range(0x1200, 0x13FF), /* GMAC1 */
	regmap_reg_range(0x1400, 0x15FF), /* GMAC2 */
	regmap_reg_range(0x1600, 0x17FF), /* GMAC3 */
	regmap_reg_range(0x1800, 0x19FF), /* GMAC4 */
	regmap_reg_range(0x1A00, 0x1BFF), /* GMAC5 */
	regmap_reg_range(0xB000, 0xEFFF), /* PRX CSR */
	regmap_reg_range(0xF000, 0x1EFFF), /* IPE IV */
	regmap_reg_range(0x20000, 0x5FFFF), /* PTX CSR */
	regmap_reg_range(0x60000, 0x9FFFF), /* IPE L2 CSR */
	regmap_reg_range(0xB0000, 0xEFFFF), /* IPO CSR */
	regmap_reg_range(0x100000, 0x17FFFF), /* IPE PC */
	regmap_reg_range(0x180000, 0x1BFFFF), /* PRE IPO CSR */
	regmap_reg_range(0x1D0000, 0x1DFFFF), /* TUNNEL PARSER CSR */
	regmap_reg_range(0x1E0000, 0x1EFFFF), /* INGRESS PARSE CSR */
	regmap_reg_range(0x200000, 0x2FFFFF), /* IPE L3 */
	regmap_reg_range(0x300000, 0x3FFFFF), /* IPE TL */
	regmap_reg_range(0x400000, 0x4FFFFF), /* TM */
	regmap_reg_range(0x500000, 0x503FFF), /* XGMAC0 */
	regmap_reg_range(0x504000, 0x507FFF), /* XGMAC1 */
	regmap_reg_range(0x508000, 0x50BFFF), /* XGMAC2 */
	regmap_reg_range(0x50C000, 0x50FFFF), /* XGMAC3 */
	regmap_reg_range(0x510000, 0x513FFF), /* XGMAC4 */
	regmap_reg_range(0x514000, 0x517FFF), /* XGMAC5 */
	regmap_reg_range(0x600000, 0x6FFFFF), /* BM */
	regmap_reg_range(0x800000, 0x9FFFFF), /* QM */
};

static const struct regmap_access_table ppe_reg_table = {
	.yes_ranges = ppe_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ppe_readable_ranges),
};

static const struct regmap_config ppe_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.rd_table = &ppe_reg_table,
	.wr_table = &ppe_reg_table,
	.max_register = 0x9FFFFF,
	.fast_io = true,
};

static struct ppe_data *ppe_data_init(struct platform_device *pdev)
{
	struct ppe_data *ppe_dev_priv;
	int ret;

	ppe_dev_priv = devm_kzalloc(&pdev->dev, sizeof(*ppe_dev_priv), GFP_KERNEL);
	if (!ppe_dev_priv)
		return ERR_PTR(-ENOMEM);

	if (of_device_is_compatible(pdev->dev.of_node, "qcom,ipq9574-ppe"))
		ppe_dev_priv->ppe_type = PPE_TYPE_APPE;
	else if (of_device_is_compatible(pdev->dev.of_node, "qcom,ipq5332-ppe"))
		ppe_dev_priv->ppe_type = PPE_TYPE_MPPE;
	else
		return ERR_PTR(-EINVAL);

	for (ret = 0; ret < PPE_CLK_MAX; ret++) {
		ppe_dev_priv->clk[ret] = devm_clk_get_optional(&pdev->dev,
							       ppe_clock_name[ret]);

		if (IS_ERR(ppe_dev_priv->clk[ret]))
			dev_err(&pdev->dev, "Failed to get the clock: %s\n",
				ppe_clock_name[ret]);
	}

	for (ret = 0; ret < PPE_RST_MAX; ret++) {
		ppe_dev_priv->rst[ret] =
			devm_reset_control_get_optional_exclusive(&pdev->dev,
								  ppe_reset_name[ret]);
		if (IS_ERR(ppe_dev_priv->rst[ret]))
			dev_err(&pdev->dev, "Failed to get the reset %s!\n",
				ppe_reset_name[ret]);
	}

	return ppe_dev_priv;
}

static int of_parse_ppe_bm(struct ppe_device *ppe_dev,
			   struct device_node *ppe_node)
{
	union ppe_bm_port_fc_cfg_u fc_cfg;
	struct device_node *bm_node;
	int ret, cnt;
	u32 *cfg, reg_val;

	bm_node = of_get_child_by_name(ppe_node, "buffer-management-config");
	if (!bm_node)
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "Fail to get buffer-management-config\n");

	cnt = of_property_count_u32_elems(bm_node, "qcom,group-config");
	if (cnt < 0)
		return dev_err_probe(ppe_dev->dev, cnt,
				     "Fail to qcom,group-config\n");

	cfg = kmalloc_array(cnt, sizeof(*cfg), GFP_KERNEL | __GFP_ZERO);
	if (!cfg)
		return -ENOMEM;

	ret = of_property_read_u32_array(bm_node, "qcom,group-config", cfg, cnt);
	if (ret) {
		dev_err(ppe_dev->dev, "Fail to get qcom,group-config %d\n", ret);
		goto parse_bm_err;
	}

	/* Parse BM group configuration,
	 * the dts propert: qcom,group-config = <group group_buf>;
	 *
	 * There are 3 kinds of buffer types, guaranteed buffer(port based),
	 * shared buffer(group based) and react buffer(cache in-flight packets).
	 *
	 * Maximum 4 groups supported by PPE.
	 */
	ret = 0;
	while ((cnt - ret) / 2) {
		if (cfg[ret] < PPE_BM_SHARED_GROUP_CFG_NUM) {
			reg_val = FIELD_PREP(PPE_BM_SHARED_GROUP_CFG_SHARED_LIMIT, cfg[ret + 1]);

			ppe_write(ppe_dev, PPE_BM_SHARED_GROUP_CFG +
				  PPE_BM_SHARED_GROUP_CFG_INC * cfg[ret], reg_val);
		}
		ret += 2;
	}

	cnt = of_property_count_u32_elems(bm_node, "qcom,port-config");
	if (cnt < 0) {
		dev_err(ppe_dev->dev, "Fail to get qcom,port-config %d\n", cnt);
		goto parse_bm_err;
	}

	cfg = krealloc_array(cfg, cnt, sizeof(*cfg), GFP_KERNEL | __GFP_ZERO);
	if (!cfg) {
		ret = -ENOMEM;
		goto parse_bm_err;
	}

	ret = of_property_read_u32_array(bm_node, "qcom,port-config", cfg, cnt);
	if (ret) {
		dev_err(ppe_dev->dev, "Fail to get qcom,port-config %d\n", ret);
		goto parse_bm_err;
	}

	/* Parse BM port configuration,
	 * the dts property: qcom,port-config = <group port prealloc react ceil
	 * weight res_off res_ceil dynamic>;
	 *
	 * The port based buffer is assigned to the group ID, which is the
	 * buffer dedicated to BM port, and the threshold to generate the
	 * pause frame, the threshold can be configured as the static value
	 * or dynamically adjusted according to the remain buffer.
	 */
	ret = 0;
	while ((cnt - ret) / 9) {
		if (cfg[ret + 1] < PPE_BM_PORT_FC_MODE_NUM) {
			memset(&fc_cfg, 0, sizeof(fc_cfg));

			fc_cfg.bf.pre_alloc = cfg[ret + 2];
			fc_cfg.bf.react_limit = cfg[ret + 3];
			fc_cfg.bf.shared_ceiling_0 = cfg[ret + 4] & 0x7;
			fc_cfg.bf.shared_ceiling_1 = cfg[ret + 4] >> 3;
			fc_cfg.bf.shared_weight = cfg[ret + 5];
			fc_cfg.bf.resum_offset = cfg[ret + 6];
			fc_cfg.bf.resum_floor_th = cfg[ret + 7];
			fc_cfg.bf.shared_dynamic = cfg[ret + 8];
			ppe_write_tbl(ppe_dev, PPE_BM_PORT_FC_CFG +
				      PPE_BM_PORT_FC_CFG_INC * cfg[ret + 1],
				      fc_cfg.val, sizeof(fc_cfg.val));

			reg_val = FIELD_PREP(PPE_BM_PORT_GROUP_ID_SHARED_GROUP_ID, cfg[ret]);
			ppe_write(ppe_dev, PPE_BM_PORT_GROUP_ID +
				  PPE_BM_PORT_GROUP_ID_INC * cfg[ret + 1], reg_val);

			reg_val = FIELD_PREP(PPE_BM_PORT_FC_MODE_EN, 1);
			ppe_write(ppe_dev, PPE_BM_PORT_FC_MODE +
				  PPE_BM_PORT_FC_MODE_INC * cfg[ret + 1], reg_val);
		}
		ret += 9;
	}
	ret = 0;

parse_bm_err:
	kfree(cfg);
	return ret;
}

static int of_parse_ppe_qm(struct ppe_device *ppe_dev,
			   struct device_node *ppe_node)
{
	union ppe_ac_uni_queue_cfg_u uni_queue_cfg;
	union ppe_ac_mul_queue_cfg_u mul_queue_cfg;
	union ppe_ac_grp_cfg_u group_cfg;
	struct device_node *qm_node;
	int ret, cnt, queue_id;
	u32 *cfg;

	qm_node = of_get_child_by_name(ppe_node, "queue-management-config");
	if (!qm_node)
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "Fail to get queue-management-config\n");

	cnt = of_property_count_u32_elems(qm_node, "qcom,group-config");
	if (cnt < 0)
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "Fail to get qcom,group-config\n");

	cfg = kmalloc_array(cnt, sizeof(*cfg), GFP_KERNEL | __GFP_ZERO);
	if (!cfg)
		return -ENOMEM;

	ret = of_property_read_u32_array(qm_node, "qcom,group-config", cfg, cnt);
	if (ret) {
		dev_err(ppe_dev->dev, "Fail to get qcom,group-config\n");
		goto parse_qm_err;
	}

	/* Parse QM group config:
	 * qcom,group-config = <group total prealloc ceil resume_off>;
	 *
	 * For packet enqueue, there are two kinds of buffer type available,
	 * queue based buffer and group(shared) buffer, the queue based buffer
	 * is used firstly, then shared buffer used.
	 *
	 * Maximum 4 groups buffer supported by PPE.
	 */
	ret = 0;
	while ((cnt - ret) / 5) {
		memset(&group_cfg, 0, sizeof(group_cfg));

		ppe_read_tbl(ppe_dev, PPE_AC_GRP_CFG_TBL +
			     PPE_AC_GRP_CFG_TBL_INC * cfg[ret],
			     group_cfg.val, sizeof(group_cfg.val));

		group_cfg.bf.limit = cfg[ret + 1];
		group_cfg.bf.prealloc_limit = cfg[ret + 2];
		group_cfg.bf.dp_thrd_0 = cfg[ret + 3] & 0x3f;
		group_cfg.bf.dp_thrd_1 = cfg[ret + 3] >> 7;
		group_cfg.bf.grn_resume = cfg[ret + 4];

		ppe_write_tbl(ppe_dev, PPE_AC_GRP_CFG_TBL +
			      PPE_AC_GRP_CFG_TBL_INC * cfg[ret],
			      group_cfg.val, sizeof(group_cfg.val));
		ret += 5;
	}

	cnt = of_property_count_u32_elems(qm_node, "qcom,queue-config");
	if (cnt < 0) {
		dev_err(ppe_dev->dev, "Fail to get qcom,queue-config\n");
		goto parse_qm_err;
	}

	cfg = krealloc_array(cfg, cnt, sizeof(*cfg), GFP_KERNEL | __GFP_ZERO);
	if (!cfg) {
		ret = -ENOMEM;
		goto parse_qm_err;
	}

	ret = of_property_read_u32_array(qm_node, "qcom,queue-config", cfg, cnt);
	if (ret) {
		dev_err(ppe_dev->dev, "Fail to get qcom,queue-config\n");
		goto parse_qm_err;
	}

	/* Parse queue based config:
	 * qcom,queue-config = <queue_base queue_num group prealloc
	 * ceil weight resume_off dynamic>;
	 *
	 * There are totally 256(queue id 0-255) unicast queues and 44(256-299)
	 * multicast queues available in PPE, each queue is assigned the
	 * dedicated buffer and ceil to drop packet, the unicast queue supports
	 * static configured ceil value and dynamic ceil value that is adjusted
	 * according to the available group buffers, multicast queue only supports
	 * static ceil.
	 */
	ret = 0;
	while ((cnt - ret) / 8) {
		queue_id = 0;
		while (queue_id < cfg[ret + 1]) {
			if (cfg[ret] + queue_id < PPE_AC_UNI_QUEUE_CFG_TBL_NUM) {
				memset(&uni_queue_cfg, 0, sizeof(uni_queue_cfg));

				ppe_read_tbl(ppe_dev, PPE_AC_UNI_QUEUE_CFG_TBL +
					     PPE_AC_UNI_QUEUE_CFG_TBL_INC * (cfg[ret] + queue_id),
					     uni_queue_cfg.val, sizeof(uni_queue_cfg.val));

				uni_queue_cfg.bf.ac_grp_id = cfg[ret + 2];
				uni_queue_cfg.bf.prealloc_limit = cfg[ret + 3];
				uni_queue_cfg.bf.shared_ceiling = cfg[ret + 4];
				uni_queue_cfg.bf.shared_weight = cfg[ret + 5];
				uni_queue_cfg.bf.grn_resume = cfg[ret + 6];
				uni_queue_cfg.bf.shared_dynamic = cfg[ret + 7];
				uni_queue_cfg.bf.ac_en = 1;

				ppe_write_tbl(ppe_dev, PPE_AC_UNI_QUEUE_CFG_TBL +
					      PPE_AC_UNI_QUEUE_CFG_TBL_INC * (cfg[ret] + queue_id),
					      uni_queue_cfg.val, sizeof(uni_queue_cfg.val));
			} else {
				memset(&mul_queue_cfg, 0, sizeof(mul_queue_cfg));

				ppe_read_tbl(ppe_dev, PPE_AC_MUL_QUEUE_CFG_TBL +
					     PPE_AC_MUL_QUEUE_CFG_TBL_INC * (cfg[ret] + queue_id),
					     mul_queue_cfg.val, sizeof(mul_queue_cfg.val));

				mul_queue_cfg.bf.ac_grp_id = cfg[ret + 2];
				mul_queue_cfg.bf.prealloc_limit = cfg[ret + 3];
				mul_queue_cfg.bf.shared_ceiling = cfg[ret + 4];
				mul_queue_cfg.bf.grn_resume = cfg[ret + 6];
				mul_queue_cfg.bf.ac_en = 1;

				ppe_write_tbl(ppe_dev, PPE_AC_MUL_QUEUE_CFG_TBL +
					      PPE_AC_MUL_QUEUE_CFG_TBL_INC * (cfg[ret] + queue_id),
					      mul_queue_cfg.val, sizeof(mul_queue_cfg.val));
			}

			ppe_mask(ppe_dev, PPE_ENQ_OPR_TBL +
				 PPE_ENQ_OPR_TBL_INC * (cfg[ret] + queue_id),
				 PPE_ENQ_OPR_TBL_DEQ_DISABLE, 0);

			ppe_mask(ppe_dev, PPE_DEQ_OPR_TBL +
				 PPE_DEQ_OPR_TBL_INC * (cfg[ret] + queue_id),
				 PPE_ENQ_OPR_TBL_DEQ_DISABLE, 0);

			queue_id++;
		}
		ret += 8;
	}

	/* Enable queue counter */
	ret = ppe_mask(ppe_dev, PPE_EG_BRIDGE_CONFIG,
		       PPE_EG_BRIDGE_CONFIG_QUEUE_CNT_EN,
		       PPE_EG_BRIDGE_CONFIG_QUEUE_CNT_EN);
parse_qm_err:
	kfree(cfg);
	return ret;
}

static int of_parse_ppe_tdm(struct ppe_device *ppe_dev,
			    struct device_node *ppe_node)
{
	struct device_node *tdm_node;
	u32 *cfg, reg_val;
	int ret, cnt;

	tdm_node = of_get_child_by_name(ppe_node, "tdm-config");
	if (!tdm_node)
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "tdm-config is not defined\n");

	cnt = of_property_count_u32_elems(tdm_node, "qcom,tdm-bm-config");
	if (cnt < 0)
		return dev_err_probe(ppe_dev->dev, -EINVAL,
				     "Fail to get qcom,tdm-bm-config\n");

	cfg = kmalloc_array(cnt, sizeof(*cfg), GFP_KERNEL | __GFP_ZERO);
	if (!cfg)
		return -ENOMEM;

	ret = of_property_read_u32_array(tdm_node, "qcom,tdm-bm-config", cfg, cnt);
	if (ret) {
		dev_err(ppe_dev->dev, "Fail to get qcom,tdm-bm-config\n");
		goto parse_tdm_err;
	}

	/* Parse TDM BM configuration,
	 * the dts property:
	 * qcom,tdm-bm-config = <valid dir port second_valid second_port>;
	 *
	 * This config decides the number ticks available for physical port
	 * to utilize buffer for receiving and transmiting packet.
	 */
	reg_val = FIELD_PREP(PPE_BM_TDM_CTRL_TDM_DEPTH, cnt / 5) |
		  FIELD_PREP(PPE_BM_TDM_CTRL_TDM_OFFSET, 0) |
		  FIELD_PREP(PPE_BM_TDM_CTRL_TDM_EN, 1);
	ret = ppe_write(ppe_dev, PPE_BM_TDM_CTRL, reg_val);
	if (ret)
		return ret;

	ret = 0;
	while ((cnt - ret) / 5) {
		reg_val = FIELD_PREP(PPE_BM_TDM_CFG_TBL_VALID, cfg[ret]) |
			  FIELD_PREP(PPE_BM_TDM_CFG_TBL_DIR, cfg[ret + 1]) |
			  FIELD_PREP(PPE_BM_TDM_CFG_TBL_PORT_NUM, cfg[ret + 2]) |
			  FIELD_PREP(PPE_BM_TDM_CFG_TBL_SECOND_PORT_VALID, cfg[ret + 3]) |
			  FIELD_PREP(PPE_BM_TDM_CFG_TBL_SECOND_PORT, cfg[ret + 4]);

		ppe_write(ppe_dev,
			  PPE_BM_TDM_CFG_TBL + (ret / 5) * PPE_BM_TDM_CFG_TBL_INC,
			  reg_val);
		ret += 5;
	}

	cnt = of_property_count_u32_elems(tdm_node, "qcom,tdm-port-scheduler-config");
	if (cnt < 0) {
		dev_err(ppe_dev->dev, "Fail to get qcom,tdm-port-scheduler-config\n");
		goto parse_tdm_err;
	}

	cfg = krealloc_array(cfg, cnt, sizeof(*cfg), GFP_KERNEL | __GFP_ZERO);
	if (!cfg) {
		ret = -ENOMEM;
		goto parse_tdm_err;
	}

	ret = of_property_read_u32_array(tdm_node, "qcom,tdm-port-scheduler-config",
					 cfg, cnt);
	if (ret) {
		dev_err(ppe_dev->dev, "Fail to get qcom,tdm-port-scheduler-config\n");
		goto parse_tdm_err;
	}

	/* Parse TDM scheduler configuration,
	 * the dts property:
	 * qcom,tdm-port-scheduler-config = <ensch_bmp ensch_port desch_port
	 * desch_second_valid desch_second_port>;
	 *
	 * This config decides the ticks number available for packet enqueue
	 * and dequeue on the physical port.
	 */
	reg_val = FIELD_PREP(PPE_PSCH_TDM_DEPTH_CFG_TDM_DEPTH, cnt / 5);
	ppe_write(ppe_dev, PPE_PSCH_TDM_DEPTH_CFG, reg_val);

	ret = 0;
	while ((cnt - ret) / 5) {
		reg_val = FIELD_PREP(PPE_PSCH_TDM_CFG_TBL_ENS_PORT_BITMAP, cfg[ret]) |
			  FIELD_PREP(PPE_PSCH_TDM_CFG_TBL_ENS_PORT, cfg[ret + 1]) |
			  FIELD_PREP(PPE_PSCH_TDM_CFG_TBL_DES_PORT, cfg[ret + 2]) |
			  FIELD_PREP(PPE_PSCH_TDM_CFG_TBL_DES_SECOND_PORT_EN, cfg[ret + 3]) |
			  FIELD_PREP(PPE_PSCH_TDM_CFG_TBL_DES_SECOND_PORT, cfg[ret + 4]);

		ppe_write(ppe_dev,
			  PPE_PSCH_TDM_CFG_TBL + (ret / 5) * PPE_PSCH_TDM_CFG_TBL_INC,
			  reg_val);
		ret += 5;
	}

	ret = 0;
parse_tdm_err:
	kfree(cfg);
	return ret;
};

static int of_parse_ppe_scheduler_resource(struct ppe_device *ppe_dev,
					   struct device_node *resource_node)
{
	struct device_node *port_node;
	u32 port;

	for_each_available_child_of_node(resource_node, port_node) {
		if (of_property_read_u32(port_node, "port-id", &port))
			return dev_err_probe(ppe_dev->dev, -ENODEV,
					     "port-id not defined on resource\n");

		if (port >= ARRAY_SIZE(ppe_scheduler_res))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid port-id defined on resource\n");

		if (of_property_read_u32_array(port_node, "qcom,ucast-queue",
					       ppe_scheduler_res[port].ucastq,
					       ARRAY_SIZE(ppe_scheduler_res[port].ucastq)))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid qcom,ucast-queue defined on resource\n");

		if (of_property_read_u32_array(port_node, "qcom,mcast-queue",
					       ppe_scheduler_res[port].mcastq,
					       ARRAY_SIZE(ppe_scheduler_res[port].mcastq)))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid qcom,mcast-queue defined on resource\n");

		if (of_property_read_u32_array(port_node, "qcom,l0sp",
					       ppe_scheduler_res[port].l0sp,
					       ARRAY_SIZE(ppe_scheduler_res[port].l0sp)))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid qcom,l0sp defined on resource\n");

		if (of_property_read_u32_array(port_node, "qcom,l0cdrr",
					       ppe_scheduler_res[port].l0cdrr,
					       ARRAY_SIZE(ppe_scheduler_res[port].l0cdrr)))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid qcom,l0cdrr defined on resource\n");

		if (of_property_read_u32_array(port_node, "qcom,l0edrr",
					       ppe_scheduler_res[port].l0edrr,
					       ARRAY_SIZE(ppe_scheduler_res[port].l0edrr)))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid qcom,l0edrr defined on resource\n");

		if (of_property_read_u32_array(port_node, "qcom,l1cdrr",
					       ppe_scheduler_res[port].l1cdrr,
					       ARRAY_SIZE(ppe_scheduler_res[port].l1cdrr)))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid qcom,l1cdrr defined on resource\n");

		if (of_property_read_u32_array(port_node, "qcom,l1edrr",
					       ppe_scheduler_res[port].l1edrr,
					       ARRAY_SIZE(ppe_scheduler_res[port].l1edrr)))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid qcom,l1edrr defined on resource\n");
	}

	return 0;
}

static int of_parse_ppe_scheduler_group_config(struct ppe_device *ppe_dev,
					       struct device_node *group_node,
					       int port,
					       const char *node_name,
					       const char *loop_name)
{
	struct ppe_qos_scheduler_cfg qos_cfg;
	const struct ppe_queue_ops *ppe_queue_ops;
	const __be32 *paddr;
	int ret, len, i, node_id, level, node_max;
	u32 tmp_cfg[5], pri_loop, max_pri;

	ppe_queue_ops = ppe_queue_config_ops_get();
	if (!ppe_queue_ops->queue_scheduler_set)
		return -EINVAL;

	/* The value of the property node_name can be single value
	 * or array value.
	 *
	 * If the array value is defined, the property loop_name should not
	 * be specified.
	 *
	 * If the single value is defined, the queue ID will be added in the
	 * loop value defined by the loop_name.
	 */
	paddr = of_get_property(group_node, node_name, &len);
	if (!paddr)
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "Fail to get queue %s of port %d\n",
				     node_name, port);

	len /= sizeof(u32);

	/* There are two levels scheduler configs, the level 0 scheduler
	 * config is configured on the queue, the level 1 scheduler is
	 * configured on the flow that is from the output of level 0
	 * scheduler.
	 */
	if (!strcmp(node_name, "qcom,flow")) {
		level = 1;
		node_max = PPE_SCHEDULER_L1_NUM;
	} else {
		level = 0;
		node_max = PPE_SCHEDULER_L0_NUM;
	}

	if (of_property_read_u32_array(group_node, "qcom,scheduler-config",
				       tmp_cfg, ARRAY_SIZE(tmp_cfg)))
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "Fail to get qcom,scheduler-config of port %d\n",
				     port);

	if (of_property_read_u32(group_node, loop_name, &pri_loop)) {
		for (i = 0; i < len; i++) {
			node_id = be32_to_cpup(paddr + i);
			if (node_id >= node_max)
				return dev_err_probe(ppe_dev->dev, -EINVAL,
						     "Invalid node ID %d of port %d\n",
						     node_id, port);

			memset(&qos_cfg, 0, sizeof(qos_cfg));

			qos_cfg.sp_id = tmp_cfg[0];
			qos_cfg.c_pri = tmp_cfg[1];
			qos_cfg.c_drr_id = tmp_cfg[2];
			qos_cfg.e_pri = tmp_cfg[3];
			qos_cfg.e_drr_id = tmp_cfg[4];
			qos_cfg.c_drr_wt = 1;
			qos_cfg.e_drr_wt = 1;
			ret = ppe_queue_ops->queue_scheduler_set(ppe_dev,
								 node_id,
								 level,
								 port,
								 qos_cfg);
			if (ret)
				return dev_err_probe(ppe_dev->dev, ret,
						     "scheduler set fail on node ID %d\n",
						     node_id);
		}
	} else {
		/* Only one base node ID allowed to loop. */
		if (len != 1)
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					"Multiple node ID defined to loop for port %d\n",
					port);

		/* Property qcom,drr-max-priority is optional for loop,
		 * if not defined, the default value PPE_SP_PRIORITY_NUM
		 * is used.
		 */
		max_pri = PPE_SP_PRIORITY_NUM;
		of_property_read_u32(group_node, "qcom,drr-max-priority", &max_pri);

		node_id = be32_to_cpup(paddr);
		if (node_id >= node_max)
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					"Invalid node ID %d defined to loop for port %d\n",
					node_id, port);

		for (i = 0; i < pri_loop; i++) {
			memset(&qos_cfg, 0, sizeof(qos_cfg));

			qos_cfg.sp_id = tmp_cfg[0] + i / max_pri;
			qos_cfg.c_pri = tmp_cfg[1] + i % max_pri;
			qos_cfg.c_drr_id = tmp_cfg[2] + i;
			qos_cfg.e_pri = tmp_cfg[3] + i % max_pri;
			qos_cfg.e_drr_id = tmp_cfg[4] + i;
			qos_cfg.c_drr_wt = 1;
			qos_cfg.e_drr_wt = 1;
			ret = ppe_queue_ops->queue_scheduler_set(ppe_dev,
								 node_id + i,
								 level,
								 port,
								 qos_cfg);
			if (ret)
				return dev_err_probe(ppe_dev->dev, ret,
						     "scheduler set fail on node ID %d\n",
						     node_id + i);
		}
	}

	return 0;
}

static int of_parse_ppe_scheduler_config(struct ppe_device *ppe_dev,
					 struct device_node *port_node)
{
	struct device_node *scheduler_node, *child;
	int port, ret;

	if (of_property_read_u32(port_node, "port-id", &port))
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "Fail to get port-id of l0scheduler\n");

	scheduler_node = of_get_child_by_name(port_node, "l0scheduler");
	if (!scheduler_node)
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "Fail to get l0scheduler config\n");

	for_each_available_child_of_node(scheduler_node, child) {
		ret = of_parse_ppe_scheduler_group_config(ppe_dev, child, port,
							  "qcom,ucast-queue",
							  "qcom,ucast-loop-priority");
		if (ret)
			return ret;

		ret = of_parse_ppe_scheduler_group_config(ppe_dev, child, port,
							  "qcom,mcast-queue",
							  "qcom,mcast-loop-priority");
		if (ret)
			return ret;
	}

	scheduler_node = of_get_child_by_name(port_node, "l1scheduler");
	if (!scheduler_node)
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "Fail to get l1scheduler config\n");

	for_each_available_child_of_node(scheduler_node, child) {
		ret = of_parse_ppe_scheduler_group_config(ppe_dev, child, port,
							  "qcom,flow",
							  "qcom,flow-loop-priority");
		if (ret)
			return ret;
	}

	return ret;
}

static int of_parse_ppe_scheduler(struct ppe_device *ppe_dev,
				  struct device_node *ppe_node)
{
	struct device_node *scheduler_node, *port_node;
	int ret;

	scheduler_node = of_get_child_by_name(ppe_node, "port-scheduler-resource");
	if (!scheduler_node)
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "port-scheduler-resource is not defined\n");

	ret = of_parse_ppe_scheduler_resource(ppe_dev, scheduler_node);
	if (ret)
		return ret;

	scheduler_node = of_get_child_by_name(ppe_node, "port-scheduler-config");
	if (!scheduler_node)
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "port-scheduler-config is not defined\n");

	for_each_available_child_of_node(scheduler_node, port_node) {
		ret = of_parse_ppe_scheduler_config(ppe_dev, port_node);
		if (ret)
			return ret;
	}

	return ret;
}

static int of_parse_ppe_config(struct ppe_device *ppe_dev,
			       struct device_node *ppe_node)
{
	int ret;

	ret = of_parse_ppe_bm(ppe_dev, ppe_node);
	if (ret)
		return ret;

	ret = of_parse_ppe_qm(ppe_dev, ppe_node);
	if (ret)
		return ret;

	ret = of_parse_ppe_tdm(ppe_dev, ppe_node);
	if (ret)
		return ret;

	return of_parse_ppe_scheduler(ppe_dev, ppe_node);
}

static int ppe_qm_init(struct ppe_device *ppe_dev)
{
	const struct ppe_queue_ops *ppe_queue_ops;
	struct ppe_queue_ucast_dest queue_dst;
	int profile_id, priority, res, class;

	ppe_queue_ops = ppe_queue_config_ops_get();

	/* Initialize the PPE queue base ID and queue priority for each
	 * physical port, the egress queue ID is decided by the queue
	 * base ID added by the queue priority class and RSS hash class.
	 *
	 * Each physical port has the independent profile ID, so that
	 * each physical port can be configured with the independent
	 * queue base and queue priority class and RSS hash class.
	 */
	profile_id = 0;
	while (profile_id < PPE_SCHEDULER_PORT_NUM) {
		memset(&queue_dst, 0, sizeof(queue_dst));

		/* The device tree property of queue-config is as below,
		 * <queue_base queue_num group prealloc ceil weight
		 * resume_off dynamic>;
		 */
		res = ppe_scheduler_res[profile_id].ucastq[0];
		queue_dst.dest_port = profile_id;

		/* Configure queue base ID and profile ID that is same as
		 * physical port ID.
		 */
		if (ppe_queue_ops->queue_ucast_base_set)
			ppe_queue_ops->queue_ucast_base_set(ppe_dev,
							    queue_dst,
							    res,
							    profile_id);

		/* Queue maximum priority supported by each phiscal port */
		res = ppe_scheduler_res[profile_id].l0cdrr[1] -
		      ppe_scheduler_res[profile_id].l0cdrr[0];

		priority = 0;
		while (priority < PPE_QUEUE_PRI_MAX) {
			if (priority > res)
				class = res;
			else
				class = priority;

			if (ppe_queue_ops->queue_ucast_pri_class_set)
				ppe_queue_ops->queue_ucast_pri_class_set(ppe_dev,
									 profile_id,
									 priority,
									 class);
			priority++;
		}

		/* Configure the queue RSS hash class value as 0 by default,
		 * which can be configured as the value same as the ARM CPU
		 * core number to distribute traffic for the traffic load balance.
		 */
		priority = 0;
		while (priority < PPE_QUEUE_HASH_MAX) {
			if (ppe_queue_ops->queue_ucast_hash_class_set)
				ppe_queue_ops->queue_ucast_hash_class_set(ppe_dev,
									  profile_id,
									  priority,
									  0);
			priority++;
		}

		profile_id++;
	}

	/* Redirect ARP reply packet with the max priority on CPU port, which
	 * keeps the ARP reply with highest priority received by EDMA when
	 * there is heavy traffic.
	 */
	memset(&queue_dst, 0, sizeof(queue_dst));
	queue_dst.cpu_code_en = true;
	queue_dst.cpu_code = 101;
	res = ppe_scheduler_res[0].ucastq[0];
	priority = ppe_scheduler_res[0].l0cdrr[1] - ppe_scheduler_res[0].l0cdrr[0];
	if (ppe_queue_ops->queue_ucast_base_set)
		ppe_queue_ops->queue_ucast_base_set(ppe_dev, queue_dst, res, priority);

	return 0;
}

static int ppe_servcode_init(struct ppe_device *ppe_dev)
{
	struct ppe_servcode_cfg servcode_cfg;

	memset(&servcode_cfg, 0, sizeof(servcode_cfg));
	servcode_cfg.bypass_bitmap[0] = (u32)(~(BIT(FAKE_MAC_HEADER_BYP) |
					BIT(SERVICE_CODE_BYP) |
					BIT(FAKE_L2_PROTO_BYP)));
	servcode_cfg.bypass_bitmap[1] = (u32)(~(BIT(ACL_POST_ROUTING_CHECK_BYP)));

	/* The default service code used by CPU port */
	return ppe_servcode_config_set(ppe_dev, 1, servcode_cfg);
}

static int ppe_port_ctrl_init(struct ppe_device *ppe_dev)
{
	union ppe_mru_mtu_ctrl_cfg_u mru_mtu_cfg;
	int ret, port_num = PPE_SCHEDULER_PORT_NUM;
	u32 reg_val;

	if (ppe_type_get(ppe_dev) == PPE_TYPE_MPPE) {
		for (ret = 0; ret < MPPE_SCHEDULER_PORT_NUM; ret++) {
			reg_val = FIELD_PREP(PPE_TX_BUFF_THRSH_XOFF, 3) |
				  FIELD_PREP(PPE_TX_BUFF_THRSH_XON, 3);
			ppe_write(ppe_dev, PPE_TX_BUFF_THRSH + PPE_TX_BUFF_THRSH_INC * ret,
				  reg_val);

			/* Fix 147B line rate on physical port */
			if (ret != 0)
				ppe_mask(ppe_dev, PPE_RX_FIFO_CFG + PPE_RX_FIFO_CFG_INC * ret,
					 PPE_RX_FIFO_CFG_THRSH,
					 FIELD_PREP(PPE_RX_FIFO_CFG_THRSH, 7));
		}

		port_num = MPPE_SCHEDULER_PORT_NUM;
	}

	for (ret = 0; ret < port_num; ret++) {
		if (ret != 0) {
			memset(&mru_mtu_cfg, 0, sizeof(mru_mtu_cfg));
			ppe_read_tbl(ppe_dev,
				     PPE_MRU_MTU_CTRL_TBL + PPE_MRU_MTU_CTRL_TBL_INC * ret,
				     mru_mtu_cfg.val, sizeof(mru_mtu_cfg.val));

			/* Drop the packet when the packet size is more than
			 * the MTU of the physical interface.
			 */
			mru_mtu_cfg.bf.mru_cmd = PPE_ACTION_DROP;
			mru_mtu_cfg.bf.mtu_cmd = PPE_ACTION_DROP;

			ppe_write_tbl(ppe_dev,
				      PPE_MRU_MTU_CTRL_TBL + PPE_MRU_MTU_CTRL_TBL_INC * ret,
				      mru_mtu_cfg.val, sizeof(mru_mtu_cfg.val));

			ppe_mask(ppe_dev,
				 PPE_MC_MTU_CTRL_TBL + PPE_MC_MTU_CTRL_TBL_INC * ret,
				 PPE_MC_MTU_CTRL_TBL_MTU_CMD,
				 FIELD_PREP(PPE_MC_MTU_CTRL_TBL_MTU_CMD, PPE_ACTION_DROP));
		}

		/* Enable PPE port counter */
		ppe_counter_set(ppe_dev, ret, true);
	}

	return 0;
}

static int ppe_rss_hash_init(struct ppe_device *ppe_dev)
{
	const struct ppe_queue_ops *ppe_queue_ops;
	struct ppe_rss_hash_cfg hash_cfg;
	int i, ret;
	u16 fins[5] = {0x205, 0x264, 0x227, 0x245, 0x201};
	u8 ips[4] = {0x13, 0xb, 0x13, 0xb};

	ppe_queue_ops = ppe_queue_config_ops_get();
	if (!ppe_queue_ops->rss_hash_config_set)
		return -EINVAL;

	hash_cfg.hash_seed = get_random_u32();
	hash_cfg.hash_mask = 0xfff;
	hash_cfg.hash_fragment_mode = false;

	i = 0;
	while (i < ARRAY_SIZE(fins)) {
		hash_cfg.hash_fin_inner[i] = fins[i] & 0x1f;
		hash_cfg.hash_fin_outer[i] = fins[i] >> 5;
		i++;
	}

	hash_cfg.hash_protocol_mix = 0x13;
	hash_cfg.hash_dport_mix = 0xb;
	hash_cfg.hash_sport_mix = 0x13;
	hash_cfg.hash_sip_mix[0] = 0x13;
	hash_cfg.hash_dip_mix[0] = 0xb;

	ret = ppe_queue_ops->rss_hash_config_set(ppe_dev,
						 PPE_RSS_HASH_MODE_IPV4,
						 hash_cfg);
	if (ret)
		return ret;

	i = 0;
	while (i < ARRAY_SIZE(ips)) {
		hash_cfg.hash_sip_mix[i] = ips[i];
		hash_cfg.hash_dip_mix[i] = ips[i];
		i++;
	}

	return ppe_queue_ops->rss_hash_config_set(ppe_dev,
						  PPE_RSS_HASH_MODE_IPV6,
						  hash_cfg);
}

static int ppe_bridge_init(struct ppe_device *ppe_dev)
{
	union ppe_l2_vp_port_tbl_u port_tbl;
	union ppe_vsi_tbl_u vsi_tbl;
	u32 reg_val = 0;
	int i = 0;

	/* CPU port0 initialization */
	reg_val = FIELD_PREP(PPE_PORT_BRIDGE_CTRL_ISOLATION_BITMAP, 0x7F) |
			PPE_PORT_BRIDGE_CTRL_PROMISC_EN;
	ppe_mask(ppe_dev,
		 PPE_PORT_BRIDGE_CTRL + PPE_PORT_BRIDGE_CTRL_INC * PPE_PORT0,
		 PPE_PORT_BRIDGE_CTRL_MASK,
		 reg_val | PPE_PORT_BRIDGE_CTRL_TXMAC_EN);

	/* Physical and virtual physical port initialization */
	reg_val |= (PPE_PORT_BRIDGE_CTRL_STATION_MODE_LRN_EN |
			PPE_PORT_BRIDGE_CTRL_NEW_ADDR_LRN_EN);
	for (i = PPE_PORT1; i <= PPE_PORT6; i++) {
		ppe_mask(ppe_dev,
			 PPE_PORT_BRIDGE_CTRL + PPE_PORT_BRIDGE_CTRL_INC * i,
			 PPE_PORT_BRIDGE_CTRL_MASK,
			 reg_val);

		/* Invalid vsi fowarding to CPU port0 */
		memset(&port_tbl, 0, sizeof(port_tbl));
		ppe_read_tbl(ppe_dev,
			     PPE_L2_VP_PORT_TBL + PPE_L2_VP_PORT_TBL_INC * i,
			     port_tbl.val,
			     sizeof(port_tbl.val));
		port_tbl.bf.invalid_vsi_forwarding_en = true;
		port_tbl.bf.dst_info = PPE_PORT0;
		ppe_write_tbl(ppe_dev,
			      PPE_L2_VP_PORT_TBL + PPE_L2_VP_PORT_TBL_INC * i,
			      port_tbl.val,
			      sizeof(port_tbl.val));
	}

	/* Internal port7 initialization */
	ppe_mask(ppe_dev,
		 PPE_PORT_BRIDGE_CTRL + PPE_PORT_BRIDGE_CTRL_INC * PPE_PORT7,
		 PPE_PORT_BRIDGE_CTRL_MASK,
		 reg_val | PPE_PORT_BRIDGE_CTRL_TXMAC_EN);

	/* Enable Global L2 Learn and Ageing */
	ppe_mask(ppe_dev,
		 PPE_L2_GLOBAL_CONFIG,
		 PPE_L2_GLOBAL_CONFIG_LRN_EN | PPE_L2_GLOBAL_CONFIG_AGE_EN,
		 PPE_L2_GLOBAL_CONFIG_LRN_EN | PPE_L2_GLOBAL_CONFIG_AGE_EN);

	/* Vsi initialization */
	for (i = 0; i < PPE_VSI_TBL_NUM; i++) {
		memset(&vsi_tbl, 0, sizeof(vsi_tbl));
		ppe_read_tbl(ppe_dev,
			     PPE_VSI_TBL + PPE_VSI_TBL_INC * i,
			     vsi_tbl.val,
			     sizeof(vsi_tbl.val));
		vsi_tbl.bf.member_port_bitmap = BIT(PPE_PORT0);
		vsi_tbl.bf.uuc_bitmap = BIT(PPE_PORT0);
		vsi_tbl.bf.umc_bitmap = BIT(PPE_PORT0);
		vsi_tbl.bf.bc_bitmap = BIT(PPE_PORT0);
		vsi_tbl.bf.new_addr_lrn_en = true;
		vsi_tbl.bf.new_addr_fwd_cmd = 0;
		vsi_tbl.bf.station_move_lrn_en = true;
		vsi_tbl.bf.station_move_fwd_cmd = 0;
		ppe_write_tbl(ppe_dev,
			      PPE_VSI_TBL + PPE_VSI_TBL_INC * i,
			      vsi_tbl.val,
			      sizeof(vsi_tbl.val));
	}

	return 0;
}

static int ppe_dev_hw_init(struct ppe_device *ppe_dev)
{
	int ret;

	ret = ppe_qm_init(ppe_dev);
	if (ret)
		return ret;

	ret = ppe_servcode_init(ppe_dev);
	if (ret)
		return ret;

	ret = ppe_port_ctrl_init(ppe_dev);
	if (ret)
		return ret;

	ret = ppe_bridge_init(ppe_dev);
	if (ret)
		return ret;

	return ppe_rss_hash_init(ppe_dev);
}

static int qcom_ppe_probe(struct platform_device *pdev)
{
	struct ppe_device *ppe_dev;
	void __iomem *base;
	int ret;

	ppe_dev = devm_kzalloc(&pdev->dev, sizeof(*ppe_dev), GFP_KERNEL);
	if (!ppe_dev)
		return -ENOMEM;

	ppe_dev->dev = &pdev->dev;
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return dev_err_probe(&pdev->dev,
				     PTR_ERR(base),
				     "Fail to ioremap\n");

	ppe_dev->regmap = devm_regmap_init_mmio(&pdev->dev, base, &ppe_regmap_config);
	if (IS_ERR(ppe_dev->regmap))
		return dev_err_probe(&pdev->dev,
				     PTR_ERR(ppe_dev->regmap),
				     "Fail to regmap\n");

	ppe_dev->ppe_priv = ppe_data_init(pdev);
	if (IS_ERR(ppe_dev->ppe_priv))
		return dev_err_probe(&pdev->dev,
				     PTR_ERR(ppe_dev->ppe_priv),
				     "Fail to init ppe data\n");

	mutex_init(&ppe_dev->reg_mutex);
	platform_set_drvdata(pdev, ppe_dev);
	ret = ppe_clock_config(pdev);
	if (ret)
		return dev_err_probe(&pdev->dev,
				     ret,
				     "ppe clock config failed\n");

	ret = of_parse_ppe_config(ppe_dev, pdev->dev.of_node);
	if (ret)
		return dev_err_probe(&pdev->dev,
				     ret,
				     "of parse ppe failed\n");

	ret = ppe_dev_hw_init(ppe_dev);
	if (ret)
		return dev_err_probe(&pdev->dev,
				     ret,
				     "ppe device hw init failed\n");

	ret = ppe_mac_init(pdev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "ppe mac initialization failed\n");

	ppe_dev->uniphy = ppe_uniphy_setup(pdev);
	if (IS_ERR(ppe_dev->uniphy))
		return dev_err_probe(&pdev->dev, ret, "ppe uniphy initialization failed\n");

	ppe_dev->ppe_ops = &qcom_ppe_ops;
	ppe_dev->is_ppe_probed = true;
	ppe_debugfs_setup(ppe_dev);

	return 0;
}

static int qcom_ppe_remove(struct platform_device *pdev)
{
	struct ppe_device *ppe_dev;
	struct ppe_ports *ppe_ports;
	struct ppe_data *ppe_dev_priv;
	int i, port;

	ppe_dev = platform_get_drvdata(pdev);
	ppe_dev_priv = ppe_dev->ppe_priv;
	ppe_ports = (struct ppe_ports *)ppe_dev->ports;

	ppe_debugfs_teardown(ppe_dev);

	for (i = 0; i < ppe_ports->num; i++) {
		/* Stop gmib statistics polling */
		cancel_delayed_work_sync(&ppe_ports->port[i].gmib_read);

		/* Reset ppe port parent clock to XO clock */
		port = ppe_ports->port[i].port_id;
		clk_set_rate(ppe_dev_priv->clk[PPE_NSS_PORT1_RX_CLK + ((port - 1) << 1)],
			     P_XO_CLOCK_RATE);
		clk_set_rate(ppe_dev_priv->clk[PPE_NSS_PORT1_TX_CLK + ((port - 1) << 1)],
			     P_XO_CLOCK_RATE);
	}

	return 0;
}

static const struct of_device_id qcom_ppe_of_match[] = {
	{ .compatible = "qcom,ipq9574-ppe", },
	{ .compatible = "qcom,ipq5332-ppe", },
	{},
};

static struct platform_driver qcom_ppe_driver = {
	.driver = {
		.name = "qcom_ppe",
		.owner  = THIS_MODULE,
		.of_match_table = qcom_ppe_of_match,
	},
	.probe	= qcom_ppe_probe,
	.remove = qcom_ppe_remove,
};
module_platform_driver(qcom_ppe_driver);

MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, qcom_ppe_of_match);
