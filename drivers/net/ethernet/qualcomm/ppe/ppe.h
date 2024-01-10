/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* PPE clock, reset and register read/write declarations. */

#ifndef __PPE_H__
#define __PPE_H__

#include <linux/clk.h>
#include <linux/reset.h>

/* PPE Ports */
#define PPE_PORT0		0
#define PPE_PORT1		1
#define PPE_PORT2		2
#define PPE_PORT3		3
#define PPE_PORT4		4
#define PPE_PORT5		5
#define PPE_PORT6		6
#define PPE_PORT7		7

/* PPE Port XO Clock Rate */
#define P_XO_CLOCK_RATE		24000000

enum ppe_clk_id {
	/* clocks for CMN PLL */
	PPE_CMN_AHB_CLK,
	PPE_CMN_SYS_CLK,
	/* clocks for PPE integrated uniphy */
	PPE_UNIPHY0_SYS_CLK,
	PPE_UNIPHY1_SYS_CLK,
	PPE_UNIPHY2_SYS_CLK,
	PPE_UNIPHY0_AHB_CLK,
	PPE_UNIPHY1_AHB_CLK,
	PPE_UNIPHY2_AHB_CLK,
	/* clocks for NSS NOC that is connected with PPE */
	PPE_NSSCC_CLK,
	PPE_NSSNOC_NSSCC_CLK,
	PPE_NSSNOC_SNOC_CLK,
	PPE_NSSNOC_SNOC_1_CLK,
	/* clock for sleep that is needed for PPE reset */
	PPE_IM_SLEEP_CLK,
	/* clocks for PPE block */
	PPE_PPE_CLK,
	PPE_PPE_CFG_CLK,
	PPE_NSSNOC_PPE_CLK,
	PPE_NSSNOC_PPE_CFG_CLK,
	/* clocks for EDMA to be enabled during the PPE initialization */
	PPE_EDMA_CLK,
	PPE_EDMA_CFG_CLK,
	/* clocks for PPE IPE/BTQ modules */
	PPE_PPE_IPE_CLK,
	PPE_PPE_BTQ_CLK,
	/* clocks for PPE integrated MAC */
	PPE_PORT1_MAC_CLK,
	PPE_PORT2_MAC_CLK,
	PPE_PORT3_MAC_CLK,
	PPE_PORT4_MAC_CLK,
	PPE_PORT5_MAC_CLK,
	PPE_PORT6_MAC_CLK,
	/* clocks for PPE port */
	PPE_NSS_PORT1_RX_CLK,
	PPE_NSS_PORT1_TX_CLK,
	PPE_NSS_PORT2_RX_CLK,
	PPE_NSS_PORT2_TX_CLK,
	PPE_NSS_PORT3_RX_CLK,
	PPE_NSS_PORT3_TX_CLK,
	PPE_NSS_PORT4_RX_CLK,
	PPE_NSS_PORT4_TX_CLK,
	PPE_NSS_PORT5_RX_CLK,
	PPE_NSS_PORT5_TX_CLK,
	PPE_NSS_PORT6_RX_CLK,
	PPE_NSS_PORT6_TX_CLK,
	/* clocks for PPE uniphy port */
	PPE_UNIPHY_PORT1_RX_CLK,
	PPE_UNIPHY_PORT1_TX_CLK,
	PPE_UNIPHY_PORT2_RX_CLK,
	PPE_UNIPHY_PORT2_TX_CLK,
	PPE_UNIPHY_PORT3_RX_CLK,
	PPE_UNIPHY_PORT3_TX_CLK,
	PPE_UNIPHY_PORT4_RX_CLK,
	PPE_UNIPHY_PORT4_TX_CLK,
	PPE_UNIPHY_PORT5_RX_CLK,
	PPE_UNIPHY_PORT5_TX_CLK,
	PPE_UNIPHY_PORT6_RX_CLK,
	PPE_UNIPHY_PORT6_TX_CLK,
	/* source clock for PPE port5 */
	PPE_NSS_PORT5_RX_CLK_SRC,
	PPE_NSS_PORT5_TX_CLK_SRC,
	PPE_CLK_MAX
};

enum ppe_rst_id {
	/* reset for PPE block */
	PPE_RST_PPE_RST,
	/* resets for uniphy */
	PPE_UNIPHY0_SYS_RST,
	PPE_UNIPHY1_SYS_RST,
	PPE_UNIPHY2_SYS_RST,
	PPE_UNIPHY0_AHB_RST,
	PPE_UNIPHY1_AHB_RST,
	PPE_UNIPHY2_AHB_RST,
	PPE_UNIPHY0_XPCS_RST,
	PPE_UNIPHY1_XPCS_RST,
	PPE_UNIPHY2_XPCS_RST,
	PPE_UNIPHY0_SOFT_RST,
	PPE_UNIPHY1_SOFT_RST,
	PPE_UNIPHY2_SOFT_RST,
	/* resets for uniphy port */
	PPE_UNIPHY_PORT1_DIS,
	PPE_UNIPHY_PORT2_DIS,
	PPE_UNIPHY_PORT3_DIS,
	PPE_UNIPHY_PORT4_DIS,
	PPE_UNIPHY_PORT1_RX_RST,
	PPE_UNIPHY_PORT1_TX_RST,
	PPE_UNIPHY_PORT2_RX_RST,
	PPE_UNIPHY_PORT2_TX_RST,
	/* resets for PPE port */
	PPE_NSS_PORT1_RX_RST,
	PPE_NSS_PORT1_TX_RST,
	PPE_NSS_PORT2_RX_RST,
	PPE_NSS_PORT2_TX_RST,
	PPE_NSS_PORT1_RST,
	PPE_NSS_PORT2_RST,
	PPE_NSS_PORT3_RST,
	PPE_NSS_PORT4_RST,
	PPE_NSS_PORT5_RST,
	PPE_NSS_PORT6_RST,
	/* resets for PPE MAC */
	PPE_NSS_PORT1_MAC_RST,
	PPE_NSS_PORT2_MAC_RST,
	PPE_NSS_PORT3_MAC_RST,
	PPE_NSS_PORT4_MAC_RST,
	PPE_NSS_PORT5_MAC_RST,
	PPE_NSS_PORT6_MAC_RST,
	PPE_RST_MAX
};

/* Different PPE type used on the different IPQ SoC platform */
enum {
	PPE_TYPE_APPE,
	PPE_TYPE_MPPE,
	PPE_TYPE_MAX = 0xff,
};

/* The action of packet received by PPE can be forwarded, dropped, copied
 * to CPU(enter multicast queue), redirected to CPU(enter unicast queue).
 */
enum {
	PPE_ACTION_FORWARD = 0,
	PPE_ACTION_DROP,
	PPE_ACTION_COPY_TO_CPU,
	PPE_ACTION_REDIRECTED_TO_CPU
};

/* PPE MAC Type */
enum {
	PPE_MAC_TYPE_NA,
	PPE_MAC_TYPE_GMAC,
	PPE_MAC_TYPE_XGMAC,
	PPE_MAC_TYPE_MAX
};

/* PPE private data of different PPE type device */
struct ppe_data {
	int ppe_type;
	struct clk *clk[PPE_CLK_MAX];
	struct reset_control *rst[PPE_RST_MAX];
};

/* PPE port QoS resource, which includes the queue range and
 * DRR(deficit round robin), SP(strict priority).
 */
struct ppe_scheduler_port_resource {
	int ucastq[2];
	int mcastq[2];
	int l0sp[2];
	int l0cdrr[2];
	int l0edrr[2];
	int l1cdrr[2];
	int l1edrr[2];
};

/* PPE GMAC statistics */
struct ppe_gmib_hw_stats {
	u64 rx_broadcast;
	u64 rx_pause;
	u64 rx_multicast;
	u64 rx_fcserr;
	u64 rx_alignerr;
	u64 rx_runt;
	u64 rx_frag;
	u64 rx_jmbfcserr;
	u64 rx_jmbalignerr;
	u64 rx_pkt64;
	u64 rx_pkt65to127;
	u64 rx_pkt128to255;
	u64 rx_pkt256to511;
	u64 rx_pkt512to1023;
	u64 rx_pkt1024to1518;
	u64 rx_pkt1519tomax;
	u64 rx_toolong;
	u64 rx_pktgoodbyte;
	u64 rx_pktbadbyte;
	u64 rx_unicast;
	u64 tx_broadcast;
	u64 tx_pause;
	u64 tx_multicast;
	u64 tx_underrun;
	u64 tx_pkt64;
	u64 tx_pkt65to127;
	u64 tx_pkt128to255;
	u64 tx_pkt256to511;
	u64 tx_pkt512to1023;
	u64 tx_pkt1024to1518;
	u64 tx_pkt1519tomax;
	u64 tx_pktbyte;
	u64 tx_collisions;
	u64 tx_abortcol;
	u64 tx_multicol;
	u64 tx_singlecol;
	u64 tx_exesdeffer;
	u64 tx_deffer;
	u64 tx_latecol;
	u64 tx_unicast;
};

/* PPE XGMAC statistics */
struct ppe_xgmib_hw_stats {
	u64 tx_bytes;
	u64 tx_frames;
	u64 tx_broadcast_g;
	u64 tx_multicast_g;
	u64 tx_pkt64;
	u64 tx_pkt65to127;
	u64 tx_pkt128to255;
	u64 tx_pkt256to511;
	u64 tx_pkt512to1023;
	u64 tx_pkt1024tomax;
	u64 tx_unicast;
	u64 tx_multicast;
	u64 tx_broadcast;
	u64 tx_underflow_err;
	u64 tx_bytes_g;
	u64 tx_frame_g;
	u64 tx_pause;
	u64 tx_vlan_g;
	u64 tx_lpi_usec;
	u64 tx_lpi_tran;
	u64 rx_frames;
	u64 rx_bytes;
	u64 rx_bytes_g;
	u64 rx_broadcast_g;
	u64 rx_multicast_g;
	u64 rx_crc_err;
	u64 rx_runt_err;
	u64 rx_jabber_err;
	u64 rx_undersize_g;
	u64 rx_oversize_g;
	u64 rx_pkt64;
	u64 rx_pkt65to127;
	u64 rx_pkt128to255;
	u64 rx_pkt256to511;
	u64 rx_pkt512to1023;
	u64 rx_pkt1024tomax;
	u64 rx_unicast_g;
	u64 rx_len_err;
	u64 rx_outofrange_err;
	u64 rx_pause;
	u64 rx_fifo_overflow;
	u64 rx_vlan;
	u64 rx_wdog_err;
	u64 rx_lpi_usec;
	u64 rx_lpi_tran;
	u64 rx_drop_frames;
	u64 rx_drop_bytes;
};

/* PPE per port data type to record port settings such as phylink
 * setting, mac type, interface mode and link speed.
 */
struct ppe_port {
	struct phylink *phylink;
	struct phylink_config phylink_config;
	struct device_node *np;
	struct ppe_device *ppe_dev;
	phy_interface_t interface;
	int mac_type;
	int port_id;
	int speed;
	int duplex;
	int pause;
	struct delayed_work gmib_read;
	struct ppe_gmib_hw_stats *gmib_stats;
	spinlock_t stats_lock; /* Protects gmib stats */
};

/* PPE ports data type */
struct ppe_ports {
	unsigned int num;
	struct ppe_port port[];
};

int ppe_type_get(struct ppe_device *ppe_dev);
struct clk **ppe_clock_get(struct ppe_device *ppe_dev);
struct reset_control **ppe_reset_get(struct ppe_device *ppe_dev);

int ppe_write(struct ppe_device *ppe_dev, u32 reg, unsigned int val);
int ppe_read(struct ppe_device *ppe_dev, u32 reg, unsigned int *val);
int ppe_mask(struct ppe_device *ppe_dev, u32 reg, u32 mask, unsigned int set);
int ppe_write_tbl(struct ppe_device *ppe_dev, u32 reg, const unsigned int *val, int cnt);
int ppe_read_tbl(struct ppe_device *ppe_dev, u32 reg, unsigned int *val, int cnt);
#endif
