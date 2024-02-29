/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __PPE_PORT_H__
#define __PPE_PORT_H__

#include <linux/phylink.h>

/**
 * enum ppe_port_clk_rst_type - PPE port clock and reset ID type
 * @PPE_PORT_CLK_RST_MAC: The clock and reset ID for port MAC
 * @PPE_PORT_CLK_RST_RX: The clock and reset ID for port receive path
 * @PPE_PORT_CLK_RST_TX: The clock and reset for port transmit path
 * @PPE_PORT_CLK_RST_MAX: The maximum of port clock and reset
 */
enum ppe_port_clk_rst_type {
	PPE_PORT_CLK_RST_MAC,
	PPE_PORT_CLK_RST_RX,
	PPE_PORT_CLK_RST_TX,
	PPE_PORT_CLK_RST_MAX,
};

/**
 * enum ppe_mac_type - PPE MAC type
 * @PPE_MAC_TYPE_GMAC: GMAC type
 * @PPE_MAC_TYPE_XGMAC: XGMAC type
 */
enum ppe_mac_type {
	PPE_MAC_TYPE_GMAC,
	PPE_MAC_TYPE_XGMAC,
};

/**
 * struct ppe_port - Private data for each PPE port
 * @phylink: Linux phylink instance
 * @phylink_config: Linux phylink configurations
 * @pcs: Linux phylink PCS instance
 * @np: Port device tree node
 * @ppe_dev: Back pointer to PPE device private data
 * @interface: Port interface mode
 * @mac_type: Port MAC type, GMAC or XGMAC
 * @port_id: Port ID
 * @clks: Port clocks
 * @rstcs: Port resets
 */
struct ppe_port {
	struct phylink *phylink;
	struct phylink_config phylink_config;
	struct phylink_pcs *pcs;
	struct device_node *np;
	struct ppe_device *ppe_dev;
	phy_interface_t interface;
	enum ppe_mac_type mac_type;
	int port_id;
	struct clk *clks[PPE_PORT_CLK_RST_MAX];
	struct reset_control *rstcs[PPE_PORT_CLK_RST_MAX];
};

/**
 * struct ppe_ports - Array of PPE ports
 * @num: Number of PPE ports
 * @port: Each PPE port private data
 */
struct ppe_ports {
	unsigned int num;
	struct ppe_port port[] __counted_by(num);
};

int ppe_port_mac_init(struct ppe_device *ppe_dev);
void ppe_port_mac_deinit(struct ppe_device *ppe_dev);
int ppe_port_phylink_setup(struct ppe_port *ppe_port,
			   struct net_device *netdev);
void ppe_port_phylink_destroy(struct ppe_port *ppe_port);
#endif
