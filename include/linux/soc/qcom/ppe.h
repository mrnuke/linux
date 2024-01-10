/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* PPE operations to be used by ethernet driver */

#ifndef __QCOM_PPE_H__
#define __QCOM_PPE_H__

#include <linux/platform_device.h>
#include <linux/phylink.h>
#include <linux/if_link.h>

/* PPE platform private data, which is used by external driver like
 * Ethernet DMA driver.
 */
struct ppe_device {
	struct device *dev;
	struct regmap *regmap;
	struct ppe_device_ops *ppe_ops;
	struct dentry *debugfs_root;
	bool is_ppe_probed;
	void *ppe_priv;
	struct mutex reg_mutex; /* Protects ppe reg operation */
	void *ports;
	void *uniphy;
};

/* PPE operations, which is used by the external driver like Ethernet
 * DMA driver to configure PPE.
 */
struct ppe_device_ops {
	/*
	 * PHYLINK integration
	 */
	struct phylink *(*phylink_setup)(struct ppe_device *ppe_dev,
					 struct net_device *netdev, int port);
	void	(*phylink_destroy)(struct ppe_device *ppe_dev,
				   int port);
	void	(*phylink_mac_config)(struct ppe_device *ppe_dev,
				      int port,
				      unsigned int mode,
				      const struct phylink_link_state *state);
	void	(*phylink_mac_link_up)(struct ppe_device *ppe_dev,
				       int port,
				       struct phy_device *phy,
				       unsigned int mode,
				       phy_interface_t interface,
				       int speed,
				       int duplex,
				       bool tx_pause,
				       bool rx_pause);
	void	(*phylink_mac_link_down)(struct ppe_device *ppe_dev,
					 int port,
					 unsigned int mode,
					 phy_interface_t interface);
	struct phylink_pcs *(*phylink_mac_select_pcs)(struct ppe_device *ppe_dev,
						      int port,
						      phy_interface_t interface);
	/*
	 * Port statistics counters
	 */
	void	(*get_stats64)(struct ppe_device *ppe_dev,
			       int port,
			       struct rtnl_link_stats64 *s);
	void	(*get_strings)(struct ppe_device *ppe_dev,
			       int port,
			       u32 stringset,
			       u8 *data);
	int	(*get_sset_count)(struct ppe_device *ppe_dev,
				  int port,
				  int sset);
	void	(*get_ethtool_stats)(struct ppe_device *ppe_dev,
				     int port,
				     u64 *data);
	/*
	 * Port MAC address setting
	 */
	int	(*set_mac_address)(struct ppe_device *ppe_dev,
				   int port,
				   u8 *macaddr);
	/*
	 * Port MAC EEE settings
	 */
	int	(*set_mac_eee)(struct ppe_device *ppe_dev, int port,
			       struct ethtool_eee *eee);
	int	(*get_mac_eee)(struct ppe_device *ppe_dev, int port,
			       struct ethtool_eee *eee);
	/*
	 * Port maximum frame size setting
	 */
	int	(*set_maxframe)(struct ppe_device *ppe_dev, int port,
				int maxframe_size);
};

/* Function used to check PPE platform dirver is registered correctly or not. */
bool ppe_is_probed(struct platform_device *pdev);

/* Function used to get the PPE device */
struct ppe_device *ppe_dev_get(struct platform_device *pdev);

/* Function used to get the operations of PPE device */
struct ppe_device_ops *ppe_ops_get(struct platform_device *pdev);
#endif
