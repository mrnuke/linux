/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __EDMA_PORTS__
#define __EDMA_PORTS__

#include "ppe_port.h"

#define EDMA_NETDEV_FEATURES		(NETIF_F_FRAGLIST \
					| NETIF_F_SG \
					| NETIF_F_RXCSUM \
					| NETIF_F_HW_CSUM \
					| NETIF_F_TSO \
					| NETIF_F_TSO6)

/**
 * struct edma_port_rx_stats - EDMA RX per CPU stats for the port.
 * @rx_pkts: Number of Rx packets
 * @rx_bytes: Number of Rx bytes
 * @rx_drops: Number of Rx drops
 * @rx_nr_frag_pkts: Number of Rx nr_frags packets
 * @rx_fraglist_pkts: Number of Rx fraglist packets
 * @rx_nr_frag_headroom_err: nr_frags headroom error packets
 * @syncp: Synchronization pointer
 */
struct edma_port_rx_stats {
	u64 rx_pkts;
	u64 rx_bytes;
	u64 rx_drops;
	u64 rx_nr_frag_pkts;
	u64 rx_fraglist_pkts;
	u64 rx_nr_frag_headroom_err;
	struct u64_stats_sync syncp;
};

/**
 * struct edma_port_pcpu_stats - EDMA per cpu stats data structure for the port.
 * @rx_stats: Per CPU Rx statistics
 */
struct edma_port_pcpu_stats {
	struct edma_port_rx_stats __percpu *rx_stats;
};

/**
 * struct edma_port_priv - EDMA port priv structure.
 * @ppe_port: Pointer to PPE port
 * @netdev: Corresponding netdevice
 * @pcpu_stats: Per CPU netdev statistics
 * @txr_map: Tx ring per-core mapping
 * @flags: Feature flags
 */
struct edma_port_priv {
	struct ppe_port *ppe_port;
	struct net_device *netdev;
	struct edma_port_pcpu_stats pcpu_stats;
	unsigned long flags;
};

void edma_port_destroy(struct ppe_port *port);
int edma_port_setup(struct ppe_port *port);
#endif
