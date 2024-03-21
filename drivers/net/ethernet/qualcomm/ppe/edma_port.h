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
 * struct edma_port_priv - EDMA port priv structure.
 * @ppe_port: Pointer to PPE port
 * @netdev: Corresponding netdevice
 * @flags: Feature flags
 */
struct edma_port_priv {
	struct ppe_port *ppe_port;
	struct net_device *netdev;
	unsigned long flags;
};

void edma_port_destroy(struct ppe_port *port);
int edma_port_setup(struct ppe_port *port);
#endif
