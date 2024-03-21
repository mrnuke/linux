/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __EDMA_CFG_TX__
#define __EDMA_CFG_TX__

/* Tx mitigation timer's default value. */
#define EDMA_TX_MITIGATION_TIMER_DEF	250

/* Tx mitigation packet count default value. */
#define EDMA_TX_MITIGATION_PKT_CNT_DEF	16

void edma_cfg_tx_rings(void);
int edma_cfg_tx_rings_alloc(void);
void edma_cfg_tx_rings_cleanup(void);
void edma_cfg_tx_disable_interrupts(u32 port_id);
void edma_cfg_tx_enable_interrupts(u32 port_id);
void edma_cfg_tx_napi_enable(u32 port_id);
void edma_cfg_tx_napi_disable(u32 port_id);
void edma_cfg_tx_napi_delete(u32 port_id);
void edma_cfg_tx_napi_add(struct net_device *netdevice, u32 macid);
void edma_cfg_tx_ring_mappings(void);
void edma_cfg_txcmpl_mapping_fill(void);
void edma_cfg_tx_rings_enable(u32 port_id);
void edma_cfg_tx_rings_disable(u32 port_id);
void edma_cfg_tx_fill_per_port_tx_map(struct net_device *netdev, u32 macid);
#endif
