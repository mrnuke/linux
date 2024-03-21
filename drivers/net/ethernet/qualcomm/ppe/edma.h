/* SPDX-License-Identifier: GPL-2.0-only
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __EDMA_MAIN__
#define __EDMA_MAIN__

#include "ppe_config.h"

/* One clock cycle = 1/(EDMA clock frequency in Mhz) micro seconds.
 *
 * One timer unit is 128 clock cycles.
 *
 * So, therefore the microsecond to timer unit calculation is:
 * Timer unit = time in microseconds / (one clock cycle in microsecond * cycles in 1 timer unit)
 *            = ('x' microsecond * EDMA clock frequency in MHz ('y') / 128).
 *
 */
#define EDMA_CYCLE_PER_TIMER_UNIT	128
#define EDMA_MICROSEC_TO_TIMER_UNIT(x, y)	((x) * (y) / EDMA_CYCLE_PER_TIMER_UNIT)
#define MHZ			1000000UL

/* EDMA profile ID. */
#define EDMA_CPU_PORT_PROFILE_ID  0

/* Number of PPE queue priorities supported per ARM core. */
#define EDMA_PRI_MAX_PER_CORE	8

/* Interface ID start. */
#define EDMA_START_IFNUM   1

/**
 * enum ppe_queue_class_type - PPE queue class type
 * @PPE_QUEUE_CLASS_PRIORITY: Queue offset configured from internal priority
 * @PPE_QUEUE_CLASS_HASH: Queue offset configured from RSS hash.
 */
enum ppe_queue_class_type {
	PPE_QUEUE_CLASS_PRIORITY,
	PPE_QUEUE_CLASS_HASH,
};

/**
 * struct edma_ring_info - EDMA ring data structure.
 * @max_rings: Maximum number of rings
 * @ring_start: Ring start ID
 * @num_rings: Number of rings
 */
struct edma_ring_info {
	u32 max_rings;
	u32 ring_start;
	u32 num_rings;
};

/**
 * struct edma_hw_info - EDMA hardware data structure.
 * @rxfill: Rx Fill ring information
 * @rx: Rx Desc ring information
 * @tx: Tx Desc ring information
 * @txcmpl: Tx complete ring information
 * @max_ports: Maximum number of ports
 * @napi_budget_rx: Rx NAPI budget
 * @napi_budget_tx: Tx NAPI budget
 */
struct edma_hw_info {
	struct edma_ring_info *rxfill;
	struct edma_ring_info *rx;
	struct edma_ring_info *tx;
	struct edma_ring_info *txcmpl;
	u32 max_ports;
	u32 napi_budget_rx;
	u32 napi_budget_tx;
};

/**
 * struct edma_intr_info - EDMA interrupt data structure.
 * @intr_mask_rx: RX interrupt mask
 * @intr_rx: Rx interrupts
 * @intr_mask_txcmpl: Tx completion interrupt mask
 * @intr_txcmpl: Tx completion interrupts
 * @intr_mask_misc: Miscellaneous interrupt mask
 * @intr_misc: Miscellaneous interrupts
 */
struct edma_intr_info {
	u32 intr_mask_rx;
	u32 *intr_rx;
	u32 intr_mask_txcmpl;
	u32 *intr_txcmpl;
	u32 intr_mask_misc;
	u32 intr_misc;
};

/**
 * struct edma_context - EDMA context.
 * @netdev_arr: Net device for each EDMA port
 * @ppe_dev: PPE device
 * @hw_info: EDMA Hardware info
 * @intr_info: EDMA Interrupt info
 */
struct edma_context {
	struct net_device **netdev_arr;
	struct ppe_device *ppe_dev;
	struct edma_hw_info *hw_info;
	struct edma_intr_info intr_info;
};

/* Global EDMA context. */
extern struct edma_context *edma_ctx;

void edma_destroy(struct ppe_device *ppe_dev);
int edma_setup(struct ppe_device *ppe_dev);
int ppe_edma_queue_offset_config(struct ppe_device *ppe_dev,
				 enum ppe_queue_class_type class,
				 int index, int queue_offset);


#endif
