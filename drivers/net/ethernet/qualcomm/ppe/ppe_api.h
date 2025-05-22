/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* Functions for low level PPE configurations which are needed during ethernet
 * driver initialization.
 */

#ifndef __PPE_API_H__
#define __PPE_API_H__

#include "ppe.h"
#include "ppe_config.h"

#define PPE_QUEUE_ID_MAX			300
#define PPE_FLOW_ID_MAX				64
#define PPE_QUEUE_INTERNAL_PRI_NUM		16
#define PPE_QUEUE_HASH_NUM			256

/**
 * enum ppe_queue_offset_type - PPE queue offset type
 * @PPE_QUEUE_CLASS_PRIORITY: Queue offset decided by PPE internal priority
 * @PPE_QUEUE_CLASS_HASH: Queue offset decided by PPE RSS hash value.
 */
enum ppe_queue_offset_type {
	PPE_QUEUE_OFFSET_BY_PRIORITY,
	PPE_QUEUE_OFFSET_BY_HASH,
};

int ppe_queue_node_priority_set(struct ppe_device *ppe_dev,
				int node_id, int priority);

int ppe_edma_queue_offset_config(struct ppe_device *ppe_dev,
				 enum ppe_queue_offset_type type,
				 int index, int queue_offset);

int ppe_edma_queue_resource_get(struct ppe_device *ppe_dev,
				enum ppe_resource_type type,
				int *res_start, int *res_end);
int ppe_edma_ring_to_queues_config(struct ppe_device *ppe_dev, int ring_id,
				   int num, int queues[]);
#endif
