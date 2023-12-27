/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* These may also be used by higher level network drivers such as ethernet or
 * QoS drivers.
 */

#ifndef __PPE_API_H__
#define __PPE_API_H__

#include "ppe.h"

#define PPE_QUEUE_ID_NUM			300
#define PPE_FLOW_ID_NUM				64
#define PPE_QUEUE_SCH_PRI_NUM			8
#define PPE_QUEUE_INTER_PRI_NUM			16
#define PPE_QUEUE_HASH_NUM			256

/* The service code is used by EDMA driver to transmit packet to PPE. */
#define PPE_EDMA_SC_BYPASS_ID			1

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
 * enum ppe_resource_type - PPE resource type
 * @PPE_RES_UCAST: Unicast queue resource
 * @PPE_RES_MCAST: Multicast queue resource
 * @PPE_RES_FLOW_ID: Flow resource
 * @PPE_RES_L0_NODE: Level 0 QoS node resource
 * @PPE_RES_L1_NODE: Level 1 QoS node resource
 */
enum ppe_resource_type {
	PPE_RES_UCAST,
	PPE_RES_MCAST,
	PPE_RES_FLOW_ID,
	PPE_RES_L0_NODE,
	PPE_RES_L1_NODE,
};

int ppe_queue_priority_set(struct ppe_device *ppe_dev,
			   int queue_id, int priority);

int ppe_edma_queue_offset_config(struct ppe_device *ppe_dev,
				 enum ppe_queue_class_type class,
				 int index, int queue_offset);
int ppe_edma_queue_resource_get(struct ppe_device *ppe_dev, int type,
				int *res_start, int *res_end);
int ppe_edma_ring_to_queues_config(struct ppe_device *ppe_dev, int ring_id,
				   int num, int queues[] __counted_by(num));
#endif
