// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "ppe.h"
#include "ppe_api.h"
#include "ppe_config.h"

/**
 * ppe_queue_node_priority_set - set scheduler priority of PPE queue or flow
 * @ppe_dev: PPE device
 * @node_id: PPE hardware node ID, which can be queue ID or flow ID.
 * @priority: PPE discipline scheduler priority
 *
 * Configure scheduler priority for a given PPE node. Node may be of type
 * PPE queue or flow. The packet is dispatched first by queue scheduler
 * (level 0), then dispatched by flow scheduler (level 1).
 *
 * Return 0 on success, negative error code on failure.
 */
int ppe_queue_node_priority_set(struct ppe_device *ppe_dev,
				int node_id, int priority)
{
	struct ppe_scheduler_cfg sch_cfg;
	int ret, port, level = 0;

	if (node_id >= PPE_QUEUE_ID_MAX + PPE_FLOW_ID_MAX)
		return -EINVAL;

	if (node_id >= PPE_QUEUE_ID_MAX) {
		level = 1;
		node_id -= PPE_QUEUE_ID_MAX;
	}

	ret = ppe_queue_scheduler_get(ppe_dev, node_id, level, &port, &sch_cfg);
	if (ret)
		return ret;

	sch_cfg.pri = priority;

	return ppe_queue_scheduler_set(ppe_dev, node_id, level, port, sch_cfg);
}

/**
 * ppe_edma_queue_offset_config - Configure queue offset for EDMA interface
 * @ppe_dev: PPE device
 * @type: The type can be internal priority or PPE hash
 * @index: Class index, which can be internal priority or hash value
 * @queue_offset: Queue offset value which is added by the queue base to get
 * 		  the egress queue ID.
 *
 * PPE EDMA queue offset is configured based on the PPE internal priority or
 * RSS hash value, the profile ID is fixed to 0 for the EDMA interface.
 *
 * Return 0 on success, negative error code on failure.
 */
int ppe_edma_queue_offset_config(struct ppe_device *ppe_dev,
				 enum ppe_queue_offset_type type,
				 int index, int queue_offset)
{
	if (type == PPE_QUEUE_OFFSET_BY_PRIORITY)
		return ppe_queue_ucast_offset_pri_set(ppe_dev, 0,
						      index, queue_offset);

	return ppe_queue_ucast_offset_hash_set(ppe_dev, 0,
					       index, queue_offset);
}

/**
 * ppe_edma_queue_resource_get - Get EDMA queue resource
 * @ppe_dev: PPE device
 * @type: Resource type
 * @res_start: Resource start ID returned
 * @res_end: Resource end ID returned
 *
 * PPE EDMA queue resource includes unicast queue and multicast queue.
 *
 * Return 0 on success, negative error code on failure.
 */
int ppe_edma_queue_resource_get(struct ppe_device *ppe_dev,
				enum ppe_resource_type type,
				int *res_start, int *res_end)
{
	if (type != PPE_RES_UCAST && type != PPE_RES_MCAST)
		return -EINVAL;

	return ppe_port_resource_get(ppe_dev, 0, type, res_start, res_end);
};

/**
 * ppe_edma_ring_to_queues_config - Configure EDMA ring to queue mapping in PPE
 * @ppe_dev: PPE device
 * @ring_id: EDMA ring ID
 * @num: Number of queues mapped to EDMA ring
 * @queues: PPE queue IDs
 *
 * Enable EDMA ring to PPE queue mapping configuration for packet
 * receive to an EDMA ring.
 *
 * Return 0 on success, negative error code on failure.
 */
int ppe_edma_ring_to_queues_config(struct ppe_device *ppe_dev, int ring_id,
				   int num, int queues[])
{
	u32 queue_bmap[PPE_RING_TO_QUEUE_BITMAP_WORD_CNT] = {};
	int index;

	for (index = 0; index < num; index++)
		queue_bmap[queues[index] / 32] |= BIT_MASK(queues[index] % 32);

	return ppe_ring_queue_map_set(ppe_dev, ring_id, queue_bmap);
}
