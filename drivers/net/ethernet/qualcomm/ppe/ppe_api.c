// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "ppe.h"
#include "ppe_api.h"
#include "ppe_config.h"

/**
 * ppe_queue_priority_set - set scheduler priority of PPE hardware queue
 * @ppe_dev: PPE device
 * @node_id: PPE hardware node ID, which is either queue ID or flow ID
 * @priority: Qos scheduler priority
 *
 * Configure scheduler priority of PPE hardware queque, the maximum node
 * ID supported is PPE_QUEUE_ID_NUM added by PPE_FLOW_ID_NUM, queue ID
 * belongs to level 0, flow ID belongs to level 1 in the packet pipeline.
 *
 * Return 0 on success, negative error code on failure.
 */
int ppe_queue_priority_set(struct ppe_device *ppe_dev,
			   int node_id, int priority)
{
	struct ppe_qos_scheduler_cfg sch_cfg;
	int ret, port, level = 0;

	if (node_id >= PPE_QUEUE_ID_NUM) {
		level = 1;
		node_id -= PPE_QUEUE_ID_NUM;
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
 * @class: The class to configure queue offset
 * @index: Class index, internal priority or hash value
 * @queue_offset: Queue offset value
 *
 * PPE EDMA queue offset is configured based on the PPE internal priority or
 * RSS hash value, the profile ID is fixed to 0 for EDMA interface.
 *
 * Return 0 on success, negative error code on failure.
 */
int ppe_edma_queue_offset_config(struct ppe_device *ppe_dev,
				 enum ppe_queue_class_type class,
				 int index, int queue_offset)
{
	if (class == PPE_QUEUE_CLASS_PRIORITY)
		return ppe_queue_ucast_pri_class_set(ppe_dev, 0,
						     index, queue_offset);

	return ppe_queue_ucast_hash_class_set(ppe_dev, 0,
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
int ppe_edma_queue_resource_get(struct ppe_device *ppe_dev, int type,
				int *res_start, int *res_end)
{
	if (type != PPE_RES_UCAST && type != PPE_RES_MCAST)
		return -EINVAL;

	return ppe_port_resource_get(ppe_dev, 0, type, res_start, res_end);
};

/**
 * ppe_edma_ring_to_queues_config - Map EDMA ring to PPE queues
 * @ppe_dev: PPE device
 * @ring_id: EDMA ring ID
 * @num: Number of queues mapped to EDMA ring
 * @queues: PPE queue IDs
 *
 * PPE queues are configured to map with the special EDMA ring ID.
 *
 * Return 0 on success, negative error code on failure.
 */
int ppe_edma_ring_to_queues_config(struct ppe_device *ppe_dev, int ring_id,
				   int num, int queues[] __counted_by(num))
{
	u32 queue_bmap[PPE_RING_MAPPED_BP_QUEUE_WORD_COUNT] = {};
	int index;

	for (index = 0; index < num; index++)
		queue_bmap[queues[index] / 32] |= BIT_MASK(queues[index] % 32);

	return ppe_ring_queue_map_set(ppe_dev, ring_id, queue_bmap);
}
