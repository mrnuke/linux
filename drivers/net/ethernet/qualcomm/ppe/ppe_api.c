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
