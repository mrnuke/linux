/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __PPE_CONFIG_H__
#define __PPE_CONFIG_H__

/**
 * struct ppe_qos_scheduler_cfg - PPE QoS scheduler configuration.
 * @flow_id: PPE flow ID.
 * @pri: Scheduler priority.
 * @drr_node_id: Node ID for scheduled traffic.
 * @drr_node_wt: weight for scheduled traffic.
 * @node_unit : Unit for scheduled traffic.
 * @node_frame_mode: Packet mode to be scheduled.
 *
 * PPE QoS feature supports the commit and exceed traffic.
 */
struct ppe_qos_scheduler_cfg {
	int flow_id;
	int pri;
	int drr_node_id;
	int drr_node_wt;
	int node_unit;
	int node_frame_mode;
};

int ppe_hw_config(struct ppe_device *ppe_dev);
int ppe_queue_scheduler_set(struct ppe_device *ppe_dev,
			    int node_id, bool flow_level, int port,
			    struct ppe_qos_scheduler_cfg scheduler_cfg);
int ppe_queue_scheduler_get(struct ppe_device *ppe_dev,
			    int node_id, bool flow_level, int *port,
			    struct ppe_qos_scheduler_cfg *scheduler_cfg);
#endif
