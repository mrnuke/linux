/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __PPE_CONFIG_H__
#define __PPE_CONFIG_H__

/* There are different queue config ranges for the destination port,
 * CPU code and service code.
 */
#define PPE_QUEUE_BASE_DEST_PORT		0
#define PPE_QUEUE_BASE_CPU_CODE			1024
#define PPE_QUEUE_BASE_SERVICE_CODE		2048

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

/**
 * struct ppe_queue_ucast_dest - PPE unicast queue destination.
 * @src_profile: Source profile.
 * @service_code_en: Enable service code.
 * @service_code: Service code.
 * @cpu_code_en: Enable CPU code.
 * @cpu_code: CPU code.
 * @dest_port: destination port.
 *
 * PPE egress queue ID is decided by the egress port ID.
 */
struct ppe_queue_ucast_dest {
	int src_profile;
	bool service_code_en;
	int service_code;
	bool cpu_code_en;
	int cpu_code;
	int dest_port;
};

int ppe_hw_config(struct ppe_device *ppe_dev);
int ppe_queue_scheduler_set(struct ppe_device *ppe_dev,
			    int node_id, bool flow_level, int port,
			    struct ppe_qos_scheduler_cfg scheduler_cfg);
int ppe_queue_scheduler_get(struct ppe_device *ppe_dev,
			    int node_id, bool flow_level, int *port,
			    struct ppe_qos_scheduler_cfg *scheduler_cfg);
int ppe_queue_ucast_base_set(struct ppe_device *ppe_dev,
			     struct ppe_queue_ucast_dest queue_dst,
			     int queue_base,
			     int profile_id);
int ppe_queue_ucast_pri_class_set(struct ppe_device *ppe_dev,
				  int profile_id,
				  int priority,
				  int class_offset);
int ppe_queue_ucast_hash_class_set(struct ppe_device *ppe_dev,
				   int profile_id,
				   int rss_hash,
				   int class_offset);
int ppe_port_resource_get(struct ppe_device *ppe_dev, int port, int type,
			  int *res_start, int *res_end);
#endif
