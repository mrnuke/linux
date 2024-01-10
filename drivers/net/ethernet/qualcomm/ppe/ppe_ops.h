/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* Low level PPE operations to be used by higher level network drivers
 * such as ethernet or QoS drivers.
 */

#ifndef __PPE_OPS_H__
#define __PPE_OPS_H__

#define PPE_QUEUE_PRI_MAX		16
#define PPE_QUEUE_HASH_MAX		256

/* PPE hardware QoS configurations used to dispatch the packet passed
 * through PPE, the scheduler supports DRR(deficit round robin with the
 * weight) and SP(strict priority).
 */
struct ppe_qos_scheduler_cfg {
	int sp_id;
	int e_pri;
	int c_pri;
	int c_drr_id;
	int e_drr_id;
	int e_drr_wt;
	int c_drr_wt;
	int c_drr_unit;
	int e_drr_unit;
	int drr_frame_mode;
};

/* The egress queue ID can be decided by service code, CPU code and
 * egress port.
 *
 * service code has the highest priority to decide queue base, then
 * CPU code, finally egress port when all are enabled.
 */
struct ppe_queue_ucast_dest {
	int src_profile;
	bool service_code_en;
	int service_code;
	bool cpu_code_en;
	int cpu_code;
	int dest_port;
};

/* The operations are used to configure the PPE queue related resource */
struct ppe_queue_ops {
	int (*queue_scheduler_set)(struct ppe_device *ppe_dev,
				   int node_id,
				   int level,
				   int port,
				   struct ppe_qos_scheduler_cfg scheduler_cfg);
	int (*queue_scheduler_get)(struct ppe_device *ppe_dev,
				   int node_id,
				   int level,
				   int *port,
				   struct ppe_qos_scheduler_cfg *scheduler_cfg);
	int (*queue_ucast_base_set)(struct ppe_device *ppe_dev,
				    struct ppe_queue_ucast_dest queue_dst,
				    int queue_base,
				    int profile_id);
	int (*queue_ucast_base_get)(struct ppe_device *ppe_dev,
				    struct ppe_queue_ucast_dest queue_dst,
				    int *queue_base,
				    int *profile_id);
	int (*queue_ucast_pri_class_set)(struct ppe_device *ppe_dev,
					 int profile_id,
					 int priority,
					 int class_offset);
	int (*queue_ucast_hash_class_set)(struct ppe_device *ppe_dev,
					  int profile_id,
					  int rss_hash,
					  int class_offset);
};

const struct ppe_queue_ops *ppe_queue_config_ops_get(void);
#endif
