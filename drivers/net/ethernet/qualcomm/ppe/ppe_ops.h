/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* Low level PPE operations to be used by higher level network drivers
 * such as ethernet or QoS drivers.
 */

#ifndef __PPE_OPS_H__
#define __PPE_OPS_H__

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
};

const struct ppe_queue_ops *ppe_queue_config_ops_get(void);
#endif
