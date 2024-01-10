// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* Low level PPE operations made available to higher level network drivers
 * such as ethernet or QoS drivers.
 */

#include <linux/soc/qcom/ppe.h>
#include "ppe_ops.h"
#include "ppe_regs.h"
#include "ppe.h"

static int ppe_scheduler_l0_queue_map_set(struct ppe_device *ppe_dev,
					  int node_id, int port,
					  struct ppe_qos_scheduler_cfg scheduler_cfg)
{
	u32 val, index;

	if (node_id >= PPE_L0_FLOW_MAP_TBL_NUM)
		return -EINVAL;

	val = FIELD_PREP(PPE_L0_FLOW_MAP_TBL_SP_ID, scheduler_cfg.sp_id) |
			 FIELD_PREP(PPE_L0_FLOW_MAP_TBL_C_PRI, scheduler_cfg.c_pri) |
			 FIELD_PREP(PPE_L0_FLOW_MAP_TBL_E_PRI, scheduler_cfg.e_pri) |
			 FIELD_PREP(PPE_L0_FLOW_MAP_TBL_C_DRR_WT, scheduler_cfg.c_drr_wt) |
			 FIELD_PREP(PPE_L0_FLOW_MAP_TBL_E_DRR_WT, scheduler_cfg.e_drr_wt);
	index = PPE_L0_FLOW_MAP_TBL + node_id * PPE_L0_FLOW_MAP_TBL_INC;
	ppe_write(ppe_dev, index, val);

	val = FIELD_PREP(PPE_L0_C_SP_CFG_TBL_DRR_ID, scheduler_cfg.c_drr_id) |
			 FIELD_PREP(PPE_L0_C_SP_CFG_TBL_DRR_CREDIT_UNIT, scheduler_cfg.c_drr_unit);
	index = PPE_L0_C_SP_CFG_TBL +
		(scheduler_cfg.sp_id * 8 + scheduler_cfg.c_pri) * PPE_L0_C_SP_CFG_TBL_INC;
	ppe_write(ppe_dev, index, val);

	val = FIELD_PREP(PPE_L0_E_SP_CFG_TBL_DRR_ID, scheduler_cfg.e_drr_id) |
			 FIELD_PREP(PPE_L0_E_SP_CFG_TBL_DRR_CREDIT_UNIT, scheduler_cfg.e_drr_unit);
	index = PPE_L0_E_SP_CFG_TBL +
		(scheduler_cfg.sp_id * 8 + scheduler_cfg.e_pri) * PPE_L0_E_SP_CFG_TBL_INC;
	ppe_write(ppe_dev, index, val);

	val = FIELD_PREP(PPE_L0_FLOW_PORT_MAP_TBL_PORT_NUM, port);
	index = PPE_L0_FLOW_PORT_MAP_TBL + node_id * PPE_L0_FLOW_PORT_MAP_TBL_INC;
	ppe_write(ppe_dev, index, val);

	index = PPE_L0_COMP_CFG_TBL + node_id * PPE_L0_COMP_CFG_TBL_INC;
	return ppe_mask(ppe_dev, index, PPE_L0_COMP_CFG_TBL_DRR_METER_LEN,
			FIELD_PREP(PPE_L0_COMP_CFG_TBL_DRR_METER_LEN,
				   scheduler_cfg.drr_frame_mode));
}

static int ppe_scheduler_l0_queue_map_get(struct ppe_device *ppe_dev,
					  int node_id, int *port,
					  struct ppe_qos_scheduler_cfg *scheduler_cfg)
{
	u32 val, index;

	if (node_id >= PPE_L0_FLOW_MAP_TBL_NUM)
		return -EINVAL;

	index = PPE_L0_FLOW_MAP_TBL + node_id * PPE_L0_FLOW_MAP_TBL_INC;
	ppe_read(ppe_dev, index, &val);
	scheduler_cfg->sp_id = FIELD_GET(PPE_L0_FLOW_MAP_TBL_SP_ID, val);
	scheduler_cfg->c_pri = FIELD_GET(PPE_L0_FLOW_MAP_TBL_C_PRI, val);
	scheduler_cfg->e_pri = FIELD_GET(PPE_L0_FLOW_MAP_TBL_E_PRI, val);
	scheduler_cfg->c_drr_wt = FIELD_GET(PPE_L0_FLOW_MAP_TBL_C_DRR_WT, val);
	scheduler_cfg->e_drr_wt = FIELD_GET(PPE_L0_FLOW_MAP_TBL_E_DRR_WT, val);

	index = PPE_L0_C_SP_CFG_TBL +
		(scheduler_cfg->sp_id * 8 + scheduler_cfg->c_pri) * PPE_L0_C_SP_CFG_TBL_INC;
	ppe_read(ppe_dev, index, &val);
	scheduler_cfg->c_drr_id = FIELD_GET(PPE_L0_C_SP_CFG_TBL_DRR_ID, val);
	scheduler_cfg->c_drr_unit = FIELD_GET(PPE_L0_C_SP_CFG_TBL_DRR_CREDIT_UNIT, val);

	index = PPE_L0_E_SP_CFG_TBL +
		(scheduler_cfg->sp_id * 8 + scheduler_cfg->e_pri) * PPE_L0_E_SP_CFG_TBL_INC;
	ppe_read(ppe_dev, index, &val);
	scheduler_cfg->e_drr_id = FIELD_GET(PPE_L0_E_SP_CFG_TBL_DRR_ID, val);
	scheduler_cfg->e_drr_unit = FIELD_GET(PPE_L0_E_SP_CFG_TBL_DRR_CREDIT_UNIT, val);

	index = PPE_L0_FLOW_PORT_MAP_TBL + node_id * PPE_L0_FLOW_PORT_MAP_TBL_INC;
	ppe_read(ppe_dev, index, &val);
	*port = FIELD_GET(PPE_L0_FLOW_PORT_MAP_TBL_PORT_NUM, val);

	index = PPE_L0_COMP_CFG_TBL + node_id * PPE_L0_COMP_CFG_TBL_INC;
	ppe_read(ppe_dev, index, &val);
	scheduler_cfg->drr_frame_mode = FIELD_GET(PPE_L0_COMP_CFG_TBL_DRR_METER_LEN, val);

	return 0;
}

static int ppe_scheduler_l1_queue_map_set(struct ppe_device *ppe_dev,
					  int node_id, int port,
					  struct ppe_qos_scheduler_cfg scheduler_cfg)
{
	u32 val, index;

	if (node_id >= PPE_L1_FLOW_MAP_TBL_NUM)
		return -EINVAL;

	val = FIELD_PREP(PPE_L1_FLOW_MAP_TBL_SP_ID, scheduler_cfg.sp_id) |
			 FIELD_PREP(PPE_L1_FLOW_MAP_TBL_C_PRI, scheduler_cfg.c_pri) |
			 FIELD_PREP(PPE_L1_FLOW_MAP_TBL_E_PRI, scheduler_cfg.e_pri) |
			 FIELD_PREP(PPE_L1_FLOW_MAP_TBL_C_DRR_WT, scheduler_cfg.c_drr_wt) |
			 FIELD_PREP(PPE_L1_FLOW_MAP_TBL_E_DRR_WT, scheduler_cfg.e_drr_wt);
	index = PPE_L1_FLOW_MAP_TBL + node_id * PPE_L1_FLOW_MAP_TBL_INC;
	ppe_write(ppe_dev, index, val);

	val = FIELD_PREP(PPE_L1_C_SP_CFG_TBL_DRR_ID, scheduler_cfg.c_drr_id) |
			 FIELD_PREP(PPE_L1_C_SP_CFG_TBL_DRR_CREDIT_UNIT, scheduler_cfg.c_drr_unit);
	index = PPE_L1_C_SP_CFG_TBL +
		(scheduler_cfg.sp_id * 8 + scheduler_cfg.c_pri) * PPE_L1_C_SP_CFG_TBL_INC;
	ppe_write(ppe_dev, index, val);

	val = FIELD_PREP(PPE_L1_E_SP_CFG_TBL_DRR_ID, scheduler_cfg.e_drr_id) |
		FIELD_PREP(PPE_L1_E_SP_CFG_TBL_DRR_CREDIT_UNIT, scheduler_cfg.e_drr_unit);
	index = PPE_L1_E_SP_CFG_TBL +
		(scheduler_cfg.sp_id * 8 + scheduler_cfg.e_pri) * PPE_L1_E_SP_CFG_TBL_INC;
	ppe_write(ppe_dev, index, val);

	val = FIELD_PREP(PPE_L1_FLOW_PORT_MAP_TBL_PORT_NUM, port);
	index = PPE_L1_FLOW_PORT_MAP_TBL + node_id * PPE_L1_FLOW_PORT_MAP_TBL_INC;
	ppe_write(ppe_dev, index, val);

	index = PPE_L1_COMP_CFG_TBL + node_id * PPE_L1_COMP_CFG_TBL_INC;
	return ppe_mask(ppe_dev, index, PPE_L1_COMP_CFG_TBL_DRR_METER_LEN,
			FIELD_PREP(PPE_L1_COMP_CFG_TBL_DRR_METER_LEN,
				   scheduler_cfg.drr_frame_mode));
}

static int ppe_scheduler_l1_queue_map_get(struct ppe_device *ppe_dev,
					  int node_id, int *port,
					  struct ppe_qos_scheduler_cfg *scheduler_cfg)
{
	u32 val, index;

	if (node_id >= PPE_L1_FLOW_MAP_TBL_NUM)
		return -EINVAL;

	index = PPE_L1_FLOW_MAP_TBL + node_id * PPE_L1_FLOW_MAP_TBL_INC;
	ppe_read(ppe_dev, index, &val);
	scheduler_cfg->sp_id = FIELD_GET(PPE_L1_FLOW_MAP_TBL_SP_ID, val);
	scheduler_cfg->c_pri = FIELD_GET(PPE_L1_FLOW_MAP_TBL_C_PRI, val);
	scheduler_cfg->e_pri = FIELD_GET(PPE_L1_FLOW_MAP_TBL_E_PRI, val);
	scheduler_cfg->c_drr_wt = FIELD_GET(PPE_L1_FLOW_MAP_TBL_C_DRR_WT, val);
	scheduler_cfg->e_drr_wt = FIELD_GET(PPE_L1_FLOW_MAP_TBL_E_DRR_WT, val);

	index = PPE_L1_C_SP_CFG_TBL +
		(scheduler_cfg->sp_id * 8 + scheduler_cfg->c_pri) * PPE_L1_C_SP_CFG_TBL_INC;
	ppe_read(ppe_dev, index, &val);
	scheduler_cfg->c_drr_id = FIELD_GET(PPE_L1_C_SP_CFG_TBL_DRR_ID, val);
	scheduler_cfg->c_drr_unit = FIELD_GET(PPE_L1_C_SP_CFG_TBL_DRR_CREDIT_UNIT, val);

	index = PPE_L1_E_SP_CFG_TBL +
		(scheduler_cfg->sp_id * 8 + scheduler_cfg->e_pri) * PPE_L1_E_SP_CFG_TBL_INC;
	ppe_read(ppe_dev, index, &val);
	scheduler_cfg->e_drr_id = FIELD_GET(PPE_L1_E_SP_CFG_TBL_DRR_ID, val);
	scheduler_cfg->e_drr_unit = FIELD_GET(PPE_L1_E_SP_CFG_TBL_DRR_CREDIT_UNIT, val);

	index = PPE_L1_FLOW_PORT_MAP_TBL + node_id * PPE_L1_FLOW_PORT_MAP_TBL_INC;
	ppe_read(ppe_dev, index, &val);
	*port = FIELD_GET(PPE_L1_FLOW_PORT_MAP_TBL_PORT_NUM, val);

	index = PPE_L1_COMP_CFG_TBL + node_id * PPE_L1_COMP_CFG_TBL_INC;
	ppe_read(ppe_dev, index, &val);
	scheduler_cfg->drr_frame_mode = FIELD_GET(PPE_L1_COMP_CFG_TBL_DRR_METER_LEN, val);

	return 0;
}

static int ppe_queue_scheduler_set(struct ppe_device *ppe_dev,
				   int node_id, int level, int port,
				   struct ppe_qos_scheduler_cfg scheduler_cfg)
{
	if (level == 0)
		return ppe_scheduler_l0_queue_map_set(ppe_dev, node_id, port, scheduler_cfg);
	else if (level == 1)
		return ppe_scheduler_l1_queue_map_set(ppe_dev, node_id, port, scheduler_cfg);
	else
		return -EINVAL;
}

static int ppe_queue_scheduler_get(struct ppe_device *ppe_dev,
				   int node_id, int level, int *port,
				   struct ppe_qos_scheduler_cfg *scheduler_cfg)
{
	if (level == 0)
		return ppe_scheduler_l0_queue_map_get(ppe_dev, node_id, port, scheduler_cfg);
	else if (level == 1)
		return ppe_scheduler_l1_queue_map_get(ppe_dev, node_id, port, scheduler_cfg);
	else
		return -EINVAL;
}

static const struct ppe_queue_ops qcom_ppe_queue_config_ops = {
	.queue_scheduler_set = ppe_queue_scheduler_set,
	.queue_scheduler_get = ppe_queue_scheduler_get,
};

const struct ppe_queue_ops *ppe_queue_config_ops_get(void)
{
	return &qcom_ppe_queue_config_ops;
}
EXPORT_SYMBOL_GPL(ppe_queue_config_ops_get);
