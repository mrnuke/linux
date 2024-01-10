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

static int ppe_queue_ucast_base_set(struct ppe_device *ppe_dev,
				    struct ppe_queue_ucast_dest queue_dst,
				    int queue_base, int profile_id)
{
	u32 reg_val;
	int index;

	if (queue_dst.service_code_en)
		index = 2048 + (queue_dst.src_profile << 8) + queue_dst.service_code;
	else if (queue_dst.cpu_code_en)
		index = 1024 + (queue_dst.src_profile << 8) + queue_dst.cpu_code;
	else
		index = (queue_dst.src_profile << 8) + queue_dst.dest_port;

	reg_val = FIELD_PREP(PPE_UCAST_QUEUE_MAP_TBL_PROFILE_ID, profile_id) |
		  FIELD_PREP(PPE_UCAST_QUEUE_MAP_TBL_QUEUE_ID, queue_base);

	return ppe_write(ppe_dev, PPE_UCAST_QUEUE_MAP_TBL + index * PPE_UCAST_QUEUE_MAP_TBL_INC,
			 reg_val);
}

static  int ppe_queue_ucast_base_get(struct ppe_device *ppe_dev,
				     struct ppe_queue_ucast_dest queue_dst,
				     int *queue_base, int *profile_id)
{
	u32 reg_val;
	int index;

	if (queue_dst.service_code_en)
		index = 2048 + (queue_dst.src_profile << 8) + queue_dst.service_code;
	else if (queue_dst.cpu_code_en)
		index = 1024 + (queue_dst.src_profile << 8) + queue_dst.cpu_code;
	else
		index = (queue_dst.src_profile << 8) + queue_dst.dest_port;

	ppe_read(ppe_dev, PPE_UCAST_QUEUE_MAP_TBL + index * PPE_UCAST_QUEUE_MAP_TBL_INC, &reg_val);

	*queue_base = FIELD_GET(PPE_UCAST_QUEUE_MAP_TBL_QUEUE_ID, reg_val);
	*profile_id = FIELD_GET(PPE_UCAST_QUEUE_MAP_TBL_PROFILE_ID, reg_val);

	return 0;
}

static int ppe_queue_ucast_pri_class_set(struct ppe_device *ppe_dev,
					 int profile_id,
					 int priority,
					 int class_offset)
{
	u32 reg_val;
	int index;

	index = (profile_id << 4) + priority;
	reg_val = FIELD_PREP(PPE_UCAST_PRIORITY_MAP_TBL_CLASS, class_offset);

	return ppe_write(ppe_dev,
			 PPE_UCAST_PRIORITY_MAP_TBL + index * PPE_UCAST_PRIORITY_MAP_TBL_INC,
			 reg_val);
}

static int ppe_queue_ucast_hash_class_set(struct ppe_device *ppe_dev,
					  int profile_id,
					  int rss_hash,
					  int class_offset)
{
	u32 reg_val;
	int index;

	index = (profile_id << 4) + rss_hash;
	reg_val = FIELD_PREP(PPE_UCAST_HASH_MAP_TBL_HASH, class_offset);

	return ppe_write(ppe_dev,
			 PPE_UCAST_HASH_MAP_TBL + index * PPE_UCAST_HASH_MAP_TBL_INC,
			 reg_val);
}

int ppe_servcode_config_set(struct ppe_device *ppe_dev,
			    int servcode,
			    struct ppe_servcode_cfg cfg)
{
	union ppe_eg_service_cfg_u eg_service_cfg;
	union ppe_service_cfg_u service_cfg;
	int val;

	memset(&service_cfg, 0, sizeof(service_cfg));
	memset(&eg_service_cfg, 0, sizeof(eg_service_cfg));

	val = FIELD_PREP(PPE_IN_L2_SERVICE_TBL_DST_PORT_ID_VALID, cfg.dest_port_valid) |
		FIELD_PREP(PPE_IN_L2_SERVICE_TBL_DST_PORT_ID, cfg.dest_port) |
		FIELD_PREP(PPE_IN_L2_SERVICE_TBL_DST_DIRECTION, cfg.is_src) |
		FIELD_PREP(PPE_IN_L2_SERVICE_TBL_DST_BYPASS_BITMAP, cfg.bypass_bitmap[1]) |
		FIELD_PREP(PPE_IN_L2_SERVICE_TBL_RX_CNT_EN,
			   cfg.bypass_bitmap[2] & BIT(1) ? 1 : 0) |
		FIELD_PREP(PPE_IN_L2_SERVICE_TBL_TX_CNT_EN,
			   cfg.bypass_bitmap[2] & BIT(3) ? 1 : 0);
	ppe_write(ppe_dev, PPE_IN_L2_SERVICE_TBL + PPE_IN_L2_SERVICE_TBL_INC * servcode, val);

	ppe_read_tbl(ppe_dev, PPE_SERVICE_TBL + PPE_SERVICE_TBL_INC * servcode,
		     service_cfg.val, sizeof(service_cfg.val));
	service_cfg.bf.bypass_bitmap = cfg.bypass_bitmap[0];
	service_cfg.bf.rx_counting_en = cfg.bypass_bitmap[2] & BIT(0);
	ppe_write_tbl(ppe_dev, PPE_SERVICE_TBL + PPE_SERVICE_TBL_INC * servcode,
		      service_cfg.val, sizeof(service_cfg.val));

	ppe_read_tbl(ppe_dev, PPE_EG_SERVICE_TBL + PPE_EG_SERVICE_TBL_INC * servcode,
		     eg_service_cfg.val, sizeof(eg_service_cfg.val));
	eg_service_cfg.bf.field_update_action = cfg.field_update_bitmap;
	eg_service_cfg.bf.next_service_code = cfg.next_service_code;
	eg_service_cfg.bf.hw_services = cfg.hw_service;
	eg_service_cfg.bf.offset_sel = cfg.offset_sel;
	eg_service_cfg.bf.tx_counting_en = cfg.bypass_bitmap[2] & BIT(2) ? 1 : 0;
	ppe_write_tbl(ppe_dev, PPE_EG_SERVICE_TBL + PPE_EG_SERVICE_TBL_INC * servcode,
		      eg_service_cfg.val, sizeof(eg_service_cfg.val));

	val = FIELD_PREP(PPE_TL_SERVICE_TBL_BYPASS_BITMAP, cfg.bypass_bitmap[3]);
	return ppe_write(ppe_dev, PPE_TL_SERVICE_TBL + PPE_TL_SERVICE_TBL_INC * servcode, val);
}

int ppe_counter_set(struct ppe_device *ppe_dev, int port, bool enable)
{
	union ppe_mru_mtu_ctrl_cfg_u mru_mtu_cfg;

	memset(&mru_mtu_cfg, 0, sizeof(mru_mtu_cfg));

	ppe_read_tbl(ppe_dev, PPE_MRU_MTU_CTRL_TBL + PPE_MRU_MTU_CTRL_TBL_INC * port,
		     mru_mtu_cfg.val, sizeof(mru_mtu_cfg.val));
	mru_mtu_cfg.bf.rx_cnt_en = enable;
	mru_mtu_cfg.bf.tx_cnt_en = enable;
	ppe_write_tbl(ppe_dev, PPE_MRU_MTU_CTRL_TBL + PPE_MRU_MTU_CTRL_TBL_INC * port,
		      mru_mtu_cfg.val, sizeof(mru_mtu_cfg.val));

	ppe_mask(ppe_dev, PPE_MC_MTU_CTRL_TBL + PPE_MC_MTU_CTRL_TBL_INC * port,
		 PPE_MC_MTU_CTRL_TBL_TX_CNT_EN,
		 FIELD_PREP(PPE_MC_MTU_CTRL_TBL_TX_CNT_EN, enable));

	return ppe_mask(ppe_dev, PPE_PORT_EG_VLAN + PPE_PORT_EG_VLAN_INC * port,
			PPE_PORT_EG_VLAN_TX_COUNTING_EN,
			FIELD_PREP(PPE_PORT_EG_VLAN_TX_COUNTING_EN, enable));
}

static int ppe_rss_hash_config_set(struct ppe_device *ppe_dev,
				   int mode,
				   struct ppe_rss_hash_cfg cfg)
{
	u32 val;
	int i;

	if (mode & PPE_RSS_HASH_MODE_IPV4) {
		val = FIELD_PREP(PPE_RSS_HASH_MASK_IPV4_HASH_MASK, cfg.hash_mask) |
				 FIELD_PREP(PPE_RSS_HASH_MASK_IPV4_FRAGMENT,
					    cfg.hash_fragment_mode);
		ppe_write(ppe_dev, PPE_RSS_HASH_MASK_IPV4, val);

		val = FIELD_PREP(PPE_RSS_HASH_SEED_IPV4_VAL, cfg.hash_seed);
		ppe_write(ppe_dev, PPE_RSS_HASH_SEED_IPV4, val);

		for (i = 0; i < PPE_RSS_HASH_MIX_IPV4_NUM; i++) {
			switch (i) {
			case 0:
				val = FIELD_PREP(PPE_RSS_HASH_MIX_IPV4_VAL,
						 cfg.hash_sip_mix[0]);
				break;
			case 1:
				val = FIELD_PREP(PPE_RSS_HASH_MIX_IPV4_VAL,
						 cfg.hash_dip_mix[0]);
				break;
			case 2:
				val = FIELD_PREP(PPE_RSS_HASH_MIX_IPV4_VAL,
						 cfg.hash_protocol_mix);
				break;
			case 3:
				val = FIELD_PREP(PPE_RSS_HASH_MIX_IPV4_VAL,
						 cfg.hash_dport_mix);
				break;
			case 4:
				val = FIELD_PREP(PPE_RSS_HASH_MIX_IPV4_VAL,
						 cfg.hash_sport_mix);
				break;
			default:
				break;
			}
			ppe_write(ppe_dev, PPE_RSS_HASH_MIX_IPV4 + i * PPE_RSS_HASH_MIX_IPV4_INC,
				  val);
		}

		for (i = 0; i < PPE_RSS_HASH_MIX_IPV4_NUM; i++) {
			val = FIELD_PREP(PPE_RSS_HASH_FIN_IPV4_INNER, cfg.hash_fin_inner[i]) |
					 FIELD_PREP(PPE_RSS_HASH_FIN_IPV4_OUTER,
						    cfg.hash_fin_outer[i]);
			ppe_write(ppe_dev, PPE_RSS_HASH_FIN_IPV4 + i * PPE_RSS_HASH_FIN_IPV4_INC,
				  val);
		}
	}

	if (mode & PPE_RSS_HASH_MODE_IPV6) {
		val = FIELD_PREP(PPE_RSS_HASH_MASK_HASH_MASK, cfg.hash_mask) |
				 FIELD_PREP(PPE_RSS_HASH_MASK_FRAGMENT, cfg.hash_fragment_mode);
		ppe_write(ppe_dev, PPE_RSS_HASH_MASK, val);

		val = FIELD_PREP(PPE_RSS_HASH_SEED_VAL, cfg.hash_seed);
		ppe_write(ppe_dev, PPE_RSS_HASH_SEED, val);

		for (i = 0; i < PPE_RSS_HASH_MIX_NUM; i++) {
			switch (i) {
			case 0 ... 3:
				val = FIELD_PREP(PPE_RSS_HASH_MIX_VAL, cfg.hash_sip_mix[i]);
				break;
			case 4 ... 7:
				val = FIELD_PREP(PPE_RSS_HASH_MIX_VAL, cfg.hash_dip_mix[i - 4]);
				break;
			case 8:
				val = FIELD_PREP(PPE_RSS_HASH_MIX_VAL, cfg.hash_protocol_mix);
				break;
			case 9:
				val = FIELD_PREP(PPE_RSS_HASH_MIX_VAL, cfg.hash_dport_mix);
				break;
			case 10:
				val = FIELD_PREP(PPE_RSS_HASH_MIX_VAL, cfg.hash_sport_mix);
				break;
			default:
				break;
			}
			ppe_write(ppe_dev, PPE_RSS_HASH_MIX + i * PPE_RSS_HASH_MIX_INC, val);
		}

		for (i = 0; i < PPE_RSS_HASH_FIN_NUM; i++) {
			val = FIELD_PREP(PPE_RSS_HASH_FIN_INNER, cfg.hash_fin_inner[i]) |
					 FIELD_PREP(PPE_RSS_HASH_FIN_OUTER, cfg.hash_fin_outer[i]);

			ppe_write(ppe_dev, PPE_RSS_HASH_FIN + i * PPE_RSS_HASH_FIN_INC, val);
		}
	}

	return 0;
}

static const struct ppe_queue_ops qcom_ppe_queue_config_ops = {
	.queue_scheduler_set = ppe_queue_scheduler_set,
	.queue_scheduler_get = ppe_queue_scheduler_get,
	.queue_ucast_base_set = ppe_queue_ucast_base_set,
	.queue_ucast_base_get = ppe_queue_ucast_base_get,
	.queue_ucast_pri_class_set = ppe_queue_ucast_pri_class_set,
	.queue_ucast_hash_class_set = ppe_queue_ucast_hash_class_set,
	.rss_hash_config_set = ppe_rss_hash_config_set,
};

const struct ppe_queue_ops *ppe_queue_config_ops_get(void)
{
	return &qcom_ppe_queue_config_ops;
}
EXPORT_SYMBOL_GPL(ppe_queue_config_ops_get);
