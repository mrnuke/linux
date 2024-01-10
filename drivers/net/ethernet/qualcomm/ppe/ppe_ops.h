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
#define PPE_RSS_HASH_MODE_IPV4		BIT(0)
#define PPE_RSS_HASH_MODE_IPV6		BIT(1)
#define PPE_QUEUE_AC_TYPE_QUEUE			0
#define PPE_QUEUE_AC_TYPE_GROUP			1
#define PPE_QUEUE_AC_UCAST_MAX			255
#define PPE_QUEUE_AC_VALUE_MASK			GENMASK(23, 0)
#define PPE_QUEUE_AC_TYPE_MASK			GENMASK(31, 24)
#define PPE_RING_MAPPED_BP_QUEUE_WORD_COUNT	10

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

/* bypss_bitmap_0 */
enum {
	IN_VLAN_TAG_FMT_CHECK_BYP = 0,
	IN_VLAN_MEMBER_CHECK_BYP,
	IN_VLAN_XLT_BYP,
	MY_MAC_CHECK_BYP,
	DIP_LOOKUP_BYP,
	FLOW_LOOKUP_BYP = 5,
	FLOW_ACTION_BYP,
	ACL_BYP,
	FAKE_MAC_HEADER_BYP,
	SERVICE_CODE_BYP,
	WRONG_PKT_FMT_L2_BYP = 10,
	WRONG_PKT_FMT_L3_IPV4_BYP,
	WRONG_PKT_FMT_L3_IPV6_BYP,
	WRONG_PKT_FMT_L4_BYP,
	FLOW_SERVICE_CODE_BYP,
	ACL_SERVICE_CODE_BYP = 15,
	FAKE_L2_PROTO_BYP,
	PPPOE_TERMINATION_BYP,
	DEFAULT_VLAN_BYP,
	DEFAULT_PCP_BYP,
	VSI_ASSIGN_BYP,
	IN_VLAN_ASSIGN_FAIL_BYP = 24,
	SOURCE_GUARD_BYP,
	MRU_MTU_CHECK_BYP,
	FLOW_SRC_CHECK_BYP,
	FLOW_QOS_BYP,
};

/* bypss_bitmap_1 */
enum {
	EG_VLAN_MEMBER_CHECK_BYP = 0,
	EG_VLAN_XLT_BYP,
	EG_VLAN_TAG_FMT_CTRL_BYP,
	FDB_LEARN_BYP,
	FDB_REFRESH_BYP,
	L2_SOURCE_SEC_BYP = 5,
	MANAGEMENT_FWD_BYP,
	BRIDGING_FWD_BYP,
	IN_STP_FLTR_BYP,
	EG_STP_FLTR_BYP,
	SOURCE_FLTR_BYP = 10,
	POLICER_BYP,
	L2_PKT_EDIT_BYP,
	L3_PKT_EDIT_BYP,
	ACL_POST_ROUTING_CHECK_BYP,
	PORT_ISOLATION_BYP = 15,
	PRE_ACL_QOS_BYP,
	POST_ACL_QOS_BYP,
	DSCP_QOS_BYP,
	PCP_QOS_BYP,
	PREHEADER_QOS_BYP = 20,
	FAKE_MAC_DROP_BYP,
	TUNL_CONTEXT_BYP,
	FLOW_POLICER_BYP,
};

/* bypss_bitmap_2 */
enum {
	RX_VLAN_COUNTER_BYP = 0,
	RX_COUNTER_BYP,
	TX_VLAN_COUNTER_BYP,
	TX_COUNTER_BYP,
};

/* bypass_bitmap_3 */
enum {
	TL_SERVICE_CODE_BYP = 0,
	TL_BYP,
	TL_L3_IF_CHECK_BYP,
	TL_VLAN_CHECK_BYP,
	TL_DMAC_CHECK_BYP,
	TL_UDP_CSUM_0_CHECK_BYP = 5,
	TL_TBL_DE_ACCE_CHECK_BYP,
	TL_PPPOE_MC_TERM_CHECK_BYP,
	TL_TTL_EXCEED_CHECK_BYP,
	TL_MAP_SRC_CHECK_BYP,
	TL_MAP_DST_CHECK_BYP = 10,
	TL_LPM_DST_LOOKUP_BYP,
	TL_LPM_LOOKUP_BYP,
	TL_WRONG_PKT_FMT_L2_BYP,
	TL_WRONG_PKT_FMT_L3_IPV4_BYP,
	TL_WRONG_PKT_FMT_L3_IPV6_BYP = 15,
	TL_WRONG_PKT_FMT_L4_BYP,
	TL_WRONG_PKT_FMT_TUNNEL_BYP,
	TL_PRE_IPO_BYP = 20,
};

/* PPE service code is used to bypass hardware handler when the packet pass
 * through PPE, the supported service code number is 256.
 */
struct ppe_servcode_cfg {
	bool dest_port_valid;
	int dest_port;
	u32 bypass_bitmap[4];
	bool is_src;
	int field_update_bitmap;
	int next_service_code;
	int hw_service;
	int offset_sel;
};

/* PPE RSS hash can be configured to generate the hash value based on
 * 5 tuples of packet, the generated hash value is used to decides the
 * final queue ID.
 */
struct ppe_rss_hash_cfg {
	u32 hash_mask;
	bool hash_fragment_mode;
	u32 hash_seed;
	u8 hash_sip_mix[4];
	u8 hash_dip_mix[4];
	u8 hash_protocol_mix;
	u8 hash_sport_mix;
	u8 hash_dport_mix;
	u8 hash_fin_inner[5];
	u8 hash_fin_outer[5];
};

/* PPE queue threshold config for the admission control, the threshold
 * decides the length of queue, the threshold can be configured statically
 * or dynamically changed with the free buffer.
 */
struct ppe_queue_ac_threshold {
	bool color_enable;
	bool wred_enable;
	bool dynamic;
	int shared_weight;
	int green_min_off;
	int yel_max_off;
	int yel_min_off;
	int red_max_off;
	int red_min_off;
	int green_resume_off;
	int yel_resume_off;
	int red_resume_off;
	int ceiling;
};

/* Admission control status of PPE queue. */
struct ppe_queue_ac_ctrl {
	bool ac_en;
	bool ac_fc_en;
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
	int (*rss_hash_config_set)(struct ppe_device *ppe_dev,
				   int mode,
				   struct ppe_rss_hash_cfg hash_cfg);
	int (*queue_ac_threshold_set)(struct ppe_device *ppe_dev,
				      int queue,
				      struct ppe_queue_ac_threshold ac_threshold);
	int (*queue_ac_threshold_get)(struct ppe_device *ppe_dev,
				      int queue,
				      struct ppe_queue_ac_threshold *ac_threshold);
	int (*queue_ac_ctrl_set)(struct ppe_device *ppe_dev,
				 u32 index,
				 struct ppe_queue_ac_ctrl ac_ctrl);
	int (*queue_ac_ctrl_get)(struct ppe_device *ppe_dev,
				 u32 index,
				 struct ppe_queue_ac_ctrl *ac_ctrl);
	int (*ring_queue_map_set)(struct ppe_device *ppe_dev,
				  int ring_id,
				  u32 *queue_map);
};

const struct ppe_queue_ops *ppe_queue_config_ops_get(void);

int ppe_servcode_config_set(struct ppe_device *ppe_dev,
			    int servcode,
			    struct ppe_servcode_cfg cfg);
int ppe_counter_set(struct ppe_device *ppe_dev, int port, bool enable);
#endif
