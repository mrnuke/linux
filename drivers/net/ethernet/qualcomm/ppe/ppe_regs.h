/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* PPE hardware register and table declarations. */
#ifndef __PPE_REGS_H__
#define __PPE_REGS_H__

#define PPE_BM_TDM_CTRL						0xb000
#define PPE_BM_TDM_CTRL_NUM					1
#define PPE_BM_TDM_CTRL_INC					4
#define PPE_BM_TDM_CTRL_TDM_DEPTH				GENMASK(7, 0)
#define PPE_BM_TDM_CTRL_TDM_OFFSET				GENMASK(14, 8)
#define PPE_BM_TDM_CTRL_TDM_EN					BIT(31)

#define PPE_RX_FIFO_CFG						0xb004
#define PPE_RX_FIFO_CFG_NUM					8
#define PPE_RX_FIFO_CFG_INC					4
#define PPE_RX_FIFO_CFG_THRSH					GENMASK(2, 0)

#define PPE_DROP_CNT						0xb024
#define PPE_DROP_CNT_NUM					8
#define PPE_DROP_CNT_INC					4
#define PPE_DROP_CNT_PKT_CNT					GENMASK(31, 0)

#define PPE_DROP_STAT						0xe000
#define PPE_DROP_STAT_NUM					30
#define PPE_DROP_STAT_INC					0x10
#define PPE_DROP_STAT_PKT_CNT					GENMASK(31, 0)

/* BM port drop counter */
struct ppe_drop_stat {
	u32 pkt_cnt;
	u32 byte_cnt_0;
	u32 byte_cnt_1:8,
	    res0:24;
};

union ppe_drop_stat_u {
	u32 val[3];
	struct ppe_drop_stat bf;
};

#define PPE_EPE_DBG_IN_CNT					0x26054
#define PPE_EPE_DBG_IN_CNT_NUM					1
#define PPE_EPE_DBG_IN_CNT_INC					0x4

#define PPE_EPE_DBG_OUT_CNT					0x26070
#define PPE_EPE_DBG_OUT_CNT_NUM					1
#define PPE_EPE_DBG_OUT_CNT_INC					0x4

#define PPE_EG_VSI_COUNTER_TBL					0x41000
#define PPE_EG_VSI_COUNTER_TBL_NUM				64
#define PPE_EG_VSI_COUNTER_TBL_INC				0x10

/* Egress VLAN counter */
struct ppe_eg_vsi_cnt_tbl {
	u32 pkt_cnt;
	u32 byte_cnt_0;
	u32 byte_cnt_1:8,
	    res0:24;
};

union ppe_eg_vsi_cnt_tbl_u {
	u32 val[3];
	struct ppe_eg_vsi_cnt_tbl bf;
};

#define PPE_PORT_TX_COUNTER_TBL					0x45000
#define PPE_PORT_TX_COUNTER_TBL_NUM				8
#define PPE_PORT_TX_COUNTER_TBL_INC				0x10

/* Port TX counter */
struct ppe_port_tx_counter_tbl {
	u32 pkt_cnt;
	u32 byte_cnt_0;
	u32 byte_cnt_1:8,
	    res0:24;
};

union ppe_port_tx_counter_tbl_u {
	u32 val[3];
	struct ppe_port_tx_counter_tbl bf;
};

#define PPE_VPORT_TX_COUNTER_TBL				0x47000
#define PPE_VPORT_TX_COUNTER_TBL_NUM				256
#define PPE_VPORT_TX_COUNTER_TBL_INC				0x10

/* Virtual port TX counter */
struct ppe_vport_tx_counter_tbl {
	u32 pkt_cnt;
	u32 byte_cnt_0;
	u32 byte_cnt_1:8,
	    res0:24;
};

union ppe_vport_tx_counter_tbl_u {
	u32 val[3];
	struct ppe_vport_tx_counter_tbl bf;
};

#define PPE_QUEUE_TX_COUNTER_TBL				0x4a000
#define PPE_QUEUE_TX_COUNTER_TBL_NUM				300
#define PPE_QUEUE_TX_COUNTER_TBL_INC				0x10

/* Queue counter */
struct ppe_queue_tx_counter_tbl {
	u32 pkt_cnt;
	u32 byte_cnt_0;
	u32 byte_cnt_1:8,
	    res0:24;
};

union ppe_queue_tx_counter_tbl_u {
	u32 val[3];
	struct ppe_queue_tx_counter_tbl bf;
};

#define PPE_RSS_HASH_MASK					0xb4318
#define PPE_RSS_HASH_MASK_NUM					1
#define PPE_RSS_HASH_MASK_INC					4
#define PPE_RSS_HASH_MASK_HASH_MASK				GENMASK(20, 0)
#define PPE_RSS_HASH_MASK_FRAGMENT				BIT(28)

#define PPE_RSS_HASH_SEED					0xb431c
#define PPE_RSS_HASH_SEED_NUM					1
#define PPE_RSS_HASH_SEED_INC					4
#define PPE_RSS_HASH_SEED_VAL					GENMASK(31, 0)

#define PPE_RSS_HASH_MIX					0xb4320
#define PPE_RSS_HASH_MIX_NUM					11
#define PPE_RSS_HASH_MIX_INC					4
#define PPE_RSS_HASH_MIX_VAL					GENMASK(4, 0)

#define PPE_RSS_HASH_FIN					0xb4350
#define PPE_RSS_HASH_FIN_NUM					5
#define PPE_RSS_HASH_FIN_INC					4
#define PPE_RSS_HASH_FIN_INNER					GENMASK(4, 0)
#define PPE_RSS_HASH_FIN_OUTER					GENMASK(9, 5)

#define PPE_RSS_HASH_MASK_IPV4					0xb4380
#define PPE_RSS_HASH_MASK_IPV4_NUM				1
#define PPE_RSS_HASH_MASK_IPV4_INC				4
#define PPE_RSS_HASH_MASK_IPV4_HASH_MASK			GENMASK(20, 0)
#define PPE_RSS_HASH_MASK_IPV4_FRAGMENT				BIT(28)

#define PPE_RSS_HASH_SEED_IPV4					0xb4384
#define PPE_RSS_HASH_SEED_IPV4_NUM				1
#define PPE_RSS_HASH_SEED_IPV4_INC				4
#define PPE_RSS_HASH_SEED_IPV4_VAL				GENMASK(31, 0)

#define PPE_RSS_HASH_MIX_IPV4					0xb4390
#define PPE_RSS_HASH_MIX_IPV4_NUM				5
#define PPE_RSS_HASH_MIX_IPV4_INC				4
#define PPE_RSS_HASH_MIX_IPV4_VAL				GENMASK(4, 0)

#define PPE_RSS_HASH_FIN_IPV4					0xb43b0
#define PPE_RSS_HASH_FIN_IPV4_NUM				5
#define PPE_RSS_HASH_FIN_IPV4_INC				4
#define PPE_RSS_HASH_FIN_IPV4_INNER				GENMASK(4, 0)
#define PPE_RSS_HASH_FIN_IPV4_OUTER				GENMASK(9, 5)

#define PPE_BM_TDM_CFG_TBL					0xc000
#define PPE_BM_TDM_CFG_TBL_NUM					128
#define PPE_BM_TDM_CFG_TBL_INC					0x10
#define PPE_BM_TDM_CFG_TBL_PORT_NUM				GENMASK(3, 0)
#define PPE_BM_TDM_CFG_TBL_DIR					BIT(4)
#define PPE_BM_TDM_CFG_TBL_VALID				BIT(5)
#define PPE_BM_TDM_CFG_TBL_SECOND_PORT_VALID			BIT(6)
#define PPE_BM_TDM_CFG_TBL_SECOND_PORT				GENMASK(11, 8)

#define PPE_SERVICE_TBL						0x15000
#define PPE_SERVICE_TBL_NUM					256
#define PPE_SERVICE_TBL_INC					0x10
#define PPE_SERVICE_TBL_BYPASS_BITMAP				GENMASK(31, 0)
#define PPE_SERVICE_TBL_RX_COUNTING_EN				BIT(32)

/* service code for the ingress packet, the PPE features can be bypassed
 * with service config.
 */
struct ppe_service_cfg {
	u32 bypass_bitmap;
	u32 rx_counting_en:1,
	    res0:31;
};

union ppe_service_cfg_u {
	u32 val[2];
	struct ppe_service_cfg bf;
};

#define PPE_PORT_EG_VLAN					0x20020
#define PPE_PORT_EG_VLAN_NUM					8
#define PPE_PORT_EG_VLAN_INC					4
#define PPE_PORT_EG_VLAN_PORT_VLAN_TYPE				BIT(0)
#define PPE_PORT_EG_VLAN_PORT_EG_VLAN_CTAG_MODE			GENMASK(2, 1)
#define PPE_PORT_EG_VLAN_PORT_EG_VLAN_STAG_MODE			GENMASK(4, 3)
#define PPE_PORT_EG_VLAN_VSI_TAG_MODE_EN			BIT(5)
#define PPE_PORT_EG_VLAN_PORT_EG_PCP_PROP_CMD			BIT(6)
#define PPE_PORT_EG_VLAN_PORT_EG_DEI_PROP_CMD			BIT(7)
#define PPE_PORT_EG_VLAN_TX_COUNTING_EN				BIT(8)

#define PPE_EG_BRIDGE_CONFIG					0x20044
#define PPE_EG_BRIDGE_CONFIG_QUEUE_CNT_EN			BIT(2)

#define PPE_EG_SERVICE_TBL					0x43000
#define PPE_EG_SERVICE_TBL_NUM					256
#define PPE_EG_SERVICE_TBL_INC					0x10

/* service code config for the egress packet, the new service code can be
 * generated and ath header can be configured.
 */
struct ppe_eg_service_cfg {
	u32 field_update_action;
	u32 next_service_code:8,
	    hw_services:6,
	    offset_sel:1,
	    tx_counting_en:1,
	    ip_length_update:1,
	    ath_hdr_insert_dis:1,
	    ath_hdr_type:3,
	    ath_from_cpu:1,
	    ath_disable_bit:1,
	    ath_port_bitmap:7,
	    res0:2;
};

union ppe_eg_service_cfg_u {
	u32 val[2];
	struct ppe_eg_service_cfg bf;
};

#define PPE_TX_BUFF_THRSH					0x26100
#define PPE_TX_BUFF_THRSH_NUM					8
#define PPE_TX_BUFF_THRSH_INC					4
#define PPE_TX_BUFF_THRSH_XOFF					GENMASK(7, 0)
#define PPE_TX_BUFF_THRSH_XON					GENMASK(15, 8)

#define PPE_L2_GLOBAL_CONFIG					0x60038
#define PPE_L2_GLOBAL_CONFIG_LRN_EN				BIT(6)
#define PPE_L2_GLOBAL_CONFIG_AGE_EN				BIT(7)

#define PPE_MIRROR_ANALYZER					0x60040
#define PPE_MIRROR_ANALYZER_NUM					1
#define PPE_MIRROR_ANALYZER_INC					4
#define PPE_MIRROR_ANALYZER_INGRESS_PORT			GENMASK(5, 0)
#define PPE_MIRROR_ANALYZER_EGRESS_PORT				GENMASK(13, 8)

#define PPE_PORT_BRIDGE_CTRL					0x60300
#define PPE_PORT_BRIDGE_CTRL_NUM				8
#define PPE_PORT_BRIDGE_CTRL_INC				4
#define PPE_PORT_BRIDGE_CTRL_NEW_ADDR_LRN_EN			BIT(0)
#define PPE_PORT_BRIDGE_CTRL_NEW_ADDR_FWD_CMD			GENMASK(2, 1)
#define PPE_PORT_BRIDGE_CTRL_STATION_MODE_LRN_EN		BIT(3)
#define PPE_PORT_BRIDGE_CTRL_STATION_MODE_FWD_CMD		GENMASK(5, 4)
#define PPE_PORT_BRIDGE_CTRL_ISOLATION_BITMAP			GENMASK(15, 8)
#define PPE_PORT_BRIDGE_CTRL_TXMAC_EN				BIT(16)
#define PPE_PORT_BRIDGE_CTRL_PROMISC_EN				BIT(17)
#define PPE_PORT_BRIDGE_CTRL_MASK				GENMASK(17, 0)

#define PPE_PORT_MIRROR						0x60800
#define PPE_PORT_MIRROR_NUM					8
#define PPE_PORT_MIRROR_INC					4
#define PPE_PORT_MIRROR_INGRESS_EN				BIT(0)
#define PPE_PORT_MIRROR_EGRESS_EN				BIT(1)

#define PPE_CST_STATE						0x60100
#define PPE_CST_STATE_NUM					8
#define PPE_CST_STATE_INC					4
#define PPE_CST_STATE_PORT_STATE				GENMASK(1, 0)

#define PPE_MC_MTU_CTRL_TBL					0x60a00
#define PPE_MC_MTU_CTRL_TBL_NUM					8
#define PPE_MC_MTU_CTRL_TBL_INC					4
#define PPE_MC_MTU_CTRL_TBL_MTU					GENMASK(13, 0)
#define PPE_MC_MTU_CTRL_TBL_MTU_CMD				GENMASK(15, 14)
#define PPE_MC_MTU_CTRL_TBL_TX_CNT_EN				BIT(16)

#define PPE_VSI_TBL						0x63800
#define PPE_VSI_TBL_NUM						64
#define PPE_VSI_TBL_INC						0x10

/* PPE vsi configurations */
struct ppe_vsi_tbl {
	u32 member_port_bitmap:8,
	    uuc_bitmap:8,
	    umc_bitmap:8,
	    bc_bitmap:8;
	u32 new_addr_lrn_en:1,
	    new_addr_fwd_cmd:2,
	    station_move_lrn_en:1,
	    station_move_fwd_cmd:2,
	    res0:26;
};

union ppe_vsi_tbl_u {
	u32 val[2];
	struct ppe_vsi_tbl bf;
};

#define PPE_MRU_MTU_CTRL_TBL					0x65000
#define PPE_MRU_MTU_CTRL_TBL_NUM				256
#define PPE_MRU_MTU_CTRL_TBL_INC				0x10

/* PPE port control configuration, the MTU and QoS are configured by
 * this table.
 */
struct ppe_mru_mtu_ctrl_cfg {
	u32 mru:14,
	    mru_cmd:2,
	    mtu:14,
	    mtu_cmd:2;

	u32 rx_cnt_en:1,
	    tx_cnt_en:1,
	    src_profile:2,
	    pcp_qos_group_id:1,
	    dscp_qos_group_id:1,
	    pcp_res_prec_force:1,
	    dscp_res_prec_force:1,
	    preheader_res_prec:3,
	    pcp_res_prec:3,
	    dscp_res_prec:3,
	    flow_res_prec:3,
	    pre_acl_res_prec:3,
	    post_acl_res_prec:3,
	    source_filtering_bypass:1,
	    source_filtering_mode:1,
	    pre_ipo_outer_res_prec:3,
	    pre_ipo_inner_res_prec_0:1;

	u32 pre_ipo_inner_res_prec_1:2,
	    res0:30;
};

union ppe_mru_mtu_ctrl_cfg_u {
	u32 val[3];
	struct ppe_mru_mtu_ctrl_cfg bf;
};

#define PPE_IN_L2_SERVICE_TBL					0x66000
#define PPE_IN_L2_SERVICE_TBL_NUM				256
#define PPE_IN_L2_SERVICE_TBL_INC				0x10
#define PPE_IN_L2_SERVICE_TBL_DST_PORT_ID_VALID			BIT(0)
#define PPE_IN_L2_SERVICE_TBL_DST_PORT_ID			GENMASK(4, 1)
#define PPE_IN_L2_SERVICE_TBL_DST_DIRECTION			BIT(5)
#define PPE_IN_L2_SERVICE_TBL_DST_BYPASS_BITMAP			GENMASK(29, 6)
#define PPE_IN_L2_SERVICE_TBL_RX_CNT_EN				BIT(30)
#define PPE_IN_L2_SERVICE_TBL_TX_CNT_EN				BIT(31)

#define PPE_L2_VP_PORT_TBL					0x98000
#define PPE_L2_VP_PORT_TBL_NUM					256
#define PPE_L2_VP_PORT_TBL_INC					0x10

/* Port configurations */
struct ppe_l2_vp_port_tbl {
	u32 invalid_vsi_forwarding_en:1,
	    promisc_en:1,
	    dst_info:8,
	    physical_port:3,
	    new_addr_lrn_en:1,
	    new_addr_fwd_cmd:2,
	    station_move_lrn_en:1,
	    station_move_fwd_cmd:2,
	    lrn_lmt_cnt:12,
	    lrn_lmt_en:1;
	u32 lrn_lmt_exceed_fwd:2,
	    eg_vlan_fltr_cmd:1,
	    port_isolation_bitmap:8,
	    isol_profile:6,
	    isol_en:1,
	    policer_en:1,
	    policer_index:9,
	    vp_state_check_en:1,
	    vp_type:1,
	    vp_context_active:1,
	    vp_eg_data_valid:1;
	u32 physical_port_mtu_check_en:1,
	    mtu_check_type:1,
	    extra_header_len:8,
	    eg_vlan_fmt_valid:1,
	    eg_stag_fmt:1,
	    eg_ctag_fmt:1,
	    exception_fmt_ctrl:1,
	    enq_service_code_en:1,
	    enq_service_code:8,
	    enq_phy_port:3,
	    app_ctrl_profile_0:6;
	u32 app_ctrl_profile_1:2,
	    res0:30;
};

union ppe_l2_vp_port_tbl_u {
	u32 val[4];
	struct ppe_l2_vp_port_tbl bf;
};

#define PPE_PORT_RX_CNT_TBL					0x150000
#define PPE_PORT_RX_CNT_TBL_NUM					256
#define PPE_PORT_RX_CNT_TBL_INC					0x20

/* Port RX counter */
struct ppe_port_rx_cnt_tbl {
	u32 pkt_cnt;
	u32 byte_cnt_0;
	u32 byte_cnt_1:8,
	    drop_pkt_cnt_0:24;
	u32 drop_pkt_cnt_1:8,
	    drop_byte_cnt_0:24;
	u32 drop_byte_cnt_1:16,
	    res0:16;
};

union ppe_port_rx_cnt_tbl_u {
	u32 val[5];
	struct ppe_port_rx_cnt_tbl bf;
};

#define PPE_PHY_PORT_RX_CNT_TBL					0x156000
#define PPE_PHY_PORT_RX_CNT_TBL_NUM				8
#define PPE_PHY_PORT_RX_CNT_TBL_INC				0x20

/* Physical port RX and RX drop counter */
struct ppe_phy_port_rx_cnt_tbl {
	u32 pkt_cnt;
	u32 byte_cnt_0;
	u32 byte_cnt_1:8,
	    drop_pkt_cnt_0:24;
	u32 drop_pkt_cnt_1:8,
	    drop_byte_cnt_0:24;
	u32 drop_byte_cnt_1:16,
	    res0:16;
};

union ppe_phy_port_rx_cnt_tbl_u {
	u32 val[5];
	struct ppe_phy_port_rx_cnt_tbl bf;
};

#define PPE_DROP_CPU_CNT_TBL					0x160000
#define PPE_DROP_CPU_CNT_TBL_NUM				1280
#define PPE_DROP_CPU_CNT_TBL_INC				0x10

/* counter for the packet to CPU port */
struct ppe_drop_cpu_cnt {
	u32 pkt_cnt;
	u32 byte_cnt_0;
	u32 byte_cnt_1:8,
	    res0:24;
};

union ppe_drop_cpu_cnt_u {
	u32 val[3];
	struct ppe_drop_cpu_cnt bf;
};

#define PPE_VLAN_CNT_TBL					0x178000
#define PPE_VLAN_CNT_TBL_NUM					64
#define PPE_VLAN_CNT_TBL_INC					0x10

/* VLAN counter */
struct ppe_vlan_cnt {
	u32 pkt_cnt;
	u32 byte_cnt_0;
	u32 byte_cnt_1:8,
	    res0:24;
};

union ppe_vlan_cnt_u {
	u32 val[3];
	struct ppe_vlan_cnt bf;
};

#define PPE_PRE_L2_CNT_TBL					0x17c000
#define PPE_PRE_L2_CNT_TBL_NUM					64
#define PPE_PRE_L2_CNT_TBL_INC					0x20

/* PPE L2 counter */
struct ppe_pre_l2_cnt_tbl {
	u32 pkt_cnt;
	u32 byte_cnt_0;
	u32 byte_cnt_1:8,
	    drop_pkt_cnt_0:24;
	u32 drop_pkt_cnt_1:8,
	    drop_byte_cnt_0:24;
	u32 drop_byte_cnt_1:16,
	    res0:16;
};

union ppe_pre_l2_cnt_tbl_u {
	u32 val[5];
	struct ppe_pre_l2_cnt_tbl bf;
};

#define PPE_PORT_TX_DROP_CNT_TBL				0x17d000
#define PPE_PORT_TX_DROP_CNT_TBL_NUM				8
#define PPE_PORT_TX_DROP_CNT_TBL_INC				0x10

/* Port TX drop counter */
struct ppe_port_tx_drop_cnt {
	u32 pkt_cnt;
	u32 byte_cnt_0;
	u32 byte_cnt_1:8,
	    res0:24;
};

union ppe_port_tx_drop_u {
	u32 val[3];
	struct ppe_port_tx_drop_cnt bf;
};

#define PPE_VPORT_TX_DROP_CNT_TBL				0x17e000
#define PPE_VPORT_TX_DROP_CNT_TBL_NUM				256
#define PPE_VPORT_TX_DROP_CNT_TBL_INC				0x10

/* Virtual port TX counter */
struct ppe_vport_tx_drop_cnt {
	u32 pkt_cnt;
	u32 byte_cnt_0;
	u32 byte_cnt_1:8,
	    res0:24;
};

union ppe_vport_tx_drop_u {
	u32 val[3];
	struct ppe_vport_tx_drop_cnt bf;
};

#define PPE_TPR_PKT_CNT						0x1d0080
#define PPE_IPR_PKT_CNT						0x1e0080
#define PPE_IPR_PKT_CNT_NUM					8
#define PPE_IPR_PKT_CNT_INC					4
#define PPE_IPR_PKT_CNT_PKT_CNT					GENMASK(31, 0)

#define PPE_TL_SERVICE_TBL					0x306000
#define PPE_TL_SERVICE_TBL_NUM					256
#define PPE_TL_SERVICE_TBL_INC					4
#define PPE_TL_SERVICE_TBL_BYPASS_BITMAP			GENMASK(31, 0)

#define PPE_PSCH_TDM_DEPTH_CFG					0x400000
#define PPE_PSCH_TDM_DEPTH_CFG_NUM				1
#define PPE_PSCH_TDM_DEPTH_CFG_INC				4
#define PPE_PSCH_TDM_DEPTH_CFG_TDM_DEPTH			GENMASK(7, 0)

#define PPE_L0_FLOW_MAP_TBL					0x402000
#define PPE_L0_FLOW_MAP_TBL_NUM					300
#define PPE_L0_FLOW_MAP_TBL_INC					0x10
#define PPE_L0_FLOW_MAP_TBL_SP_ID				GENMASK(5, 0)
#define PPE_L0_FLOW_MAP_TBL_C_PRI				GENMASK(8, 6)
#define PPE_L0_FLOW_MAP_TBL_E_PRI				GENMASK(11, 9)
#define PPE_L0_FLOW_MAP_TBL_C_DRR_WT				GENMASK(21, 12)
#define PPE_L0_FLOW_MAP_TBL_E_DRR_WT				GENMASK(31, 22)

#define PPE_L0_C_SP_CFG_TBL					0x404000
#define PPE_L0_C_SP_CFG_TBL_NUM					512
#define PPE_L0_C_SP_CFG_TBL_INC					0x10
#define PPE_L0_C_SP_CFG_TBL_DRR_ID				GENMASK(7, 0)
#define PPE_L0_C_SP_CFG_TBL_DRR_CREDIT_UNIT			BIT(8)

#define PPE_L0_E_SP_CFG_TBL					0x406000
#define PPE_L0_E_SP_CFG_TBL_NUM					512
#define PPE_L0_E_SP_CFG_TBL_INC					0x10
#define PPE_L0_E_SP_CFG_TBL_DRR_ID				GENMASK(7, 0)
#define PPE_L0_E_SP_CFG_TBL_DRR_CREDIT_UNIT			BIT(8)

#define PPE_L0_FLOW_PORT_MAP_TBL				0x408000
#define PPE_L0_FLOW_PORT_MAP_TBL_NUM				300
#define PPE_L0_FLOW_PORT_MAP_TBL_INC				0x10
#define PPE_L0_FLOW_PORT_MAP_TBL_PORT_NUM			GENMASK(3, 0)

#define PPE_L0_COMP_CFG_TBL					0x428000
#define PPE_L0_COMP_CFG_TBL_NUM					300
#define PPE_L0_COMP_CFG_TBL_INC					0x10
#define PPE_L0_COMP_CFG_TBL_SHAPER_METER_LEN			GENMASK(1, 0)
#define PPE_L0_COMP_CFG_TBL_DRR_METER_LEN			GENMASK(3, 2)

#define PPE_RING_Q_MAP_TBL					0x42a000
#define PPE_RING_Q_MAP_TBL_NUM					24
#define PPE_RING_Q_MAP_TBL_INC					0x40

/* The queue bitmap for the back pressure from EDAM RX ring to PPE queue */
struct ppe_ring_q_map_cfg {
	u32 queue_bitmap_0;
	u32 queue_bitmap_1;
	u32 queue_bitmap_2;
	u32 queue_bitmap_3;
	u32 queue_bitmap_4;
	u32 queue_bitmap_5;
	u32 queue_bitmap_6;
	u32 queue_bitmap_7;
	u32 queue_bitmap_8;
	u32 queue_bitmap_9:12,
	    res0:20;
};

union ppe_ring_q_map_cfg_u {
	u32 val[10];
	struct ppe_ring_q_map_cfg bf;
};

#define PPE_DEQ_OPR_TBL						0x430000
#define PPE_DEQ_OPR_TBL_NUM					300
#define PPE_DEQ_OPR_TBL_INC					0x10
#define PPE_ENQ_OPR_TBL_DEQ_DISABLE				BIT(0)

#define PPE_L1_FLOW_MAP_TBL					0x440000
#define PPE_L1_FLOW_MAP_TBL_NUM					64
#define PPE_L1_FLOW_MAP_TBL_INC					0x10
#define PPE_L1_FLOW_MAP_TBL_SP_ID				GENMASK(3, 0)
#define PPE_L1_FLOW_MAP_TBL_C_PRI				GENMASK(6, 4)
#define PPE_L1_FLOW_MAP_TBL_E_PRI				GENMASK(9, 7)
#define PPE_L1_FLOW_MAP_TBL_C_DRR_WT				GENMASK(19, 10)
#define PPE_L1_FLOW_MAP_TBL_E_DRR_WT				GENMASK(29, 20)

#define PPE_L1_C_SP_CFG_TBL					0x442000
#define PPE_L1_C_SP_CFG_TBL_NUM					64
#define PPE_L1_C_SP_CFG_TBL_INC					0x10
#define PPE_L1_C_SP_CFG_TBL_DRR_ID				GENMASK(5, 0)
#define PPE_L1_C_SP_CFG_TBL_DRR_CREDIT_UNIT			BIT(6)

#define PPE_L1_E_SP_CFG_TBL					0x444000
#define PPE_L1_E_SP_CFG_TBL_NUM					64
#define PPE_L1_E_SP_CFG_TBL_INC					0x10
#define PPE_L1_E_SP_CFG_TBL_DRR_ID				GENMASK(5, 0)
#define PPE_L1_E_SP_CFG_TBL_DRR_CREDIT_UNIT			BIT(6)

#define PPE_L1_FLOW_PORT_MAP_TBL				0x446000
#define PPE_L1_FLOW_PORT_MAP_TBL_NUM				64
#define PPE_L1_FLOW_PORT_MAP_TBL_INC				0x10
#define PPE_L1_FLOW_PORT_MAP_TBL_PORT_NUM			GENMASK(3, 0)

#define PPE_L1_COMP_CFG_TBL					0x46a000
#define PPE_L1_COMP_CFG_TBL_NUM					64
#define PPE_L1_COMP_CFG_TBL_INC					0x10
#define PPE_L1_COMP_CFG_TBL_SHAPER_METER_LEN			GENMASK(1, 0)
#define PPE_L1_COMP_CFG_TBL_DRR_METER_LEN			GENMASK(3, 2)

#define PPE_PSCH_TDM_CFG_TBL					0x47a000
#define PPE_PSCH_TDM_CFG_TBL_NUM				128
#define PPE_PSCH_TDM_CFG_TBL_INC				0x10
#define PPE_PSCH_TDM_CFG_TBL_DES_PORT				GENMASK(3, 0)
#define PPE_PSCH_TDM_CFG_TBL_ENS_PORT				GENMASK(7, 4)
#define PPE_PSCH_TDM_CFG_TBL_ENS_PORT_BITMAP			GENMASK(15, 8)
#define PPE_PSCH_TDM_CFG_TBL_DES_SECOND_PORT_EN			BIT(16)
#define PPE_PSCH_TDM_CFG_TBL_DES_SECOND_PORT			GENMASK(20, 17)

#define PPE_BM_PORT_FC_MODE					0x600100
#define PPE_BM_PORT_FC_MODE_NUM					15
#define PPE_BM_PORT_FC_MODE_INC					4
#define PPE_BM_PORT_FC_MODE_EN					BIT(0)

#define PPE_BM_PORT_GROUP_ID					0x600180
#define PPE_BM_PORT_GROUP_ID_NUM				15
#define PPE_BM_PORT_GROUP_ID_INC				4
#define PPE_BM_PORT_GROUP_ID_SHARED_GROUP_ID			GENMASK(1, 0)

#define PPE_BM_USED_CNT						0x6001c0
#define PPE_BM_USED_CNT_NUM					15
#define PPE_BM_USED_CNT_INC					0x4
#define PPE_BM_USED_CNT_VAL					GENMASK(10, 0)

#define PPE_BM_REACT_CNT					0x600240
#define PPE_BM_REACT_CNT_NUM					15
#define PPE_BM_REACT_CNT_INC					0x4
#define PPE_BM_REACT_CNT_VAL					GENMASK(8, 0)

#define PPE_BM_SHARED_GROUP_CFG					0x600290
#define PPE_BM_SHARED_GROUP_CFG_NUM				4
#define PPE_BM_SHARED_GROUP_CFG_INC				4
#define PPE_BM_SHARED_GROUP_CFG_SHARED_LIMIT			GENMASK(10, 0)

#define PPE_BM_PORT_FC_CFG					0x601000
#define PPE_BM_PORT_FC_CFG_NUM					15
#define PPE_BM_PORT_FC_CFG_INC					0x10

/* BM port configurations, BM port(0-7) for CPU port, BM port(8-13) for physical
 * port 1-6.
 */
struct ppe_bm_port_fc_cfg {
	u32 react_limit:9,
	    resum_floor_th:9,
	    resum_offset:11,
	    shared_ceiling_0:3;
	u32 shared_ceiling_1:8,
	    shared_weight:3,
	    shared_dynamic:1,
	    pre_alloc:11,
	    res0:9;
};

union ppe_bm_port_fc_cfg_u {
	u32 val[2];
	struct ppe_bm_port_fc_cfg bf;
};

#define PPE_UCAST_QUEUE_MAP_TBL					0x810000
#define PPE_UCAST_QUEUE_MAP_TBL_NUM				3072
#define PPE_UCAST_QUEUE_MAP_TBL_INC				0x10
#define PPE_UCAST_QUEUE_MAP_TBL_PROFILE_ID			GENMASK(3, 0)
#define PPE_UCAST_QUEUE_MAP_TBL_QUEUE_ID			GENMASK(11, 4)

#define PPE_UCAST_HASH_MAP_TBL					0x830000
#define PPE_UCAST_HASH_MAP_TBL_NUM				4096
#define PPE_UCAST_HASH_MAP_TBL_INC				0x10
#define PPE_UCAST_HASH_MAP_TBL_HASH				GENMASK(7, 0)

#define PPE_UCAST_PRIORITY_MAP_TBL				0x842000
#define PPE_UCAST_PRIORITY_MAP_TBL_NUM				256
#define PPE_UCAST_PRIORITY_MAP_TBL_INC				0x10
#define PPE_UCAST_PRIORITY_MAP_TBL_CLASS			GENMASK(3, 0)

#define PPE_AC_UNI_QUEUE_CFG_TBL				0x848000
#define PPE_AC_UNI_QUEUE_CFG_TBL_NUM				256
#define PPE_AC_UNI_QUEUE_CFG_TBL_INC				0x10

/* PPE unicast queue(0-255) configurations, the threshold supports to be
 * configured static or dynamic.
 *
 * For dynamic threshold, the queue threshold depends on the remain buffer.
 */
struct ppe_ac_uni_queue_cfg {
	u32 ac_en:1,
	    wred_en:1,
	    force_ac_en:1,
	    color_aware:1,
	    ac_grp_id:2,
	    prealloc_limit:11,
	    shared_dynamic:1,
	    shared_weight:3,
	    shared_ceiling:11;
	u32 gap_grn_grn_min:11,
	    gap_grn_yel_max:11,
	    gap_grn_yel_min_0:10;
	u32 gap_grn_yel_min_1:1,
	    gap_grn_red_max:11,
	    gap_grn_red_min:11,
	    red_resume_0:9;
	u32 red_resume_1:2,
	    yel_resume:11,
	    grn_resume:11,
	    res0:8;
};

union ppe_ac_uni_queue_cfg_u {
	u32 val[4];
	struct ppe_ac_uni_queue_cfg bf;
};

#define PPE_AC_MUL_QUEUE_CFG_TBL				0x84a000
#define PPE_AC_MUL_QUEUE_CFG_TBL_NUM				44
#define PPE_AC_MUL_QUEUE_CFG_TBL_INC				0x10

/* PPE multicast queue(256-299) configurations, the mutlicast queues are
 * fixed to the PPE ports, which only support static threshold.
 */
struct ppe_ac_mul_queue_cfg {
	u32 ac_en:1,
	    force_ac_en:1,
	    color_aware:1,
	    ac_grp_id:2,
	    prealloc_limit:11,
	    shared_ceiling:11,
	    gap_grn_yel_0:5;
	u32 gap_grn_yel_1:6,
	    gap_grn_red:11,
	    red_resume:11,
	    yel_resume_0:4;
	u32 yel_resume_1:7,
	    grn_resume:11,
	    res0:14;
};

union ppe_ac_mul_queue_cfg_u {
	u32 val[3];
	struct ppe_ac_mul_queue_cfg bf;
};

#define PPE_AC_GRP_CFG_TBL					0x84c000
#define PPE_AC_GRP_CFG_TBL_NUM					4
#define PPE_AC_GRP_CFG_TBL_INC					0x10

/* PPE admission control of group configurations */
struct ppe_ac_grp_cfg {
	u32 ac_en:1,
	    force_ac_en:1,
	    color_aware:1,
	    gap_grn_red:11,
	    gap_grn_yel:11,
	    dp_thrd_0:7;
	u32 dp_thrd_1:4,
	    limit:11,
	    red_resume:11,
	    yel_resume_0:6;
	u32 yel_resume_1:5,
	    grn_resume:11,
	    prealloc_limit:11,
	    res0:5;
};

union ppe_ac_grp_cfg_u {
	u32 val[3];
	struct ppe_ac_grp_cfg bf;
};

#define PPE_AC_UNI_QUEUE_CNT_TBL				0x84e000
#define PPE_AC_UNI_QUEUE_CNT_TBL_NUM				256
#define PPE_AC_UNI_QUEUE_CNT_TBL_INC				0x10
#define PPE_AC_UNI_QUEUE_CNT_TBL_PEND_CNT			GENMASK(12, 0)

#define PPE_AC_MUL_QUEUE_CNT_TBL				0x852000
#define PPE_AC_MUL_QUEUE_CNT_TBL_NUM				44
#define PPE_AC_MUL_QUEUE_CNT_TBL_INC				0x10
#define PPE_AC_MUL_QUEUE_CNT_TBL_PEND_CNT			GENMASK(12, 0)

#define PPE_ENQ_OPR_TBL						0x85c000
#define PPE_ENQ_OPR_TBL_NUM					300
#define PPE_ENQ_OPR_TBL_INC					0x10
#define PPE_ENQ_OPR_TBL_ENQ_DISABLE				BIT(0)

#endif
