/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* PPE hardware register and table declarations. */
#ifndef __PPE_REGS_H__
#define __PPE_REGS_H__

#define PPE_PORT_MUX_CTRL					0x10
#define PPE_PORT6_MAC_SEL					BIT(13)
#define PPE_PORT5_MAC_SEL					BIT(12)
#define PPE_PORT4_MAC_SEL					BIT(11)
#define PPE_PORT3_MAC_SEL					BIT(10)
#define PPE_PORT2_MAC_SEL					BIT(9)
#define PPE_PORT1_MAC_SEL					BIT(8)
#define PPE_PORT5_PCS_SEL					BIT(4)
#define PPE_PORT_MAC_SEL(x)					(PPE_PORT1_MAC_SEL << ((x) - 1))

#define PPE_LPI_LPI_EN						0x400
#define PPE_LPI_PORT1_EN					BIT(0)
#define PPE_LPI_PORT2_EN					BIT(1)
#define PPE_LPI_PORT3_EN					BIT(2)
#define PPE_LPI_PORT4_EN					BIT(3)
#define PPE_LPI_PORT5_EN					BIT(4)
#define PPE_LPI_PORT6_EN					BIT(5)
#define PPE_LPI_PORT_EN(x)					(PPE_LPI_PORT1_EN << ((x) - 1))

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

/* PPE MAC Address */
#define PPE_PORT_GMAC_ADDR(x)					(0x001000 + ((x) - 1) * 0x200)
#define PPE_PORT_XGMAC_ADDR(x)					(0x500000 + ((x) - 1) * 0x4000)

/* GMAC Registers */
#define GMAC_ENABLE						0x0
#define GMAC_TX_FLOW_EN						BIT(6)
#define GMAC_RX_FLOW_EN						BIT(5)
#define GMAC_DUPLEX_FULL					BIT(4)
#define GMAC_TXMAC_EN						BIT(1)
#define GMAC_RXMAC_EN						BIT(0)
#define GMAC_MAC_EN						(GMAC_RXMAC_EN | GMAC_TXMAC_EN)

#define GMAC_SPEED						0x4
#define GMAC_SPEED_MASK						GENMASK(1, 0)
#define GMAC_SPEED_10						0
#define GMAC_SPEED_100						1
#define GMAC_SPEED_1000						2

#define GMAC_GOL_MAC_ADDR0					0x8
#define MAC_ADDR_BYTE5						GENMASK(15, 8)
#define MAC_ADDR_BYTE4						GENMASK(7, 0)

#define GMAC_GOL_MAC_ADDR1					0xC
#define MAC_ADDR_BYTE0						GENMASK(31, 24)
#define MAC_ADDR_BYTE1						GENMASK(23, 16)
#define MAC_ADDR_BYTE2						GENMASK(15, 8)
#define MAC_ADDR_BYTE3						GENMASK(7, 0)

#define GMAC_MAC_CTRL2						0x18
#define GMAC_TX_THD_MASK					GENMASK(27, 24)
#define GMAC_MAXFR_MASK						GENMASK(21, 8)
#define GMAC_CRS_SEL						BIT(6)
#define GMAC_TX_THD						0x1
#define GMAC_INIT_CTRL2_FIELD					(GMAC_MAXFR_MASK | \
								GMAC_CRS_SEL | GMAC_TX_THD_MASK)
#define GMAC_INIT_CTRL2						(FIELD_PREP(GMAC_MAXFR_MASK, \
				MAC_MAX_FRAME_SIZE) | FIELD_PREP(GMAC_TX_THD_MASK, GMAC_TX_THD))

#define GMAC_MAC_DBG_CTRL					0x1c
#define GMAC_HIGH_IPG_MASK					GENMASK(15, 8)
#define GMAC_IPG_CHECK						0xc

#define GMAC_MAC_JUMBO_SIZE					0x30
#define GMAC_JUMBO_SIZE_MASK					GENMASK(13, 0)
#define MAC_MAX_FRAME_SIZE					0x3000

#define GMAC_MAC_MIB_CTRL					0x34
#define MAC_MIB_RD_CLR						BIT(2)
#define MAC_MIB_RESET						BIT(1)
#define MAC_MIB_EN						BIT(0)

#define GMAC_RXBROAD						0x40
#define GMAC_RXPAUSE						0x44
#define GMAC_RXMULTI						0x48
#define GMAC_RXFCSERR						0x4C
#define GMAC_RXALIGNERR						0x50
#define GMAC_RXRUNT						0x54
#define GMAC_RXFRAG						0x58
#define GMAC_RXJUMBOFCSERR					0x5C
#define GMAC_RXJUMBOALIGNERR					0x60
#define GMAC_RXPKT64						0x64
#define GMAC_RXPKT65TO127					0x68
#define GMAC_RXPKT128TO255					0x6C
#define GMAC_RXPKT256TO511					0x70
#define GMAC_RXPKT512TO1023					0x74
#define GMAC_RXPKT1024TO1518					0x78
#define GMAC_RXPKT1519TOX					0x7C
#define GMAC_RXTOOLONG						0x80
#define GMAC_RXGOODBYTE_L					0x84
#define GMAC_RXGOODBYTE_H					0x88
#define GMAC_RXBADBYTE_L					0x8C
#define GMAC_RXBADBYTE_H					0x90
#define GMAC_RXUNI						0x94
#define GMAC_TXBROAD						0xA0
#define GMAC_TXPAUSE						0xA4
#define GMAC_TXMULTI						0xA8
#define GMAC_TXUNDERRUN						0xAC
#define GMAC_TXPKT64						0xB0
#define GMAC_TXPKT65TO127					0xB4
#define GMAC_TXPKT128TO255					0xB8
#define GMAC_TXPKT256TO511					0xBC
#define GMAC_TXPKT512TO1023					0xC0
#define GMAC_TXPKT1024TO1518					0xC4
#define GMAC_TXPKT1519TOX					0xC8
#define GMAC_TXBYTE_L						0xCC
#define GMAC_TXBYTE_H						0xD0
#define GMAC_TXCOLLISIONS					0xD4
#define GMAC_TXABORTCOL						0xD8
#define GMAC_TXMULTICOL						0xDC
#define GMAC_TXSINGLECOL					0xE0
#define GMAC_TXEXCESSIVEDEFER					0xE4
#define GMAC_TXDEFER						0xE8
#define GMAC_TXLATECOL						0xEC
#define GMAC_TXUNI						0xF0

/* XGMAC Registers */
#define XGMAC_TX_CONFIGURATION					0x0
#define XGMAC_SPEED_MASK					GENMASK(31, 29)
#define XGMAC_SPEED_10000_USXGMII				FIELD_PREP(XGMAC_SPEED_MASK, 4)
#define XGMAC_SPEED_10000					FIELD_PREP(XGMAC_SPEED_MASK, 0)
#define XGMAC_SPEED_5000					FIELD_PREP(XGMAC_SPEED_MASK, 5)
#define XGMAC_SPEED_2500_USXGMII				FIELD_PREP(XGMAC_SPEED_MASK, 6)
#define XGMAC_SPEED_2500					FIELD_PREP(XGMAC_SPEED_MASK, 2)
#define XGMAC_SPEED_1000					FIELD_PREP(XGMAC_SPEED_MASK, 3)
#define XGMAC_SPEED_100						XGMAC_SPEED_1000
#define XGMAC_SPEED_10						XGMAC_SPEED_1000

#define XGMAC_JD						BIT(16)
#define XGMAC_TE						BIT(0)
#define XGMAC_INIT_TX_CONFIG_FIELD				(XGMAC_JD | XGMAC_TE)
#define XGMAC_INIT_TX_CONFIG					XGMAC_JD

#define XGMAC_RX_CONFIGURATION					0x4
#define XGMAC_GPSL_MASK						GENMASK(29, 16)
#define XGMAC_WD						BIT(7)
#define XGMAC_GPSLCE						BIT(6)
#define XGMAC_CST						BIT(2)
#define XGMAC_ACS						BIT(1)
#define XGMAC_RE						BIT(0)
#define XGMAC_INIT_RX_CONFIG_FIELD				(XGMAC_RE | XGMAC_ACS | \
					XGMAC_CST | XGMAC_WD | XGMAC_GPSLCE | XGMAC_GPSL_MASK)
#define XGMAC_INIT_RX_CONFIG					(XGMAC_ACS | XGMAC_CST | \
				XGMAC_GPSLCE | FIELD_PREP(XGMAC_GPSL_MASK, MAC_MAX_FRAME_SIZE))

#define XGMAC_PACKET_FILTER					0x8
#define XGMAC_RA						BIT(31)
#define XGMAC_PCF_MASK						GENMASK(7, 6)
#define XGMAC_PR						BIT(0)
#define XGMAC_PASS_CONTROL_PACKET				0x2
#define XGMAC_INIT_FILTER_FIELD					(XGMAC_RA | XGMAC_PR | \
									XGMAC_PCF_MASK)
#define XGMAC_INIT_FILTER					(XGMAC_RA | XGMAC_PR | \
								FIELD_PREP(XGMAC_PCF_MASK, \
									XGMAC_PASS_CONTROL_PACKET))

#define XGMAC_WATCHDOG_TIMEOUT					0xc
#define XGMAC_PWE						BIT(8)
#define XGMAC_WTO_MASK						GENMASK(3, 0)
#define XGMAC_WTO_LIMIT_13K					0xb
#define XGMAC_INIT_WATCHDOG_FIELD				(XGMAC_PWE | XGMAC_WTO_MASK)
#define XGMAC_INIT_WATCHDOG					(XGMAC_PWE | \
						FIELD_PREP(XGMAC_WTO_MASK, XGMAC_WTO_LIMIT_13K))

#define XGMAC_Q0_TX_FLOW_CTRL					0x70
#define XGMAC_PT_MASK						GENMASK(31, 16)
#define XGMAC_PAUSE_TIME					FIELD_PREP(XGMAC_PT_MASK, 0xffff)
#define XGMAC_TFE						BIT(1)

#define XGMAC_RX_FLOW_CTRL					0x90
#define XGMAC_RFE						BIT(0)

#define XGMAC_LPI_CONTROL_STATUS				0xd0
#define XGMAC_LPI_TXEN						BIT(16)
#define XGMAC_LPI_PLS						BIT(17)
#define XGMAC_LPI_TXA						BIT(19)
#define XGMAC_LPI_TE						BIT(20)

#define XGMAC_LPI_TIMERS_CONTROL				0xd4
#define XGMAC_LPI_TWT						GENMASK(15, 0)
#define XGMAC_LPI_LST						GENMASK(25, 16)

#define XGMAC_LPI_AUTO_ENTRY_TIMER				0xd8
#define XGMAC_LPI_ET						GENMASK(19, 3)

#define XGMAC_1US_TIC_COUNTER					0xdc
#define XGMAC_1US_TIC_CNTR					GENMASK(11, 0)

#define XGMAC_MAC_ADDR0_HIGH					0x300
#define XGMAC_ADDR_EN						BIT(31)
#define XGMAC_ADDRHI						GENMASK(15, 0)

#define XGMAC_MAC_ADDR0_LOW					0x304
#define XGMAC_ADDRLO						GENMASK(31, 0)

#define XGMAC_MMC_CONTROL					0x800
#define XGMAC_MCF						BIT(3)
#define XGMAC_CNTRST						BIT(0)

#define XGMAC_TX_OCTET_COUNT_GOOD_BAD_LOW			0x814
#define XGMAC_TX_OCTET_COUNT_GOOD_BAD_HIGH			0x818
#define XGMAC_TX_FRAME_COUNT_GOOD_BAD_LOW			0x81C
#define XGMAC_TX_FRAME_COUNT_GOOD_BAD_HIGH			0x820
#define XGMAC_TX_BROADCAST_FRAMES_GOOD_LOW			0x824
#define XGMAC_TX_BROADCAST_FRAMES_GOOD_HIGH			0x828
#define XGMAC_TX_MULTICAST_FRAMES_GOOD_LOW			0x82C
#define XGMAC_TX_MULTICAST_FRAMES_GOOD_HIGH			0x830
#define XGMAC_TX_64OCTETS_FRAMES_GOOD_BAD_LOW			0x834
#define XGMAC_TX_64OCTETS_FRAMES_GOOD_BAD_HIGH			0x838
#define XGMAC_TX_65TO127OCTETS_FRAMES_GOOD_BAD_LOW		0x83C
#define XGMAC_TX_65TO127OCTETS_FRAMES_GOOD_BAD_HIGH		0x840
#define XGMAC_TX_128TO255OCTETS_FRAMES_GOOD_BAD_LOW		0x844
#define XGMAC_TX_128TO255OCTETS_FRAMES_GOOD_BAD_HIGH		0x848
#define XGMAC_TX_256TO511OCTETS_FRAMES_GOOD_BAD_LOW		0x84C
#define XGMAC_TX_256TO511OCTETS_FRAMES_GOOD_BAD_HIGH		0x850
#define XGMAC_TX_512TO1023OCTETS_FRAMES_GOOD_BAD_LOW		0x854
#define XGMAC_TX_512TO1023OCTETS_FRAMES_GOOD_BAD_HIGH		0x858
#define XGMAC_TX_1024TOMAXOCTETS_FRAMES_GOOD_BAD_LOW		0x85C
#define XGMAC_TX_1024TOMAXOCTETS_FRAMES_GOOD_BAD_HIGH		0x860
#define XGMAC_TX_UNICAST_FRAMES_GOOD_BAD_LOW			0x864
#define XGMAC_TX_UNICAST_FRAMES_GOOD_BAD_HIGH			0x868
#define XGMAC_TX_MULTICAST_FRAMES_GOOD_BAD_LOW			0x86C
#define XGMAC_TX_MULTICAST_FRAMES_GOOD_BAD_HIGH			0x870
#define XGMAC_TX_BROADCAST_FRAMES_GOOD_BAD_LOW			0x874
#define XGMAC_TX_BROADCAST_FRAMES_GOOD_BAD_HIGH			0x878
#define XGMAC_TX_UNDERFLOW_ERROR_FRAMES_LOW			0x87C
#define XGMAC_TX_UNDERFLOW_ERROR_FRAMES_HIGH			0x880
#define XGMAC_TX_OCTET_COUNT_GOOD_LOW				0x884
#define XGMAC_TX_OCTET_COUNT_GOOD_HIGH				0x888
#define XGMAC_TX_FRAME_COUNT_GOOD_LOW				0x88C
#define XGMAC_TX_FRAME_COUNT_GOOD_HIGH				0x890
#define XGMAC_TX_PAUSE_FRAMES_LOW				0x894
#define XGMAC_TX_PAUSE_FRAMES_HIGH				0x898
#define XGMAC_TX_VLAN_FRAMES_GOOD_LOW				0x89C
#define XGMAC_TX_VLAN_FRAMES_GOOD_HIGH				0x8A0
#define XGMAC_TX_LPI_USEC_CNTR					0x8A4
#define XGMAC_TX_LPI_TRAN_CNTR					0x8A8
#define XGMAC_RX_FRAME_COUNT_GOOD_BAD_LOW			0x900
#define XGMAC_RX_FRAME_COUNT_GOOD_BAD_HIGH			0x904
#define XGMAC_RX_OCTET_COUNT_GOOD_BAD_LOW			0x908
#define XGMAC_RX_OCTET_COUNT_GOOD_BAD_HIGH			0x90C
#define XGMAC_RX_OCTET_COUNT_GOOD_LOW				0x910
#define XGMAC_RX_OCTET_COUNT_GOOD_HIGH				0x914
#define XGMAC_RX_BROADCAST_FRAMES_GOOD_LOW			0x918
#define XGMAC_RX_BROADCAST_FRAMES_GOOD_HIGH			0x91C
#define XGMAC_RX_MULTICAST_FRAMES_GOOD_LOW			0x920
#define XGMAC_RX_MULTICAST_FRAMES_GOOD_HIGH			0x924
#define XGMAC_RX_CRC_ERROR_FRAMES_LOW				0x928
#define XGMAC_RX_CRC_ERROR_FRAMES_HIGH				0x92C
#define XGMAC_RX_FRAG_ERROR_FRAMES				0x930
#define XGMAC_RX_JABBER_ERROR_FRAMES				0x934
#define XGMAC_RX_UNDERSIZE_FRAMES_GOOD				0x938
#define XGMAC_RX_OVERSIZE_FRAMES_GOOD				0x93C
#define XGMAC_RX_64OCTETS_FRAMES_GOOD_BAD_LOW			0x940
#define XGMAC_RX_64OCTETS_FRAMES_GOOD_BAD_HIGH			0x944
#define XGMAC_RX_65TO127OCTETS_FRAMES_GOOD_BAD_LOW		0x948
#define XGMAC_RX_65TO127OCTETS_FRAMES_GOOD_BAD_HIGH		0x94C
#define XGMAC_RX_128TO255OCTETS_FRAMES_GOOD_BAD_LOW		0x950
#define XGMAC_RX_128TO255OCTETS_FRAMES_GOOD_BAD_HIGH		0x954
#define XGMAC_RX_256TO511OCTETS_FRAMES_GOOD_BAD_LOW		0x958
#define XGMAC_RX_256TO511OCTETS_FRAMES_GOOD_BAD_HIGH		0x95C
#define XGMAC_RX_512TO1023OCTETS_FRAMES_GOOD_BAD_LOW		0x960
#define XGMAC_RX_512TO1023OCTETS_FRAMES_GOOD_BAD_HIGH		0x964
#define XGMAC_RX_1024TOMAXOCTETS_FRAMES_GOOD_BAD_LOW		0x968
#define XGMAC_RX_1024TOMAXOCTETS_FRAMES_GOOD_BAD_HIGH		0x96C
#define XGMAC_RX_UNICAST_FRAMES_GOOD_LOW			0x970
#define XGMAC_RX_UNICAST_FRAMES_GOOD_HIGH			0x974
#define XGMAC_RX_LENGTH_ERROR_FRAMES_LOW			0x978
#define XGMAC_RX_LENGTH_ERROR_FRAMES_HIGH			0x97C
#define XGMAC_RX_OUTOFRANGE_FRAMES_LOW				0x980
#define XGMAC_RX_OUTOFRANGE_FRAMES_HIGH				0x984
#define XGMAC_RX_PAUSE_FRAMES_LOW				0x988
#define XGMAC_RX_PAUSE_FRAMES_HIGH				0x98C
#define XGMAC_RX_FIFOOVERFLOW_FRAMES_LOW			0x990
#define XGMAC_RX_FIFOOVERFLOW_FRAMES_HIGH			0x994
#define XGMAC_RX_VLAN_FRAMES_GOOD_BAD_LOW			0x998
#define XGMAC_RX_VLAN_FRAMES_GOOD_BAD_HIGH			0x99C
#define XGMAC_RX_WATCHDOG_ERROR_FRAMES				0x9A0
#define XGMAC_RX_LPI_USEC_CNTR					0x9A4
#define XGMAC_RX_LPI_TRAN_CNTR					0x9A8
#define XGMAC_RX_DISCARD_FRAME_COUNT_GOOD_BAD_LOW		0x9AC
#define XGMAC_RX_DISCARD_FRAME_COUNT_GOOD_BAD_HIGH		0x9B0
#define XGMAC_RX_DISCARD_OCTET_COUNT_GOOD_BAD_LOW		0x9B4
#define XGMAC_RX_DISCARD_OCTET_COUNT_GOOD_BAD_HIGH		0x9B8

#endif
