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

#define PPE_BM_TDM_CFG_TBL					0xc000
#define PPE_BM_TDM_CFG_TBL_NUM					128
#define PPE_BM_TDM_CFG_TBL_INC					0x10
#define PPE_BM_TDM_CFG_TBL_PORT_NUM				GENMASK(3, 0)
#define PPE_BM_TDM_CFG_TBL_DIR					BIT(4)
#define PPE_BM_TDM_CFG_TBL_VALID				BIT(5)
#define PPE_BM_TDM_CFG_TBL_SECOND_PORT_VALID			BIT(6)
#define PPE_BM_TDM_CFG_TBL_SECOND_PORT				GENMASK(11, 8)

#define PPE_EG_BRIDGE_CONFIG					0x20044
#define PPE_EG_BRIDGE_CONFIG_QUEUE_CNT_EN			BIT(2)

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

#define PPE_ENQ_OPR_TBL						0x85c000
#define PPE_ENQ_OPR_TBL_NUM					300
#define PPE_ENQ_OPR_TBL_INC					0x10
#define PPE_ENQ_OPR_TBL_ENQ_DISABLE				BIT(0)

#endif
