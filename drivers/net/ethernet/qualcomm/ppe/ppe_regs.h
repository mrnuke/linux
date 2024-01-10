/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* PPE hardware register and table declarations. */
#ifndef __PPE_REGS_H__
#define __PPE_REGS_H__

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

#endif
