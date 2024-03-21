// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* PPE debugfs routines for display of PPE counters useful for debug. */

#include <linux/debugfs.h>
#include <linux/netdevice.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>

#include "edma.h"
#include "ppe.h"
#include "ppe_config.h"
#include "ppe_debugfs.h"
#include "ppe_regs.h"

#define PPE_PKT_CNT_TBL_SIZE		3
#define PPE_DROP_PKT_CNT_TBL_SIZE	5

#define PREFIX_S(desc, cnt_type) \
	seq_printf(seq, "%-16s %16s", desc, cnt_type)
#define CNT_ONE_TYPE(cnt, str, index) \
	seq_printf(seq, "%10u(%s=%04d)", cnt, str, index)
#define CNT_TWO_TYPE(cnt, cnt1, str, index) \
	seq_printf(seq, "%10u/%u(%s=%04d)", cnt, cnt1, str, index)
#define CNT_CPU_CODE(cnt, index) \
	seq_printf(seq, "%10u(cpucode:%d)", cnt, index)
#define CNT_DROP_CODE(cnt, port, index) \
	seq_printf(seq, "%10u(port=%d),dropcode:%d", cnt, port, index)

#define PPE_W0_PKT_CNT				GENMASK(31, 0)
#define PPE_W2_DROP_PKT_CNT_LOW			GENMASK(31, 8)
#define PPE_W3_DROP_PKT_CNT_HIGH		GENMASK(7, 0)

#define PPE_GET_PKT_CNT(tbl_cfg)		\
	u32_get_bits(*((u32 *)(tbl_cfg)), PPE_W0_PKT_CNT)
#define PPE_GET_DROP_PKT_CNT_LOW(tbl_cfg)	\
	u32_get_bits(*((u32 *)(tbl_cfg) + 0x2), PPE_W2_DROP_PKT_CNT_LOW)
#define PPE_GET_DROP_PKT_CNT_HIGH(tbl_cfg)	\
	u32_get_bits(*((u32 *)(tbl_cfg) + 0x3), PPE_W3_DROP_PKT_CNT_HIGH)

/**
 * enum ppe_cnt_size_type - PPE counter size type
 * @PPE_PKT_CNT_SIZE_1WORD: Counter size with single register
 * @PPE_PKT_CNT_SIZE_3WORD: Counter size with table of 3 words
 * @PPE_PKT_CNT_SIZE_5WORD: Counter size with table of 5 words
 *
 * PPE takes the different register size to record the packet counter,
 * which uses single register or register table with 3 words or 5 words.
 * The counter with table size 5 words also records the drop counter.
 * There are also some other counters only occupying several bits less than
 * 32 bits, which is not covered by this enumeration type.
 */
enum ppe_cnt_size_type {
	PPE_PKT_CNT_SIZE_1WORD,
	PPE_PKT_CNT_SIZE_3WORD,
	PPE_PKT_CNT_SIZE_5WORD,
};

static int ppe_pkt_cnt_get(struct ppe_device *ppe_dev, u32 reg,
			   enum ppe_cnt_size_type cnt_type,
			   u32 *cnt, u32 *drop_cnt)
{
	u32 drop_pkt_cnt[PPE_DROP_PKT_CNT_TBL_SIZE];
	u32 pkt_cnt[PPE_PKT_CNT_TBL_SIZE];
	u32 value;
	int ret;

	switch (cnt_type) {
	case PPE_PKT_CNT_SIZE_1WORD:
		ret = regmap_read(ppe_dev->regmap, reg, &value);
		if (ret)
			return ret;

		*cnt = value;
		break;
	case PPE_PKT_CNT_SIZE_3WORD:
		ret = regmap_bulk_read(ppe_dev->regmap, reg,
				       pkt_cnt, ARRAY_SIZE(pkt_cnt));
		if (ret)
			return ret;

		*cnt = PPE_GET_PKT_CNT(pkt_cnt);
		break;
	case PPE_PKT_CNT_SIZE_5WORD:
		ret = regmap_bulk_read(ppe_dev->regmap, reg,
				       drop_pkt_cnt, ARRAY_SIZE(drop_pkt_cnt));
		if (ret)
			return ret;

		*cnt = PPE_GET_PKT_CNT(drop_pkt_cnt);

		/* Drop counter with low 24 bits. */
		value  = PPE_GET_DROP_PKT_CNT_LOW(drop_pkt_cnt);
		*drop_cnt = FIELD_PREP(GENMASK(23, 0), value);

		/* Drop counter with high 8 bits. */
		value  = PPE_GET_DROP_PKT_CNT_HIGH(drop_pkt_cnt);
		*drop_cnt |= FIELD_PREP(GENMASK(31, 24), value);
		break;
	}

	return 0;
}

static void ppe_tbl_pkt_cnt_clear(struct ppe_device *ppe_dev, u32 reg,
				  enum ppe_cnt_size_type cnt_type)
{
	u32 drop_pkt_cnt[PPE_DROP_PKT_CNT_TBL_SIZE] = {};
	u32 pkt_cnt[PPE_PKT_CNT_TBL_SIZE] = {};

	switch (cnt_type) {
	case PPE_PKT_CNT_SIZE_1WORD:
		regmap_write(ppe_dev->regmap, reg, 0);
		break;
	case PPE_PKT_CNT_SIZE_3WORD:
		regmap_bulk_write(ppe_dev->regmap, reg,
				  pkt_cnt, ARRAY_SIZE(pkt_cnt));
		break;
	case PPE_PKT_CNT_SIZE_5WORD:
		regmap_bulk_write(ppe_dev->regmap, reg,
				  drop_pkt_cnt, ARRAY_SIZE(drop_pkt_cnt));
		break;
	}
}

/* The number of packets dropped because of no buffer available. */
static void ppe_prx_drop_counter_get(struct ppe_device *ppe_dev,
				     struct seq_file *seq)
{
	int ret, i, tag = 0;
	u32 reg, drop_cnt;

	PREFIX_S("PRX_DROP_CNT", "SILENT_DROP:");
	for (i = 0; i < PPE_DROP_CNT_NUM; i++) {
		reg = PPE_DROP_CNT_ADDR + i * PPE_DROP_CNT_INC;
		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_1WORD,
				      &drop_cnt, NULL);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		if (drop_cnt > 0) {
			tag++;
			if (!(tag % 4)) {
				seq_putc(seq, '\n');
				PREFIX_S("", "");
			}

			CNT_ONE_TYPE(drop_cnt, "port", i);
		}
	}

	seq_putc(seq, '\n');
}

/* The number of packet dropped because of no enough buffer to cache
 * packet, some buffer allocated for the part of packet.
 */
static void ppe_prx_bm_drop_counter_get(struct ppe_device *ppe_dev,
					struct seq_file *seq)
{
	u32 reg, pkt_cnt = 0;
	int ret, i, tag = 0;

	PREFIX_S("PRX_BM_DROP_CNT", "OVERFLOW_DROP:");
	for (i = 0; i < PPE_DROP_STAT_NUM; i++) {
		reg = PPE_DROP_STAT_ADDR + PPE_DROP_STAT_INC * i;

		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD,
				      &pkt_cnt, NULL);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		if (pkt_cnt > 0) {
			tag++;
			if (!(tag % 4)) {
				seq_putc(seq, '\n');
				PREFIX_S("", "");
			}

			CNT_ONE_TYPE(pkt_cnt, "port", i);
		}
	}

	seq_putc(seq, '\n');
}

/* The number of currently occupied buffers, that can't be flushed. */
static void ppe_prx_bm_port_counter_get(struct ppe_device *ppe_dev,
					struct seq_file *seq)
{
	int used_cnt, react_cnt;
	int ret, i, tag = 0;
	u32 reg, val;

	PREFIX_S("PRX_BM_PORT_CNT", "USED/REACT:");
	for (i = 0; i < PPE_BM_USED_CNT_NUM; i++) {
		reg = PPE_BM_USED_CNT_ADDR + i * PPE_BM_USED_CNT_INC;
		ret = regmap_read(ppe_dev->regmap, reg, &val);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		used_cnt = FIELD_GET(PPE_BM_USED_CNT_VAL, val);

		reg = PPE_BM_REACT_CNT_ADDR + i * PPE_BM_REACT_CNT_INC;
		ret = regmap_read(ppe_dev->regmap, reg, &val);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		react_cnt = FIELD_GET(PPE_BM_REACT_CNT_VAL, val);

		if (used_cnt > 0 || react_cnt > 0) {
			tag++;
			if (!(tag % 4)) {
				seq_putc(seq, '\n');
				PREFIX_S("", "");
			}

			CNT_TWO_TYPE(used_cnt, react_cnt, "port", i);
		}
	}

	seq_putc(seq, '\n');
}

/* The number of ingress packets. */
static void ppe_ipx_pkt_counter_get(struct ppe_device *ppe_dev,
				    struct seq_file *seq)
{
	u32 reg, cnt, tunnel_cnt;
	int i, ret, tag = 0;

	PREFIX_S("IPR_PKT_CNT", "TPRX/IPRX:");
	for (i = 0; i < PPE_IPR_PKT_CNT_NUM; i++) {
		reg = PPE_TPR_PKT_CNT_ADDR + i * PPE_IPR_PKT_CNT_INC;
		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_1WORD,
				      &tunnel_cnt, NULL);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		reg = PPE_IPR_PKT_CNT_ADDR + i * PPE_IPR_PKT_CNT_INC;
		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_1WORD,
				      &cnt, NULL);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		if (tunnel_cnt > 0 || cnt > 0) {
			tag++;
			if (!(tag % 4)) {
				seq_putc(seq, '\n');
				PREFIX_S("", "");
			}

			CNT_TWO_TYPE(tunnel_cnt, cnt, "port", i);
		}
	}

	seq_putc(seq, '\n');
}

/* The number of packet received or dropped on the ingress direction. */
static void ppe_port_rx_counter_get(struct ppe_device *ppe_dev,
				    struct seq_file *seq)
{
	u32 reg, pkt_cnt, drop_cnt;
	int ret, i, tag = 0;

	PREFIX_S("PORT_RX_CNT", "RX/RX_DROP:");
	for (i = 0; i < PPE_PHY_PORT_RX_CNT_TBL_NUM; i++) {
		reg = PPE_PHY_PORT_RX_CNT_TBL_ADDR + PPE_PHY_PORT_RX_CNT_TBL_INC * i;
		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_5WORD,
				      &pkt_cnt, &drop_cnt);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		if (pkt_cnt > 0) {
			tag++;
			if (!(tag % 4)) {
				seq_putc(seq, '\n');
				PREFIX_S("", "");
			}

			CNT_TWO_TYPE(pkt_cnt, drop_cnt, "port", i);
		}
	}

	seq_putc(seq, '\n');
}

/* The number of packet received or dropped by the port. */
static void ppe_vp_rx_counter_get(struct ppe_device *ppe_dev,
				  struct seq_file *seq)
{
	u32 reg, pkt_cnt, drop_cnt;
	int ret, i, tag = 0;

	PREFIX_S("VPORT_RX_CNT", "RX/RX_DROP:");
	for (i = 0; i < PPE_PORT_RX_CNT_TBL_NUM; i++) {
		reg = PPE_PORT_RX_CNT_TBL_ADDR + PPE_PORT_RX_CNT_TBL_INC * i;
		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_5WORD,
				      &pkt_cnt, &drop_cnt);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		if (pkt_cnt > 0) {
			tag++;
			if (!(tag % 4)) {
				seq_putc(seq, '\n');
				PREFIX_S("", "");
			}

			CNT_TWO_TYPE(pkt_cnt, drop_cnt, "port", i);
		}
	}

	seq_putc(seq, '\n');
}

/* The number of packet received or dropped by layer 2 processing. */
static void ppe_pre_l2_counter_get(struct ppe_device *ppe_dev,
				   struct seq_file *seq)
{
	u32 reg, pkt_cnt, drop_cnt;
	int ret, i, tag = 0;

	PREFIX_S("PRE_L2_CNT", "RX/RX_DROP:");
	for (i = 0; i < PPE_PRE_L2_CNT_TBL_NUM; i++) {
		reg = PPE_PRE_L2_CNT_TBL_ADDR + PPE_PRE_L2_CNT_TBL_INC * i;
		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_5WORD,
				      &pkt_cnt, &drop_cnt);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		if (pkt_cnt > 0) {
			tag++;
			if (!(tag % 4)) {
				seq_putc(seq, '\n');
				PREFIX_S("", "");
			}

			CNT_TWO_TYPE(pkt_cnt, drop_cnt, "vsi", i);
		}
	}

	seq_putc(seq, '\n');
}

/* The number of packet received for VLAN handler. */
static void ppe_vlan_counter_get(struct ppe_device *ppe_dev,
				 struct seq_file *seq)
{
	u32 reg, pkt_cnt = 0;
	int ret, i, tag = 0;

	PREFIX_S("VLAN_CNT", "RX:");
	for (i = 0; i < PPE_VLAN_CNT_TBL_NUM; i++) {
		reg = PPE_VLAN_CNT_TBL_ADDR + PPE_VLAN_CNT_TBL_INC * i;

		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD,
				      &pkt_cnt, NULL);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		if (pkt_cnt > 0) {
			tag++;
			if (!(tag % 4)) {
				seq_putc(seq, '\n');
				PREFIX_S("", "");
			}

			CNT_ONE_TYPE(pkt_cnt, "vsi", i);
		}
	}

	seq_putc(seq, '\n');
}

/* The number of packet forwarded to CPU handler. */
static void ppe_cpu_code_counter_get(struct ppe_device *ppe_dev,
				     struct seq_file *seq)
{
	u32 reg, pkt_cnt = 0;
	int ret, i;

	PREFIX_S("CPU_CODE_CNT", "CODE:");
	for (i = 0; i < PPE_DROP_CPU_CNT_TBL_NUM; i++) {
		reg = PPE_DROP_CPU_CNT_TBL_ADDR + PPE_DROP_CPU_CNT_TBL_INC * i;

		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD,
				      &pkt_cnt, NULL);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		if (!pkt_cnt)
			continue;

		if (i < 256)
			CNT_CPU_CODE(pkt_cnt, i);
		else
			CNT_DROP_CODE(pkt_cnt, (i - 256) % 8, (i - 256) / 8);

		seq_putc(seq, '\n');
		PREFIX_S("", "");
	}

	seq_putc(seq, '\n');
}

/* The number of packet forwarded by VLAN on the egress direction. */
static void ppe_eg_vsi_counter_get(struct ppe_device *ppe_dev,
				   struct seq_file *seq)
{
	u32 reg, pkt_cnt = 0;
	int ret, i, tag = 0;

	PREFIX_S("EG_VSI_CNT", "TX:");
	for (i = 0; i < PPE_EG_VSI_COUNTER_TBL_NUM; i++) {
		reg = PPE_EG_VSI_COUNTER_TBL_ADDR + PPE_EG_VSI_COUNTER_TBL_INC * i;

		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD,
				      &pkt_cnt, NULL);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		if (pkt_cnt > 0) {
			tag++;
			if (!(tag % 4)) {
				seq_putc(seq, '\n');
				PREFIX_S("", "");
			}

			CNT_ONE_TYPE(pkt_cnt, "vsi", i);
		}
	}

	seq_putc(seq, '\n');
}

/* The number of packet trasmitted or dropped by port. */
static void ppe_vp_tx_counter_get(struct ppe_device *ppe_dev,
				  struct seq_file *seq)
{
	u32 reg, pkt_cnt = 0, drop_cnt = 0;
	int ret, i, tag = 0;

	PREFIX_S("VPORT_TX_CNT", "TX/TX_DROP:");
	for (i = 0; i < PPE_VPORT_TX_COUNTER_TBL_NUM; i++) {
		reg = PPE_VPORT_TX_COUNTER_TBL_ADDR + PPE_VPORT_TX_COUNTER_TBL_INC * i;
		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD,
				      &pkt_cnt, NULL);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		reg = PPE_VPORT_TX_DROP_CNT_TBL_ADDR + PPE_VPORT_TX_DROP_CNT_TBL_INC * i;
		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD,
				      &drop_cnt, NULL);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		if (pkt_cnt > 0 || drop_cnt > 0) {
			tag++;
			if (!(tag % 4)) {
				seq_putc(seq, '\n');
				PREFIX_S("", "");
			}

			CNT_TWO_TYPE(pkt_cnt, drop_cnt, "port", i);
		}
	}

	seq_putc(seq, '\n');
}

/* The number of packet trasmitted or dropped on the egress direction. */
static void ppe_port_tx_counter_get(struct ppe_device *ppe_dev,
				    struct seq_file *seq)
{
	u32 reg, pkt_cnt = 0, drop_cnt = 0;
	int ret, i, tag = 0;

	PREFIX_S("PORT_TX_CNT", "TX/TX_DROP:");
	for (i = 0; i < PPE_PORT_TX_COUNTER_TBL_NUM; i++) {
		reg = PPE_PORT_TX_COUNTER_TBL_ADDR + PPE_PORT_TX_COUNTER_TBL_INC * i;
		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD,
				      &pkt_cnt, NULL);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		reg = PPE_PORT_TX_DROP_CNT_TBL_ADDR + PPE_PORT_TX_DROP_CNT_TBL_INC * i;
		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD,
				      &drop_cnt, NULL);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		if (pkt_cnt > 0 || drop_cnt > 0) {
			tag++;
			if (!(tag % 4)) {
				seq_putc(seq, '\n');
				PREFIX_S("", "");
			}

			CNT_TWO_TYPE(pkt_cnt, drop_cnt, "port", i);
		}
	}

	seq_putc(seq, '\n');
}

/* The number of packet trasmitted or pended by the PPE queue. */
static void ppe_queue_tx_counter_get(struct ppe_device *ppe_dev,
				     struct seq_file *seq)
{
	u32 reg, val, pkt_cnt = 0, pend_cnt = 0;
	int ret, i, tag = 0;

	PREFIX_S("QUEUE_TX_CNT", "TX/PEND:");
	for (i = 0; i < PPE_QUEUE_TX_COUNTER_TBL_NUM; i++) {
		reg = PPE_QUEUE_TX_COUNTER_TBL_ADDR + PPE_QUEUE_TX_COUNTER_TBL_INC * i;
		ret = ppe_pkt_cnt_get(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD,
				      &pkt_cnt, NULL);
		if (ret) {
			seq_printf(seq, "ERROR %d\n", ret);
			return;
		}

		if (i < PPE_AC_UNI_QUEUE_CFG_TBL_NUM) {
			reg = PPE_AC_UNI_QUEUE_CNT_TBL_ADDR + PPE_AC_UNI_QUEUE_CNT_TBL_INC * i;
			ret = regmap_read(ppe_dev->regmap, reg, &val);
			if (ret) {
				seq_printf(seq, "ERROR %d\n", ret);
				return;
			}

			pend_cnt = FIELD_GET(PPE_AC_UNI_QUEUE_CNT_TBL_PEND_CNT, val);
		} else {
			reg = PPE_AC_MUL_QUEUE_CNT_TBL_ADDR +
			      PPE_AC_MUL_QUEUE_CNT_TBL_INC * (i - PPE_AC_UNI_QUEUE_CFG_TBL_NUM);
			ret = regmap_read(ppe_dev->regmap, reg, &val);
			if (ret) {
				seq_printf(seq, "ERROR %d\n", ret);
				return;
			}

			pend_cnt = FIELD_GET(PPE_AC_MUL_QUEUE_CNT_TBL_PEND_CNT, val);
		}

		if (pkt_cnt > 0 || pend_cnt > 0) {
			tag++;
			if (!(tag % 4)) {
				seq_putc(seq, '\n');
				PREFIX_S("", "");
			}

			CNT_TWO_TYPE(pkt_cnt, pend_cnt, "queue", i);
		}
	}

	seq_putc(seq, '\n');
}

/* Display the packet counter of PPE. */
static int ppe_packet_counter_show(struct seq_file *seq, void *v)
{
	struct ppe_device *ppe_dev = seq->private;

	ppe_prx_drop_counter_get(ppe_dev, seq);
	ppe_prx_bm_drop_counter_get(ppe_dev, seq);
	ppe_prx_bm_port_counter_get(ppe_dev, seq);
	ppe_ipx_pkt_counter_get(ppe_dev, seq);
	ppe_port_rx_counter_get(ppe_dev, seq);
	ppe_vp_rx_counter_get(ppe_dev, seq);
	ppe_pre_l2_counter_get(ppe_dev, seq);
	ppe_vlan_counter_get(ppe_dev, seq);
	ppe_cpu_code_counter_get(ppe_dev, seq);
	ppe_eg_vsi_counter_get(ppe_dev, seq);
	ppe_vp_tx_counter_get(ppe_dev, seq);
	ppe_port_tx_counter_get(ppe_dev, seq);
	ppe_queue_tx_counter_get(ppe_dev, seq);

	return 0;
}

static int ppe_packet_counter_open(struct inode *inode, struct file *file)
{
	return single_open(file, ppe_packet_counter_show, inode->i_private);
}

static ssize_t ppe_packet_counter_clear(struct file *file,
					const char __user *buf,
					size_t count, loff_t *pos)
{
	struct ppe_device *ppe_dev = file_inode(file)->i_private;
	u32 reg;
	int i;

	for (i = 0; i < PPE_DROP_CNT_NUM; i++) {
		reg = PPE_DROP_CNT_ADDR + i * PPE_DROP_CNT_INC;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_1WORD);
	}

	for (i = 0; i < PPE_DROP_STAT_NUM; i++) {
		reg = PPE_DROP_STAT_ADDR + PPE_DROP_STAT_INC * i;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD);
	}

	for (i = 0; i < PPE_IPR_PKT_CNT_NUM; i++) {
		reg = PPE_IPR_PKT_CNT_ADDR + i * PPE_IPR_PKT_CNT_INC;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_1WORD);

		reg = PPE_TPR_PKT_CNT_ADDR + i * PPE_IPR_PKT_CNT_INC;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_1WORD);
	}

	for (i = 0; i < PPE_VLAN_CNT_TBL_NUM; i++) {
		reg = PPE_VLAN_CNT_TBL_ADDR + PPE_VLAN_CNT_TBL_INC * i;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD);
	}

	for (i = 0; i < PPE_PRE_L2_CNT_TBL_NUM; i++) {
		reg = PPE_PRE_L2_CNT_TBL_ADDR + PPE_PRE_L2_CNT_TBL_INC * i;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_5WORD);
	}

	for (i = 0; i < PPE_PORT_TX_COUNTER_TBL_NUM; i++) {
		reg = PPE_PORT_TX_DROP_CNT_TBL_ADDR + PPE_PORT_TX_DROP_CNT_TBL_INC * i;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD);

		reg = PPE_PORT_TX_COUNTER_TBL_ADDR + PPE_PORT_TX_COUNTER_TBL_INC * i;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD);
	}

	for (i = 0; i < PPE_EG_VSI_COUNTER_TBL_NUM; i++) {
		reg = PPE_EG_VSI_COUNTER_TBL_ADDR + PPE_EG_VSI_COUNTER_TBL_INC * i;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD);
	}

	for (i = 0; i < PPE_VPORT_TX_COUNTER_TBL_NUM; i++) {
		reg = PPE_VPORT_TX_COUNTER_TBL_ADDR + PPE_VPORT_TX_COUNTER_TBL_INC * i;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD);

		reg = PPE_VPORT_TX_DROP_CNT_TBL_ADDR + PPE_VPORT_TX_DROP_CNT_TBL_INC * i;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD);
	}

	for (i = 0; i < PPE_QUEUE_TX_COUNTER_TBL_NUM; i++) {
		reg = PPE_QUEUE_TX_COUNTER_TBL_ADDR + PPE_QUEUE_TX_COUNTER_TBL_INC * i;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD);
	}

	ppe_tbl_pkt_cnt_clear(ppe_dev, PPE_EPE_DBG_IN_CNT_ADDR, PPE_PKT_CNT_SIZE_1WORD);
	ppe_tbl_pkt_cnt_clear(ppe_dev, PPE_EPE_DBG_OUT_CNT_ADDR, PPE_PKT_CNT_SIZE_1WORD);

	for (i = 0; i < PPE_DROP_CPU_CNT_TBL_NUM; i++) {
		reg = PPE_DROP_CPU_CNT_TBL_ADDR + PPE_DROP_CPU_CNT_TBL_INC * i;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_3WORD);
	}

	for (i = 0; i < PPE_PORT_RX_CNT_TBL_NUM; i++) {
		reg = PPE_PORT_RX_CNT_TBL_ADDR + PPE_PORT_RX_CNT_TBL_INC * i;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_5WORD);
	}

	for (i = 0; i < PPE_PHY_PORT_RX_CNT_TBL_NUM; i++) {
		reg = PPE_PHY_PORT_RX_CNT_TBL_ADDR + PPE_PHY_PORT_RX_CNT_TBL_INC * i;
		ppe_tbl_pkt_cnt_clear(ppe_dev, reg, PPE_PKT_CNT_SIZE_5WORD);
	}

	return count;
}

static const struct file_operations ppe_debugfs_packet_counter_fops = {
	.owner   = THIS_MODULE,
	.open    = ppe_packet_counter_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
	.write   = ppe_packet_counter_clear,
};

void ppe_debugfs_setup(struct ppe_device *ppe_dev)
{
	int ret;

	ppe_dev->debugfs_root = debugfs_create_dir("ppe", NULL);
	debugfs_create_file("packet_counter", 0444,
			    ppe_dev->debugfs_root,
			    ppe_dev,
			    &ppe_debugfs_packet_counter_fops);

	if (!ppe_dev->debugfs_root) {
		dev_err(ppe_dev->dev, "Error in PPE debugfs setup\n");
		return;
	}

	ret = edma_debugfs_setup(ppe_dev);
	if (ret) {
		dev_err(ppe_dev->dev, "Error in EDMA debugfs setup API. ret: %d\n", ret);
		debugfs_remove_recursive(ppe_dev->debugfs_root);
		ppe_dev->debugfs_root = NULL;
	}
}

void ppe_debugfs_teardown(struct ppe_device *ppe_dev)
{
	edma_debugfs_teardown();
	debugfs_remove_recursive(ppe_dev->debugfs_root);
	ppe_dev->debugfs_root = NULL;
}
