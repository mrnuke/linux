// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* PPE platform device probe, DTSI read and basic HW initialization functions
 * such as BM, QM, TDM and scheduler configs.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/soc/qcom/ppe.h>
#include "ppe.h"
#include "ppe_regs.h"

#define PPE_SCHEDULER_PORT_NUM		8
static const char * const ppe_clock_name[PPE_CLK_MAX] = {
	"cmn_ahb",
	"cmn_sys",
	"uniphy0_sys",
	"uniphy1_sys",
	"uniphy2_sys",
	"uniphy0_ahb",
	"uniphy1_ahb",
	"uniphy2_ahb",
	"gcc_nsscc",
	"gcc_nssnoc_nsscc",
	"gcc_nssnoc_snoc",
	"gcc_nssnoc_snoc_1",
	"gcc_im_sleep",
	"nss_ppe",
	"nss_ppe_cfg",
	"nssnoc_ppe",
	"nssnoc_ppe_cfg",
	"nss_edma",
	"nss_edma_cfg",
	"nss_ppe_ipe",
	"nss_ppe_btq",
	"port1_mac",
	"port2_mac",
	"port3_mac",
	"port4_mac",
	"port5_mac",
	"port6_mac",
	"nss_port1_rx",
	"nss_port1_tx",
	"nss_port2_rx",
	"nss_port2_tx",
	"nss_port3_rx",
	"nss_port3_tx",
	"nss_port4_rx",
	"nss_port4_tx",
	"nss_port5_rx",
	"nss_port5_tx",
	"nss_port6_rx",
	"nss_port6_tx",
	"uniphy_port1_rx",
	"uniphy_port1_tx",
	"uniphy_port2_rx",
	"uniphy_port2_tx",
	"uniphy_port3_rx",
	"uniphy_port3_tx",
	"uniphy_port4_rx",
	"uniphy_port4_tx",
	"uniphy_port5_rx",
	"uniphy_port5_tx",
	"uniphy_port6_rx",
	"uniphy_port6_tx",
	"nss_port5_rx_clk_src",
	"nss_port5_tx_clk_src",
};

static const char * const ppe_reset_name[PPE_RST_MAX] = {
	"ppe",
	"uniphy0_sys",
	"uniphy1_sys",
	"uniphy2_sys",
	"uniphy0_ahb",
	"uniphy1_ahb",
	"uniphy2_ahb",
	"uniphy0_xpcs",
	"uniphy1_xpcs",
	"uniphy2_xpcs",
	"uniphy0_soft",
	"uniphy1_soft",
	"uniphy2_soft",
	"uniphy_port1_dis",
	"uniphy_port2_dis",
	"uniphy_port3_dis",
	"uniphy_port4_dis",
	"uniphy_port1_rx",
	"uniphy_port1_tx",
	"uniphy_port2_rx",
	"uniphy_port2_tx",
	"nss_port1_rx",
	"nss_port1_tx",
	"nss_port2_rx",
	"nss_port2_tx",
	"nss_port1",
	"nss_port2",
	"nss_port3",
	"nss_port4",
	"nss_port5",
	"nss_port6",
	"nss_port1_mac",
	"nss_port2_mac",
	"nss_port3_mac",
	"nss_port4_mac",
	"nss_port5_mac",
	"nss_port6_mac",
};

static struct ppe_scheduler_port_resource ppe_scheduler_res[PPE_SCHEDULER_PORT_NUM];

int ppe_write(struct ppe_device *ppe_dev, u32 reg, unsigned int val)
{
	return regmap_write(ppe_dev->regmap, reg, val);
}

int ppe_read(struct ppe_device *ppe_dev, u32 reg, unsigned int *val)
{
	return regmap_read(ppe_dev->regmap, reg, val);
}

int ppe_mask(struct ppe_device *ppe_dev, u32 reg, u32 mask, unsigned int set)
{
	return regmap_update_bits(ppe_dev->regmap, reg, mask, set);
}

int ppe_write_tbl(struct ppe_device *ppe_dev, u32 reg,
		  const unsigned int *val, int cnt)
{
	int i, ret;

	for (i = 0; i < cnt / 4; i++) {
		ret = ppe_write(ppe_dev, reg + i * 4, val[i]);
		if (ret)
			return ret;
	}

	return ret;
}

int ppe_read_tbl(struct ppe_device *ppe_dev, u32 reg,
		 unsigned int *val, int cnt)
{
	int i, ret;

	for (i = 0; i < cnt / 4; i++) {
		ret = ppe_read(ppe_dev, reg + i * 4, &val[i]);
		if (ret)
			return ret;
	}

	return ret;
}

int ppe_type_get(struct ppe_device *ppe_dev)
{
	struct ppe_data *ppe_dev_priv = ppe_dev->ppe_priv;

	if (!ppe_dev_priv)
		return PPE_TYPE_MAX;

	return ppe_dev_priv->ppe_type;
}

static int ppe_clock_set_enable(struct ppe_device *ppe_dev,
				enum ppe_clk_id clk_id, unsigned long rate)
{
	struct ppe_data *ppe_dev_priv = ppe_dev->ppe_priv;

	if (clk_id >= PPE_CLK_MAX)
		return -EINVAL;

	if (rate != 0)
		clk_set_rate(ppe_dev_priv->clk[clk_id], rate);

	return clk_prepare_enable(ppe_dev_priv->clk[clk_id]);
}

static int ppe_fix_clock_init(struct ppe_device *ppe_dev)
{
	unsigned long noc_rate, ppe_rate;
	enum ppe_clk_id clk_id;
	int ppe_type = ppe_type_get(ppe_dev);

	switch (ppe_type) {
	case PPE_TYPE_APPE:
		noc_rate = 342857143;
		ppe_rate = 353000000;
		break;
	case PPE_TYPE_MPPE:
		noc_rate = 266660000;
		ppe_rate = 200000000;
		ppe_clock_set_enable(ppe_dev, PPE_IM_SLEEP_CLK, 0);
		break;
	default:
		return -EINVAL;
	}

	ppe_clock_set_enable(ppe_dev, PPE_CMN_AHB_CLK, 0);
	ppe_clock_set_enable(ppe_dev, PPE_CMN_SYS_CLK, 0);
	ppe_clock_set_enable(ppe_dev, PPE_NSSCC_CLK, 100000000);
	ppe_clock_set_enable(ppe_dev, PPE_NSSNOC_NSSCC_CLK, 100000000);

	ppe_clock_set_enable(ppe_dev, PPE_NSSNOC_SNOC_CLK, noc_rate);
	ppe_clock_set_enable(ppe_dev, PPE_NSSNOC_SNOC_1_CLK, noc_rate);

	ppe_clock_set_enable(ppe_dev, PPE_UNIPHY0_SYS_CLK, 24000000);
	ppe_clock_set_enable(ppe_dev, PPE_UNIPHY1_SYS_CLK, 24000000);
	ppe_clock_set_enable(ppe_dev, PPE_UNIPHY0_AHB_CLK, 100000000);
	ppe_clock_set_enable(ppe_dev, PPE_UNIPHY1_AHB_CLK, 100000000);

	if (ppe_type == PPE_TYPE_APPE) {
		ppe_clock_set_enable(ppe_dev, PPE_UNIPHY2_SYS_CLK, 24000000);
		ppe_clock_set_enable(ppe_dev, PPE_UNIPHY2_AHB_CLK, 100000000);
	}

	ppe_clock_set_enable(ppe_dev, PPE_PORT1_MAC_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_PORT2_MAC_CLK, ppe_rate);

	if (ppe_type == PPE_TYPE_APPE) {
		ppe_clock_set_enable(ppe_dev, PPE_PORT3_MAC_CLK, ppe_rate);
		ppe_clock_set_enable(ppe_dev, PPE_PORT4_MAC_CLK, ppe_rate);
		ppe_clock_set_enable(ppe_dev, PPE_PORT5_MAC_CLK, ppe_rate);
		ppe_clock_set_enable(ppe_dev, PPE_PORT6_MAC_CLK, ppe_rate);
	}

	ppe_clock_set_enable(ppe_dev, PPE_PPE_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_PPE_CFG_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_NSSNOC_PPE_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_NSSNOC_PPE_CFG_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_EDMA_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_EDMA_CFG_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_PPE_IPE_CLK, ppe_rate);
	ppe_clock_set_enable(ppe_dev, PPE_PPE_BTQ_CLK, ppe_rate);

	/* Enable uniphy port clocks */
	for (clk_id = PPE_NSS_PORT1_RX_CLK; clk_id <= PPE_UNIPHY_PORT6_TX_CLK; clk_id++)
		ppe_clock_set_enable(ppe_dev, clk_id, 0);

	return 0;
}

static int ppe_clock_config(struct platform_device *pdev)
{
	struct ppe_device *ppe_dev = platform_get_drvdata(pdev);
	struct ppe_data *ppe_dev_priv = ppe_dev->ppe_priv;
	int ret;

	ret = ppe_fix_clock_init(ppe_dev);
	if (ret)
		return ret;

	/* Reset PPE */
	reset_control_assert(ppe_dev_priv->rst[PPE_RST_PPE_RST]);
	fsleep(100000);
	reset_control_deassert(ppe_dev_priv->rst[PPE_RST_PPE_RST]);
	fsleep(100000);

	/* Reset the ahb uniphy connected with the PHY chip */
	if (ppe_type_get(ppe_dev) == PPE_TYPE_MPPE) {
		reset_control_assert(ppe_dev_priv->rst[PPE_UNIPHY1_AHB_RST]);
		fsleep(100000);
		reset_control_deassert(ppe_dev_priv->rst[PPE_UNIPHY1_AHB_RST]);
		fsleep(100000);
	}

	return 0;
}

bool ppe_is_probed(struct platform_device *pdev)
{
	struct ppe_device *ppe_dev = platform_get_drvdata(pdev);

	return ppe_dev && ppe_dev->is_ppe_probed;
}
EXPORT_SYMBOL_GPL(ppe_is_probed);

struct ppe_device *ppe_dev_get(struct platform_device *pdev)
{
	return platform_get_drvdata(pdev);
}
EXPORT_SYMBOL_GPL(ppe_dev_get);

static const struct regmap_range ppe_readable_ranges[] = {
	regmap_reg_range(0x0, 0x1FF), /* GLB */
	regmap_reg_range(0x400, 0x5FF), /* LPI CSR */
	regmap_reg_range(0x1000, 0x11FF), /* GMAC0 */
	regmap_reg_range(0x1200, 0x13FF), /* GMAC1 */
	regmap_reg_range(0x1400, 0x15FF), /* GMAC2 */
	regmap_reg_range(0x1600, 0x17FF), /* GMAC3 */
	regmap_reg_range(0x1800, 0x19FF), /* GMAC4 */
	regmap_reg_range(0x1A00, 0x1BFF), /* GMAC5 */
	regmap_reg_range(0xB000, 0xEFFF), /* PRX CSR */
	regmap_reg_range(0xF000, 0x1EFFF), /* IPE IV */
	regmap_reg_range(0x20000, 0x5FFFF), /* PTX CSR */
	regmap_reg_range(0x60000, 0x9FFFF), /* IPE L2 CSR */
	regmap_reg_range(0xB0000, 0xEFFFF), /* IPO CSR */
	regmap_reg_range(0x100000, 0x17FFFF), /* IPE PC */
	regmap_reg_range(0x180000, 0x1BFFFF), /* PRE IPO CSR */
	regmap_reg_range(0x1D0000, 0x1DFFFF), /* TUNNEL PARSER CSR */
	regmap_reg_range(0x1E0000, 0x1EFFFF), /* INGRESS PARSE CSR */
	regmap_reg_range(0x200000, 0x2FFFFF), /* IPE L3 */
	regmap_reg_range(0x300000, 0x3FFFFF), /* IPE TL */
	regmap_reg_range(0x400000, 0x4FFFFF), /* TM */
	regmap_reg_range(0x500000, 0x503FFF), /* XGMAC0 */
	regmap_reg_range(0x504000, 0x507FFF), /* XGMAC1 */
	regmap_reg_range(0x508000, 0x50BFFF), /* XGMAC2 */
	regmap_reg_range(0x50C000, 0x50FFFF), /* XGMAC3 */
	regmap_reg_range(0x510000, 0x513FFF), /* XGMAC4 */
	regmap_reg_range(0x514000, 0x517FFF), /* XGMAC5 */
	regmap_reg_range(0x600000, 0x6FFFFF), /* BM */
	regmap_reg_range(0x800000, 0x9FFFFF), /* QM */
};

static const struct regmap_access_table ppe_reg_table = {
	.yes_ranges = ppe_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ppe_readable_ranges),
};

static const struct regmap_config ppe_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.rd_table = &ppe_reg_table,
	.wr_table = &ppe_reg_table,
	.max_register = 0x9FFFFF,
	.fast_io = true,
};

static struct ppe_data *ppe_data_init(struct platform_device *pdev)
{
	struct ppe_data *ppe_dev_priv;
	int ret;

	ppe_dev_priv = devm_kzalloc(&pdev->dev, sizeof(*ppe_dev_priv), GFP_KERNEL);
	if (!ppe_dev_priv)
		return ERR_PTR(-ENOMEM);

	if (of_device_is_compatible(pdev->dev.of_node, "qcom,ipq9574-ppe"))
		ppe_dev_priv->ppe_type = PPE_TYPE_APPE;
	else if (of_device_is_compatible(pdev->dev.of_node, "qcom,ipq5332-ppe"))
		ppe_dev_priv->ppe_type = PPE_TYPE_MPPE;
	else
		return ERR_PTR(-EINVAL);

	for (ret = 0; ret < PPE_CLK_MAX; ret++) {
		ppe_dev_priv->clk[ret] = devm_clk_get_optional(&pdev->dev,
							       ppe_clock_name[ret]);

		if (IS_ERR(ppe_dev_priv->clk[ret]))
			dev_err(&pdev->dev, "Failed to get the clock: %s\n",
				ppe_clock_name[ret]);
	}

	for (ret = 0; ret < PPE_RST_MAX; ret++) {
		ppe_dev_priv->rst[ret] =
			devm_reset_control_get_optional_exclusive(&pdev->dev,
								  ppe_reset_name[ret]);
		if (IS_ERR(ppe_dev_priv->rst[ret]))
			dev_err(&pdev->dev, "Failed to get the reset %s!\n",
				ppe_reset_name[ret]);
	}

	return ppe_dev_priv;
}

static int of_parse_ppe_bm(struct ppe_device *ppe_dev,
			   struct device_node *ppe_node)
{
	union ppe_bm_port_fc_cfg_u fc_cfg;
	struct device_node *bm_node;
	int ret, cnt;
	u32 *cfg, reg_val;

	bm_node = of_get_child_by_name(ppe_node, "buffer-management-config");
	if (!bm_node)
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "Fail to get buffer-management-config\n");

	cnt = of_property_count_u32_elems(bm_node, "qcom,group-config");
	if (cnt < 0)
		return dev_err_probe(ppe_dev->dev, cnt,
				     "Fail to qcom,group-config\n");

	cfg = kmalloc_array(cnt, sizeof(*cfg), GFP_KERNEL | __GFP_ZERO);
	if (!cfg)
		return -ENOMEM;

	ret = of_property_read_u32_array(bm_node, "qcom,group-config", cfg, cnt);
	if (ret) {
		dev_err(ppe_dev->dev, "Fail to get qcom,group-config %d\n", ret);
		goto parse_bm_err;
	}

	/* Parse BM group configuration,
	 * the dts propert: qcom,group-config = <group group_buf>;
	 *
	 * There are 3 kinds of buffer types, guaranteed buffer(port based),
	 * shared buffer(group based) and react buffer(cache in-flight packets).
	 *
	 * Maximum 4 groups supported by PPE.
	 */
	ret = 0;
	while ((cnt - ret) / 2) {
		if (cfg[ret] < PPE_BM_SHARED_GROUP_CFG_NUM) {
			reg_val = FIELD_PREP(PPE_BM_SHARED_GROUP_CFG_SHARED_LIMIT, cfg[ret + 1]);

			ppe_write(ppe_dev, PPE_BM_SHARED_GROUP_CFG +
				  PPE_BM_SHARED_GROUP_CFG_INC * cfg[ret], reg_val);
		}
		ret += 2;
	}

	cnt = of_property_count_u32_elems(bm_node, "qcom,port-config");
	if (cnt < 0) {
		dev_err(ppe_dev->dev, "Fail to get qcom,port-config %d\n", cnt);
		goto parse_bm_err;
	}

	cfg = krealloc_array(cfg, cnt, sizeof(*cfg), GFP_KERNEL | __GFP_ZERO);
	if (!cfg) {
		ret = -ENOMEM;
		goto parse_bm_err;
	}

	ret = of_property_read_u32_array(bm_node, "qcom,port-config", cfg, cnt);
	if (ret) {
		dev_err(ppe_dev->dev, "Fail to get qcom,port-config %d\n", ret);
		goto parse_bm_err;
	}

	/* Parse BM port configuration,
	 * the dts property: qcom,port-config = <group port prealloc react ceil
	 * weight res_off res_ceil dynamic>;
	 *
	 * The port based buffer is assigned to the group ID, which is the
	 * buffer dedicated to BM port, and the threshold to generate the
	 * pause frame, the threshold can be configured as the static value
	 * or dynamically adjusted according to the remain buffer.
	 */
	ret = 0;
	while ((cnt - ret) / 9) {
		if (cfg[ret + 1] < PPE_BM_PORT_FC_MODE_NUM) {
			memset(&fc_cfg, 0, sizeof(fc_cfg));

			fc_cfg.bf.pre_alloc = cfg[ret + 2];
			fc_cfg.bf.react_limit = cfg[ret + 3];
			fc_cfg.bf.shared_ceiling_0 = cfg[ret + 4] & 0x7;
			fc_cfg.bf.shared_ceiling_1 = cfg[ret + 4] >> 3;
			fc_cfg.bf.shared_weight = cfg[ret + 5];
			fc_cfg.bf.resum_offset = cfg[ret + 6];
			fc_cfg.bf.resum_floor_th = cfg[ret + 7];
			fc_cfg.bf.shared_dynamic = cfg[ret + 8];
			ppe_write_tbl(ppe_dev, PPE_BM_PORT_FC_CFG +
				      PPE_BM_PORT_FC_CFG_INC * cfg[ret + 1],
				      fc_cfg.val, sizeof(fc_cfg.val));

			reg_val = FIELD_PREP(PPE_BM_PORT_GROUP_ID_SHARED_GROUP_ID, cfg[ret]);
			ppe_write(ppe_dev, PPE_BM_PORT_GROUP_ID +
				  PPE_BM_PORT_GROUP_ID_INC * cfg[ret + 1], reg_val);

			reg_val = FIELD_PREP(PPE_BM_PORT_FC_MODE_EN, 1);
			ppe_write(ppe_dev, PPE_BM_PORT_FC_MODE +
				  PPE_BM_PORT_FC_MODE_INC * cfg[ret + 1], reg_val);
		}
		ret += 9;
	}
	ret = 0;

parse_bm_err:
	kfree(cfg);
	return ret;
}

static int of_parse_ppe_qm(struct ppe_device *ppe_dev,
			   struct device_node *ppe_node)
{
	union ppe_ac_uni_queue_cfg_u uni_queue_cfg;
	union ppe_ac_mul_queue_cfg_u mul_queue_cfg;
	union ppe_ac_grp_cfg_u group_cfg;
	struct device_node *qm_node;
	int ret, cnt, queue_id;
	u32 *cfg;

	qm_node = of_get_child_by_name(ppe_node, "queue-management-config");
	if (!qm_node)
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "Fail to get queue-management-config\n");

	cnt = of_property_count_u32_elems(qm_node, "qcom,group-config");
	if (cnt < 0)
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "Fail to get qcom,group-config\n");

	cfg = kmalloc_array(cnt, sizeof(*cfg), GFP_KERNEL | __GFP_ZERO);
	if (!cfg)
		return -ENOMEM;

	ret = of_property_read_u32_array(qm_node, "qcom,group-config", cfg, cnt);
	if (ret) {
		dev_err(ppe_dev->dev, "Fail to get qcom,group-config\n");
		goto parse_qm_err;
	}

	/* Parse QM group config:
	 * qcom,group-config = <group total prealloc ceil resume_off>;
	 *
	 * For packet enqueue, there are two kinds of buffer type available,
	 * queue based buffer and group(shared) buffer, the queue based buffer
	 * is used firstly, then shared buffer used.
	 *
	 * Maximum 4 groups buffer supported by PPE.
	 */
	ret = 0;
	while ((cnt - ret) / 5) {
		memset(&group_cfg, 0, sizeof(group_cfg));

		ppe_read_tbl(ppe_dev, PPE_AC_GRP_CFG_TBL +
			     PPE_AC_GRP_CFG_TBL_INC * cfg[ret],
			     group_cfg.val, sizeof(group_cfg.val));

		group_cfg.bf.limit = cfg[ret + 1];
		group_cfg.bf.prealloc_limit = cfg[ret + 2];
		group_cfg.bf.dp_thrd_0 = cfg[ret + 3] & 0x3f;
		group_cfg.bf.dp_thrd_1 = cfg[ret + 3] >> 7;
		group_cfg.bf.grn_resume = cfg[ret + 4];

		ppe_write_tbl(ppe_dev, PPE_AC_GRP_CFG_TBL +
			      PPE_AC_GRP_CFG_TBL_INC * cfg[ret],
			      group_cfg.val, sizeof(group_cfg.val));
		ret += 5;
	}

	cnt = of_property_count_u32_elems(qm_node, "qcom,queue-config");
	if (cnt < 0) {
		dev_err(ppe_dev->dev, "Fail to get qcom,queue-config\n");
		goto parse_qm_err;
	}

	cfg = krealloc_array(cfg, cnt, sizeof(*cfg), GFP_KERNEL | __GFP_ZERO);
	if (!cfg) {
		ret = -ENOMEM;
		goto parse_qm_err;
	}

	ret = of_property_read_u32_array(qm_node, "qcom,queue-config", cfg, cnt);
	if (ret) {
		dev_err(ppe_dev->dev, "Fail to get qcom,queue-config\n");
		goto parse_qm_err;
	}

	/* Parse queue based config:
	 * qcom,queue-config = <queue_base queue_num group prealloc
	 * ceil weight resume_off dynamic>;
	 *
	 * There are totally 256(queue id 0-255) unicast queues and 44(256-299)
	 * multicast queues available in PPE, each queue is assigned the
	 * dedicated buffer and ceil to drop packet, the unicast queue supports
	 * static configured ceil value and dynamic ceil value that is adjusted
	 * according to the available group buffers, multicast queue only supports
	 * static ceil.
	 */
	ret = 0;
	while ((cnt - ret) / 8) {
		queue_id = 0;
		while (queue_id < cfg[ret + 1]) {
			if (cfg[ret] + queue_id < PPE_AC_UNI_QUEUE_CFG_TBL_NUM) {
				memset(&uni_queue_cfg, 0, sizeof(uni_queue_cfg));

				ppe_read_tbl(ppe_dev, PPE_AC_UNI_QUEUE_CFG_TBL +
					     PPE_AC_UNI_QUEUE_CFG_TBL_INC * (cfg[ret] + queue_id),
					     uni_queue_cfg.val, sizeof(uni_queue_cfg.val));

				uni_queue_cfg.bf.ac_grp_id = cfg[ret + 2];
				uni_queue_cfg.bf.prealloc_limit = cfg[ret + 3];
				uni_queue_cfg.bf.shared_ceiling = cfg[ret + 4];
				uni_queue_cfg.bf.shared_weight = cfg[ret + 5];
				uni_queue_cfg.bf.grn_resume = cfg[ret + 6];
				uni_queue_cfg.bf.shared_dynamic = cfg[ret + 7];
				uni_queue_cfg.bf.ac_en = 1;

				ppe_write_tbl(ppe_dev, PPE_AC_UNI_QUEUE_CFG_TBL +
					      PPE_AC_UNI_QUEUE_CFG_TBL_INC * (cfg[ret] + queue_id),
					      uni_queue_cfg.val, sizeof(uni_queue_cfg.val));
			} else {
				memset(&mul_queue_cfg, 0, sizeof(mul_queue_cfg));

				ppe_read_tbl(ppe_dev, PPE_AC_MUL_QUEUE_CFG_TBL +
					     PPE_AC_MUL_QUEUE_CFG_TBL_INC * (cfg[ret] + queue_id),
					     mul_queue_cfg.val, sizeof(mul_queue_cfg.val));

				mul_queue_cfg.bf.ac_grp_id = cfg[ret + 2];
				mul_queue_cfg.bf.prealloc_limit = cfg[ret + 3];
				mul_queue_cfg.bf.shared_ceiling = cfg[ret + 4];
				mul_queue_cfg.bf.grn_resume = cfg[ret + 6];
				mul_queue_cfg.bf.ac_en = 1;

				ppe_write_tbl(ppe_dev, PPE_AC_MUL_QUEUE_CFG_TBL +
					      PPE_AC_MUL_QUEUE_CFG_TBL_INC * (cfg[ret] + queue_id),
					      mul_queue_cfg.val, sizeof(mul_queue_cfg.val));
			}

			ppe_mask(ppe_dev, PPE_ENQ_OPR_TBL +
				 PPE_ENQ_OPR_TBL_INC * (cfg[ret] + queue_id),
				 PPE_ENQ_OPR_TBL_DEQ_DISABLE, 0);

			ppe_mask(ppe_dev, PPE_DEQ_OPR_TBL +
				 PPE_DEQ_OPR_TBL_INC * (cfg[ret] + queue_id),
				 PPE_ENQ_OPR_TBL_DEQ_DISABLE, 0);

			queue_id++;
		}
		ret += 8;
	}

	/* Enable queue counter */
	ret = ppe_mask(ppe_dev, PPE_EG_BRIDGE_CONFIG,
		       PPE_EG_BRIDGE_CONFIG_QUEUE_CNT_EN,
		       PPE_EG_BRIDGE_CONFIG_QUEUE_CNT_EN);
parse_qm_err:
	kfree(cfg);
	return ret;
}

static int of_parse_ppe_tdm(struct ppe_device *ppe_dev,
			    struct device_node *ppe_node)
{
	struct device_node *tdm_node;
	u32 *cfg, reg_val;
	int ret, cnt;

	tdm_node = of_get_child_by_name(ppe_node, "tdm-config");
	if (!tdm_node)
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "tdm-config is not defined\n");

	cnt = of_property_count_u32_elems(tdm_node, "qcom,tdm-bm-config");
	if (cnt < 0)
		return dev_err_probe(ppe_dev->dev, -EINVAL,
				     "Fail to get qcom,tdm-bm-config\n");

	cfg = kmalloc_array(cnt, sizeof(*cfg), GFP_KERNEL | __GFP_ZERO);
	if (!cfg)
		return -ENOMEM;

	ret = of_property_read_u32_array(tdm_node, "qcom,tdm-bm-config", cfg, cnt);
	if (ret) {
		dev_err(ppe_dev->dev, "Fail to get qcom,tdm-bm-config\n");
		goto parse_tdm_err;
	}

	/* Parse TDM BM configuration,
	 * the dts property:
	 * qcom,tdm-bm-config = <valid dir port second_valid second_port>;
	 *
	 * This config decides the number ticks available for physical port
	 * to utilize buffer for receiving and transmiting packet.
	 */
	reg_val = FIELD_PREP(PPE_BM_TDM_CTRL_TDM_DEPTH, cnt / 5) |
		  FIELD_PREP(PPE_BM_TDM_CTRL_TDM_OFFSET, 0) |
		  FIELD_PREP(PPE_BM_TDM_CTRL_TDM_EN, 1);
	ret = ppe_write(ppe_dev, PPE_BM_TDM_CTRL, reg_val);
	if (ret)
		return ret;

	ret = 0;
	while ((cnt - ret) / 5) {
		reg_val = FIELD_PREP(PPE_BM_TDM_CFG_TBL_VALID, cfg[ret]) |
			  FIELD_PREP(PPE_BM_TDM_CFG_TBL_DIR, cfg[ret + 1]) |
			  FIELD_PREP(PPE_BM_TDM_CFG_TBL_PORT_NUM, cfg[ret + 2]) |
			  FIELD_PREP(PPE_BM_TDM_CFG_TBL_SECOND_PORT_VALID, cfg[ret + 3]) |
			  FIELD_PREP(PPE_BM_TDM_CFG_TBL_SECOND_PORT, cfg[ret + 4]);

		ppe_write(ppe_dev,
			  PPE_BM_TDM_CFG_TBL + (ret / 5) * PPE_BM_TDM_CFG_TBL_INC,
			  reg_val);
		ret += 5;
	}

	cnt = of_property_count_u32_elems(tdm_node, "qcom,tdm-port-scheduler-config");
	if (cnt < 0) {
		dev_err(ppe_dev->dev, "Fail to get qcom,tdm-port-scheduler-config\n");
		goto parse_tdm_err;
	}

	cfg = krealloc_array(cfg, cnt, sizeof(*cfg), GFP_KERNEL | __GFP_ZERO);
	if (!cfg) {
		ret = -ENOMEM;
		goto parse_tdm_err;
	}

	ret = of_property_read_u32_array(tdm_node, "qcom,tdm-port-scheduler-config",
					 cfg, cnt);
	if (ret) {
		dev_err(ppe_dev->dev, "Fail to get qcom,tdm-port-scheduler-config\n");
		goto parse_tdm_err;
	}

	/* Parse TDM scheduler configuration,
	 * the dts property:
	 * qcom,tdm-port-scheduler-config = <ensch_bmp ensch_port desch_port
	 * desch_second_valid desch_second_port>;
	 *
	 * This config decides the ticks number available for packet enqueue
	 * and dequeue on the physical port.
	 */
	reg_val = FIELD_PREP(PPE_PSCH_TDM_DEPTH_CFG_TDM_DEPTH, cnt / 5);
	ppe_write(ppe_dev, PPE_PSCH_TDM_DEPTH_CFG, reg_val);

	ret = 0;
	while ((cnt - ret) / 5) {
		reg_val = FIELD_PREP(PPE_PSCH_TDM_CFG_TBL_ENS_PORT_BITMAP, cfg[ret]) |
			  FIELD_PREP(PPE_PSCH_TDM_CFG_TBL_ENS_PORT, cfg[ret + 1]) |
			  FIELD_PREP(PPE_PSCH_TDM_CFG_TBL_DES_PORT, cfg[ret + 2]) |
			  FIELD_PREP(PPE_PSCH_TDM_CFG_TBL_DES_SECOND_PORT_EN, cfg[ret + 3]) |
			  FIELD_PREP(PPE_PSCH_TDM_CFG_TBL_DES_SECOND_PORT, cfg[ret + 4]);

		ppe_write(ppe_dev,
			  PPE_PSCH_TDM_CFG_TBL + (ret / 5) * PPE_PSCH_TDM_CFG_TBL_INC,
			  reg_val);
		ret += 5;
	}

	ret = 0;
parse_tdm_err:
	kfree(cfg);
	return ret;
};

static int of_parse_ppe_scheduler_resource(struct ppe_device *ppe_dev,
					   struct device_node *resource_node)
{
	struct device_node *port_node;
	u32 port;

	for_each_available_child_of_node(resource_node, port_node) {
		if (of_property_read_u32(port_node, "port-id", &port))
			return dev_err_probe(ppe_dev->dev, -ENODEV,
					     "port-id not defined on resource\n");

		if (port >= ARRAY_SIZE(ppe_scheduler_res))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid port-id defined on resource\n");

		if (of_property_read_u32_array(port_node, "qcom,ucast-queue",
					       ppe_scheduler_res[port].ucastq,
					       ARRAY_SIZE(ppe_scheduler_res[port].ucastq)))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid qcom,ucast-queue defined on resource\n");

		if (of_property_read_u32_array(port_node, "qcom,mcast-queue",
					       ppe_scheduler_res[port].mcastq,
					       ARRAY_SIZE(ppe_scheduler_res[port].mcastq)))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid qcom,mcast-queue defined on resource\n");

		if (of_property_read_u32_array(port_node, "qcom,l0sp",
					       ppe_scheduler_res[port].l0sp,
					       ARRAY_SIZE(ppe_scheduler_res[port].l0sp)))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid qcom,l0sp defined on resource\n");

		if (of_property_read_u32_array(port_node, "qcom,l0cdrr",
					       ppe_scheduler_res[port].l0cdrr,
					       ARRAY_SIZE(ppe_scheduler_res[port].l0cdrr)))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid qcom,l0cdrr defined on resource\n");

		if (of_property_read_u32_array(port_node, "qcom,l0edrr",
					       ppe_scheduler_res[port].l0edrr,
					       ARRAY_SIZE(ppe_scheduler_res[port].l0edrr)))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid qcom,l0edrr defined on resource\n");

		if (of_property_read_u32_array(port_node, "qcom,l1cdrr",
					       ppe_scheduler_res[port].l1cdrr,
					       ARRAY_SIZE(ppe_scheduler_res[port].l1cdrr)))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid qcom,l1cdrr defined on resource\n");

		if (of_property_read_u32_array(port_node, "qcom,l1edrr",
					       ppe_scheduler_res[port].l1edrr,
					       ARRAY_SIZE(ppe_scheduler_res[port].l1edrr)))
			return dev_err_probe(ppe_dev->dev, -EINVAL,
					     "Invalid qcom,l1edrr defined on resource\n");
	}

	return 0;
}

static int of_parse_ppe_scheduler(struct ppe_device *ppe_dev,
				  struct device_node *ppe_node)
{
	struct device_node *scheduler_node;

	scheduler_node = of_get_child_by_name(ppe_node, "port-scheduler-resource");
	if (!scheduler_node)
		return dev_err_probe(ppe_dev->dev, -ENODEV,
				     "port-scheduler-resource is not defined\n");

	return of_parse_ppe_scheduler_resource(ppe_dev, scheduler_node);
}

static int of_parse_ppe_config(struct ppe_device *ppe_dev,
			       struct device_node *ppe_node)
{
	int ret;

	ret = of_parse_ppe_bm(ppe_dev, ppe_node);
	if (ret)
		return ret;

	ret = of_parse_ppe_qm(ppe_dev, ppe_node);
	if (ret)
		return ret;

	ret = of_parse_ppe_tdm(ppe_dev, ppe_node);
	if (ret)
		return ret;

	return of_parse_ppe_scheduler(ppe_dev, ppe_node);
}

static int qcom_ppe_probe(struct platform_device *pdev)
{
	struct ppe_device *ppe_dev;
	void __iomem *base;
	int ret;

	ppe_dev = devm_kzalloc(&pdev->dev, sizeof(*ppe_dev), GFP_KERNEL);
	if (!ppe_dev)
		return -ENOMEM;

	ppe_dev->dev = &pdev->dev;
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return dev_err_probe(&pdev->dev,
				     PTR_ERR(base),
				     "Fail to ioremap\n");

	ppe_dev->regmap = devm_regmap_init_mmio(&pdev->dev, base, &ppe_regmap_config);
	if (IS_ERR(ppe_dev->regmap))
		return dev_err_probe(&pdev->dev,
				     PTR_ERR(ppe_dev->regmap),
				     "Fail to regmap\n");

	ppe_dev->ppe_priv = ppe_data_init(pdev);
	if (IS_ERR(ppe_dev->ppe_priv))
		return dev_err_probe(&pdev->dev,
				     PTR_ERR(ppe_dev->ppe_priv),
				     "Fail to init ppe data\n");

	platform_set_drvdata(pdev, ppe_dev);
	ret = ppe_clock_config(pdev);
	if (ret)
		return dev_err_probe(&pdev->dev,
				     ret,
				     "ppe clock config failed\n");

	ret = of_parse_ppe_config(ppe_dev, pdev->dev.of_node);
	if (ret)
		return dev_err_probe(&pdev->dev,
				     ret,
				     "of parse ppe failed\n");

	ppe_dev->is_ppe_probed = true;
	return 0;
}

static int qcom_ppe_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id qcom_ppe_of_match[] = {
	{ .compatible = "qcom,ipq9574-ppe", },
	{ .compatible = "qcom,ipq5332-ppe", },
	{},
};

static struct platform_driver qcom_ppe_driver = {
	.driver = {
		.name = "qcom_ppe",
		.owner  = THIS_MODULE,
		.of_match_table = qcom_ppe_of_match,
	},
	.probe	= qcom_ppe_probe,
	.remove = qcom_ppe_remove,
};
module_platform_driver(qcom_ppe_driver);

MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, qcom_ppe_of_match);
