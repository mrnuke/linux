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
