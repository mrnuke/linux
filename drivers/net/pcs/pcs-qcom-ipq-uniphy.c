// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pcs/pcs-qcom-ipq-uniphy.h>
#include <linux/phylink.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

/* Maximum PCS channel numbers, For PSGMII it has 5 channels */
#define PCS_MAX_CHANNELS		5

#define PCS_CALIBRATION			0x1e0
#define PCS_CALIBRATION_DONE		BIT(7)

#define PCS_MODE_CTRL			0x46c
#define PCS_MODE_SEL_MASK		GENMASK(12, 8)
#define PCS_MODE_SGMII			FIELD_PREP(PCS_MODE_SEL_MASK, 0x4)
#define PCS_MODE_QSGMII			FIELD_PREP(PCS_MODE_SEL_MASK, 0x1)
#define PCS_MODE_PSGMII			FIELD_PREP(PCS_MODE_SEL_MASK, 0x2)
#define PCS_MODE_SGMII_PLUS		FIELD_PREP(PCS_MODE_SEL_MASK, 0x8)
#define PCS_MODE_XPCS			FIELD_PREP(PCS_MODE_SEL_MASK, 0x10)
#define PCS_MODE_SGMII_CTRL_MASK	GENMASK(6, 4)
#define PCS_MODE_SGMII_CTRL_1000BASEX	FIELD_PREP(PCS_MODE_SGMII_CTRL_MASK, \
						   0x0)
#define PCS_MODE_AN_MODE		BIT(0)

#define PCS_CHANNEL_CTRL(x)		(0x480 + 0x18 * (x))
#define PCS_CHANNEL_ADPT_RESET		BIT(11)
#define PCS_CHANNEL_FORCE_MODE		BIT(3)
#define PCS_CHANNEL_SPEED_MASK		GENMASK(2, 1)
#define PCS_CHANNEL_SPEED_1000		FIELD_PREP(PCS_CHANNEL_SPEED_MASK, 0x2)
#define PCS_CHANNEL_SPEED_100		FIELD_PREP(PCS_CHANNEL_SPEED_MASK, 0x1)
#define PCS_CHANNEL_SPEED_10		FIELD_PREP(PCS_CHANNEL_SPEED_MASK, 0x0)

#define PCS_CHANNEL_STS(x)		(0x488 + 0x18 * (x))
#define PCS_CHANNEL_LINK_STS		BIT(7)
#define PCS_CHANNEL_STS_DUPLEX_FULL	BIT(6)
#define PCS_CHANNEL_STS_SPEED_MASK	GENMASK(5, 4)
#define PCS_CHANNEL_STS_SPEED_10	0
#define PCS_CHANNEL_STS_SPEED_100	1
#define PCS_CHANNEL_STS_SPEED_1000	2
#define PCS_CHANNEL_STS_PAUSE_TX_EN	BIT(1)
#define PCS_CHANNEL_STS_PAUSE_RX_EN	BIT(0)

#define PCS_QP_USXG_OPTION		0x584
#define PCS_QP_USXG_GMII_SRC_XPCS	BIT(0)

#define PCS_PLL_RESET			0x780
#define PCS_ANA_SW_RESET		BIT(6)

#define XPCS_INDIRECT_ADDR		0x8000
#define XPCS_INDIRECT_AHB_ADDR		0x83fc
#define XPCS_INDIRECT_ADDR_H		GENMASK(20, 8)
#define XPCS_INDIRECT_ADDR_L		GENMASK(7, 0)
#define XPCS_INDIRECT_DATA_ADDR(reg)	(FIELD_PREP(GENMASK(15, 10), 0x20) | \
					 FIELD_PREP(GENMASK(9, 2), \
					 FIELD_GET(XPCS_INDIRECT_ADDR_L, reg)))

#define XPCS_10GBASER_STS		0x30020
#define XPCS_10GBASER_LINK_STS		BIT(12)

#define XPCS_DIG_CTRL			0x38000
#define XPCS_SOFT_RESET			BIT(15)
#define XPCS_USXG_ADPT_RESET		BIT(10)
#define XPCS_USXG_EN			BIT(9)

#define XPCS_KR_CTRL			0x38007
#define XPCS_USXG_MODE_MASK		GENMASK(12, 10)
#define XPCS_10G_QXGMII_MODE		FIELD_PREP(XPCS_USXG_MODE_MASK, 0x5)

#define XPCS_DIG_STS			0x3800a
#define XPCS_DIG_STS_AM_COUNT		GENMASK(14, 0)

#define XPCS_CHANNEL_DIG_CTRL(x)	(0x1a8000 + 0x10000 * ((x) - 1))
#define XPCS_CHANNEL_USXG_ADPT_RESET	BIT(5)

#define XPCS_MII_CTRL			0x1f0000
#define XPCS_CHANNEL_MII_CTRL(x)	(0x1a0000 + 0x10000 * ((x) - 1))
#define XPCS_MII_AN_EN			BIT(12)
#define XPCS_DUPLEX_FULL		BIT(8)
#define XPCS_SPEED_MASK			(BIT(13) | BIT(6) | BIT(5))
#define XPCS_SPEED_10000		(BIT(13) | BIT(6))
#define XPCS_SPEED_5000			(BIT(13) | BIT(5))
#define XPCS_SPEED_2500			BIT(5)
#define XPCS_SPEED_1000			BIT(6)
#define XPCS_SPEED_100			BIT(13)
#define XPCS_SPEED_10			0

#define XPCS_MII_AN_CTRL		0x1f8001
#define XPCS_CHANNEL_MII_AN_CTRL(x)	(0x1a8001 + 0x10000 * ((x) - 1))
#define XPCS_MII_AN_8BIT		BIT(8)

#define XPCS_MII_AN_INTR_STS		0x1f8002
#define XPCS_CHANNEL_MII_AN_INTR_STS(x)	(0x1a8002 + 0x10000 * ((x) - 1))
#define XPCS_USXG_AN_LINK_STS		BIT(14)
#define XPCS_USXG_AN_DUPLEX_FULL	BIT(13)
#define XPCS_USXG_AN_SPEED_MASK		GENMASK(12, 10)
#define XPCS_USXG_AN_SPEED_10		0
#define XPCS_USXG_AN_SPEED_100		1
#define XPCS_USXG_AN_SPEED_1000		2
#define XPCS_USXG_AN_SPEED_2500		4
#define XPCS_USXG_AN_SPEED_5000		5
#define XPCS_USXG_AN_SPEED_10000	3

#define XPCS_XAUI_MODE_CTRL		0x1f8004
#define XPCS_CHANNEL_XAUI_MODE_CTRL(x)	(0x1a8004 + 0x10000 * ((x) - 1))
#define XPCS_TX_IPG_CHECK_DIS		BIT(0)

/* UNIPHY PCS RAW clock ID */
enum {
	PCS_RAW_RX_CLK = 0,
	PCS_RAW_TX_CLK,
	PCS_RAW_CLK_MAX
};

/* UNIPHY PCS raw clock */
struct ipq_unipcs_raw_clk {
	struct clk_hw hw;
	unsigned long rate;
};

/* UNIPHY PCS clock ID */
enum {
	PCS_SYS_CLK,
	PCS_AHB_CLK,
	PCS_CLK_MAX
};

/* UNIPHY PCS reset ID */
enum {
	PCS_SYS_RESET,
	PCS_AHB_RESET,
	XPCS_RESET,
	PCS_RESET_MAX
};

/* UNIPHY PCS clock name */
static const char *const pcs_clock_name[PCS_CLK_MAX] = {
	"sys",
	"ahb",
};

/* UNIPHY PCS reset name */
static const char *const pcs_reset_name[PCS_RESET_MAX] = {
	"sys",
	"ahb",
	"xpcs",
};

/* UNIPHY PCS channel clock ID */
enum {
	PCS_CH_RX_CLK,
	PCS_CH_TX_CLK,
	PCS_CH_CLK_MAX
};

/* UNIPHY PCS channel clock name */
static const char *const pcs_ch_clock_name[PCS_CH_CLK_MAX] = {
	"ch_rx",
	"ch_tx",
};

/* UNIPHY PCS private data instance */
struct ipq_uniphy_pcs {
	void __iomem *base;
	struct device *dev;
	phy_interface_t interface;
	struct mutex shared_lock; /* Lock to protect shared config */
	spinlock_t reg_lock; /* Lock for register access */
	struct clk *clk[PCS_CLK_MAX];
	struct reset_control *reset[PCS_RESET_MAX];
	struct ipq_unipcs_raw_clk raw_clk[PCS_RAW_CLK_MAX];
};

/* UNIPHY PCS channel private data instance */
struct ipq_uniphy_pcs_ch {
	struct ipq_uniphy_pcs *qunipcs;
	struct phylink_pcs pcs;
	int channel;
	struct clk *clk[PCS_CH_CLK_MAX];
};

#define to_unipcs_raw_clk(_hw)		\
	container_of(_hw, struct ipq_unipcs_raw_clk, hw)
#define phylink_pcs_to_unipcs(_pcs)	\
	container_of(_pcs, struct ipq_uniphy_pcs_ch, pcs)

static unsigned long ipq_unipcs_raw_clk_recalc_rate(struct clk_hw *hw,
						    unsigned long parent_rate)
{
	struct ipq_unipcs_raw_clk *raw_clk = to_unipcs_raw_clk(hw);

	return raw_clk->rate;
}

static int ipq_unipcs_raw_clk_determine_rate(struct clk_hw *hw,
					     struct clk_rate_request *req)
{
	switch (req->rate) {
	case 125000000:
	case 312500000:
		return 0;
	default:
		return -EINVAL;
	}
}

static int ipq_unipcs_raw_clk_set_rate(struct clk_hw *hw,
				       unsigned long rate,
				       unsigned long parent_rate)
{
	struct ipq_unipcs_raw_clk *raw_clk = to_unipcs_raw_clk(hw);

	switch (rate) {
	case 125000000:
	case 312500000:
		raw_clk->rate = rate;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct clk_ops ipq_unipcs_raw_clk_ops = {
	.recalc_rate = ipq_unipcs_raw_clk_recalc_rate,
	.determine_rate = ipq_unipcs_raw_clk_determine_rate,
	.set_rate = ipq_unipcs_raw_clk_set_rate,
};

static u32 ipq_unipcs_reg_read32(struct ipq_uniphy_pcs *qunipcs, u32 reg)
{
	u32 val;

	/* PCS use direct AHB access while XPCS use indirect AHB access */
	if (reg >= XPCS_INDIRECT_ADDR) {
		/* For XPCS, althrough the register is different for different
		 * channels, but they use the same indirect AHB address to
		 * access, so add protects here.
		 */
		spin_lock(&qunipcs->reg_lock);

		writel(FIELD_GET(XPCS_INDIRECT_ADDR_H, reg),
		       qunipcs->base + XPCS_INDIRECT_AHB_ADDR);
		val = readl(qunipcs->base + XPCS_INDIRECT_DATA_ADDR(reg));

		spin_unlock(&qunipcs->reg_lock);
		return val;
	} else {
		return readl(qunipcs->base + reg);
	}
}

static void ipq_unipcs_reg_modify32(struct ipq_uniphy_pcs *qunipcs,
				    u32 reg, u32 mask, u32 set)
{
	u32 val;

	if (reg >= XPCS_INDIRECT_ADDR) {
		spin_lock(&qunipcs->reg_lock);

		/* XPCS read */
		writel(FIELD_GET(XPCS_INDIRECT_ADDR_H, reg),
		       qunipcs->base + XPCS_INDIRECT_AHB_ADDR);
		val = readl(qunipcs->base + XPCS_INDIRECT_DATA_ADDR(reg));

		val &= ~mask;
		val |= set;

		/* XPCS write */
		writel(val, qunipcs->base + XPCS_INDIRECT_DATA_ADDR(reg));

		spin_unlock(&qunipcs->reg_lock);
	} else {
		val = readl(qunipcs->base + reg);
		val &= ~mask;
		val |= set;
		writel(val, qunipcs->base + reg);
	}
}

static void ipq_unipcs_get_state_sgmii(struct ipq_uniphy_pcs *qunipcs,
				       int channel,
				       struct phylink_link_state *state)
{
	u32 val;

	val = ipq_unipcs_reg_read32(qunipcs, PCS_CHANNEL_STS(channel));

	state->link = !!(val & PCS_CHANNEL_LINK_STS);

	if (!state->link)
		return;

	switch (FIELD_GET(PCS_CHANNEL_STS_SPEED_MASK, val)) {
	case PCS_CHANNEL_STS_SPEED_1000:
		state->speed = SPEED_1000;
		break;
	case PCS_CHANNEL_STS_SPEED_100:
		state->speed = SPEED_100;
		break;
	case PCS_CHANNEL_STS_SPEED_10:
		state->speed = SPEED_10;
		break;
	default:
		return;
	}

	if (val & PCS_CHANNEL_STS_DUPLEX_FULL)
		state->duplex = DUPLEX_FULL;
	else
		state->duplex = DUPLEX_HALF;

	if (val & PCS_CHANNEL_STS_PAUSE_TX_EN)
		state->pause |= MLO_PAUSE_TX;
	if (val & PCS_CHANNEL_STS_PAUSE_RX_EN)
		state->pause |= MLO_PAUSE_RX;
}

static void ipq_unipcs_get_state_2500basex(struct ipq_uniphy_pcs *qunipcs,
					   int channel,
					   struct phylink_link_state *state)
{
	u32 val;

	val = ipq_unipcs_reg_read32(qunipcs, PCS_CHANNEL_STS(channel));

	state->link = !!(val & PCS_CHANNEL_LINK_STS);

	if (!state->link)
		return;

	state->speed = SPEED_2500;
	state->duplex = DUPLEX_FULL;
	state->pause |= MLO_PAUSE_TXRX_MASK;
}

static void ipq_unipcs_get_state_usxgmii(struct ipq_uniphy_pcs *qunipcs,
					 int channel,
					 struct phylink_link_state *state)
{
	u32 val, reg;

	reg = (channel == 0) ? XPCS_MII_AN_INTR_STS :
		XPCS_CHANNEL_MII_AN_INTR_STS(channel);

	val = ipq_unipcs_reg_read32(qunipcs, reg);

	state->link = !!(val & XPCS_USXG_AN_LINK_STS);

	if (!state->link)
		return;

	switch (FIELD_GET(XPCS_USXG_AN_SPEED_MASK, val)) {
	case XPCS_USXG_AN_SPEED_10000:
		state->speed = SPEED_10000;
		break;
	case XPCS_USXG_AN_SPEED_5000:
		state->speed = SPEED_5000;
		break;
	case XPCS_USXG_AN_SPEED_2500:
		state->speed = SPEED_2500;
		break;
	case XPCS_USXG_AN_SPEED_1000:
		state->speed = SPEED_1000;
		break;
	case XPCS_USXG_AN_SPEED_100:
		state->speed = SPEED_100;
		break;
	case XPCS_USXG_AN_SPEED_10:
		state->speed = SPEED_10;
		break;
	default:
		return;
	}

	if (val & XPCS_USXG_AN_DUPLEX_FULL)
		state->duplex = DUPLEX_FULL;
	else
		state->duplex = DUPLEX_HALF;
}

static void ipq_unipcs_get_state_10gbaser(struct ipq_uniphy_pcs *qunipcs,
					  struct phylink_link_state *state)
{
	u32 val;

	val = ipq_unipcs_reg_read32(qunipcs, XPCS_10GBASER_STS);

	state->link = !!(val & XPCS_10GBASER_LINK_STS);

	if (!state->link)
		return;

	state->speed = SPEED_10000;
	state->duplex = DUPLEX_FULL;
	state->pause |= MLO_PAUSE_TXRX_MASK;
}

static int ipq_unipcs_config_mode(struct ipq_uniphy_pcs *qunipcs,
				  phy_interface_t interface)
{
	unsigned long rate = 0;
	u32 val;
	int ret;

	/* Assert XPCS reset */
	reset_control_assert(qunipcs->reset[XPCS_RESET]);

	/* Config PCS interface mode */
	switch (interface) {
	case PHY_INTERFACE_MODE_SGMII:
		rate = 125000000;
		/* Select Qualcomm SGMII AN mode */
		ipq_unipcs_reg_modify32(qunipcs, PCS_MODE_CTRL,
					PCS_MODE_SEL_MASK | PCS_MODE_AN_MODE,
					PCS_MODE_SGMII);
		break;
	case PHY_INTERFACE_MODE_QSGMII:
		rate = 125000000;
		/* Select Qualcomm SGMII AN mode */
		ipq_unipcs_reg_modify32(qunipcs, PCS_MODE_CTRL,
					PCS_MODE_SEL_MASK | PCS_MODE_AN_MODE,
					PCS_MODE_QSGMII);
		break;
	case PHY_INTERFACE_MODE_PSGMII:
		rate = 125000000;
		/* Select Qualcomm SGMII AN mode */
		ipq_unipcs_reg_modify32(qunipcs, PCS_MODE_CTRL,
					PCS_MODE_SEL_MASK | PCS_MODE_AN_MODE,
					PCS_MODE_PSGMII);
		break;
	case PHY_INTERFACE_MODE_1000BASEX:
		ipq_unipcs_reg_modify32(qunipcs, PCS_MODE_CTRL,
					PCS_MODE_SEL_MASK |
					PCS_MODE_SGMII_CTRL_MASK,
					PCS_MODE_SGMII |
					PCS_MODE_SGMII_CTRL_1000BASEX);
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
		rate = 312500000;
		ipq_unipcs_reg_modify32(qunipcs, PCS_MODE_CTRL,
					PCS_MODE_SEL_MASK,
					PCS_MODE_SGMII_PLUS);
		break;
	case PHY_INTERFACE_MODE_USXGMII:
	case PHY_INTERFACE_MODE_10GBASER:
		rate = 312500000;
		ipq_unipcs_reg_modify32(qunipcs, PCS_MODE_CTRL,
					PCS_MODE_SEL_MASK,
					PCS_MODE_XPCS);
		break;
	case PHY_INTERFACE_MODE_10G_QXGMII:
		rate = 312500000;
		ipq_unipcs_reg_modify32(qunipcs, PCS_MODE_CTRL,
					PCS_MODE_SEL_MASK,
					PCS_MODE_XPCS);
		ipq_unipcs_reg_modify32(qunipcs, PCS_QP_USXG_OPTION,
					PCS_QP_USXG_GMII_SRC_XPCS,
					PCS_QP_USXG_GMII_SRC_XPCS);
		break;
	default:
		dev_err(qunipcs->dev,
			"interface %s not supported\n", phy_modes(interface));
		return -EOPNOTSUPP;
	}

	/* PCS PLL reset */
	ipq_unipcs_reg_modify32(qunipcs, PCS_PLL_RESET, PCS_ANA_SW_RESET, 0);
	fsleep(10000);
	ipq_unipcs_reg_modify32(qunipcs, PCS_PLL_RESET,
				PCS_ANA_SW_RESET, PCS_ANA_SW_RESET);

	/* Wait for calibration completion */
	ret = read_poll_timeout(ipq_unipcs_reg_read32, val,
				val & PCS_CALIBRATION_DONE,
				1000, 100000, true,
				qunipcs, PCS_CALIBRATION);
	if (ret) {
		dev_err(qunipcs->dev, "UNIPHY PCS calibration timed-out\n");
		return ret;
	}

	/* Configure raw clock rate */
	clk_set_rate(qunipcs->raw_clk[PCS_RAW_RX_CLK].hw.clk, rate);
	clk_set_rate(qunipcs->raw_clk[PCS_RAW_TX_CLK].hw.clk, rate);

	return 0;
}

static int ipq_unipcs_config_sgmii(struct ipq_uniphy_pcs *qunipcs,
				   int channel,
				   unsigned int neg_mode,
				   phy_interface_t interface)
{
	int ret;

	/* PCS configurations shared by multi channels should be
	 * configured for only once.
	 */
	if (phy_interface_num_ports(interface) > 1)
		mutex_lock(&qunipcs->shared_lock);

	if (qunipcs->interface != interface) {
		ret = ipq_unipcs_config_mode(qunipcs, interface);
		if (ret)
			goto err;

		qunipcs->interface = interface;
	}

	if (phy_interface_num_ports(interface) > 1)
		mutex_unlock(&qunipcs->shared_lock);

	/* In-band autoneg mode is enabled by default for each PCS channel */
	if (neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED)
		return 0;

	/* Force speed mode */
	ipq_unipcs_reg_modify32(qunipcs, PCS_CHANNEL_CTRL(channel),
				PCS_CHANNEL_FORCE_MODE, PCS_CHANNEL_FORCE_MODE);

	return 0;

err:
	if (phy_interface_num_ports(interface) > 1)
		mutex_unlock(&qunipcs->shared_lock);

	return ret;
}

static int ipq_unipcs_config_2500basex(struct ipq_uniphy_pcs *qunipcs,
				       phy_interface_t interface)
{
	int ret;

	if (qunipcs->interface != interface) {
		ret = ipq_unipcs_config_mode(qunipcs, interface);
		if (ret)
			return ret;

		qunipcs->interface = interface;
	}

	return 0;
}

static int ipq_unipcs_config_usxgmii(struct ipq_uniphy_pcs *qunipcs,
				     int channel,
				     unsigned int neg_mode,
				     phy_interface_t interface)
{
	int ret;
	u32 reg;

	/* Only in-band autoneg mode is supported currently */
	if (neg_mode != PHYLINK_PCS_NEG_INBAND_ENABLED)
		return -EOPNOTSUPP;

	if (interface == PHY_INTERFACE_MODE_10G_QXGMII)
		mutex_lock(&qunipcs->shared_lock);

	if (qunipcs->interface != interface) {
		ret = ipq_unipcs_config_mode(qunipcs, interface);
		if (ret)
			goto err;

		/* Deassert XPCS and configure XPCS USXGMII or 10G_QXGMII */
		reset_control_deassert(qunipcs->reset[XPCS_RESET]);

		ipq_unipcs_reg_modify32(qunipcs, XPCS_DIG_CTRL,
					XPCS_USXG_EN, XPCS_USXG_EN);

		if (interface == PHY_INTERFACE_MODE_10G_QXGMII) {
			ipq_unipcs_reg_modify32(qunipcs, XPCS_KR_CTRL,
						XPCS_USXG_MODE_MASK,
						XPCS_10G_QXGMII_MODE);

			/* Set Alignment Marker Interval */
			ipq_unipcs_reg_modify32(qunipcs, XPCS_DIG_STS,
						XPCS_DIG_STS_AM_COUNT,
						0x6018);

			ipq_unipcs_reg_modify32(qunipcs, XPCS_DIG_CTRL,
						XPCS_SOFT_RESET,
						XPCS_SOFT_RESET);
		}

		qunipcs->interface = interface;
	}

	if (interface == PHY_INTERFACE_MODE_10G_QXGMII)
		mutex_unlock(&qunipcs->shared_lock);

	/* Disable Tx IPG check for 10G_QXGMII */
	if (interface == PHY_INTERFACE_MODE_10G_QXGMII) {
		reg = (channel == 0) ? XPCS_XAUI_MODE_CTRL :
			XPCS_CHANNEL_XAUI_MODE_CTRL(channel);

		ipq_unipcs_reg_modify32(qunipcs, reg,
					XPCS_TX_IPG_CHECK_DIS,
					XPCS_TX_IPG_CHECK_DIS);
	}

	/* Enable autoneg */
	reg = (channel == 0) ? XPCS_MII_AN_CTRL :
		XPCS_CHANNEL_MII_AN_CTRL(channel);

	ipq_unipcs_reg_modify32(qunipcs, reg,
				XPCS_MII_AN_8BIT, XPCS_MII_AN_8BIT);

	reg = (channel == 0) ? XPCS_MII_CTRL :
		XPCS_CHANNEL_MII_CTRL(channel);

	ipq_unipcs_reg_modify32(qunipcs, reg,
				XPCS_MII_AN_EN, XPCS_MII_AN_EN);

	return 0;

err:
	if (interface == PHY_INTERFACE_MODE_10G_QXGMII)
		mutex_unlock(&qunipcs->shared_lock);

	return ret;
}

static int ipq_unipcs_config_10gbaser(struct ipq_uniphy_pcs *qunipcs,
				      phy_interface_t interface)
{
	int ret;

	if (qunipcs->interface != interface) {
		ret = ipq_unipcs_config_mode(qunipcs, interface);
		if (ret)
			return ret;

		/* Deassert XPCS */
		reset_control_deassert(qunipcs->reset[XPCS_RESET]);

		qunipcs->interface = interface;
	}

	return 0;
}

static unsigned long ipq_unipcs_clock_rate_get_gmii(int speed)
{
	unsigned long rate = 0;

	switch (speed) {
	case SPEED_1000:
		rate = 125000000;
		break;
	case SPEED_100:
		rate = 25000000;
		break;
	case SPEED_10:
		rate = 2500000;
		break;
	default:
		break;
	}

	return rate;
}

static unsigned long ipq_unipcs_clock_rate_get_gmiiplus(int speed)
{
	unsigned long rate = 0;

	switch (speed) {
	case SPEED_2500:
		rate = 312500000;
		break;
	default:
		break;
	}

	return rate;
}

static unsigned long ipq_unipcs_clock_rate_get_xgmii(int speed)
{
	unsigned long rate = 0;

	switch (speed) {
	case SPEED_10000:
		rate = 312500000;
		break;
	case SPEED_5000:
		rate = 156250000;
		break;
	case SPEED_2500:
		rate = 78125000;
		break;
	case SPEED_1000:
		rate = 125000000;
		break;
	case SPEED_100:
		rate = 12500000;
		break;
	case SPEED_10:
		rate = 1250000;
		break;
	default:
		break;
	}

	return rate;
}

static void
ipq_unipcs_link_up_clock_rate_set(struct ipq_uniphy_pcs_ch *qunipcs_ch,
				  phy_interface_t interface,
				  int speed)
{
	struct ipq_uniphy_pcs *qunipcs = qunipcs_ch->qunipcs;
	unsigned long rate = 0;

	switch (interface) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
	case PHY_INTERFACE_MODE_PSGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
		rate = ipq_unipcs_clock_rate_get_gmii(speed);
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
		rate = ipq_unipcs_clock_rate_get_gmiiplus(speed);
		break;
	case PHY_INTERFACE_MODE_USXGMII:
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_10G_QXGMII:
		rate = ipq_unipcs_clock_rate_get_xgmii(speed);
		break;
	default:
		dev_err(qunipcs->dev,
			"interface %s not supported\n", phy_modes(interface));
		return;
	}

	if (rate == 0) {
		dev_err(qunipcs->dev, "Invalid PCS clock rate\n");
		return;
	}

	clk_set_rate(qunipcs_ch->clk[PCS_CH_RX_CLK], rate);
	clk_set_rate(qunipcs_ch->clk[PCS_CH_TX_CLK], rate);
	fsleep(10000);
}

static void ipq_unipcs_link_up_config_sgmii(struct ipq_uniphy_pcs *qunipcs,
					    int channel,
					    unsigned int neg_mode,
					    int speed)
{
	/* No need to config PCS speed if in-band autoneg is enabled */
	if (neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED)
		goto pcs_adapter_reset;

	/* PCS speed set for force mode */
	switch (speed) {
	case SPEED_1000:
		ipq_unipcs_reg_modify32(qunipcs, PCS_CHANNEL_CTRL(channel),
					PCS_CHANNEL_SPEED_MASK,
					PCS_CHANNEL_SPEED_1000);
		break;
	case SPEED_100:
		ipq_unipcs_reg_modify32(qunipcs, PCS_CHANNEL_CTRL(channel),
					PCS_CHANNEL_SPEED_MASK,
					PCS_CHANNEL_SPEED_100);
		break;
	case SPEED_10:
		ipq_unipcs_reg_modify32(qunipcs, PCS_CHANNEL_CTRL(channel),
					PCS_CHANNEL_SPEED_MASK,
					PCS_CHANNEL_SPEED_10);
		break;
	default:
		dev_err(qunipcs->dev, "Force speed %d not supported\n", speed);
		return;
	}

pcs_adapter_reset:
	/* PCS channel adapter reset */
	ipq_unipcs_reg_modify32(qunipcs, PCS_CHANNEL_CTRL(channel),
				PCS_CHANNEL_ADPT_RESET,
				0);
	ipq_unipcs_reg_modify32(qunipcs, PCS_CHANNEL_CTRL(channel),
				PCS_CHANNEL_ADPT_RESET,
				PCS_CHANNEL_ADPT_RESET);
}

static void ipq_unipcs_link_up_config_2500basex(struct ipq_uniphy_pcs *qunipcs,
						int channel,
						int speed)
{
	/* 2500BASEX do not support autoneg and do not need to
	 * configure PCS speed, only reset PCS adapter here.
	 */
	ipq_unipcs_reg_modify32(qunipcs, PCS_CHANNEL_CTRL(channel),
				PCS_CHANNEL_ADPT_RESET,
				0);
	ipq_unipcs_reg_modify32(qunipcs, PCS_CHANNEL_CTRL(channel),
				PCS_CHANNEL_ADPT_RESET,
				PCS_CHANNEL_ADPT_RESET);
}

static void ipq_unipcs_link_up_config_usxgmii(struct ipq_uniphy_pcs *qunipcs,
					      int channel,
					      int speed)
{
	u32 val, reg;

	switch (speed) {
	case SPEED_10000:
		val = XPCS_SPEED_10000;
		break;
	case SPEED_5000:
		val = XPCS_SPEED_5000;
		break;
	case SPEED_2500:
		val = XPCS_SPEED_2500;
		break;
	case SPEED_1000:
		val = XPCS_SPEED_1000;
		break;
	case SPEED_100:
		val = XPCS_SPEED_100;
		break;
	case SPEED_10:
		val = XPCS_SPEED_10;
		break;
	default:
		return;
	}

	/* USXGMII only support full duplex mode */
	val |= XPCS_DUPLEX_FULL;

	/* Config XPCS speed */
	reg = (channel == 0) ? XPCS_MII_CTRL : XPCS_CHANNEL_MII_CTRL(channel);
	ipq_unipcs_reg_modify32(qunipcs, reg,
				XPCS_SPEED_MASK | XPCS_DUPLEX_FULL,
				val);

	/* XPCS adapter reset */
	if (channel == 0)
		ipq_unipcs_reg_modify32(qunipcs, XPCS_DIG_CTRL,
					XPCS_USXG_ADPT_RESET,
					XPCS_USXG_ADPT_RESET);
	else
		ipq_unipcs_reg_modify32(qunipcs, XPCS_CHANNEL_DIG_CTRL(channel),
					XPCS_CHANNEL_USXG_ADPT_RESET,
					XPCS_CHANNEL_USXG_ADPT_RESET);
}

static int ipq_unipcs_validate(struct phylink_pcs *pcs,
			       unsigned long *supported,
			       const struct phylink_link_state *state)
{
	/* In-band autoneg is not supported for 2500BASEX */
	if (state->interface == PHY_INTERFACE_MODE_2500BASEX)
		phylink_clear(supported, Autoneg);

	return 0;
}

static void ipq_unipcs_get_state(struct phylink_pcs *pcs,
				 struct phylink_link_state *state)
{
	struct ipq_uniphy_pcs_ch *qunipcs_ch = phylink_pcs_to_unipcs(pcs);
	struct ipq_uniphy_pcs *qunipcs = qunipcs_ch->qunipcs;
	int channel = qunipcs_ch->channel;

	switch (state->interface) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
	case PHY_INTERFACE_MODE_PSGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
		/* SGMII and 1000BASEX in-band autoneg word format are decoded
		 * by PCS hardware and both placed to the same status register.
		 */
		ipq_unipcs_get_state_sgmii(qunipcs, channel, state);
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
		ipq_unipcs_get_state_2500basex(qunipcs, channel, state);
		break;
	case PHY_INTERFACE_MODE_USXGMII:
	case PHY_INTERFACE_MODE_10G_QXGMII:
		ipq_unipcs_get_state_usxgmii(qunipcs, channel, state);
		break;
	case PHY_INTERFACE_MODE_10GBASER:
		ipq_unipcs_get_state_10gbaser(qunipcs, state);
		break;
	default:
		break;
	}

	dev_dbg(qunipcs->dev,
		"mode=%s/%s/%s link=%u\n",
		phy_modes(state->interface),
		phy_speed_to_str(state->speed),
		phy_duplex_to_str(state->duplex),
		state->link);
}

static int ipq_unipcs_config(struct phylink_pcs *pcs,
			     unsigned int neg_mode,
			     phy_interface_t interface,
			     const unsigned long *advertising,
			     bool permit)
{
	struct ipq_uniphy_pcs_ch *qunipcs_ch = phylink_pcs_to_unipcs(pcs);
	struct ipq_uniphy_pcs *qunipcs = qunipcs_ch->qunipcs;
	int channel = qunipcs_ch->channel;

	switch (interface) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
	case PHY_INTERFACE_MODE_PSGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
		return ipq_unipcs_config_sgmii(qunipcs, channel,
					       neg_mode, interface);
	case PHY_INTERFACE_MODE_2500BASEX:
		return ipq_unipcs_config_2500basex(qunipcs, interface);
	case PHY_INTERFACE_MODE_USXGMII:
	case PHY_INTERFACE_MODE_10G_QXGMII:
		return ipq_unipcs_config_usxgmii(qunipcs, channel,
						 neg_mode, interface);
	case PHY_INTERFACE_MODE_10GBASER:
		return ipq_unipcs_config_10gbaser(qunipcs, interface);
	default:
		dev_err(qunipcs->dev,
			"interface %s not supported\n", phy_modes(interface));
		return -EOPNOTSUPP;
	};
}

static void qcom_ipq_unipcs_an_restart(struct phylink_pcs *pcs)
{
	/* Currently not used */
}

static void ipq_unipcs_link_up(struct phylink_pcs *pcs,
			       unsigned int neg_mode,
			       phy_interface_t interface,
			       int speed, int duplex)
{
	struct ipq_uniphy_pcs_ch *qunipcs_ch = phylink_pcs_to_unipcs(pcs);
	struct ipq_uniphy_pcs *qunipcs = qunipcs_ch->qunipcs;
	int channel = qunipcs_ch->channel;

	/* Configure PCS channel interface clock rate */
	ipq_unipcs_link_up_clock_rate_set(qunipcs_ch, interface, speed);

	/* Configure PCS speed and reset PCS adapter */
	switch (interface) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
	case PHY_INTERFACE_MODE_PSGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
		ipq_unipcs_link_up_config_sgmii(qunipcs, channel,
						neg_mode, speed);
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
		ipq_unipcs_link_up_config_2500basex(qunipcs,
						    channel, speed);
		break;
	case PHY_INTERFACE_MODE_USXGMII:
	case PHY_INTERFACE_MODE_10G_QXGMII:
		ipq_unipcs_link_up_config_usxgmii(qunipcs, channel, speed);
		break;
	case PHY_INTERFACE_MODE_10GBASER:
		break;
	default:
		dev_err(qunipcs->dev,
			"interface %s not supported\n", phy_modes(interface));
		break;
	}
}

static const struct phylink_pcs_ops ipq_unipcs_phylink_ops = {
	.pcs_validate = ipq_unipcs_validate,
	.pcs_get_state = ipq_unipcs_get_state,
	.pcs_config = ipq_unipcs_config,
	.pcs_an_restart = qcom_ipq_unipcs_an_restart,
	.pcs_link_up = ipq_unipcs_link_up,
};

/**
 * ipq_unipcs_create() - Create Qualcomm IPQ UNIPHY PCS
 * @np: Device tree node to the PCS
 *
 * Description: Create a phylink PCS instance for a PCS node @np.
 *
 * Return: A pointer to the phylink PCS instance or an error-pointer value.
 */
struct phylink_pcs *ipq_unipcs_create(struct device_node *np)
{
	struct ipq_uniphy_pcs_ch *qunipcs_ch;
	struct ipq_uniphy_pcs *qunipcs;
	struct device_node *uniphy_np;
	struct platform_device *pdev;
	u32 channel;
	int i, j;

	if (!of_device_is_available(np))
		return ERR_PTR(-ENODEV);

	if (of_property_read_u32(np, "reg", &channel))
		return ERR_PTR(-EINVAL);

	if (channel >= PCS_MAX_CHANNELS)
		return ERR_PTR(-EINVAL);

	uniphy_np = of_get_parent(np);
	if (!uniphy_np)
		return ERR_PTR(-ENODEV);

	if (!of_device_is_available(uniphy_np)) {
		of_node_put(uniphy_np);
		return ERR_PTR(-ENODEV);
	}

	pdev = of_find_device_by_node(uniphy_np);
	of_node_put(uniphy_np);
	if (!pdev)
		return ERR_PTR(-ENODEV);

	qunipcs = platform_get_drvdata(pdev);
	platform_device_put(pdev);

	/* If probe is not yet completed, return DEFER to
	 * the dependent driver.
	 */
	if (!qunipcs)
		return ERR_PTR(-EPROBE_DEFER);

	qunipcs_ch = kzalloc(sizeof(*qunipcs_ch), GFP_KERNEL);
	if (!qunipcs_ch)
		return ERR_PTR(-ENOMEM);

	qunipcs_ch->qunipcs = qunipcs;
	qunipcs_ch->channel = channel;
	qunipcs_ch->pcs.ops = &ipq_unipcs_phylink_ops;
	qunipcs_ch->pcs.neg_mode = true;
	qunipcs_ch->pcs.poll = true;

	for (i = 0; i < PCS_CH_CLK_MAX; i++) {
		qunipcs_ch->clk[i] = of_clk_get_by_name(np,
							pcs_ch_clock_name[i]);
		if (IS_ERR(qunipcs_ch->clk[i])) {
			dev_err(qunipcs->dev,
				"Failed to get PCS channel %d clock ID %s\n",
				channel, pcs_ch_clock_name[i]);
			goto free_pcs;
		}

		clk_prepare_enable(qunipcs_ch->clk[i]);
	}

	return &qunipcs_ch->pcs;

free_pcs:
	for (j = 0; j < i; j++) {
		clk_disable_unprepare(qunipcs_ch->clk[j]);
		clk_put(qunipcs_ch->clk[j]);
	}

	kfree(qunipcs_ch);
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL(ipq_unipcs_create);

/**
 * ipq_unipcs_destroy() - Destroy Qualcomm IPQ UNIPHY PCS
 * @pcs: PCS instance
 *
 * Description: Destroy a phylink PCS instance.
 */
void ipq_unipcs_destroy(struct phylink_pcs *pcs)
{
	struct ipq_uniphy_pcs_ch *qunipcs_ch;
	int i;

	if (!pcs)
		return;

	qunipcs_ch = phylink_pcs_to_unipcs(pcs);

	for (i = 0; i < PCS_CH_CLK_MAX; i++) {
		clk_disable_unprepare(qunipcs_ch->clk[i]);
		clk_put(qunipcs_ch->clk[i]);
	}

	kfree(qunipcs_ch);
}
EXPORT_SYMBOL(ipq_unipcs_destroy);

static int ipq_uniphy_clk_register(struct ipq_uniphy_pcs *qunipcs)
{
	struct ipq_unipcs_raw_clk *raw_clk;
	struct device *dev = qunipcs->dev;
	struct clk_hw_onecell_data *data;

	struct clk_init_data init = { };
	int i, ret;

	data = devm_kzalloc(dev,
			    struct_size(data, hws, PCS_RAW_CLK_MAX),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->num = PCS_RAW_CLK_MAX;
	for (i = 0; i < PCS_RAW_CLK_MAX; i++) {
		ret = of_property_read_string_index(dev->of_node,
						    "clock-output-names",
						    i, &init.name);
		if (ret) {
			dev_err(dev,
				"%pOFn: No clock-output-names\n", dev->of_node);
			return ret;
		}

		init.ops = &ipq_unipcs_raw_clk_ops;
		raw_clk = &qunipcs->raw_clk[i];

		raw_clk->rate = 125000000;
		raw_clk->hw.init = &init;

		ret = devm_clk_hw_register(dev, &raw_clk->hw);
		if (ret) {
			dev_err(dev, "Failed to register UNIPHY PCS raw clock\n");
			return ret;
		}

		data->hws[i] = &raw_clk->hw;
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
}

static int ipq_uniphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ipq_uniphy_pcs *priv;
	int i, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	for (i = 0; i < PCS_CLK_MAX; i++) {
		priv->clk[i] = devm_clk_get_optional_enabled(dev,
							     pcs_clock_name[i]);

		if (IS_ERR(priv->clk[i]))
			dev_err(dev, "Failed to get the clock ID %s\n",
				pcs_clock_name[i]);
	}

	for (i = 0; i < PCS_RESET_MAX; i++) {
		priv->reset[i] =
			devm_reset_control_get_optional_exclusive(dev,
								  pcs_reset_name[i]);

		if (IS_ERR(priv->reset[i]))
			dev_err(dev, "Failed to get the reset ID %s\n",
				pcs_reset_name[i]);
	}

	/* Set UNIPHY PCS system and AHB clock rate */
	clk_set_rate(priv->clk[PCS_SYS_CLK], 24000000);
	clk_set_rate(priv->clk[PCS_AHB_CLK], 100000000);

	ret = ipq_uniphy_clk_register(priv);
	if (ret)
		return ret;

	mutex_init(&priv->shared_lock);
	spin_lock_init(&priv->reg_lock);

	platform_set_drvdata(pdev, priv);

	return 0;
}

static const struct of_device_id ipq_uniphy_of_mtable[] = {
	{ .compatible = "qcom,ipq5332-uniphy" },
	{ .compatible = "qcom,ipq9574-uniphy" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ipq_uniphy_of_mtable);

static struct platform_driver ipq_uniphy_driver = {
	.driver = {
		.name = "ipq_uniphy",
		.of_match_table = ipq_uniphy_of_mtable,
	},
	.probe = ipq_uniphy_probe,
};
module_platform_driver(ipq_uniphy_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Qualcomm IPQ UNIPHY PCS driver");
MODULE_AUTHOR("Lei Wei <quic_leiwei@quicinc.com>");
