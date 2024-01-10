// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* PPE UNIPHY clock register and UNIPHY PCS operations for phylink.
 *
 * The PPE UNIPHY block is specifically used by PPE to connect the PPE MAC
 * with the external PHYs or SFPs or Switches (fixed link). The PPE UNIPHY
 * block includes serdes, PCS or XPCS and the control logic to support PPE
 * ports to work in different interface mode and different link speed.
 *
 * The PPE UNIPHY block provides raw clock as the parent clock to NSSCC
 * clocks and the NSSCC clocks can be configured to generate different
 * port Tx and Rx clocks to PPE ports in different port link speed.
 */

#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/clk-provider.h>
#include <linux/soc/qcom/ppe.h>
#include "ppe.h"
#include "ppe_uniphy.h"

/* UNIPHY clock direction */
enum {
	UNIPHY_RX = 0,
	UNIPHY_TX,
};

/* UNIPHY clock data type */
struct clk_uniphy {
	struct clk_hw hw;
	u8 index;
	u8 dir;
	unsigned long rate;
};

#define to_clk_uniphy(_hw)		container_of(_hw, struct clk_uniphy, hw)
/* UNIPHY clock rate */
#define UNIPHY_CLK_RATE_125M		125000000
#define UNIPHY_CLK_RATE_312P5M		312500000

static void ppe_uniphy_write(struct ppe_uniphy *uniphy, u32 val, u32 reg)
{
	if (reg >= UNIPHY_INDIRECT_ADDR_START) {
		writel(FIELD_GET(UNIPHY_INDIRECT_ADDR_HIGH, reg),
		       uniphy->base + UNIPHY_INDIRECT_AHB_ADDR);
		writel(val, uniphy->base + UNIPHY_INDIRECT_DATA_ADDR(reg));
	} else {
		writel(val, uniphy->base + reg);
	}
}

static u32 ppe_uniphy_read(struct ppe_uniphy *uniphy, u32 reg)
{
	if (reg >= UNIPHY_INDIRECT_ADDR_START) {
		writel(FIELD_GET(UNIPHY_INDIRECT_ADDR_HIGH, reg),
		       uniphy->base + UNIPHY_INDIRECT_AHB_ADDR);
		return readl(uniphy->base + UNIPHY_INDIRECT_DATA_ADDR(reg));
	} else {
		return readl(uniphy->base + reg);
	}
}

static int ppe_uniphy_mask(struct ppe_uniphy *uniphy, u32 reg, u32 mask, u32 set)
{
	u32 val;

	val = ppe_uniphy_read(uniphy, reg);
	val &= ~mask;
	val |= set;
	ppe_uniphy_write(uniphy, val, reg);

	return 0;
}

static unsigned long clk_uniphy_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct clk_uniphy *uniphy = to_clk_uniphy(hw);

	return uniphy->rate;
}

static int clk_uniphy_determine_rate(struct clk_hw *hw,
				     struct clk_rate_request *req)
{
	if (req->rate <= UNIPHY_CLK_RATE_125M)
		req->rate = UNIPHY_CLK_RATE_125M;
	else
		req->rate = UNIPHY_CLK_RATE_312P5M;

	return 0;
}

static int clk_uniphy_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct clk_uniphy *uniphy = to_clk_uniphy(hw);

	if (rate != UNIPHY_CLK_RATE_125M && rate != UNIPHY_CLK_RATE_312P5M)
		return -1;

	uniphy->rate = rate;

	return 0;
}

static const struct clk_ops clk_uniphy_ops = {
	.recalc_rate = clk_uniphy_recalc_rate,
	.determine_rate = clk_uniphy_determine_rate,
	.set_rate = clk_uniphy_set_rate,
};

static struct clk_uniphy uniphy0_gcc_rx_clk = {
	.hw.init = &(struct clk_init_data){
		.name = "uniphy0_gcc_rx_clk",
		.ops = &clk_uniphy_ops,
	},
	.index = 0,
	.dir = UNIPHY_RX,
	.rate = UNIPHY_CLK_RATE_125M,
};

static struct clk_uniphy uniphy0_gcc_tx_clk = {
	.hw.init = &(struct clk_init_data){
		.name = "uniphy0_gcc_tx_clk",
		.ops = &clk_uniphy_ops,
	},
	.index = 0,
	.dir = UNIPHY_TX,
	.rate = UNIPHY_CLK_RATE_125M,
};

static struct clk_uniphy uniphy1_gcc_rx_clk = {
	.hw.init = &(struct clk_init_data){
		.name = "uniphy1_gcc_rx_clk",
		.ops = &clk_uniphy_ops,
	},
	.index = 1,
	.dir = UNIPHY_RX,
	.rate = UNIPHY_CLK_RATE_312P5M,
};

static struct clk_uniphy uniphy1_gcc_tx_clk = {
	.hw.init = &(struct clk_init_data){
		.name = "uniphy1_gcc_tx_clk",
		.ops = &clk_uniphy_ops,
	},
	.index = 1,
	.dir = UNIPHY_TX,
	.rate = UNIPHY_CLK_RATE_312P5M,
};

static struct clk_uniphy uniphy2_gcc_rx_clk = {
	.hw.init = &(struct clk_init_data){
		.name = "uniphy2_gcc_rx_clk",
		.ops = &clk_uniphy_ops,
	},
	.index = 2,
	.dir = UNIPHY_RX,
	.rate = UNIPHY_CLK_RATE_312P5M,
};

static struct clk_uniphy uniphy2_gcc_tx_clk = {
	.hw.init = &(struct clk_init_data){
		.name = "uniphy2_gcc_tx_clk",
		.ops = &clk_uniphy_ops,
	},
	.index = 2,
	.dir = UNIPHY_TX,
	.rate = UNIPHY_CLK_RATE_312P5M,
};

static struct clk_hw *uniphy_raw_clks[] = {
	&uniphy0_gcc_rx_clk.hw, &uniphy0_gcc_tx_clk.hw,
	&uniphy1_gcc_rx_clk.hw, &uniphy1_gcc_tx_clk.hw,
	&uniphy2_gcc_rx_clk.hw, &uniphy2_gcc_tx_clk.hw,
};

int ppe_uniphy_port_gcc_clock_en_set(struct ppe_uniphy *uniphy, int port, bool enable)
{
	struct clk **clock = ppe_clock_get(uniphy->ppe_dev);
	enum ppe_clk_id rx_id, tx_id;
	int err = 0;

	rx_id = PPE_UNIPHY_PORT1_RX_CLK + ((port - 1) << 1);
	tx_id = PPE_UNIPHY_PORT1_TX_CLK + ((port - 1) << 1);

	if (enable) {
		if (!IS_ERR(clock[rx_id])) {
			err = clk_prepare_enable(clock[rx_id]);
			if (err) {
				dev_err(uniphy->ppe_dev->dev,
					"Failed to enable uniphy port %d rx_clk(%d)\n",
					port, rx_id);
				return err;
			}
		}

		if (!IS_ERR(clock[tx_id])) {
			err = clk_prepare_enable(clock[tx_id]);
			if (err) {
				dev_err(uniphy->ppe_dev->dev,
					"Failed to enable uniphy port %d tx_clk(%d)\n",
					port, tx_id);
				return err;
			}
		}
	} else {
		clk_disable_unprepare(clock[rx_id]);
		clk_disable_unprepare(clock[tx_id]);
	}

	return 0;
}

static int ppe_uniphy_interface_gcc_clock_en_set(struct ppe_uniphy *uniphy, bool enable)
{
	int ppe_type = ppe_type_get(uniphy->ppe_dev);
	int port = 0;

	switch (uniphy->index) {
	case 2:
		ppe_uniphy_port_gcc_clock_en_set(uniphy, PPE_PORT6, enable);
		break;
	case 1:
		if (ppe_type == PPE_TYPE_APPE)
			ppe_uniphy_port_gcc_clock_en_set(uniphy, PPE_PORT5, enable);
		else if (ppe_type == PPE_TYPE_MPPE)
			ppe_uniphy_port_gcc_clock_en_set(uniphy, PPE_PORT2, enable);
		break;
	case 0:
		if (ppe_type == PPE_TYPE_APPE) {
			for (port = PPE_PORT1; port <= PPE_PORT4; port++)
				ppe_uniphy_port_gcc_clock_en_set(uniphy, port, enable);
		} else if (ppe_type == PPE_TYPE_MPPE) {
			ppe_uniphy_port_gcc_clock_en_set(uniphy, PPE_PORT1, enable);
		}
		break;
	default:
		break;
	}

	return 0;
}

static int ppe_uniphy_gcc_xpcs_reset(struct ppe_uniphy *uniphy, bool enable)
{
	struct reset_control **reset = ppe_reset_get(uniphy->ppe_dev);
	enum ppe_rst_id id = PPE_UNIPHY0_XPCS_RST + uniphy->index;

	if (IS_ERR(reset[id]))
		return PTR_ERR(reset[id]);

	if (enable)
		return reset_control_assert(reset[id]);
	else
		return reset_control_deassert(reset[id]);
}

static int ppe_uniphy_gcc_software_reset(struct ppe_uniphy *uniphy)
{
	struct reset_control **reset = ppe_reset_get(uniphy->ppe_dev);
	int ppe_type = ppe_type_get(uniphy->ppe_dev);
	unsigned int index = uniphy->index;
	int err = 0, port = 0;

	/* Assert uniphy sys reset control */
	if (!IS_ERR(reset[PPE_UNIPHY0_SYS_RST + index])) {
		err = reset_control_assert(reset[PPE_UNIPHY0_SYS_RST + index]);
		if (err)
			return err;
	}

	/* Assert uniphy port reset control */
	switch (ppe_type) {
	case PPE_TYPE_APPE:
		if (index == 0) {
			for (port = PPE_PORT1; port <= PPE_PORT4; port++) {
				if (!IS_ERR(reset[PPE_UNIPHY_PORT1_DIS + port - 1])) {
					err = reset_control_assert(reset[PPE_UNIPHY_PORT1_DIS +
								   port - 1]);
					if (err)
						return err;
				}
			}
		} else {
			if (!IS_ERR(reset[PPE_UNIPHY0_SOFT_RST + index])) {
				err = reset_control_assert(reset[PPE_UNIPHY0_SOFT_RST + index]);
				if (err)
					return err;
			}
		}
		break;
	case PPE_TYPE_MPPE:
		if (!IS_ERR(reset[PPE_UNIPHY_PORT1_RX_RST + (index << 1)])) {
			err = reset_control_assert(reset[PPE_UNIPHY_PORT1_RX_RST + (index << 1)]);
			if (err)
				return err;
		}

		if (!IS_ERR(reset[PPE_UNIPHY_PORT1_TX_RST + (index << 1)])) {
			err = reset_control_assert(reset[PPE_UNIPHY_PORT1_TX_RST + (index << 1)]);
			if (err)
				return err;
		}
		break;
	default:
		break;
	}
	fsleep(100000);

	/* Deassert uniphy sys reset control */
	if (!IS_ERR(reset[PPE_UNIPHY0_SYS_RST + index])) {
		err = reset_control_deassert(reset[PPE_UNIPHY0_SYS_RST + index]);
		if (err)
			return err;
	}

	/* Deassert uniphy port reset control */
	switch (ppe_type) {
	case PPE_TYPE_APPE:
		if (index == 0) {
			for (port = PPE_PORT1; port <= PPE_PORT4; port++) {
				if (!IS_ERR(reset[PPE_UNIPHY_PORT1_DIS + port - 1])) {
					err = reset_control_deassert(reset[PPE_UNIPHY_PORT1_DIS +
								     port - 1]);
					if (err)
						return err;
				}
			}
		} else {
			if (!IS_ERR(reset[PPE_UNIPHY0_SOFT_RST + index])) {
				err = reset_control_deassert(reset[PPE_UNIPHY0_SOFT_RST + index]);
				if (err)
					return err;
			}
		}
		break;
	case PPE_TYPE_MPPE:
		if (!IS_ERR(reset[PPE_UNIPHY_PORT1_RX_RST + (index << 1)])) {
			err = reset_control_deassert(reset[PPE_UNIPHY_PORT1_RX_RST + (index << 1)]);
			if (err)
				return err;
		}

		if (!IS_ERR(reset[PPE_UNIPHY_PORT1_TX_RST + (index << 1)])) {
			err = reset_control_deassert(reset[PPE_UNIPHY_PORT1_TX_RST + (index << 1)]);
			if (err)
				return err;
		}
		break;
	default:
		break;
	}
	fsleep(100000);

	return err;
}

int ppe_uniphy_autoneg_complete_check(struct ppe_uniphy *uniphy, int port)
{
	u32 reg, val;
	int channel, ret;

	if (uniphy->interface == PHY_INTERFACE_MODE_USXGMII ||
	    uniphy->interface == PHY_INTERFACE_MODE_QUSGMII) {
		/* Only uniphy0 may have multi channels */
		channel = (uniphy->index == 0) ? (port - 1) : 0;
		reg = (channel == 0) ? VR_MII_AN_INTR_STS_ADDR :
		       VR_MII_AN_INTR_STS_CHANNEL_ADDR(channel);

		/* Wait auto negotiation complete */
		ret = read_poll_timeout(ppe_uniphy_read, val,
					(val & CL37_ANCMPLT_INTR),
					1000, 100000, true,
					uniphy, reg);
		if (ret) {
			dev_err(uniphy->ppe_dev->dev,
				"uniphy %d auto negotiation timeout\n", uniphy->index);
			return ret;
		}

		/* Clear auto negotiation complete interrupt */
		ppe_uniphy_mask(uniphy, reg, CL37_ANCMPLT_INTR, 0);
	}

	return 0;
}

int ppe_uniphy_speed_set(struct ppe_uniphy *uniphy, int port, int speed)
{
	u32 reg, val;
	int channel;

	if (uniphy->interface == PHY_INTERFACE_MODE_USXGMII ||
	    uniphy->interface == PHY_INTERFACE_MODE_QUSGMII) {
		/* Only uniphy0 may have multiple channels */
		channel = (uniphy->index == 0) ? (port - 1) : 0;

		reg = (channel == 0) ? SR_MII_CTRL_ADDR :
		       SR_MII_CTRL_CHANNEL_ADDR(channel);

		switch (speed) {
		case SPEED_100:
			val = USXGMII_SPEED_100;
			break;
		case SPEED_1000:
			val = USXGMII_SPEED_1000;
			break;
		case SPEED_2500:
			val = USXGMII_SPEED_2500;
			break;
		case SPEED_5000:
			val = USXGMII_SPEED_5000;
			break;
		case SPEED_10000:
			val = USXGMII_SPEED_10000;
			break;
		case SPEED_10:
			val = USXGMII_SPEED_10;
			break;
		default:
			val = 0;
			break;
		}

		ppe_uniphy_mask(uniphy, reg, USXGMII_SPEED_MASK, val);
	}

	return 0;
}

int ppe_uniphy_duplex_set(struct ppe_uniphy *uniphy, int port, int duplex)
{
	u32 reg;
	int channel;

	if (uniphy->interface == PHY_INTERFACE_MODE_USXGMII &&
	    uniphy->interface == PHY_INTERFACE_MODE_QUSGMII) {
		/* Only uniphy0 may have multiple channels */
		channel = (uniphy->index == 0) ? (port - 1) : 0;

		reg = (channel == 0) ? SR_MII_CTRL_ADDR :
		       SR_MII_CTRL_CHANNEL_ADDR(channel);

		ppe_uniphy_mask(uniphy, reg, USXGMII_DUPLEX_FULL,
				(duplex == DUPLEX_FULL) ? USXGMII_DUPLEX_FULL : 0);
	}

	return 0;
}

int ppe_uniphy_adapter_reset(struct ppe_uniphy *uniphy, int port)
{
	int channel;

	/* Only uniphy0 may have multiple channels */
	channel = (uniphy->index == 0) ? (port - 1) : 0;

	switch (uniphy->interface) {
	case PHY_INTERFACE_MODE_USXGMII:
	case PHY_INTERFACE_MODE_QUSGMII:
		if (channel == 0)
			ppe_uniphy_mask(uniphy,
					VR_XS_PCS_DIG_CTRL1_ADDR,
					USRA_RST, USRA_RST);
		else
			ppe_uniphy_mask(uniphy,
					VR_MII_DIG_CTRL1_CHANNEL_ADDR(channel),
					CHANNEL_USRA_RST, CHANNEL_USRA_RST);
		break;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
	case PHY_INTERFACE_MODE_2500BASEX:
	case PHY_INTERFACE_MODE_QSGMII:
		ppe_uniphy_mask(uniphy,
				UNIPHY_CHANNEL_INPUT_OUTPUT_4_ADDR(channel),
				NEWADDEDFROMHERE_CH_ADP_SW_RSTN, 0);
		ppe_uniphy_mask(uniphy,
				UNIPHY_CHANNEL_INPUT_OUTPUT_4_ADDR(channel),
				NEWADDEDFROMHERE_CH_ADP_SW_RSTN,
				NEWADDEDFROMHERE_CH_ADP_SW_RSTN);
		break;
	default:
		break;
	}

	return 0;
}

static int ppe_pcs_config(struct phylink_pcs *pcs, unsigned int mode,
			  phy_interface_t interface,
			  const unsigned long *advertising,
			  bool permit_pause_to_mac)
{
	struct ppe_uniphy *uniphy = pcs_to_ppe_uniphy(pcs);
	unsigned long rate = 0;
	int ret, channel = 0;
	u32 val = 0;

	if (uniphy->interface == interface)
		return 0;

	uniphy->interface = interface;

	/* Disable gcc uniphy interface clock */
	ppe_uniphy_interface_gcc_clock_en_set(uniphy, false);

	/* Assert gcc uniphy xpcs reset control */
	ppe_uniphy_gcc_xpcs_reset(uniphy, true);

	/* Configure uniphy mode */
	switch (interface) {
	case PHY_INTERFACE_MODE_USXGMII:
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_QUSGMII:
		rate = UNIPHY_CLK_RATE_312P5M;
		ppe_uniphy_mask(uniphy, UNIPHY_MODE_CTRL_ADDR,
				USXGMII_MODE_CTRL_MASK, USXGMII_MODE_CTRL);
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
		rate = UNIPHY_CLK_RATE_312P5M;
		ppe_uniphy_mask(uniphy, UNIPHY_MODE_CTRL_ADDR,
				SGMIIPLUS_MODE_CTRL_MASK, SGMIIPLUS_MODE_CTRL);
		break;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
		rate = UNIPHY_CLK_RATE_125M;
		ppe_uniphy_mask(uniphy, UNIPHY_MODE_CTRL_ADDR,
				SGMII_MODE_CTRL_MASK, SGMII_MODE_CTRL);
		break;
	case PHY_INTERFACE_MODE_QSGMII:
		rate = UNIPHY_CLK_RATE_125M;
		ppe_uniphy_mask(uniphy, UNIPHY_MODE_CTRL_ADDR,
				QSGMII_MODE_CTRL_MASK, QSGMII_MODE_CTRL);
		break;
	default:
		break;
	}

	if (interface == PHY_INTERFACE_MODE_QUSGMII)
		ppe_uniphy_mask(uniphy, UNIPHY_QP_USXG_OPITON1_ADDR,
				GMII_SRC_SEL, GMII_SRC_SEL);

	if (interface == PHY_INTERFACE_MODE_10GBASER)
		ppe_uniphy_mask(uniphy, UNIPHY_LINK_DETECT_ADDR,
				DETECT_LOS_FROM_SFP, UNIPHY_10GR_LINK_LOSS);

	/* Reset uniphy gcc software reset control */
	ppe_uniphy_gcc_software_reset(uniphy);

	/* Wait uniphy calibration completion */
	ret = read_poll_timeout(ppe_uniphy_read, val,
				(val & MMD1_REG_CALIBRATION_DONE_REG),
				1000, 100000, true,
				uniphy, UNIPHY_OFFSET_CALIB_4_ADDR);
	if (ret) {
		dev_err(uniphy->ppe_dev->dev,
			"uniphy %d calibration timeout\n", uniphy->index);
		return ret;
	}

	/* Enable gcc uniphy interface clk */
	ppe_uniphy_interface_gcc_clock_en_set(uniphy, true);

	/* Deassert gcc uniphy xpcs reset control */
	if (interface == PHY_INTERFACE_MODE_USXGMII ||
	    interface == PHY_INTERFACE_MODE_10GBASER ||
		interface == PHY_INTERFACE_MODE_QUSGMII)
		ppe_uniphy_gcc_xpcs_reset(uniphy, false);

	if (interface == PHY_INTERFACE_MODE_USXGMII ||
	    interface == PHY_INTERFACE_MODE_QUSGMII) {
		/* Wait 10gr link up */
		ret = read_poll_timeout(ppe_uniphy_read, val,
					(val & SR_XS_PCS_KR_STS1_PLU),
					1000, 100000, true,
					uniphy, SR_XS_PCS_KR_STS1_ADDR);
		if (ret)
			dev_warn(uniphy->ppe_dev->dev,
				 "uniphy %d 10gr linkup timeout\n", uniphy->index);

		/* Enable usxgmii */
		ppe_uniphy_mask(uniphy, VR_XS_PCS_DIG_CTRL1_ADDR, USXGMII_EN, USXGMII_EN);

		if (interface == PHY_INTERFACE_MODE_QUSGMII) {
			/* XPCS set quxgmii mode */
			ppe_uniphy_mask(uniphy, VR_XS_PCS_DIG_STS_ADDR, AM_COUNT, QUXGMII_AM_COUNT);
			ppe_uniphy_mask(uniphy, VR_XS_PCS_KR_CTRL_ADDR, USXG_MODE, QUXGMII_MODE);
			/* XPCS software reset */
			ppe_uniphy_mask(uniphy, VR_XS_PCS_DIG_CTRL1_ADDR, VR_RST, VR_RST);
		}

		/* Enable autoneg complete interrupt and 10M/100M 8bit mii width */
		ppe_uniphy_mask(uniphy, VR_MII_AN_CTRL_ADDR,
				MII_AN_INTR_EN | MII_CTRL, MII_AN_INTR_EN | MII_CTRL);

		if (interface == PHY_INTERFACE_MODE_QUSGMII) {
			for (channel = 1; channel <= 3; channel++)
				ppe_uniphy_mask(uniphy, VR_MII_AN_CTRL_CHANNEL_ADDR(channel),
						MII_AN_INTR_EN | MII_CTRL,
						MII_AN_INTR_EN | MII_CTRL);
			/* Disable TICD */
			ppe_uniphy_mask(uniphy, VR_XAUI_MODE_CTRL_ADDR, IPG_CHECK, IPG_CHECK);
			for (channel = 1; channel <= 3; channel++)
				ppe_uniphy_mask(uniphy, VR_XAUI_MODE_CTRL_CHANNEL_ADDR(channel),
						IPG_CHECK, IPG_CHECK);
		}

		/* Enable autoneg ability and usxgmii 10g speed and full duplex */
		ppe_uniphy_mask(uniphy, SR_MII_CTRL_ADDR,
				USXGMII_SPEED_MASK | AN_ENABLE | USXGMII_DUPLEX_FULL,
				USXGMII_SPEED_10000 | AN_ENABLE | USXGMII_DUPLEX_FULL);
		if (interface == PHY_INTERFACE_MODE_QUSGMII) {
			for (channel = 1; channel <= 3; channel++)
				ppe_uniphy_mask(uniphy, SR_MII_CTRL_CHANNEL_ADDR(channel),
						USXGMII_SPEED_MASK | AN_ENABLE |
						USXGMII_DUPLEX_FULL,
						USXGMII_SPEED_10000 | AN_ENABLE |
						USXGMII_DUPLEX_FULL);

			/* Enable eee transparent mode */
			ppe_uniphy_mask(uniphy, VR_XS_PCS_EEE_MCTRL0_ADDR,
					MULT_FACT_100NS | SIGN_BIT,
					FIELD_PREP(MULT_FACT_100NS, 0x1) | SIGN_BIT);
			ppe_uniphy_mask(uniphy, VR_XS_PCS_EEE_TXTIMER_ADDR,
					TSL_RES | T1U_RES | TWL_RES,
					UNIPHY_XPCS_TSL_TIMER |
					UNIPHY_XPCS_T1U_TIMER | UNIPHY_XPCS_TWL_TIMER);
			ppe_uniphy_mask(uniphy, VR_XS_PCS_EEE_RXTIMER_ADDR,
					RES_100U | TWR_RES,
					UNIPHY_XPCS_100US_TIMER | UNIPHY_XPCS_TWR_TIMER);

			ppe_uniphy_mask(uniphy, VR_XS_PCS_EEE_MCTRL1_ADDR,
					TRN_LPI | TRN_RXLPI, TRN_LPI | TRN_RXLPI);
			ppe_uniphy_mask(uniphy, VR_XS_PCS_EEE_MCTRL0_ADDR,
					LTX_EN | LRX_EN, LTX_EN | LRX_EN);
		}
	}

	/* Set uniphy raw clk rate */
	clk_set_rate(uniphy_raw_clks[(uniphy->index << 1) + UNIPHY_RX]->clk,
		     rate);
	clk_set_rate(uniphy_raw_clks[(uniphy->index << 1) + UNIPHY_TX]->clk,
		     rate);

	dev_info(uniphy->ppe_dev->dev,
		 "ppe pcs config uniphy index %d, interface %s\n",
		 uniphy->index, phy_modes(interface));

	return 0;
}

static void ppe_pcs_get_state(struct phylink_pcs *pcs,
			      struct phylink_link_state *state)
{
	struct ppe_uniphy *uniphy = pcs_to_ppe_uniphy(pcs);
	u32 val;

	switch (state->interface) {
	case PHY_INTERFACE_MODE_10GBASER:
		val = ppe_uniphy_read(uniphy, SR_XS_PCS_KR_STS1_ADDR);
		state->link = (val & SR_XS_PCS_KR_STS1_PLU) ? 1 : 0;
		state->duplex = DUPLEX_FULL;
		state->speed = SPEED_10000;
		state->pause |= (MLO_PAUSE_RX | MLO_PAUSE_TX);
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
		val = ppe_uniphy_read(uniphy, UNIPHY_CHANNEL0_INPUT_OUTPUT_6_ADDR);
		state->link = (val & NEWADDEDFROMHERE_CH0_LINK_MAC) ? 1 : 0;
		state->duplex = DUPLEX_FULL;
		state->speed = SPEED_2500;
		state->pause |= (MLO_PAUSE_RX | MLO_PAUSE_TX);
		break;
	case PHY_INTERFACE_MODE_1000BASEX:
	case PHY_INTERFACE_MODE_SGMII:
		val = ppe_uniphy_read(uniphy, UNIPHY_CHANNEL0_INPUT_OUTPUT_6_ADDR);
		state->link = (val & NEWADDEDFROMHERE_CH0_LINK_MAC) ? 1 : 0;
		state->duplex = (val & NEWADDEDFROMHERE_CH0_DUPLEX_MODE_MAC) ?
			DUPLEX_FULL : DUPLEX_HALF;
		if (FIELD_GET(NEWADDEDFROMHERE_CH0_SPEED_MODE_MAC, val) == UNIPHY_SPEED_10M)
			state->speed = SPEED_10;
		else if (FIELD_GET(NEWADDEDFROMHERE_CH0_SPEED_MODE_MAC, val) == UNIPHY_SPEED_100M)
			state->speed = SPEED_100;
		else if (FIELD_GET(NEWADDEDFROMHERE_CH0_SPEED_MODE_MAC, val) == UNIPHY_SPEED_1000M)
			state->speed = SPEED_1000;
		state->pause |= (MLO_PAUSE_RX | MLO_PAUSE_TX);
		break;
	default:
		break;
	}
}

static void ppe_pcs_an_restart(struct phylink_pcs *pcs)
{
}

static const struct phylink_pcs_ops ppe_pcs_ops = {
	.pcs_get_state = ppe_pcs_get_state,
	.pcs_config = ppe_pcs_config,
	.pcs_an_restart = ppe_pcs_an_restart,
};

static void uniphy_clk_release_provider(void *res)
{
	of_clk_del_provider(res);
}

struct ppe_uniphy *ppe_uniphy_setup(struct platform_device *pdev)
{
	struct clk_hw_onecell_data *uniphy_clk_data = NULL;
	struct device_node *np;
	struct ppe_device *ppe_dev = platform_get_drvdata(pdev);
	struct ppe_uniphy *uniphy;
	int i, ret, clk_num = 0;

	np = of_get_child_by_name(pdev->dev.of_node, "qcom-uniphy");
	if (!np) {
		dev_err(&pdev->dev, "Failed to find uniphy node\n");
		return ERR_PTR(-ENODEV);
	}

	/* Register uniphy raw clock */
	clk_num = of_property_count_strings(np, "clock-output-names");
	if (clk_num < 0) {
		dev_err(&pdev->dev, "%pOFn: invalid clock output count\n", np);
		goto err_node_put;
	}

	uniphy_clk_data = devm_kzalloc(&pdev->dev,
				       struct_size(uniphy_clk_data, hws, clk_num),
				       GFP_KERNEL);
	if (!uniphy_clk_data) {
		ret = -ENOMEM;
		goto err_node_put;
	}

	uniphy_clk_data->num = clk_num;
	for (i = 0; i < clk_num; i++) {
		ret = of_property_read_string_index(np, "clock-output-names", i,
						    (const char **)&uniphy_raw_clks[i]->init->name);
		if (ret) {
			dev_err(&pdev->dev, "invalid clock name @ %pOFn\n", np);
			goto err_node_put;
		}

		ret = devm_clk_hw_register(&pdev->dev, uniphy_raw_clks[i]);
		if (ret)
			goto err_node_put;
		uniphy_clk_data->hws[i] = uniphy_raw_clks[i];
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, uniphy_clk_data);
	if (ret)
		goto err_node_put;

	ret = devm_add_action_or_reset(&pdev->dev, uniphy_clk_release_provider, np);
	if (ret)
		goto err_node_put;

	/* Initialize each uniphy structure */
	uniphy = devm_kzalloc(&pdev->dev, sizeof(*uniphy) * (clk_num >> 1), GFP_KERNEL);
	if (!uniphy) {
		ret = -ENOMEM;
		goto err_node_put;
	}

	for (i = 0; i < (clk_num >> 1); i++) {
		uniphy[i].base = devm_of_iomap(&pdev->dev, np, i, NULL);
		if (IS_ERR(uniphy[i].base)) {
			ret = PTR_ERR(uniphy[i].base);
			goto err_node_put;
		}
		uniphy[i].index = i;
		uniphy[i].interface = PHY_INTERFACE_MODE_NA;
		uniphy[i].ppe_dev = ppe_dev;
		uniphy[i].pcs.ops = &ppe_pcs_ops;
		uniphy[i].pcs.poll = true;
	}
	of_node_put(np);
	return uniphy;

err_node_put:
	of_node_put(np);
	return ERR_PTR(ret);
}
