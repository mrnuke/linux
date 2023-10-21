// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __LINUX_PCS_QCA8K_H
#define __LINUX_PCS_QCA8K_H

#include <linux/phylink.h>

/**
 * define P_XO_CLOCK_RATE - Clock frequency of crystal(External clock)
 *
 * The reference clock frequency of QCAK is fixed to 50000000 HZ,
 * which is used to restore the parent of PCS clocks to crystal clock
 * (External clock connected to QCA8K) by configuring the clock rate of
 * PCS to 50000000 HZ.
 */
#define P_XO_CLOCK_RATE		50000000

struct phylink_pcs *qca8k_pcs_create_fwnode(struct fwnode_handle *node);
void qca8k_pcs_destroy(struct phylink_pcs *pcs);

#endif /* __LINUX_PCS_QCA8K_H */
