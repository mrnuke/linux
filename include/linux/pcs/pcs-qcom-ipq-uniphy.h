/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 */

#ifndef __LINUX_PCS_QCOM_IPQ_UNIPHY_H
#define __LINUX_PCS_QCOM_IPQ_UNIPHY_H

struct phylink_pcs *ipq_unipcs_create(struct device_node *np);
void ipq_unipcs_destroy(struct phylink_pcs *pcs);

#endif /* __LINUX_PCS_QCOM_IPQ_UNIPHY_H */
