/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* PPE operations to be used by ethernet driver */

#ifndef __QCOM_PPE_H__
#define __QCOM_PPE_H__

#include <linux/platform_device.h>

/* PPE platform private data, which is used by external driver like
 * Ethernet DMA driver.
 */
struct ppe_device {
	struct device *dev;
	struct regmap *regmap;
	struct ppe_device_ops *ppe_ops;
	struct dentry *debugfs_root;
	bool is_ppe_probed;
	void *ppe_priv;
	void *uniphy;
};

/* PPE operations, which is used by the external driver like Ethernet
 * DMA driver to configure PPE.
 */
struct ppe_device_ops {
	int	(*set_maxframe)(struct ppe_device *ppe_dev, int port,
				int maxframe_size);
};

/* Function used to check PPE platform dirver is registered correctly or not. */
bool ppe_is_probed(struct platform_device *pdev);

/* Function used to get the PPE device */
struct ppe_device *ppe_dev_get(struct platform_device *pdev);

/* Function used to get the operations of PPE device */
struct ppe_device_ops *ppe_ops_get(struct platform_device *pdev);
#endif
