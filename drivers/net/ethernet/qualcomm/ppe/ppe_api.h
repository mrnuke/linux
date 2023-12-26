/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* These may also be used by higher level network drivers such as ethernet or
 * QoS drivers.
 */

#ifndef __PPE_API_H__
#define __PPE_API_H__

#include "ppe.h"

#define PPE_QUEUE_ID_NUM			300
#define PPE_FLOW_ID_NUM				64
#define PPE_QUEUE_SCH_PRI_NUM			8

int ppe_queue_priority_set(struct ppe_device *ppe_dev,
			   int queue_id, int priority);
#endif
