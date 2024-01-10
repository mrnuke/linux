/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/* PPE debugfs counters setup. */

#ifndef __PPE_DEBUGFS_H__
#define __PPE_DEBUGFS_H__

#define PREFIX_S(desc, cnt_type) \
	seq_printf(seq, "%-16s %16s", desc, cnt_type)
#define CNT_ONE_TYPE(cnt, str, index) \
	seq_printf(seq, "%10u(%s=%04d)", cnt, str, index)
#define CNT_TWO_TYPE(cnt, cnt1, str, index) \
	seq_printf(seq, "%10u/%u(%s=%04d)", cnt, cnt1, str, index)
#define CNT_CPU_CODE(cnt, str, index) \
	seq_printf(seq, "%10u(%s),cpucode:%d", cnt, str, index)
#define CNT_DROP_CODE(cnt, str, port, index) \
	seq_printf(seq, "%10u(port=%d:%s),dropcode:%d", cnt, port, str, index)

int ppe_debugfs_setup(struct ppe_device *ppe_dev);
void ppe_debugfs_teardown(struct ppe_device *ppe_dev);

#endif
