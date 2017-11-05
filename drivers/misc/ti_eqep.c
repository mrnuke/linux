/*
 * TI eQEP driver for AM33xx SOCs
 *
 * Copyright (C) 2013 Nathaniel R. Lewis - http://teknoman117.wordpress.com/
 * Copyright (C) 2015 SoftPLC Corporation, Dick Hollenbeck <dick@softplc.com>
 * Copyright (C) 2017 Alexandru Gagniuc <mr.nuke.me@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * sysfs entries
 *	 - position = absolute - current position; relative - last latched value
 *	 - mode => "absolute"; "relative"
 *	 - period_ns => sampling period for the hardware
 *	 - enable => 0 - eQEP disabled, 1 - eQEP enabled
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/input.h>

/* eQEP register map */
#define EQEP_POSCNT		0x00
#define EQEP_POSINIT		0x04
#define EQEP_POSMAX		0x08
#define EQEP_POSCMP		0x0C
#define EQEP_POSILAT		0x10
#define EQEP_POSSLAT		0x14
#define EQEP_POSLAT		0x18
#define EQEP_UTMR		0x1C
#define EQEP_UPRD		0x20
#define EQEP_WDTMR		0x24
#define EQEP_WDPRD		0x26
#define EQEP_DECCTL		0x28
#define EQEP_CTL		0x2A
#define EQEP_CAPCTL		0x2C
#define EQEP_QPOSCTL		0x2E
#define EQEP_EINT		0x30
#define EQEP_FLG		0x32
#define EQEP_CLR		0x34
#define EQEP_FRC		0x36
#define EQEP_STS		0x38
#define EQEP_CTMR		0x3A
#define EQEP_CPRD		0x3C
#define EQEP_CTMRLAT		0x3E
#define EQEP_CPRDLAT		0x40
#define EQEP_REVID		0x5C

#define DECTL_QSRC_MASK		(3 << 14)
#define DECTL_QSRC(src)		((src << 14) & DECTL_QSRC_MASK)
#define DECTL_XCR		(1 << 11)
#define DECTL_SWAP		(1 << 10)
#define DECTL_QAP		(1 << 8)
#define DECTL_QBP		(1 << 7)
#define DECTL_QIP		(1 << 6)
#define DECTL_QSP		(1 << 5)

#define CTL_FREESOFT_MASK	(3 << 14)
#define CTL_FREESOFT(x)		((x << 14) & DECTL_QSRC_MASK)
#define CTL_PCRM_MASK		(3 << 12)
#define CTL_PCRM_INDEX		(0 << 12)
#define CTL_PCRM_MAX		(1 << 12)
#define CTL_PCRM_1ST_INDEX	(2 << 12)
#define CTL_PCRM_TIME_EVENT	(3 << 12)
#define CTL_SWI			(1 << 7)
#define CTL_IEL_MASK		(3 << 4)
#define CTL_IEL_RISING		(1 << 4)
#define CTL_IEL_FALLING		(2 << 4)
#define CTL_IEL_SOFTWARE	(3 << 4)
#define CTL_PHEN		(1 << 3)
#define CTL_QCLM		(1 << 2)
#define CTL_UTE			(1 << 1)

#define EQEP_INTERRUPT_MASK	0x0FFF
#define IRQ_UTOF		(1 << 11)

/*
 * Modes for the eQEP unit:
 *  Absolute - the position entry represents the current position of the
 *	   encoder. Poll this value and it will be notified every
 *	   period nanoseconds
 *  Relative - the position entry represents the last latched position of
 *	   the encoder. This value is latched every period nanoseconds
 *	   and the internal counter is subsequenty reset
 */
enum eqep_mode {
	TIEQEP_MODE_ABSOLUTE,
	TIEQEP_MODE_RELATIVE,
};

struct encoder_settings {
	unsigned swap_qa_qb : 1;
	unsigned qa_inverted : 1;
	unsigned qb_inverted : 1;
	unsigned qi_active_low : 1;
	unsigned qs_active_low : 1;
	unsigned count_mode : 2;
};

struct eqep {
	struct work_struct notify_work;
	struct encoder_settings encoder;
	struct platform_device *pdev;
	void __iomem *mmio_base;

	/* SYSCLKOUT to the eQEP unit */
	uint32_t clk_rate;
	uint16_t irq;

	/* Backup for driver suspension */
	uint16_t prior_qepctl;
	uint16_t prior_qeint;

	enum eqep_mode op_mode;
};

static const char *const eqep_decode_modes[] = {
	"quadrature",
	"direction-count",
	"up",
	"down"
};

static const char *const eqep_drv_modes[] = {
	[TIEQEP_MODE_ABSOLUTE] = "absolute",
	[TIEQEP_MODE_RELATIVE] = "relative"
};

static uint16_t eqep_read16(struct eqep *eqep, uint16_t reg)
{
	uintptr_t addr = (uintptr_t)eqep->mmio_base + (reg & ~1);

	return readw((void *)addr);
}

static void eqep_write16(struct eqep *eqep, uint16_t reg, uint16_t val)
{
	uintptr_t addr = (uintptr_t)eqep->mmio_base + (reg & ~1);

	writew(val, (void *)addr);
}

static uint32_t eqep_read32(struct eqep *eqep, uint16_t reg)
{
	uintptr_t addr = (uintptr_t)eqep->mmio_base + (reg & ~3);

	return readl((void *)addr);
}

static void eqep_write32(struct eqep *eqep, uint16_t reg, uint32_t val)
{
	uintptr_t addr = (uintptr_t)eqep->mmio_base + (reg & ~3);

	writel(val, (void *)addr);
}

uint64_t eqep_period_ns_to_uprd(struct eqep *eqep, uint64_t period)
{
	period *= eqep->clk_rate;
	do_div(period, NSEC_PER_SEC);
	return period;
}

static void notify_handler(struct work_struct *work)
{
	struct eqep *eqep;

	eqep = container_of(work, struct eqep, notify_work);
	sysfs_notify(&eqep->pdev->dev.kobj, NULL, "position");
}

static irqreturn_t eqep_irq_handler(int irq, void *dev_id)
{
	uint16_t irq_flags;
	struct platform_device	*pdev = dev_id;
	struct eqep	*eqep = platform_get_drvdata(pdev);

	irq_flags = eqep_read16(eqep, EQEP_FLG) & EQEP_INTERRUPT_MASK;
	if (irq_flags & IRQ_UTOF) {
		/* Unit timer overflow interrupt - notify potential pollers */
		schedule_work(&eqep->notify_work);
	}

	/* Clear interrupt flags */
	eqep_write16(eqep, EQEP_CLR, irq_flags);

	return IRQ_HANDLED;
}

static ssize_t eqep_is_enabled(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint16_t ctl;
	struct eqep *eqep = dev_get_drvdata(dev);

	pm_runtime_get_sync(dev);

	ctl = eqep_read16(eqep, EQEP_CTL) & CTL_PHEN;
	return sprintf(buf, "%u\n", !!(ctl & CTL_PHEN));
}

static ssize_t eqep_set_enabled(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	uint16_t qe_ctl;
	uint8_t	enabled;
	struct eqep *eqep = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &enabled);
	if (ret)
		return ret;

	pm_runtime_get_sync(dev);

	qe_ctl = eqep_read16(eqep, EQEP_CTL);
	qe_ctl &= ~CTL_PHEN;
	qe_ctl |= enabled ? CTL_PHEN : 0;
	eqep_write16(eqep, EQEP_CTL, qe_ctl);

	return count;
}

/* Read the current position counter.
 * In absolute mode, get the current position, min and max reset values, and
 * the position that was latched on the last index event.
 * In relative mode, only the last latched position is relevant.
 */
static ssize_t eqep_get_position(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	uint32_t pos, min, max, latched;
	struct eqep *eqep = dev_get_drvdata(dev);

	pm_runtime_get_sync(dev);

	pos = eqep_read32(eqep, EQEP_POSCNT);
	min = eqep_read32(eqep, EQEP_POSINIT);
	max = eqep_read32(eqep, EQEP_POSMAX);
	latched = eqep_read32(eqep, EQEP_POSILAT);

	if (eqep->op_mode == TIEQEP_MODE_ABSOLUTE)
		return sprintf(buf, "%u, [%u, %u], %u\n",
			pos, min, max, latched);

	return sprintf(buf, "%d\n", eqep_read32(eqep, EQEP_POSLAT));
}

#include <linux/slab.h>
/* Adjust/calibrate the position counter. */
static ssize_t eqep_set_position(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	int ret, i;
	char *next, *buf_copy;
	uint32_t bounds[3];
	struct eqep *eqep = dev_get_drvdata(dev);


	/* It only makes sense to adjust the position in absolute mode. */
	if (eqep->op_mode != TIEQEP_MODE_ABSOLUTE)
		return -ENOTSUPP;

	pm_runtime_get_sync(dev);


	buf_copy = kstrdup(buf, GFP_KERNEL);
	next = buf_copy;

	for (i = 0; i < ARRAY_SIZE(bounds); i++) {
		next = strsep(&buf_copy, " ");
		if (!next)
			return -EINVAL;

		ret = kstrtou32(next, 0, bounds + i);
		if (ret)
			return ret;
	}

	dev_err(&eqep->pdev->dev, "%u, %u, %u\n", bounds[0], bounds[1], bounds[2]);

	kfree(buf_copy);

	eqep_write32(eqep, EQEP_POSCNT, bounds[0]);
	eqep_write32(eqep, EQEP_POSINIT, bounds[1]);
	eqep_write32(eqep, EQEP_POSMAX, bounds[2]);

	return count;
}

static ssize_t eqep_get_timer_period(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct eqep *eqep = dev_get_drvdata(dev);
	uint64_t period;

	pm_runtime_get_sync(dev);

	period = eqep_read32(eqep, EQEP_UPRD) * NSEC_PER_SEC;
	do_div(period, eqep->clk_rate);
	return sprintf(buf, "%llu\n", period);
}

static ssize_t eqep_set_timer_period(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int ret;
	uint16_t qe_ctl;
	uint64_t period;

	struct eqep *eqep = dev_get_drvdata(dev);

	ret = kstrtou64(buf, 0, &period);
	if (ret)
		return ret;

	pm_runtime_get_sync(dev);

	/* Disable the unit timer before messing with its settings. */
	qe_ctl = eqep_read16(eqep, EQEP_CTL);
	qe_ctl &= ~(CTL_UTE | CTL_QCLM);
	eqep_write16(eqep, EQEP_CTL, qe_ctl);

	/* If the timer is enabled (a non-zero period has been passed) */
	if (period) {
		period = eqep_period_ns_to_uprd(eqep, period);
		eqep_write32(eqep, EQEP_UPRD, period);

		/* Reset the timer. */
		eqep_write16(eqep, EQEP_UTMR, 0);
		/* Re-enable unit timer */
		qe_ctl |= CTL_UTE | CTL_QCLM;
		eqep_write16(eqep, EQEP_CTL, qe_ctl);
	}

	return count;
}

static ssize_t eqep_get_mode(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct eqep *eqep = dev_get_drvdata(dev);

	if (eqep->op_mode >= ARRAY_SIZE(eqep_drv_modes))
		return sprintf(buf, "%s\n", "unknown");

	return sprintf(buf, "%s\n", eqep_drv_modes[eqep->op_mode]);
}

/* Set the operating mode:
 *   "absolute": In absolute mode, the counter is reset by the index signal.
 *	The position counter reset mode (PCRM) contrpls this.
 *
 *   "relative": In relative mode, latch the value of the eQEP hardware on the
 *	overflow of the unit timer. So enable the unit timer position reset.
 */
static ssize_t eqep_set_mode(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	uint8_t mode;
	uint16_t qe_ctl;
	struct eqep *eqep = dev_get_drvdata(dev);

	for (mode = 0; mode < ARRAY_SIZE(eqep_drv_modes); mode++) {
		if (!strcmp(buf, eqep_drv_modes[mode]))
			break;
	}

	if (mode >  ARRAY_SIZE(eqep_drv_modes))
		return -EINVAL;

	/* Increment the device usage count and run pm_runtime_resume() */
	pm_runtime_get_sync(dev);
	qe_ctl = eqep_read16(eqep, EQEP_CTL);
	qe_ctl &= ~CTL_PCRM_MASK;

	if (mode == TIEQEP_MODE_ABSOLUTE) {
		qe_ctl |= CTL_PCRM_INDEX;
		eqep->op_mode = TIEQEP_MODE_ABSOLUTE;
	} else if (mode == TIEQEP_MODE_RELATIVE) {
		qe_ctl |= CTL_PCRM_TIME_EVENT;
		eqep->op_mode = TIEQEP_MODE_RELATIVE;
	}

	eqep_write16(eqep, EQEP_CTL, qe_ctl);

	return count;
}

static DEVICE_ATTR(enabled,   0644, eqep_is_enabled, eqep_set_enabled);
static DEVICE_ATTR(position,  0644, eqep_get_position, eqep_set_position);
static DEVICE_ATTR(period_ns, 0644, eqep_get_timer_period,
				    eqep_set_timer_period);
static DEVICE_ATTR(mode,      0644, eqep_get_mode, eqep_set_mode);

static const struct attribute *eqep_attrs[] = {
	&dev_attr_enabled.attr,
	&dev_attr_position.attr,
	&dev_attr_period_ns.attr,
	&dev_attr_mode.attr,
	NULL,
};

static const struct attribute_group eqep_device_attr_group = {
	.attrs = (struct attribute **) eqep_attrs,
};

static int eqep_match_mode(const char *name)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(eqep_decode_modes); i++) {
		if (!strcmp(name, eqep_decode_modes[i]))
			return i;
	}

	return -EINVAL;
}

static void eqep_init(struct eqep *eqep)
{
	uint16_t decoder_ctl, qe_ctl;
	uint32_t rev;
	struct device *dev = &eqep->pdev->dev;

	rev = eqep_read32(eqep, EQEP_REVID);
	dev_info(dev, "TI eQEP HW revision %x\n", rev);

	decoder_ctl = DECTL_QSRC(eqep->encoder.count_mode);
	/* In up or down count modes, count on rising edge only. */
	if (eqep->encoder.count_mode > 2)
		decoder_ctl |= DECTL_XCR;

	decoder_ctl |= eqep->encoder.qa_inverted ? DECTL_QAP : 0;
	decoder_ctl |= eqep->encoder.qb_inverted ? DECTL_QBP : 0;
	decoder_ctl |= eqep->encoder.qi_active_low ? DECTL_QIP : 0;
	decoder_ctl |= eqep->encoder.qs_active_low ? DECTL_QSP : 0;
	decoder_ctl |= eqep->encoder.swap_qa_qb ? DECTL_SWAP : 0;

	dev_info(dev, "count_mode: %s\n",
		 eqep_decode_modes[eqep->encoder.count_mode]);
	dev_info(dev, "invert_qa: %d\n", eqep->encoder.qa_inverted);
	dev_info(dev, "invert_qb: %d\n", eqep->encoder.qb_inverted);
	dev_info(dev, "qi_active_low: %d\n", eqep->encoder.qi_active_low);
	dev_info(dev, "qs_active_low: %d\n", eqep->encoder.qs_active_low);
	dev_info(dev, "swap_inputs: %d\n", eqep->encoder.swap_qa_qb);

	eqep_write16(eqep, EQEP_DECCTL, decoder_ctl);
	eqep_write32(eqep, EQEP_POSINIT, 0);
	eqep_write32(eqep, EQEP_POSMAX, ~0);
	eqep_write32(eqep, EQEP_POSCNT, 0);
	eqep_write16(eqep, EQEP_EINT, IRQ_UTOF);

	/* Default timer period to 1 second */
	eqep_write32(eqep, EQEP_UPRD, eqep_period_ns_to_uprd(eqep, 1000000000));

	/* Enable quadrature position counting, unit timer */
	qe_ctl = CTL_PHEN | CTL_UTE | CTL_QCLM | CTL_IEL_RISING | CTL_SWI;
	eqep_write16(eqep, EQEP_CTL, qe_ctl);

	/* Default to absolute mode */
	eqep->op_mode = TIEQEP_MODE_ABSOLUTE;
}

static int eqep_probe_dt(struct eqep *eqep, const struct device_node *node)
{
	const char *str;
	struct device *dev = &eqep->pdev->dev;

	if (of_get_property(node, "qa_inverted", NULL))
		eqep->encoder.qa_inverted = 1;

	if (of_get_property(node, "qb_inverted", NULL))
		eqep->encoder.qb_inverted = 1;

	if (of_get_property(node, "qi_active_low", NULL))
		eqep->encoder.qi_active_low = 1;

	if (of_get_property(node, "qs_active_low", NULL))
		eqep->encoder.qs_active_low = 1;

	if (of_get_property(node, "ti,swap_qepa_qepb", NULL))
		eqep->encoder.swap_qa_qb = 1;

	eqep->encoder.count_mode = 0;
	if (!of_property_read_string_index(node, "count_mode", 0, &str))
		eqep->encoder.count_mode = eqep_match_mode(str);

	if (eqep->encoder.count_mode < 0) {
		dev_err(dev, "Invalid 'count_mode' property: '%s'\n", str);
		eqep->encoder.count_mode = 0;
	}

	return 0;
}

static int eqep_probe(struct platform_device *pdev)
{
	int ret;
	struct resource	 *res;
	struct clk	 *clk;
	struct eqep *eqep;

	eqep = devm_kzalloc(&pdev->dev, sizeof(*eqep), GFP_KERNEL);
	if (!eqep)
		return -ENOMEM;

	clk = devm_clk_get(pdev->dev.parent, "fck");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return PTR_ERR(clk);
	}

	eqep->clk_rate = clk_get_rate(clk);
	if (!eqep->clk_rate) {
		dev_err(&pdev->dev, "failed to get clock rate\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	eqep->mmio_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(eqep->mmio_base))
		return PTR_ERR(eqep->mmio_base);

	eqep->pdev = pdev;
	platform_set_drvdata(pdev, eqep);

	eqep->irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, eqep->irq, eqep_irq_handler,
			       IRQF_IRQPOLL, dev_name(&pdev->dev), pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Cannot request IRQ\n");
		return -ENODEV;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &eqep_device_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "failed to create sysfs entries\n");
		return -ret;
	}

	eqep_probe_dt(eqep, pdev->dev.of_node);
	eqep_init(eqep);

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	INIT_WORK(&eqep->notify_work, notify_handler);

	return 0;
}

static int eqep_remove(struct platform_device *pdev)
{
	struct eqep *eqep = platform_get_drvdata(pdev);

	cancel_work_sync(&eqep->notify_work);
	sysfs_remove_group(&pdev->dev.kobj, &eqep_device_attr_group);
	devm_free_irq(&eqep->pdev->dev, eqep->irq, pdev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static int eqep_suspend(struct device *dev)
{
	struct eqep *eqep = dev_get_drvdata(dev);

	/* Save critical regisdters. */
	eqep->prior_qepctl = eqep_read16(eqep, EQEP_CTL);
	eqep->prior_qeint = eqep_read16(eqep, EQEP_EINT);
	/* Disable interrupts. */
	eqep_write16(eqep, EQEP_EINT, 0);
	/* Disable eQEP controller */
	eqep_write16(eqep, EQEP_CTL, eqep->prior_qepctl & ~CTL_PHEN);

	pm_runtime_put_sync(dev);
	return 0;
}

static int eqep_resume(struct device *dev)
{
	struct eqep *eqep = dev_get_drvdata(dev);

	pm_runtime_get_sync(dev);
	/* Restore critical registers. */
	eqep_write16(eqep, EQEP_EINT, eqep->prior_qeint);
	eqep_write16(eqep, EQEP_CTL, eqep->prior_qepctl);
	return 0;
}

static SIMPLE_DEV_PM_OPS(eqep_pm_ops, eqep_suspend, eqep_resume);

static const struct of_device_id eqep_of_match[] = {
	{ .compatible = "ti,am33xx-eqep" },
	{ }
};

MODULE_DEVICE_TABLE(of, eqep_of_match);

static struct platform_driver eqep_driver = {
	.driver = {
		.name	= "ti-eqep",
		.owner	= THIS_MODULE,
		.pm	= &eqep_pm_ops,
		.of_match_table = eqep_of_match,
	},
	.probe = eqep_probe,
	.remove = eqep_remove,
};

module_platform_driver(eqep_driver);

MODULE_DESCRIPTION("TI eQEP driver");
MODULE_AUTHOR("Nathaniel R. Lewis");
MODULE_AUTHOR("Alexandru Gagniuc <mr.nuke.me@gmail.com>");
MODULE_LICENSE("GPL");
