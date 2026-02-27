// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Photonicat Power Manager Driver - Sysfs Module
 *
 * Handles sysfs attributes.
 *
 * Copyright (c) 2025, Kyosuke Nekoyashiki <supercatexpert@gmail.com>
 * Copyright (c) 2026, HackingGate <i@hackinggate.com>
 */

#include "photonicat-pm.h"

static ssize_t movement_trigger_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct pcat_pm_data *pm_data;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	return scnprintf(buf, PAGE_SIZE, "%u\n", pm_data->movement_activated ? 1 : 0);
}

static ssize_t status_led_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct pcat_pm_data *pm_data;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	return scnprintf(buf, PAGE_SIZE, "%u\n", pm_data->status_led_enabled ? 1 : 0);
}

static ssize_t status_led_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct pcat_pm_data *pm_data;
	unsigned int val;
	u8 state;
	int ret;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	pm_data->status_led_enabled = !!val;

	state = pm_data->status_led_enabled ? 1 : 0;
	state |= pm_data->beeper_enabled ? 2 : 0;

	pcat_pm_uart_write_data(pm_data,
		PCAT_PM_COMMAND_STATUS_LED_BEEPER_V2_SET, &state, 1, true, 0);

	return count;
}

static ssize_t beeper_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct pcat_pm_data *pm_data;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	return scnprintf(buf, PAGE_SIZE, "%u\n", pm_data->beeper_enabled ? 1 : 0);
}

static ssize_t beeper_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct pcat_pm_data *pm_data;
	unsigned int val;
	u8 state;
	int ret;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	pm_data->beeper_enabled = !!val;

	state = pm_data->status_led_enabled ? 1 : 0;
	state |= pm_data->beeper_enabled ? 2 : 0;

	pcat_pm_uart_write_data(pm_data,
		PCAT_PM_COMMAND_STATUS_LED_BEEPER_V2_SET, &state, 1, true, 0);

	return count;
}

static ssize_t pmu_hw_version_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct pcat_pm_data *pm_data;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	return scnprintf(buf, PAGE_SIZE, "%s\n", pm_data->pmu_hw_version);
}

static ssize_t pmu_fw_version_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct pcat_pm_data *pm_data;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	return scnprintf(buf, PAGE_SIZE, "%s\n", pm_data->pmu_fw_version);
}

static ssize_t power_on_event_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct pcat_pm_data *pm_data;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	return scnprintf(buf, PAGE_SIZE, "%u\n", pm_data->power_on_event);
}

static void pcat_pm_sysfs_net_status_led_send(struct pcat_pm_data *pm_data)
{
	u8 buffer[6];
	u16 v;

	v = cpu_to_le16(pm_data->net_status_led_on_time);
	memcpy(buffer, &v, 2);

	v = cpu_to_le16(pm_data->net_status_led_off_time);
	memcpy(buffer + 2, &v, 2);

	v = cpu_to_le16(pm_data->net_status_led_repeat);
	memcpy(buffer + 4, &v, 2);

	pcat_pm_uart_write_data(pm_data,
		PCAT_PM_COMMAND_NET_STATUS_LED_SETUP, buffer, 6, true, 0);
}

static ssize_t net_status_led_on_time_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct pcat_pm_data *pm_data;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	return scnprintf(buf, PAGE_SIZE, "%u\n", pm_data->net_status_led_on_time);
}

static ssize_t net_status_led_on_time_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct pcat_pm_data *pm_data;
	unsigned int val;
	int ret;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;
	if (val > 65535)
		return -EINVAL;

	pm_data->net_status_led_on_time = val;
	pcat_pm_sysfs_net_status_led_send(pm_data);

	return count;
}

static ssize_t net_status_led_off_time_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct pcat_pm_data *pm_data;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	return scnprintf(buf, PAGE_SIZE, "%u\n", pm_data->net_status_led_off_time);
}

static ssize_t net_status_led_off_time_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct pcat_pm_data *pm_data;
	unsigned int val;
	int ret;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;
	if (val > 65535)
		return -EINVAL;

	pm_data->net_status_led_off_time = val;
	pcat_pm_sysfs_net_status_led_send(pm_data);

	return count;
}

static ssize_t net_status_led_repeat_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct pcat_pm_data *pm_data;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	return scnprintf(buf, PAGE_SIZE, "%u\n", pm_data->net_status_led_repeat);
}

static ssize_t net_status_led_repeat_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct pcat_pm_data *pm_data;
	unsigned int val;
	int ret;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;
	if (val > 65535)
		return -EINVAL;

	pm_data->net_status_led_repeat = val;
	pcat_pm_sysfs_net_status_led_send(pm_data);

	return count;
}

static ssize_t charger_on_auto_start_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct pcat_pm_data *pm_data;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	return scnprintf(buf, PAGE_SIZE, "%u\n", pm_data->charger_on_auto_start ? 1 : 0);
}

static ssize_t charger_on_auto_start_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct pcat_pm_data *pm_data;
	unsigned int val;
	u8 state;
	int ret;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	pm_data->charger_on_auto_start = !!val;
	state = pm_data->charger_on_auto_start ? 1 : 0;

	pcat_pm_uart_write_data(pm_data,
		PCAT_PM_COMMAND_CHARGER_ON_AUTO_START, &state, 1, true, 0);

	return count;
}

static ssize_t fan_state_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct pcat_pm_data *pm_data;

	pm_data = container_of(kobj, struct pcat_pm_data, kobject);

	if (!pm_data->fan_managed)
		return scnprintf(buf, PAGE_SIZE, "unmanaged\n");

	return scnprintf(buf, PAGE_SIZE, "%u\n", pm_data->fan_ctrl_speed);
}

static const struct kobj_type pcat_pm_kobj_ktype = {
	.sysfs_ops = &kobj_sysfs_ops,
};

static struct kobj_attribute pcat_pm_sysfs_movement_trigger_attribute =
	__ATTR_RO(movement_trigger);

static struct kobj_attribute pcat_pm_sysfs_status_led_attribute =
	__ATTR_RW(status_led);

static struct kobj_attribute pcat_pm_sysfs_beeper_attribute =
	__ATTR_RW(beeper);

static struct kobj_attribute pcat_pm_sysfs_pmu_hw_version_attribute =
	__ATTR_RO(pmu_hw_version);

static struct kobj_attribute pcat_pm_sysfs_pmu_fw_version_attribute =
	__ATTR_RO(pmu_fw_version);

static struct kobj_attribute pcat_pm_sysfs_power_on_event_attribute =
	__ATTR_RO(power_on_event);

static struct kobj_attribute pcat_pm_sysfs_net_status_led_on_time_attribute =
	__ATTR_RW(net_status_led_on_time);

static struct kobj_attribute pcat_pm_sysfs_net_status_led_off_time_attribute =
	__ATTR_RW(net_status_led_off_time);

static struct kobj_attribute pcat_pm_sysfs_net_status_led_repeat_attribute =
	__ATTR_RW(net_status_led_repeat);

static struct kobj_attribute pcat_pm_sysfs_charger_on_auto_start_attribute =
	__ATTR_RW(charger_on_auto_start);

static struct kobj_attribute pcat_pm_sysfs_fan_state_attribute =
	__ATTR_RO(fan_state);

static struct attribute *pcat_pm_sysfs_attrs[] = {
	&pcat_pm_sysfs_movement_trigger_attribute.attr,
	&pcat_pm_sysfs_status_led_attribute.attr,
	&pcat_pm_sysfs_beeper_attribute.attr,
	&pcat_pm_sysfs_pmu_hw_version_attribute.attr,
	&pcat_pm_sysfs_pmu_fw_version_attribute.attr,
	&pcat_pm_sysfs_power_on_event_attribute.attr,
	&pcat_pm_sysfs_net_status_led_on_time_attribute.attr,
	&pcat_pm_sysfs_net_status_led_off_time_attribute.attr,
	&pcat_pm_sysfs_net_status_led_repeat_attribute.attr,
	&pcat_pm_sysfs_charger_on_auto_start_attribute.attr,
	&pcat_pm_sysfs_fan_state_attribute.attr,
	NULL,
};

static struct attribute_group pcat_pm_sysfs_attr_group = {
	.attrs = pcat_pm_sysfs_attrs,
};

int pcat_pm_sysfs_init(struct pcat_pm_data *pm_data)
{
	struct device *dev = &pm_data->serdev->dev;
	int ret;

	ret = kobject_init_and_add(&pm_data->kobject, &pcat_pm_kobj_ktype,
		kernel_kobj, "%s", "photonicat-pm");
	if (!ret) {
		ret = sysfs_create_group(&pm_data->kobject,
			&pcat_pm_sysfs_attr_group);
		if (ret)
			dev_err(dev, "Failed to create sysfs files: %d\n", ret);
	} else {
		dev_err(dev, "Failed to initialize kernel object: %d\n", ret);
	}

	return ret;
}

void pcat_pm_sysfs_cleanup(struct pcat_pm_data *pm_data)
{
	sysfs_remove_group(&pm_data->kobject, &pcat_pm_sysfs_attr_group);
	kobject_put(&pm_data->kobject);
}
