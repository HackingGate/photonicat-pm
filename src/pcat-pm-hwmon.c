// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Photonicat Power Manager Driver - Hardware Monitoring Module
 *
 * Handles temperature and fan speed monitoring via hwmon.
 *
 * Copyright (c) 2025, Kyosuke Nekoyashiki <supercatexpert@gmail.com>
 * Copyright (c) 2026, HackingGate <i@hackinggate.com>
 */

#include <linux/hwmon.h>

#include "photonicat-pm.h"

static umode_t pcat_pm_hwmon_temp_mb_is_visible(const void *data,
					enum hwmon_sensor_types type,
					u32 attr, int channel)
{
	if (type != hwmon_temp)
		return 0;

	switch (attr) {
	case hwmon_temp_input:
		return 0444;
	default:
		return 0;
	}
}

static umode_t pcat_pm_hwmon_speed_fan_is_visible(const void *data,
					enum hwmon_sensor_types type,
					u32 attr, int channel)
{
	if (type != hwmon_fan)
		return 0;

	switch (attr) {
	case hwmon_fan_input:
		return 0444;
	default:
		return 0;
	}
}

static int pcat_pm_hwmon_temp_mb_read(struct device *dev,
			      enum hwmon_sensor_types type,
			      u32 attr, int channel, long *val)
{
	struct pcat_pm_data *pm_data = pcat_pm_get_data_from_dev(dev);
	int err = 0;

	switch (attr) {
	case hwmon_temp_input:
		mutex_lock(&pm_data->mutex);
		*val = pm_data->board_temp * 1000;
		mutex_unlock(&pm_data->mutex);
		break;
	default:
		err = -EOPNOTSUPP;
		break;
	}

	return err;
}

static int pcat_pm_hwmon_speed_fan_read(struct device *dev,
			      enum hwmon_sensor_types type,
			      u32 attr, int channel, long *val)
{
	struct pcat_pm_data *pm_data = pcat_pm_get_data_from_dev(dev);
	int err = 0;

	switch (attr) {
	case hwmon_fan_input:
		mutex_lock(&pm_data->mutex);
		*val = pm_data->fan_rpm;
		mutex_unlock(&pm_data->mutex);
		break;
	default:
		err = -EOPNOTSUPP;
		break;
	}

	return err;
}

static int pcat_pm_tz_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct pcat_pm_data *pm_data = thermal_zone_device_priv(tz);

	mutex_lock(&pm_data->mutex);
	*temp = pm_data->board_temp * 1000;
	mutex_unlock(&pm_data->mutex);

	return 0;
}

static const struct thermal_zone_device_ops pcat_pm_tz_ops = {
	.get_temp = pcat_pm_tz_get_temp,
};

static u32 pcat_pm_hwmon_temp_config[] = {
	HWMON_T_INPUT,
	0
};

static u32 pcat_pm_hwmon_fan_config[] = {
	HWMON_F_INPUT,
	0
};

static const struct hwmon_channel_info pcat_pm_hwmon_temp_mb_cinfo = {
	.type = hwmon_temp,
	.config = pcat_pm_hwmon_temp_config,
};

static const struct hwmon_channel_info pcat_pm_hwmon_speed_fan_cinfo = {
	.type = hwmon_fan,
	.config = pcat_pm_hwmon_fan_config,
};

static const struct hwmon_channel_info *pcat_pm_hwmon_temp_mb_info[] = {
	&pcat_pm_hwmon_temp_mb_cinfo,
	NULL
};

static const struct hwmon_channel_info *pcat_pm_hwmon_speed_fan_info[] = {
	&pcat_pm_hwmon_speed_fan_cinfo,
	NULL
};

static const struct hwmon_ops pcat_pm_hwmon_temp_mb_ops = {
	.is_visible = pcat_pm_hwmon_temp_mb_is_visible,
	.read = pcat_pm_hwmon_temp_mb_read,
};

static const struct hwmon_ops pcat_pm_hwmon_speed_fan_ops = {
	.is_visible = pcat_pm_hwmon_speed_fan_is_visible,
	.read = pcat_pm_hwmon_speed_fan_read,
};

static const struct hwmon_chip_info pcat_pm_hwmon_temp_mb_chip_info = {
	.ops = &pcat_pm_hwmon_temp_mb_ops,
	.info = pcat_pm_hwmon_temp_mb_info,
};

static const struct hwmon_chip_info pcat_pm_hwmon_speed_fan_chip_info = {
	.ops = &pcat_pm_hwmon_speed_fan_ops,
	.info = pcat_pm_hwmon_speed_fan_info,
};

int pcat_pm_hwmon_probe(struct pcat_pm_data *pm_data)
{
	struct device *dev = &pm_data->serdev->dev;
	int ret = 0, err;

	pm_data->hwmon_temp_mb_dev = devm_hwmon_device_register_with_info(
		dev, "pcat_pm_hwmon_temp_mb", pm_data,
		&pcat_pm_hwmon_temp_mb_chip_info, NULL);
	if (IS_ERR(pm_data->hwmon_temp_mb_dev)) {
		err = PTR_ERR(pm_data->hwmon_temp_mb_dev);
		dev_err(dev, "Failed to register hwmon for MB temperature: %d\n",
			err);
		ret = err;
	}

	pm_data->hwmon_speed_fan_dev = devm_hwmon_device_register_with_info(
		dev, "pcat_pm_hwmon_speed_fan", pm_data,
		&pcat_pm_hwmon_speed_fan_chip_info, NULL);
	if (IS_ERR(pm_data->hwmon_speed_fan_dev)) {
		err = PTR_ERR(pm_data->hwmon_speed_fan_dev);
		dev_err(dev, "Failed to register hwmon for fan speed: %d\n",
			err);
		if (!ret)
			ret = err;
	}

	pm_data->tzdev = devm_thermal_of_zone_register(dev, 0, pm_data,
		&pcat_pm_tz_ops);
	if (IS_ERR(pm_data->tzdev)) {
		dev_warn(dev,
			"MB temperature not registered as thermal zone (no DT binding?): %ld\n",
			PTR_ERR(pm_data->tzdev));
		pm_data->tzdev = NULL;
	}

	return ret;
}
