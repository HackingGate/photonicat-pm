// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Photonicat Power Manager Driver - Fan/Thermal Control Module
 *
 * Handles thermal cooling device for fan control.
 *
 * WARNING: The PMU uses unmanaged fan speed by default. Once the driver sends a
 * SET command (via this cooling device, DT thermal policy, or other
 * software), the PMU switches to managed fan speed.
 *
 * CAUTION: Unmanaged may refer the last fixed speed sometimes (not the PMU auto),
 * and is dangerous for thermal management.
 *
 * When in managed fan speed, after shutdown, the fan stays at the last fixed speed.
 * If the device is still charging or thermally active, the fan will not adjust on its
 * own.
 *
 * To restore PMU auto speed:
 *
 * State 1: After a power button shutdown.
 * Do one of:
 * 1. Unplug AC power
 * 2. Hold to power on again
 *
 * State 2: After a software shutdown.
 * Do:
 * 1. Unplug AC power
 *
 * Copyright (c) 2025, Kyosuke Nekoyashiki <supercatexpert@gmail.com>
 * Copyright (c) 2026, HackingGate <i@hackinggate.com>
 */

#include "photonicat-pm.h"

static int pcat_pm_fan_get_max_state(struct thermal_cooling_device *cdev,
	unsigned long *state)
{
	*state = PCAT_PM_FAN_MAX_STATE;

	return 0;
}

static int pcat_pm_fan_get_cur_state(struct thermal_cooling_device *cdev,
	unsigned long *state)
{
	struct pcat_pm_data *pm_data = cdev->devdata;

	*state = pm_data->fan_ctrl_speed;

	return 0;
}

/*
 * WARNING: Sending a SET command switches the PMU from unmanaged fan speed to
 * managed fan speed. See module header for how to restore PMU auto speed.
 */
static int pcat_pm_fan_set_cur_state(struct thermal_cooling_device *cdev,
	unsigned long state)
{
	struct pcat_pm_data *pm_data = cdev->devdata;
	u8 speed = (u8)state;

	pm_data->fan_ctrl_speed = speed;
	pm_data->fan_managed = true;

	pcat_pm_uart_write_data(pm_data, PCAT_PM_COMMAND_FAN_SET,
		&speed, 1, true, 0);

	return 0;
}

static const struct thermal_cooling_device_ops pcat_pm_fan_cooling_ops = {
	.get_max_state = pcat_pm_fan_get_max_state,
	.get_cur_state = pcat_pm_fan_get_cur_state,
	.set_cur_state = pcat_pm_fan_set_cur_state,
};

int pcat_pm_fan_probe(struct pcat_pm_data *pm_data)
{
	struct serdev_device *serdev = pm_data->serdev;
	struct device *dev = &serdev->dev;
	struct thermal_cooling_device *cdev;
	struct device_node *fan_node;

	fan_node = of_get_child_by_name(dev->of_node, "fan");
	if (!fan_node) {
		dev_err(dev, "Missing fan node!\n");
		return -ENODEV;
	}

	cdev = devm_thermal_of_cooling_device_register(dev, fan_node,
		"pcat-pm-fan", pm_data, &pcat_pm_fan_cooling_ops);
	of_node_put(fan_node);

	if (IS_ERR(cdev))
		return -ENODEV;

	pm_data->cdev = cdev;

	return 0;
}
