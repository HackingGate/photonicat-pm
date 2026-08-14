// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Photonicat Power Manager Driver - Power Supply Module
 *
 * Handles battery and charger subsystems.
 *
 * Copyright (c) 2025, Kyosuke Nekoyashiki <supercatexpert@gmail.com>
 * Copyright (c) 2026, HackingGate <i@hackinggate.com>
 */

#include "photonicat-pm.h"

static enum power_supply_property pcat_pm_battery_v1_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static enum power_supply_property pcat_pm_battery_v2_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static enum power_supply_property pcat_pm_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static bool pcat_pm_charge_threshold_enabled(struct pcat_pm_data *pm_data)
{
	return pcat_pm_probe_capability_enabled(
		READ_ONCE(pm_data->pmu_fw_caps.charge_threshold_capability));
}

void pcat_pm_charge_threshold_query(struct pcat_pm_data *pm_data)
{
	pcat_pm_uart_write_data(pm_data, PCAT_PM_COMMAND_CHARGE_THRESHOLD_GET,
		NULL, 0, true, 0);
}

void pcat_pm_charge_threshold_report(struct pcat_pm_data *pm_data, u8 threshold)
{
	bool promoted = false;

	if (threshold < PCAT_PM_CHARGE_THRESHOLD_MIN ||
	    threshold > PCAT_PM_CHARGE_THRESHOLD_MAX) {
		dev_warn(&pm_data->serdev->dev,
			"Ignoring out-of-range PMU charge threshold %u.\n",
			threshold);
		return;
	}

	mutex_lock(&pm_data->mutex);
	pm_data->charge_threshold = threshold;
	if (pm_data->pmu_fw_caps.charge_threshold_capability !=
	    PCAT_PM_PROBE_CAP_ENABLED) {
		pm_data->pmu_fw_caps.charge_threshold_capability =
			PCAT_PM_PROBE_CAP_ENABLED;
		promoted = true;
	}
	mutex_unlock(&pm_data->mutex);

	if (promoted)
		dev_info(&pm_data->serdev->dev,
			"PMU charge threshold enabled after runtime validation (%u%%).\n",
			threshold);

	if (!IS_ERR_OR_NULL(pm_data->battery_psy))
		power_supply_changed(pm_data->battery_psy);
}

/**
 * pcat_pm_charge_threshold_set - Program the PMU charge stop threshold
 * @pm_data: Driver data
 * @threshold: Requested threshold percentage
 *
 * Sends CHARGE_THRESHOLD_SET and waits for the PMU ACK. The threshold is
 * not gated on the runtime capability state: firmware without support does
 * not answer, which surfaces as -ETIMEDOUT, and a firmware that answers
 * promotes the capability.
 *
 * Return: 0 on success, -EINVAL out of range, -EIO if the PMU refused the
 * value, -ETIMEDOUT if the PMU did not answer.
 */
static int pcat_pm_charge_threshold_set(struct pcat_pm_data *pm_data,
	int threshold)
{
	u8 value = threshold;
	int ret;

	if (threshold < PCAT_PM_CHARGE_THRESHOLD_MIN ||
	    threshold > PCAT_PM_CHARGE_THRESHOLD_MAX)
		return -EINVAL;

	ret = pcat_pm_cmd_send_wait(pm_data, &pm_data->charge_threshold_ack,
		PCAT_PM_COMMAND_CHARGE_THRESHOLD_SET, &value, 1,
		PCAT_PM_CMD_ACK_TIMEOUT_MS);
	if (ret) {
		if (ret == -EIO)
			dev_err(&pm_data->serdev->dev,
				"Failed to set charge threshold %u%%.\n", value);
		return ret;
	}

	pcat_pm_charge_threshold_report(pm_data, value);

	/* Read the threshold back so the exported value stays the PMU's own
	 * state rather than the requested one.
	 */
	pcat_pm_charge_threshold_query(pm_data);

	return 0;
}

static int pcat_pm_battery_get_prop(struct power_supply *ps,
		enum power_supply_property prop,
		union power_supply_propval *val)
{
	struct pcat_pm_data *pm_data = power_supply_get_drvdata(ps);

	switch (prop) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = pm_data->battery_technology;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "photonicat-pm";
		break;
	case POWER_SUPPLY_PROP_STATUS:
		mutex_lock(&pm_data->mutex);
		if (pm_data->on_battery && !pm_data->on_charger) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		} else if (pm_data->on_charger) {
			if (pm_data->battery_soc >= 100)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else if (pm_data->on_battery)
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		} else {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		mutex_unlock(&pm_data->mutex);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = pm_data->battery_design_max_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = pm_data->battery_design_min_uv;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		mutex_lock(&pm_data->mutex);
		val->intval = pm_data->battery_energy_full;
		mutex_unlock(&pm_data->mutex);
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		val->intval = pm_data->battery_design_uwh;
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		mutex_lock(&pm_data->mutex);
		val->intval = pm_data->battery_energy_now;
		mutex_unlock(&pm_data->mutex);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		mutex_lock(&pm_data->mutex);
		val->intval = pm_data->battery_voltage_now;
		mutex_unlock(&pm_data->mutex);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		mutex_lock(&pm_data->mutex);
		val->intval = pm_data->battery_current_now;
		mutex_unlock(&pm_data->mutex);
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		mutex_lock(&pm_data->mutex);
		val->intval = (pm_data->battery_voltage_now / 1000) *
			(pm_data->battery_current_now / 1000);
		mutex_unlock(&pm_data->mutex);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (!pm_data->battery_info)
			return -ENODEV;
		mutex_lock(&pm_data->mutex);
		val->intval = pm_data->battery_soc;
		mutex_unlock(&pm_data->mutex);
		if (val->intval > 100)
			val->intval = 100;
		else if (val->intval < 0)
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		if (!pcat_pm_charge_threshold_enabled(pm_data))
			return -ENODATA;
		mutex_lock(&pm_data->mutex);
		val->intval = pm_data->charge_threshold;
		mutex_unlock(&pm_data->mutex);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int pcat_pm_battery_set_prop(struct power_supply *ps,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct pcat_pm_data *pm_data = power_supply_get_drvdata(ps);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return pcat_pm_charge_threshold_set(pm_data, val->intval);
	default:
		return -EINVAL;
	}
}

static int pcat_pm_battery_prop_is_writeable(struct power_supply *ps,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return 1;
	default:
		return 0;
	}
}

static int pcat_pm_charger_get_prop(struct power_supply *ps,
		enum power_supply_property prop,
		union power_supply_propval *val)
{
	struct pcat_pm_data *pm_data = power_supply_get_drvdata(ps);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = pm_data->on_charger ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		mutex_lock(&pm_data->mutex);
		val->intval = pm_data->charger_voltage_now;
		mutex_unlock(&pm_data->mutex);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct power_supply_desc pcat_pm_battery_v1_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = pcat_pm_battery_v1_properties,
	.num_properties = ARRAY_SIZE(pcat_pm_battery_v1_properties),
	.get_property = pcat_pm_battery_get_prop,
	.set_property = pcat_pm_battery_set_prop,
	.property_is_writeable = pcat_pm_battery_prop_is_writeable,
};

static const struct power_supply_desc pcat_pm_battery_v2_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = pcat_pm_battery_v2_properties,
	.num_properties = ARRAY_SIZE(pcat_pm_battery_v2_properties),
	.get_property = pcat_pm_battery_get_prop,
	.set_property = pcat_pm_battery_set_prop,
	.property_is_writeable = pcat_pm_battery_prop_is_writeable,
};

static const struct power_supply_desc pcat_pm_charger_desc = {
	.name = "charger",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = pcat_pm_ac_properties,
	.num_properties = ARRAY_SIZE(pcat_pm_ac_properties),
	.get_property = pcat_pm_charger_get_prop,
};

int pcat_pm_charger_probe(struct pcat_pm_data *pm_data)
{
	struct serdev_device *serdev = pm_data->serdev;
	struct device *dev = &serdev->dev;
	const struct power_supply_desc *battery_psy_desc;
	struct power_supply_config pscfg = {};
	struct device_node *charger_node;
	struct power_supply_battery_info *bat_info;
	int ret;

	if (device_property_read_u32(dev, "pm-version", &pm_data->pm_version))
		pm_data->pm_version = 1;

	if (pm_data->pm_version > 1)
		battery_psy_desc = &pcat_pm_battery_v2_desc;
	else
		battery_psy_desc = &pcat_pm_battery_v1_desc;

	charger_node = of_get_child_by_name(dev->of_node, "charger");
	if (!charger_node) {
		dev_err(dev, "Missing charger node!\n");
		return -ENODEV;
	}

	pscfg.drv_data = pm_data;
	pscfg.fwnode = of_fwnode_handle(charger_node);

	pm_data->battery_psy = power_supply_register(dev, battery_psy_desc, &pscfg);
	of_node_put(charger_node);
	if (IS_ERR(pm_data->battery_psy)) {
		ret = PTR_ERR(pm_data->battery_psy);
		dev_err(dev, "Failed to register battery power supply: %d\n", ret);
		return ret;
	}

	pm_data->charger_psy = power_supply_register(dev, &pcat_pm_charger_desc, &pscfg);
	if (IS_ERR(pm_data->charger_psy)) {
		ret = PTR_ERR(pm_data->charger_psy);
		dev_err(dev, "Failed to register charger power supply: %d\n", ret);
		goto err_unregister_battery;
	}

	ret = power_supply_get_battery_info(pm_data->battery_psy, &bat_info);
	if (ret) {
		dev_err(dev, "Failed to get battery info: %d\n", ret);
		goto err_unregister_charger;
	}

	pm_data->battery_technology = bat_info->technology;
	pm_data->battery_design_uwh = bat_info->energy_full_design_uwh;
	pm_data->battery_design_min_uv = bat_info->voltage_min_design_uv;
	pm_data->battery_design_max_uv = bat_info->voltage_max_design_uv;
	pm_data->battery_voltage_now = pm_data->battery_design_max_uv;
	pm_data->battery_energy_full = pm_data->battery_design_uwh;
	pm_data->battery_soc = 100;
	pm_data->battery_info = bat_info;
	pm_data->on_charger = true;

	return 0;

err_unregister_charger:
	power_supply_unregister(pm_data->charger_psy);
	pm_data->charger_psy = NULL;
err_unregister_battery:
	power_supply_unregister(pm_data->battery_psy);
	pm_data->battery_psy = NULL;
	return ret;
}

void pcat_pm_charger_remove(struct pcat_pm_data *pm_data)
{
	if (!IS_ERR_OR_NULL(pm_data->charger_psy)) {
		power_supply_unregister(pm_data->charger_psy);
		pm_data->charger_psy = NULL;
	}
	if (!IS_ERR_OR_NULL(pm_data->battery_psy)) {
		power_supply_unregister(pm_data->battery_psy);
		pm_data->battery_psy = NULL;
	}
}
