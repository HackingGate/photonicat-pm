// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Photonicat Power Manager Driver - Power Button Module
 *
 * Decides what a PMU power button press means. The PMU announces a press
 * with PMU_REQUEST_SHUTDOWN; the pmu-button-mode device tree property
 * selects whether the driver powers the system off itself, reports
 * KEY_POWER and lets userspace decide, or ignores the press entirely.
 *
 * The PMU sends one frame per press and never announces a release, so the
 * input device reports a press immediately followed by a release. Userspace
 * therefore sees every press as a short press; long-press policy such as
 * HandlePowerKeyLongPress= in logind.conf(5) cannot trigger from this button.
 *
 * Copyright (c) 2026, HackingGate <i@hackinggate.com>
 */

#include "photonicat-pm.h"

static char pcat_pm_button_mode_param[16];
module_param_string(button_mode, pcat_pm_button_mode_param,
	sizeof(pcat_pm_button_mode_param), 0444);
MODULE_PARM_DESC(button_mode,
	"PMU power button mode: poweroff, input, or ignore (overrides the pmu-button-mode device tree property)");

static const char * const pcat_pm_button_mode_names[] = {
	[PCAT_PM_BUTTON_MODE_POWEROFF] = "poweroff",
	[PCAT_PM_BUTTON_MODE_INPUT] = "input",
	[PCAT_PM_BUTTON_MODE_IGNORE] = "ignore",
};

/**
 * pcat_pm_button_mode_name - Name of a power button mode
 * @mode: Button mode
 *
 * Return: Device tree string for @mode
 */
static const char *pcat_pm_button_mode_name(enum pcat_pm_button_mode mode)
{
	if (mode >= ARRAY_SIZE(pcat_pm_button_mode_names))
		return "poweroff";

	return pcat_pm_button_mode_names[mode];
}

/**
 * pcat_pm_button_mode_parse - Parse a button mode name
 * @name: Mode name to parse
 * @mode: Parsed mode on success, untouched on failure
 *
 * Return: true if @name is a known mode
 */
static bool pcat_pm_button_mode_parse(const char *name,
	enum pcat_pm_button_mode *mode)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(pcat_pm_button_mode_names); i++) {
		if (!strcmp(name, pcat_pm_button_mode_names[i])) {
			*mode = i;
			return true;
		}
	}

	return false;
}

/**
 * pcat_pm_input_probe - Read the button mode and register the input device
 * @pm_data: Driver data
 *
 * Picks the button mode from the button_mode module parameter when set,
 * otherwise from the optional pmu-button-mode device tree property,
 * defaulting to poweroff so device trees written for earlier driver
 * versions keep the behavior they were written against. An unknown value
 * from either source is rejected with a warning and falls back to the
 * next source.
 *
 * The input device is registered only in input mode; the other modes never
 * report a key, and an input device that emits nothing is misleading.
 *
 * Return: 0 on success, negative error otherwise
 */
int pcat_pm_input_probe(struct pcat_pm_data *pm_data)
{
	struct device *dev = &pm_data->serdev->dev;
	struct input_dev *input;
	const char *mode_name;
	const char *source = "default";
	int ret;

	pm_data->button_mode = PCAT_PM_BUTTON_MODE_POWEROFF;

	if (!device_property_read_string(dev, "pmu-button-mode", &mode_name)) {
		if (pcat_pm_button_mode_parse(mode_name, &pm_data->button_mode))
			source = "device tree";
		else
			dev_warn(dev,
				"Unknown pmu-button-mode \"%s\", using poweroff.\n",
				mode_name);
	}

	if (pcat_pm_button_mode_param[0]) {
		if (pcat_pm_button_mode_parse(pcat_pm_button_mode_param,
					      &pm_data->button_mode))
			source = "module parameter";
		else
			dev_warn(dev,
				"Unknown button_mode parameter \"%s\", ignored.\n",
				pcat_pm_button_mode_param);
	}

	dev_info(dev, "PMU button mode: %s (%s)\n",
		pcat_pm_button_mode_name(pm_data->button_mode), source);

	if (pm_data->button_mode != PCAT_PM_BUTTON_MODE_INPUT)
		return 0;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = "photonicat-pm power button";
	input->phys = "photonicat-pm/input0";
	input->id.bustype = BUS_HOST;

	input_set_capability(input, EV_KEY, KEY_POWER);

	ret = input_register_device(input);
	if (ret)
		return ret;

	pm_data->input = input;

	return 0;
}

/**
 * pcat_pm_button_event - Handle a PMU power button press
 * @pm_data: Driver data
 *
 * Called from the UART receive path on PMU_REQUEST_SHUTDOWN. Runs in the
 * serdev receive context, so it must not sleep; orderly_poweroff() and the
 * input calls are both safe there.
 */
void pcat_pm_button_event(struct pcat_pm_data *pm_data)
{
	struct device *dev = &pm_data->serdev->dev;

	switch (pm_data->button_mode) {
	case PCAT_PM_BUTTON_MODE_INPUT:
		if (!pm_data->input) {
			dev_warn(dev,
				"PMU request shutdown with no input device, powering off.\n");
			orderly_poweroff(true);
			break;
		}

		dev_info(dev, "PMU request shutdown, reporting KEY_POWER.\n");
		input_report_key(pm_data->input, KEY_POWER, 1);
		input_sync(pm_data->input);
		input_report_key(pm_data->input, KEY_POWER, 0);
		input_sync(pm_data->input);
		break;

	case PCAT_PM_BUTTON_MODE_IGNORE:
		dev_info(dev, "PMU request shutdown ignored.\n");
		break;

	case PCAT_PM_BUTTON_MODE_POWEROFF:
	default:
		dev_info(dev, "PMU request shutdown.");
		orderly_poweroff(true);
		break;
	}
}
