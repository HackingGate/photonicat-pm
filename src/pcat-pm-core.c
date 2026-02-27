// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Photonicat Power Manager Driver - Core Module
 *
 * Main driver orchestration, probe/remove, power management, and
 * worker thread for periodic heartbeat and status monitoring.
 *
 * Copyright (c) 2025, Kyosuke Nekoyashiki <supercatexpert@gmail.com>
 * Copyright (c) 2026, HackingGate <i@hackinggate.com>
 */

#include "photonicat-pm.h"

/**
 * pcat_pm_check_work - Periodic worker function
 * @work: kthread_work structure
 *
 * Sends heartbeat to PMU and checks for status report timeouts.
 * Also updates movement detection state.
 */
static void pcat_pm_check_work(struct kthread_work *work)
{
	struct pcat_pm_data *pm_data;
	u64 now;

	pm_data = container_of(work, struct pcat_pm_data, check_work);

	if (pm_data->work_flag)
		pcat_pm_uart_write_data(pm_data, PCAT_PM_COMMAND_HEARTBEAT,
			NULL, 0, false, 0);

	now = ktime_get_boottime_ns();

	mutex_lock(&pm_data->mutex);
	if (now >= pm_data->status_report_timestamp + 15 * NSEC_PER_SEC &&
		now >= pm_data->status_report_timeout_warn_timestamp + 15 * NSEC_PER_SEC) {
		dev_warn(&pm_data->serdev->dev, "Status report timeout!");
		pm_data->status_report_timeout_warn_timestamp = now;
	}

	if (now >= pm_data->movement_timestamp &&
		now < pm_data->movement_timestamp + 5 * NSEC_PER_SEC) {
		if (!pm_data->movement_activated)
			pm_data->movement_activated = true;
	} else {
		if (pm_data->movement_activated)
			pm_data->movement_activated = false;
	}
	mutex_unlock(&pm_data->mutex);
}

/**
 * pcat_pm_check_timer_expired - Timer callback for worker scheduling
 * @timer: High-resolution timer
 *
 * Queues the check work and reschedules for 1 second later.
 *
 * Return: HRTIMER_RESTART to keep timer running
 */
static enum hrtimer_restart pcat_pm_check_timer_expired(struct hrtimer *timer)
{
	struct pcat_pm_data *pm_data;

	pm_data = container_of(timer, struct pcat_pm_data, check_timer);

	kthread_queue_work(pm_data->kworker, &pm_data->check_work);

	hrtimer_forward_now(timer, ms_to_ktime(1000));

	return HRTIMER_RESTART;
}

/**
 * pcat_pm_watchdog_timeout_set - Configure PMU watchdog timeouts
 * @pm_data: Driver data
 * @interval: Heartbeat interval in seconds (0 to disable)
 * @timeout: UART write timeout in jiffies
 *
 * Sends watchdog configuration to PMU. The PMU will power off the
 * system if heartbeats stop arriving within the configured timeout.
 */
static void pcat_pm_watchdog_timeout_set(struct pcat_pm_data *pm_data,
	u8 interval, long timeout)
{
	u8 timeouts[3] = {60, pm_data->force_poweroff_timeout, interval};

	pcat_pm_uart_write_data(pm_data, PCAT_PM_COMMAND_WATCHDOG_TIMEOUT_SET,
		timeouts, 3, true, timeout);
}

/**
 * pcat_pm_worker_stop - Stop the worker thread and timer
 * @pm_data: Driver data
 */
static void pcat_pm_worker_stop(struct pcat_pm_data *pm_data)
{
	if (IS_ERR(pm_data->kworker))
		return;

	hrtimer_cancel(&pm_data->check_timer);
	kthread_cancel_work_sync(&pm_data->check_work);
}

/**
 * pcat_pm_do_poweroff - System power off handler
 * @data: sys_off_data containing driver data
 *
 * Called during system poweroff. Notifies PMU to cut power after
 * the host has shut down.
 *
 * Return: NOTIFY_DONE
 */
static int pcat_pm_do_poweroff(struct sys_off_data *data)
{
	struct pcat_pm_data *pm_data = data->cb_data;
	unsigned int try_count;

	dev_info(&pm_data->serdev->dev, "Shutdown process starts.\n");

	pm_data->work_flag = false;
	pcat_pm_worker_stop(pm_data);

	pcat_pm_uart_write_data(pm_data, PCAT_PM_COMMAND_HOST_REQUEST_SHUTDOWN,
		NULL, 0, true, msecs_to_jiffies(1000));

	for (try_count = 0; try_count < 50; try_count++) {
		if (READ_ONCE(pm_data->poweroff_ok))
			break;
		mdelay(100);
	}

	if (READ_ONCE(pm_data->poweroff_ok))
		dev_info(&pm_data->serdev->dev, "Shutdown request received.\n");
	else
		dev_err(&pm_data->serdev->dev, "Shutdown request timeout!\n");

	if (!IS_ERR(pm_data->power_gpio))
		gpiod_direction_output(pm_data->power_gpio, 0);
	mdelay(100);

	return NOTIFY_DONE;
}

/**
 * pcat_pm_do_restart - System restart handler
 * @data: sys_off_data containing driver data
 *
 * Called during system restart. Disables watchdog to allow reboot.
 *
 * Return: NOTIFY_DONE
 */
static int pcat_pm_do_restart(struct sys_off_data *data)
{
	struct pcat_pm_data *pm_data = data->cb_data;

	dev_info(&pm_data->serdev->dev, "Reboot process starts.\n");

	pcat_pm_watchdog_timeout_set(pm_data, 0, msecs_to_jiffies(1000));

	mdelay(100);

	pm_data->work_flag = false;
	pcat_pm_worker_stop(pm_data);

	dev_info(&pm_data->serdev->dev, "Reboot process completed.\n");

	return NOTIFY_DONE;
}

/**
 * pcat_pm_probe - Driver probe function
 * @serdev: Serial device
 *
 * Initializes all driver subsystems: UART, power supply, RTC,
 * hwmon, fan control, misc device, and sysfs.
 *
 * Return: 0 on success, negative error otherwise
 */
static int pcat_pm_probe(struct serdev_device *serdev)
{
	struct pcat_pm_data *pm_data;
	struct device *dev = &serdev->dev;
	int ret;

	pm_data = devm_kzalloc(dev, sizeof(*pm_data), GFP_KERNEL);
	if (!pm_data)
		return -ENOMEM;

	pm_data->serdev = serdev;
	pm_data->work_flag = true;

	mutex_init(&pm_data->mutex);
	mutex_init(&pm_data->ctl_mutex);
	init_waitqueue_head(&pm_data->ctl_wait);

	pm_data->status_report_timestamp = ktime_get_boottime_ns();
	pm_data->status_report_timeout_warn_timestamp = pm_data->status_report_timestamp;

	ret = pcat_pm_sysfs_init(pm_data);
	if (ret)
		dev_err(dev, "Failed to initialize sysfs: %d\n", ret);

	if (device_property_read_u32(dev, "baudrate", &pm_data->baudrate))
		pm_data->baudrate = 115200;

	if (device_property_read_u32(dev, "force-poweroff-timeout",
				    &pm_data->force_poweroff_timeout))
		pm_data->force_poweroff_timeout = 0;

	serdev_device_set_drvdata(serdev, pm_data);
	serdev_device_set_client_ops(serdev, &pcat_pm_serdev_ops);

	pm_data->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_HIGH);
	if (IS_ERR(pm_data->power_gpio))
		dev_err(dev, "Failed to setup power GPIO!\n");

	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF_PREPARE,
		SYS_OFF_PRIO_FIRMWARE, pcat_pm_do_poweroff, pm_data);
	if (ret)
		dev_err(dev, "Cannot register poweroff handler: %d\n", ret);

	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_RESTART,
		SYS_OFF_PRIO_HIGH, pcat_pm_do_restart, pm_data);
	if (ret)
		dev_err(dev, "Cannot register restart handler: %d\n", ret);

	ret = pcat_pm_uart_serdev_open(pm_data);
	if (ret)
		dev_err(dev, "Cannot open serial device: %d\n", ret);

	kthread_init_work(&pm_data->check_work, pcat_pm_check_work);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
	hrtimer_setup(&pm_data->check_timer, pcat_pm_check_timer_expired,
		CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
#else
	hrtimer_init(&pm_data->check_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
	pm_data->check_timer.function = pcat_pm_check_timer_expired;
#endif

	hrtimer_start(&pm_data->check_timer, ms_to_ktime(1000), HRTIMER_MODE_REL_HARD);

	pcat_pm_watchdog_timeout_set(pm_data, PCAT_PM_WATCHDOG_DEFAULT_INTERVAL, 0);

	ret = pcat_pm_charger_probe(pm_data);
	if (ret)
		dev_err(dev, "Failed to probe charger: %d\n", ret);

	ret = pcat_pm_rtc_probe(pm_data);
	if (ret)
		dev_err(dev, "Failed to probe RTC: %d\n", ret);

	ret = pcat_pm_hwmon_probe(pm_data);
	if (ret)
		dev_err(dev, "Failed to probe hwmon: %d\n", ret);

	pm_data->ctl_device.minor = MISC_DYNAMIC_MINOR;
	pm_data->ctl_device.name = "pcat-pm-ctl";
	pm_data->ctl_device.fops = &pcat_pm_ctl_dev_ops;
	pm_data->ctl_device.mode = 0;
	ret = misc_register(&pm_data->ctl_device);
	if (ret)
		dev_err(dev, "Failed to register control device: %d\n", ret);

	ret = pcat_pm_fan_probe(pm_data);
	if (ret)
		dev_err(dev, "Failed to register cooling device: %d\n", ret);

	/* Query PMU hardware version */
	pcat_pm_uart_write_data(pm_data,
		PCAT_PM_COMMAND_PMU_HW_VERSION_GET, NULL, 0, true, 0);

	/* Query PMU firmware version */
	pcat_pm_uart_write_data(pm_data,
		PCAT_PM_COMMAND_PMU_FW_VERSION_GET, NULL, 0, true, 0);

	/* Query power-on event */
	pcat_pm_uart_write_data(pm_data,
		PCAT_PM_COMMAND_POWER_ON_EVENT_GET, NULL, 0, true, 0);

	/* Query initial LED/beeper state from PMU (0xFF = get current state) */
	{
		u8 query = 0xFF;

		pcat_pm_uart_write_data(pm_data,
			PCAT_PM_COMMAND_STATUS_LED_BEEPER_V2_SET,
			&query, 1, true, 0);
	}

	dev_info(dev, "photonicat power manager initialized OK.\n");

	return 0;
}

#ifdef CONFIG_PM_SLEEP
/**
 * pcat_pm_pm_suspend - System suspend handler
 * @dev: Device
 *
 * Disables watchdog during suspend.
 *
 * Return: 0
 */
static int pcat_pm_pm_suspend(struct device *dev)
{
	struct pcat_pm_data *pm_data = dev_get_drvdata(dev);

	pcat_pm_watchdog_timeout_set(pm_data, 0, 0);

	return 0;
}

/**
 * pcat_pm_pm_resume - System resume handler
 * @dev: Device
 *
 * Re-enables watchdog after resume.
 *
 * Return: 0
 */
static int pcat_pm_pm_resume(struct device *dev)
{
	struct pcat_pm_data *pm_data = dev_get_drvdata(dev);

	pcat_pm_watchdog_timeout_set(pm_data, PCAT_PM_WATCHDOG_DEFAULT_INTERVAL, 0);

	return 0;
}
#endif

static const struct of_device_id pcat_pm_of_match[] = {
	{ .compatible = "photonicat-pm" },
	{}
};
MODULE_DEVICE_TABLE(of, pcat_pm_of_match);

static const struct dev_pm_ops pcat_pm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pcat_pm_pm_suspend, pcat_pm_pm_resume)
};

/**
 * pcat_pm_remove - Driver remove function
 * @serdev: Serial device
 *
 * Cleans up all non-devm-managed resources.
 */
static void pcat_pm_remove(struct serdev_device *serdev)
{
	struct pcat_pm_data *pm_data = serdev_device_get_drvdata(serdev);

	pm_data->work_flag = false;
	pcat_pm_worker_stop(pm_data);

	if (!IS_ERR(pm_data->kworker))
		kthread_destroy_worker(pm_data->kworker);

	misc_deregister(&pm_data->ctl_device);
	pcat_pm_charger_remove(pm_data);
	pcat_pm_sysfs_cleanup(pm_data);
}

static struct serdev_device_driver pcat_pm_driver = {
	.driver = {
		.name = "photonicat-pm",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pcat_pm_of_match),
		.pm = &pcat_pm_pm_ops,
	},
	.probe = pcat_pm_probe,
	.remove = pcat_pm_remove,
};

module_serdev_device_driver(pcat_pm_driver);

MODULE_DESCRIPTION("Photonicat Power Manager Driver");
MODULE_AUTHOR("Kyosuke Nekoyashiki <supercatexpert@gmail.com>");
MODULE_AUTHOR("HackingGate <i@hackinggate.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:photonicat-pm");
