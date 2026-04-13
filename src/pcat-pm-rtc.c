// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Photonicat Power Manager Driver - RTC Module
 *
 * Handles real-time clock interface.
 *
 * Copyright (c) 2025, Kyosuke Nekoyashiki <supercatexpert@gmail.com>
 * Copyright (c) 2026, HackingGate <i@hackinggate.com>
 */

#include "photonicat-pm.h"

#define PCAT_PM_RTC_ACK_TIMEOUT_MS 1000

static void pcat_pm_rtc_ack_reset(struct pcat_pm_data *pm_data, bool *ack_seen,
	u16 *ack_frame, u8 *ack_status)
{
	mutex_lock(&pm_data->mutex);
	*ack_seen = false;
	*ack_frame = 0;
	*ack_status = 0;
	mutex_unlock(&pm_data->mutex);
}

static int pcat_pm_rtc_wait_for_ack(struct pcat_pm_data *pm_data, bool *ack_seen,
	u16 *ack_frame, u8 *ack_status, u16 expected_frame)
{
	long timeout;

	timeout = wait_event_timeout(pm_data->rtc_cmd_wait,
		READ_ONCE(*ack_seen) && READ_ONCE(*ack_frame) == expected_frame,
		msecs_to_jiffies(PCAT_PM_RTC_ACK_TIMEOUT_MS));
	if (!timeout)
		return -ETIMEDOUT;

	return READ_ONCE(*ack_status) ? -EIO : 0;
}

static int pcat_pm_rtc_read_time(struct device *dev, struct rtc_time *t)
{
	struct pcat_pm_data *pm_data = pcat_pm_get_data_from_dev(dev);
	struct rtc_time rtc_time = { 0 };
	unsigned int try_count;

	/* Wait up to 2 seconds for the first status report from PMU */
	for (try_count = 0; try_count < 20; try_count++) {
		mutex_lock(&pm_data->mutex);
		if (pm_data->rtc_year != 0) {
			rtc_time.tm_year = pm_data->rtc_year - 1900;
			rtc_time.tm_mon = pm_data->rtc_month;
			rtc_time.tm_mday = pm_data->rtc_day;
			rtc_time.tm_hour = pm_data->rtc_hour;
			rtc_time.tm_min = pm_data->rtc_min;
			rtc_time.tm_sec = pm_data->rtc_sec;
			rtc_time.tm_wday = pm_data->rtc_wday;
			mutex_unlock(&pm_data->mutex);

			if (rtc_valid_tm(&rtc_time))
				return -EINVAL;

			*t = rtc_time;
			return 0;
		}
		mutex_unlock(&pm_data->mutex);

		msleep(100);
	}

	return -EINVAL;
}

static int pcat_pm_rtc_set_time(struct device *dev, struct rtc_time *t)
{
	struct pcat_pm_data *pm_data = pcat_pm_get_data_from_dev(dev);
	u8 date_data[7];
	u16 y;
	u16 frame_num;
	int ret;

	if (rtc_valid_tm(t))
		return -EINVAL;

	y = t->tm_year + 1900;
	date_data[0] = y & 0xFF;
	date_data[1] = (y >> 8) & 0xFF;
	date_data[2] = t->tm_mon + 1;
	date_data[3] = t->tm_mday;
	date_data[4] = t->tm_hour;
	date_data[5] = t->tm_min;
	date_data[6] = t->tm_sec;

	mutex_lock(&pm_data->rtc_cmd_mutex);
	pcat_pm_rtc_ack_reset(pm_data, &pm_data->rtc_sync_ack_seen,
		&pm_data->rtc_sync_ack_frame, &pm_data->rtc_sync_ack_status);
	ret = pcat_pm_uart_write_data_frame(pm_data, PCAT_PM_COMMAND_DATE_TIME_SYNC,
		date_data, 7, true, 0, &frame_num);
	if (ret < 0)
		goto out_unlock;

	ret = pcat_pm_rtc_wait_for_ack(pm_data, &pm_data->rtc_sync_ack_seen,
		&pm_data->rtc_sync_ack_frame, &pm_data->rtc_sync_ack_status,
		frame_num);
	if (ret < 0)
		goto out_unlock;

	mutex_lock(&pm_data->mutex);
	pm_data->rtc_year = y;
	pm_data->rtc_month = t->tm_mon;
	pm_data->rtc_day = t->tm_mday;
	pm_data->rtc_hour = t->tm_hour;
	pm_data->rtc_min = t->tm_min;
	pm_data->rtc_sec = t->tm_sec;
	pm_data->rtc_wday = t->tm_wday % 7;
	mutex_unlock(&pm_data->mutex);

	ret = 0;

out_unlock:
	mutex_unlock(&pm_data->rtc_cmd_mutex);
	return ret;
}

/**
 * pcat_pm_schedule_boot_send - Send schedule boot data to PMU
 * @pm_data: Driver data
 *
 * Sends the current alarm time to the PMU as a scheduled startup entry,
 * or clears the schedule if the alarm is disabled.
 *
 * Return: 0 on success, negative error otherwise
 */
static int pcat_pm_schedule_boot_send(struct pcat_pm_data *pm_data)
{
	u8 schedule_data[8];
	bool already_sent, enabled;
	struct rtc_time alarm;
	u16 frame_num;
	u16 year;
	int ret;

	mutex_lock(&pm_data->rtc_cmd_mutex);
	mutex_lock(&pm_data->mutex);
	already_sent = pm_data->schedule_boot_sent;
	enabled = pm_data->alarm_enabled;
	alarm = pm_data->alarm_time;
	mutex_unlock(&pm_data->mutex);

	if (already_sent) {
		ret = 0;
		goto out_unlock;
	}

	pcat_pm_rtc_ack_reset(pm_data, &pm_data->schedule_boot_ack_seen,
		&pm_data->schedule_boot_ack_frame,
		&pm_data->schedule_boot_ack_status);

	if (!enabled) {
		ret = pcat_pm_uart_write_data_frame(pm_data,
			PCAT_PM_COMMAND_SCHEDULE_STARTUP_TIME_SET,
			NULL, 0, true, 0, &frame_num);
		if (ret < 0)
			goto out_unlock;

		ret = pcat_pm_rtc_wait_for_ack(pm_data,
			&pm_data->schedule_boot_ack_seen,
			&pm_data->schedule_boot_ack_frame,
			&pm_data->schedule_boot_ack_status, frame_num);
		if (ret < 0)
			goto out_unlock;
		goto mark_sent;
	}

	year = alarm.tm_year + 1900;
	schedule_data[0] = year & 0xFF;
	schedule_data[1] = (year >> 8) & 0xFF;
	schedule_data[2] = alarm.tm_mon + 1;
	schedule_data[3] = alarm.tm_mday;
	schedule_data[4] = alarm.tm_hour;
	schedule_data[5] = alarm.tm_min;
	schedule_data[6] = 0; /* dow_bits: unused for specific date */
	schedule_data[7] = 0x1F; /* enable year|month|day|hour|minute */

	ret = pcat_pm_uart_write_data_frame(pm_data,
		PCAT_PM_COMMAND_SCHEDULE_STARTUP_TIME_SET,
		schedule_data, 8, true, 0, &frame_num);
	if (ret < 0)
		goto out_unlock;

	ret = pcat_pm_rtc_wait_for_ack(pm_data, &pm_data->schedule_boot_ack_seen,
		&pm_data->schedule_boot_ack_frame,
		&pm_data->schedule_boot_ack_status, frame_num);
	if (ret < 0)
		goto out_unlock;

mark_sent:
	mutex_lock(&pm_data->mutex);
	pm_data->schedule_boot_sent = true;
	mutex_unlock(&pm_data->mutex);
	ret = 0;

out_unlock:
	mutex_unlock(&pm_data->rtc_cmd_mutex);
	return ret;
}

static int pcat_pm_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct pcat_pm_data *pm_data = pcat_pm_get_data_from_dev(dev);

	mutex_lock(&pm_data->mutex);
	alrm->time = pm_data->alarm_time;
	alrm->enabled = pm_data->alarm_enabled;
	alrm->pending = 0;
	mutex_unlock(&pm_data->mutex);

	return 0;
}

static int pcat_pm_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct pcat_pm_data *pm_data = pcat_pm_get_data_from_dev(dev);

	mutex_lock(&pm_data->mutex);
	if (pm_data->alarm_enabled != alrm->enabled ||
	    memcmp(&pm_data->alarm_time, &alrm->time, sizeof(struct rtc_time)))
		pm_data->schedule_boot_sent = false;
	pm_data->alarm_time = alrm->time;
	pm_data->alarm_enabled = alrm->enabled;
	mutex_unlock(&pm_data->mutex);

	return pcat_pm_schedule_boot_send(pm_data);
}

static int pcat_pm_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct pcat_pm_data *pm_data = pcat_pm_get_data_from_dev(dev);

	mutex_lock(&pm_data->mutex);
	if (pm_data->alarm_enabled != !!enabled)
		pm_data->schedule_boot_sent = false;
	pm_data->alarm_enabled = !!enabled;
	mutex_unlock(&pm_data->mutex);

	return pcat_pm_schedule_boot_send(pm_data);
}

static int pcat_pm_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct pcat_pm_data *pm_data = pcat_pm_get_data_from_dev(dev);
	struct rtc_time rtc_time = { 0 };
	bool rtc_ready;
	int flags = 0;

	switch (cmd) {
	case RTC_VL_READ:
		mutex_lock(&pm_data->mutex);
		rtc_ready = pm_data->rtc_year != 0;
		rtc_time.tm_year = pm_data->rtc_year ? pm_data->rtc_year - 1900 : 0;
		rtc_time.tm_mon = pm_data->rtc_month;
		rtc_time.tm_mday = pm_data->rtc_day;
		rtc_time.tm_hour = pm_data->rtc_hour;
		rtc_time.tm_min = pm_data->rtc_min;
		rtc_time.tm_sec = pm_data->rtc_sec;
		rtc_time.tm_wday = pm_data->rtc_wday;
		mutex_unlock(&pm_data->mutex);

		if (!rtc_ready || rtc_valid_tm(&rtc_time))
			flags |= RTC_VL_DATA_INVALID;

		return put_user(flags, (unsigned int __user *)arg);

	default:
		return -ENOIOCTLCMD;
	}
}

static const struct rtc_class_ops pcat_pm_rtcops = {
	.read_time		= pcat_pm_rtc_read_time,
	.set_time		= pcat_pm_rtc_set_time,
	.read_alarm		= pcat_pm_rtc_read_alarm,
	.set_alarm		= pcat_pm_rtc_set_alarm,
	.alarm_irq_enable	= pcat_pm_rtc_alarm_irq_enable,
	.ioctl			= pcat_pm_rtc_ioctl,
};

int pcat_pm_rtc_probe(struct pcat_pm_data *pm_data)
{
	int ret;

	pm_data->rtc = devm_rtc_allocate_device(&pm_data->serdev->dev);
	if (IS_ERR(pm_data->rtc)) {
		ret = PTR_ERR(pm_data->rtc);
		dev_err(&pm_data->serdev->dev, "Cannot allocate RTC device: %d\n", ret);
		return ret;
	}

	pm_data->rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	pm_data->rtc->range_max = RTC_TIMESTAMP_END_2099;
	pm_data->rtc->ops = &pcat_pm_rtcops;
	set_bit(RTC_FEATURE_ALARM, pm_data->rtc->features);

	ret = devm_rtc_register_device(pm_data->rtc);
	if (ret) {
		dev_err(&pm_data->serdev->dev, "Failed to register RTC device: %d\n", ret);
		return ret;
	}

	return 0;
}
