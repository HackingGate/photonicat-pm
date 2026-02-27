// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Photonicat Power Manager Driver - Control Device Module
 *
 * Handles misc device for userspace control.
 *
 * Copyright (c) 2025, Kyosuke Nekoyashiki <supercatexpert@gmail.com>
 * Copyright (c) 2026, HackingGate <i@hackinggate.com>
 */

#include <linux/poll.h>

#include "photonicat-pm.h"

void pcat_pm_ctl_cmd_exec(struct pcat_pm_data *pm_data,
	const u8 *rawdata, size_t rawdata_len, u8 src, u8 dst, u16 frame_num,
	u16 command, const u8 *extra_data, u16 extra_data_len, bool need_ack)
{
	bool forward_cmd = false;
	int ret;

	switch (command) {
	case PCAT_PM_COMMAND_HEARTBEAT:
		break;
	case PCAT_PM_COMMAND_HEARTBEAT_ACK:
		break;
	case PCAT_PM_COMMAND_STATUS_REPORT:
		break;
	case PCAT_PM_COMMAND_STATUS_REPORT_ACK:
		break;
	case PCAT_PM_COMMAND_DATE_TIME_SYNC:
		break;
	case PCAT_PM_COMMAND_DATE_TIME_SYNC_ACK:
		break;
	case PCAT_PM_COMMAND_HOST_REQUEST_SHUTDOWN:
		break;
	case PCAT_PM_COMMAND_HOST_REQUEST_SHUTDOWN_ACK:
		break;
	case PCAT_PM_COMMAND_PMU_REQUEST_SHUTDOWN:
		break;
	case PCAT_PM_COMMAND_PMU_REQUEST_SHUTDOWN_ACK:
		break;
	case PCAT_PM_COMMAND_WATCHDOG_TIMEOUT_SET:
		break;
	case PCAT_PM_COMMAND_WATCHDOG_TIMEOUT_SET_ACK:
		break;
	case PCAT_PM_COMMAND_FAN_SET:
		break;
	case PCAT_PM_COMMAND_FAN_SET_ACK:
		break;
	default:
		forward_cmd = true;
		break;
	}

	if (!forward_cmd)
		return;

	ret = serdev_device_write_buf(pm_data->serdev, rawdata, rawdata_len);

	if (ret < 0)
		dev_err(&pm_data->serdev->dev, "Failed to write serial port: %d\n", ret);
}

static ssize_t pcat_pm_ctl_dev_read(struct file *file, char *buffer,
	size_t count, loff_t *ppos)
{
	struct miscdevice *mdev = file->private_data;
	struct pcat_pm_data *pm_data;
	int ret;
	size_t copy_size = count;

	pm_data = container_of(mdev, struct pcat_pm_data, ctl_device);

	if (!pm_data->ctl_write_buffer_ready) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(pm_data->ctl_wait,
			pm_data->ctl_write_buffer_ready);
		if (ret)
			return ret;
	}

	mutex_lock(&pm_data->ctl_mutex);
	if (pm_data->ctl_write_buffer_used > 0) {
		if (copy_size > pm_data->ctl_write_buffer_used)
			copy_size = pm_data->ctl_write_buffer_used;
		ret = copy_to_user(buffer, pm_data->ctl_write_buffer, copy_size);
		if (!ret) {
			if (copy_size < pm_data->ctl_write_buffer_used) {
				memmove(pm_data->ctl_write_buffer,
					pm_data->ctl_write_buffer + copy_size,
					pm_data->ctl_write_buffer_used - copy_size);
				pm_data->ctl_write_buffer_used -= copy_size;
			} else {
				pm_data->ctl_write_buffer_used = 0;
			}

			if (pm_data->ctl_write_buffer_used == 0)
				pm_data->ctl_write_buffer_ready = false;
		}
		mutex_unlock(&pm_data->ctl_mutex);

		if (ret)
			return -EFAULT;
	} else {
		pm_data->ctl_write_buffer_ready = false;
		mutex_unlock(&pm_data->ctl_mutex);
		copy_size = 0;
	}

	return copy_size;
}

static ssize_t pcat_pm_ctl_dev_write(struct file *file, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	struct miscdevice *mdev = file->private_data;
	struct pcat_pm_data *pm_data;
	u16 crc;
	u16 expect_len;

	pm_data = container_of(mdev, struct pcat_pm_data, ctl_device);

	if (count > PCAT_PM_BUFFER_SIZE)
		count = PCAT_PM_BUFFER_SIZE;

	mutex_lock(&pm_data->ctl_mutex);

	if (copy_from_user(pm_data->ctl_read_buffer, buffer, count)) {
		mutex_unlock(&pm_data->ctl_mutex);
		return -EFAULT;
	}

	if (count < 13) {
		mutex_unlock(&pm_data->ctl_mutex);
		return count;
	}

	pm_data->ctl_read_buffer_used = count;
	expect_len = pm_data->ctl_read_buffer[5] +
		((u16)pm_data->ctl_read_buffer[6] << 8);
	if (expect_len + 10 > count) {
		mutex_unlock(&pm_data->ctl_mutex);
		return count;
	}

	mutex_lock(&pm_data->mutex);
	pm_data->ctl_read_buffer[3] = pm_data->write_framenum & 0xFF;
	pm_data->ctl_read_buffer[4] = (pm_data->write_framenum >> 8) & 0xFF;
	pm_data->write_framenum++;
	mutex_unlock(&pm_data->mutex);

	crc = pcat_pm_compute_crc16(pm_data->ctl_read_buffer + 1, 6 + expect_len);
	pm_data->ctl_read_buffer[7 + expect_len] = crc & 0xFF;
	pm_data->ctl_read_buffer[8 + expect_len] = (crc >> 8) & 0xFF;

	pcat_pm_uart_receive_parse(pm_data, pm_data->ctl_read_buffer,
		&pm_data->ctl_read_buffer_used, pcat_pm_ctl_cmd_exec);

	mutex_unlock(&pm_data->ctl_mutex);

	return count;
}

static __poll_t pcat_pm_ctl_dev_poll(struct file *file, poll_table *pt)
{
	struct miscdevice *mdev = file->private_data;
	struct pcat_pm_data *pm_data;
	__poll_t mask;

	pm_data = container_of(mdev, struct pcat_pm_data, ctl_device);

	poll_wait(file, &pm_data->ctl_wait, pt);

	mask = EPOLLOUT | EPOLLWRNORM;
	if (pm_data->ctl_write_buffer_ready)
		mask |= EPOLLIN | EPOLLRDNORM;

	return mask;
}

const struct file_operations pcat_pm_ctl_dev_ops = {
	.owner = THIS_MODULE,
	.read = pcat_pm_ctl_dev_read,
	.write = pcat_pm_ctl_dev_write,
	.llseek = noop_llseek,
	.poll = pcat_pm_ctl_dev_poll,
};
