// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Photonicat Power Manager Driver - UART Protocol Module
 *
 * Handles serial communication with the PMU including:
 * - CRC16 checksum calculation (Modbus style)
 * - Packet framing and parsing
 * - Status report parsing
 * - Command execution dispatch
 *
 * Protocol Frame Format:
 *   [0xA5] [src] [dst] [frame_lo] [frame_hi] [len_lo] [len_hi]
 *   [cmd_lo] [cmd_hi] [payload...] [need_ack] [crc_lo] [crc_hi] [0x5A]
 *
 * Copyright (c) 2025, Kyosuke Nekoyashiki <supercatexpert@gmail.com>
 * Copyright (c) 2026, HackingGate <i@hackinggate.com>
 */

#include "photonicat-pm.h"

/**
 * pcat_pm_compute_crc16 - Compute Modbus CRC16 checksum
 * @data: Input data buffer
 * @len: Length of data
 *
 * Uses polynomial 0xA001 (bit-reversed 0x8005).
 *
 * Return: 16-bit CRC value
 */
u16 pcat_pm_compute_crc16(const u8 *data, size_t len)
{
	u16 crc = 0xFFFF;
	size_t i;
	unsigned int j;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (j = 0; j < 8; j++) {
			if (crc & 1)
				crc = (crc >> 1) ^ 0xA001;
			else
				crc >>= 1;
		}
	}

	return crc;
}

/**
 * pcat_pm_uart_write_data - Send a command packet to PMU
 * @pm_data: Driver data
 * @command: Command type (PCatPMCommandType)
 * @extra_data: Optional payload (NULL if none)
 * @extra_data_len: Payload length (max 512 bytes)
 * @need_ack: Request acknowledgment from PMU
 * @timeout: Write timeout in jiffies (0 for non-blocking)
 *
 * Constructs a framed packet with CRC and sends via serial.
 *
 * Return: Bytes written on success, negative errno on failure
 */
int pcat_pm_uart_write_data(struct pcat_pm_data *pm_data,
	u16 command, const u8 *extra_data, u16 extra_data_len,
	bool need_ack, long timeout)
{
	u8 data[1024];
	size_t data_size = 0;
	u16 sv;
	u16 dp_size;
	int ret;

	/* Header: sync byte + source (0x01 = host) + destination (0x81 = PMU) */
	memcpy(data, (const u8 *)"\xA5\x01\x81", 3);
	data_size += 3;

	/* Frame number (auto-incrementing) */
	mutex_lock(&pm_data->mutex);
	data[data_size] = pm_data->write_framenum & 0xFF;
	data[data_size + 1] = (pm_data->write_framenum >> 8) & 0xFF;
	data_size += 2;
	pm_data->write_framenum++;
	mutex_unlock(&pm_data->mutex);

	/* Payload length + command */
	if (extra_data != NULL && extra_data_len > 0 && extra_data_len <= 512) {
		sv = extra_data_len + 3;
		data[data_size] = sv & 0xFF;
		data[data_size + 1] = (sv >> 8) & 0xFF;
		data_size += 2;

		sv = command;
		data[data_size] = sv & 0xFF;
		data[data_size + 1] = (sv >> 8) & 0xFF;
		data_size += 2;

		memcpy(data + data_size, extra_data, extra_data_len);
		data_size += extra_data_len;

		dp_size = extra_data_len + 3;
	} else {
		sv = 3;
		data[data_size] = sv & 0xFF;
		data[data_size + 1] = (sv >> 8) & 0xFF;
		data_size += 2;

		sv = command;
		data[data_size] = sv & 0xFF;
		data[data_size + 1] = (sv >> 8) & 0xFF;
		data_size += 2;

		dp_size = 3;
	}

	/* ACK request flag */
	data[data_size] = need_ack ? 1 : 0;
	data_size++;

	/* CRC16 */
	sv = pcat_pm_compute_crc16(data + 1, dp_size + 6);
	data[data_size] = sv & 0xFF;
	data[data_size + 1] = (sv >> 8) & 0xFF;
	data_size += 2;

	/* End marker */
	data[data_size] = 0x5A;
	data_size++;

	if (timeout > 0)
		ret = serdev_device_write(pm_data->serdev, data, data_size, timeout);
	else
		ret = serdev_device_write_buf(pm_data->serdev, data, data_size);

	if (ret < 0)
		dev_err(&pm_data->serdev->dev, "Failed to write serial port: %d\n", ret);

	return ret;
}

/**
 * pcat_pm_status_report_parse - Parse PMU status report payload
 * @pm_data: Driver data
 * @data: Status report data
 * @data_len: Data length
 *
 * Extracts battery voltage, charger voltage, temperature, current,
 * SOC, energy, accelerometer data, and fan speed from status reports.
 */
static void pcat_pm_status_report_parse(struct pcat_pm_data *pm_data,
	const u8 *data, size_t data_len)
{
	u16 battery_voltage, charger_voltage;
	u16 gpio_input, gpio_output;
	int temp = 0;
	u16 battery_current_raw = 0;
	s16 battery_current = 0;
	bool on_battery, on_charger;
	bool prev_on_battery, prev_on_charger;
	bool notify_power_supply = false;
	int soc;
	u32 energy_now = 0, energy_full = 0;
	int gs_x = 0, gs_y = 0, gs_z = 0;
	bool gs_ready = false;
	u32 fan_rpm = 0;

	if (data_len < 16)
		return;

	/* Bytes 0-7: Voltages and GPIO states */
	battery_voltage = data[0] | ((u16)data[1] << 8);
	charger_voltage = data[2] | ((u16)data[3] << 8);
	on_charger = (charger_voltage >= 4200);
	gpio_input = data[4] | ((u16)data[5] << 8);
	gpio_output = data[6] | ((u16)data[7] << 8);

	(void)gpio_input;
	(void)gpio_output;

	/* Bytes 17-19: Temperature and current (v2 protocol) */
	if (data_len >= 20) {
		temp = (int)data[17] - 100;
		battery_current_raw = data[18] + ((u16)data[19] << 8);
		battery_current = (s16)battery_current_raw;
		/*
		 * PMU raw current sign: positive = discharging (out of battery),
		 * negative = charging (into battery).
		 * Stored value is negated (line ~227) for Linux power_supply convention.
		 * Keep unplugged path forced to battery mode even if current sign is noisy.
		 */
		on_battery = (battery_current > 0);
		if (!on_charger)
			on_battery = true;
	} else {
		on_battery = !on_charger;
	}

	/* Bytes 22-30: SOC and energy (v2 protocol) */
	if (data_len >= 31) {
		energy_now = data[23] | ((u32)data[24] << 8) |
			((u32)data[25] << 16) | ((u32)data[26] << 24);
		energy_full = data[27] | ((u32)data[28] << 8) |
			((u32)data[29] << 16) | ((u32)data[30] << 24);

		soc = data[22];
	} else {
		soc = power_supply_batinfo_ocv2cap(
			pm_data->battery_info, (int)battery_voltage * 1000, 20);
	}

	/* Bytes 35-51: Accelerometer and fan speed */
	if (data_len >= 52) {
		gs_x = data[35] | ((u32)data[36] << 8) |
			((u32)data[37] << 16) | ((u32)data[38] << 24);
		gs_y = data[39] | ((u32)data[40] << 8) |
			((u32)data[41] << 16) | ((u32)data[42] << 24);
		gs_z = data[43] | ((u32)data[44] << 8) |
			((u32)data[45] << 16) | ((u32)data[46] << 24);
		gs_ready = (data[47] != 0);
		fan_rpm = data[48] | ((u32)data[49] << 8) |
			((u32)data[50] << 16) | ((u32)data[51] << 24);
	}

	mutex_lock(&pm_data->mutex);
	prev_on_battery = pm_data->on_battery;
	prev_on_charger = pm_data->on_charger;
	pm_data->status_report_timestamp = ktime_get_boottime_ns();
	pm_data->battery_voltage_now = battery_voltage * 1000;
	pm_data->charger_voltage_now = charger_voltage * 1000;
	pm_data->battery_current_now = -battery_current * 1000;
	pm_data->battery_soc = soc;
	pm_data->battery_energy_now = energy_now * 1000;
	pm_data->battery_energy_full = energy_full * 1000;
	pm_data->on_battery = on_battery;
	pm_data->on_charger = on_charger;
	pm_data->rtc_year = data[8] + ((u16)data[9] << 8);
	pm_data->rtc_month = data[10] - 1;
	pm_data->rtc_day = data[11];
	pm_data->rtc_hour = data[12];
	pm_data->rtc_min = data[13];
	pm_data->rtc_sec = data[14];
	pm_data->rtc_status = data[15];
	pm_data->board_temp = temp;
	pm_data->gs_x = gs_x;
	pm_data->gs_y = gs_y;
	pm_data->gs_z = gs_z;
	pm_data->gs_ready = gs_ready;
	pm_data->fan_rpm = fan_rpm;
	if (!pm_data->ps_initialized) {
		pm_data->ps_initialized = true;
		notify_power_supply = true;
	} else if (on_battery != prev_on_battery || on_charger != prev_on_charger) {
		notify_power_supply = true;
	}
	mutex_unlock(&pm_data->mutex);

	if (notify_power_supply) {
		if (!IS_ERR_OR_NULL(pm_data->battery_psy))
			power_supply_changed(pm_data->battery_psy);
		if (!IS_ERR_OR_NULL(pm_data->charger_psy))
			power_supply_changed(pm_data->charger_psy);
	}
}

/**
 * pcat_pm_uart_cmd_exec - Execute received PMU commands
 * @pm_data: Driver data
 * @rawdata: Raw packet data (for forwarding to userspace)
 * @rawdata_len: Packet length
 * @src: Source address
 * @dst: Destination address
 * @frame_num: Frame sequence number
 * @command: Command type
 * @extra_data: Command payload
 * @extra_data_len: Payload length
 * @need_ack: Acknowledgment requested
 *
 * Handles known commands internally and forwards unknown commands
 * to the userspace control device.
 */
void pcat_pm_uart_cmd_exec(struct pcat_pm_data *pm_data,
	const u8 *rawdata, size_t rawdata_len, u8 src, u8 dst, u16 frame_num,
	u16 command, const u8 *extra_data, u16 extra_data_len, bool need_ack)
{
	size_t overflow_size;

	/* Filter by destination: host (0x01), broadcast (0x80), or all (0xFF) */
	if (dst != 0x1 && dst != 0x80 && dst != 0xFF)
		return;

	/* Log all received PMU commands for debugging.
	 * Uses dev_dbg so these are compiled out by default.
	 * Enable via: CONFIG_DYNAMIC_DEBUG at runtime, or -DDEBUG at build time.
	 */
	if (extra_data_len > 0) {
		char hex_buf[16 * 3 + 1];
		size_t dump_len = min_t(size_t, extra_data_len, 16);
		size_t k;

		for (k = 0; k < dump_len; k++)
			snprintf(hex_buf + k * 3, 4, "%02X ", extra_data[k]);
		hex_buf[dump_len * 3] = '\0';

		dev_dbg(&pm_data->serdev->dev,
			"PMU cmd=0x%04X src=0x%02X dst=0x%02X frame=%u len=%u data=[%s]\n",
			command, src, dst, frame_num, extra_data_len, hex_buf);
	} else {
		dev_dbg(&pm_data->serdev->dev,
			"PMU cmd=0x%04X src=0x%02X dst=0x%02X frame=%u len=0\n",
			command, src, dst, frame_num);
	}

	switch (command) {
	case PCAT_PM_COMMAND_HOST_REQUEST_SHUTDOWN_ACK:
		WRITE_ONCE(pm_data->poweroff_ok, true);
		need_ack = false;
		break;

	case PCAT_PM_COMMAND_PMU_REQUEST_SHUTDOWN:
		dev_info(&pm_data->serdev->dev, "PMU request shutdown.");
		orderly_poweroff(true);
		break;

	case PCAT_PM_COMMAND_STATUS_REPORT:
		pcat_pm_status_report_parse(pm_data, extra_data, extra_data_len);
		break;

	case PCAT_PM_COMMAND_DATE_TIME_SYNC_ACK:
		if (extra_data_len > 0 && extra_data[0])
			dev_err(&pm_data->serdev->dev, "Failed to sync date: %d\n",
				extra_data[0]);
		break;

	case PCAT_PM_COMMAND_SCHEDULE_STARTUP_TIME_SET_ACK:
		if (extra_data_len > 0 && extra_data[0])
			dev_err(&pm_data->serdev->dev,
				"Failed to set schedule boot: %d\n",
				extra_data[0]);
		else
			dev_dbg(&pm_data->serdev->dev,
				"Schedule boot updated.\n");
		break;

	case PCAT_PM_COMMAND_CHARGER_ON_AUTO_START_ACK:
		if (extra_data_len > 0 && extra_data[0] > 1)
			dev_err(&pm_data->serdev->dev,
				"Failed to set charger auto-start: %d\n",
				extra_data[0]);
		break;

	case PCAT_PM_COMMAND_PMU_HW_VERSION_GET_ACK:
		if (extra_data_len > 0) {
			size_t copy_len = min_t(size_t, extra_data_len,
				sizeof(pm_data->pmu_hw_version) - 1);

			mutex_lock(&pm_data->mutex);
			memcpy(pm_data->pmu_hw_version, extra_data, copy_len);
			pm_data->pmu_hw_version[copy_len] = '\0';
			mutex_unlock(&pm_data->mutex);

			dev_info(&pm_data->serdev->dev,
				"PMU HW Version: %s\n", pm_data->pmu_hw_version);
		}
		break;

	case PCAT_PM_COMMAND_PMU_FW_VERSION_GET_ACK:
		if (extra_data_len > 0) {
			size_t copy_len = min_t(size_t, extra_data_len,
				sizeof(pm_data->pmu_fw_version) - 1);

			mutex_lock(&pm_data->mutex);
			memcpy(pm_data->pmu_fw_version, extra_data, copy_len);
			pm_data->pmu_fw_version[copy_len] = '\0';
			mutex_unlock(&pm_data->mutex);

			dev_info(&pm_data->serdev->dev,
				"PMU FW Version: %s\n", pm_data->pmu_fw_version);
		}
		break;

	case PCAT_PM_COMMAND_POWER_ON_EVENT_GET_ACK:
		if (extra_data_len >= 1) {
			const char *event_str = "unknown";

			pm_data->power_on_event = extra_data[0];

			switch (pm_data->power_on_event) {
			case 1:
				event_str = "power button";
				break;
			case 2:
				event_str = "scheduled";
				break;
			case 3:
				event_str = "charger connected";
				break;
			case 4:
				event_str = "USB";
				break;
			}

			dev_info(&pm_data->serdev->dev,
				"Power-on event: %u (%s)\n",
				pm_data->power_on_event, event_str);
		}
		break;

	case PCAT_PM_COMMAND_STATUS_LED_BEEPER_V2_SET_ACK:
		if (extra_data_len >= 1 && extra_data[0] <= 3) {
			mutex_lock(&pm_data->mutex);
			pm_data->status_led_enabled = extra_data[0] & 1;
			pm_data->beeper_enabled = (extra_data[0] >> 1) & 1;
			mutex_unlock(&pm_data->mutex);
		}
		break;

	case PCAT_PM_COMMAND_DEVICE_MOVEMENT:
		mutex_lock(&pm_data->mutex);
		pm_data->movement_timestamp = ktime_get_boottime_ns();
		mutex_unlock(&pm_data->mutex);
		break;

	case PCAT_PM_COMMAND_FAN_SET_ACK:
		WRITE_ONCE(pm_data->fan_set_ok, true);
		if (extra_data_len > 0 && extra_data[0])
			dev_err(&pm_data->serdev->dev,
				"Failed to set fan speed: %d\n",
				extra_data[0]);
		break;

	case PCAT_PM_COMMAND_NET_STATUS_LED_SETUP_ACK:
		if (extra_data_len > 0 && extra_data[0])
			dev_err(&pm_data->serdev->dev,
				"Failed to set net status LED: %d\n",
				extra_data[0]);
		break;

	case PCAT_PM_COMMAND_WATCHDOG_TIMEOUT_SET_ACK:
		if (extra_data_len > 0 && extra_data[0])
			dev_err(&pm_data->serdev->dev,
				"Failed to set watchdog timeout: %d\n",
				extra_data[0]);
		break;

	/* ACKs that need no action and should not be forwarded to ctl device */
	case PCAT_PM_COMMAND_HEARTBEAT_ACK:
	case PCAT_PM_COMMAND_STATUS_REPORT_ACK:
		need_ack = false;
		break;

	default:
		/* Forward unknown commands to userspace control device */
		mutex_lock(&pm_data->ctl_mutex);
		if (pm_data->ctl_write_buffer_used + rawdata_len > PCAT_PM_BUFFER_SIZE) {
			overflow_size = pm_data->ctl_write_buffer_used +
				rawdata_len - PCAT_PM_BUFFER_SIZE;
			memmove(pm_data->ctl_write_buffer,
				pm_data->ctl_write_buffer + overflow_size,
				PCAT_PM_BUFFER_SIZE - overflow_size);
			memcpy(pm_data->ctl_write_buffer + PCAT_PM_BUFFER_SIZE - rawdata_len,
				rawdata, rawdata_len);
			pm_data->ctl_write_buffer_used = PCAT_PM_BUFFER_SIZE;
		} else {
			memcpy(pm_data->ctl_write_buffer + pm_data->ctl_write_buffer_used,
				rawdata, rawdata_len);
			pm_data->ctl_write_buffer_used += rawdata_len;
		}
		pm_data->ctl_write_buffer_ready = true;
		mutex_unlock(&pm_data->ctl_mutex);
		wake_up_interruptible_all(&pm_data->ctl_wait);

		dev_dbg(&pm_data->serdev->dev,
			"Got command %X from %X to %X, frame num %d, "
			"need ACK %d.\n", command, src, dst, frame_num, need_ack);
		need_ack = false;
		break;
	}

	if (need_ack)
		pcat_pm_uart_write_data(pm_data, command + 1, NULL, 0, false, 0);
}

/**
 * pcat_pm_uart_receive_parse - Parse UART receive buffer for complete packets
 * @pm_data: Driver data
 * @buffer: Receive buffer
 * @buffer_used: Pointer to used bytes count (updated on return)
 * @cmd_exec_func: Callback function for executing parsed commands
 *
 * Scans buffer for valid packets (0xA5 header, valid length, CRC, 0x5A tail).
 * Consumed bytes are removed from the buffer.
 *
 * Return: Number of bytes consumed
 */
size_t pcat_pm_uart_receive_parse(struct pcat_pm_data *pm_data,
	u8 *buffer, size_t *buffer_used, pcat_pm_cmd_exec_func cmd_exec_func)
{
	size_t used_size = 0, remaining_size;
	size_t i = 0;
	const u8 *p, *extra_data;
	u16 expect_len, extra_data_len;
	u16 checksum, rchecksum;
	u16 command;
	u8 src, dst;
	bool need_ack;
	u16 frame_num;

	if (*buffer_used < 13)
		return 0;

	while (i < *buffer_used) {
		/* Find sync byte */
		if (buffer[i] != 0xA5) {
			used_size = i + 1;
			i++;
			continue;
		}

		p = buffer + i;
		remaining_size = *buffer_used - i;

		if (remaining_size < 13)
			break;

		/* Validate payload length */
		expect_len = p[5] + ((u16)p[6] << 8);
		if (expect_len < 3 || expect_len > 515) {
			used_size = i + 1;
			i++;
			dev_err(&pm_data->serdev->dev,
				"Invalid command data length!");
			continue;
		}

		/* Check if we have complete packet */
		if (expect_len + 10 > remaining_size)
			break;

		/* Validate tail byte */
		if (p[9 + expect_len] != 0x5A) {
			used_size = i + 1;
			i++;
			dev_err(&pm_data->serdev->dev,
				"Serial port data missing valid tail!");
			continue;
		}

		/* Validate CRC */
		checksum = p[7 + expect_len] + ((u16)p[8 + expect_len] << 8);
		rchecksum = pcat_pm_compute_crc16(p + 1, 6 + expect_len);

		if (checksum != rchecksum) {
			dev_err(&pm_data->serdev->dev,
				"Serial port got incorrect checksum %04X, "
				"should be %04X!", checksum, rchecksum);
			i += 10 + expect_len;
			used_size = i;
			continue;
		}

		/* Extract packet fields */
		src = p[1];
		dst = p[2];
		frame_num = p[3] + ((u16)p[4] << 8);
		command = p[7] + ((u16)p[8] << 8);
		extra_data_len = expect_len - 3;

		if (expect_len > 3)
			extra_data = p + 9;
		else
			extra_data = NULL;
		need_ack = (p[6 + expect_len] != 0);

		/* Execute command */
		cmd_exec_func(pm_data, p, 10 + expect_len, src, dst, frame_num,
			command, extra_data, extra_data_len, need_ack);

		i += 10 + expect_len;
		used_size = i;
	}

	/* Remove consumed bytes from buffer */
	if (used_size > 0) {
		memmove(buffer, buffer + used_size,
			*buffer_used - used_size);
		*buffer_used -= used_size;
	}

	return used_size;
}

/**
 * pcat_pm_uart_serdev_receive_buf - Serdev receive callback
 * @serdev: Serial device
 * @buf: Received data
 * @count: Number of bytes received
 *
 * Called by serdev layer when data arrives. Buffers data and
 * parses complete packets.
 *
 * Return: Number of bytes consumed (always @count)
 */
static size_t pcat_pm_uart_serdev_receive_buf(
	struct serdev_device *serdev, const u8 *buf, size_t count)
{
	struct pcat_pm_data *pm_data = serdev_device_get_drvdata(serdev);
	size_t used_size = 0;

	while (used_size < count) {
		if (pm_data->read_buffer_used + count - used_size >
			PCAT_PM_BUFFER_SIZE) {
			size_t copy_count = PCAT_PM_BUFFER_SIZE -
				pm_data->read_buffer_used;

			memcpy(pm_data->read_buffer + pm_data->read_buffer_used,
				buf + used_size, copy_count);
			pm_data->read_buffer_used = PCAT_PM_BUFFER_SIZE;
			used_size += copy_count;
		} else {
			memcpy(pm_data->read_buffer + pm_data->read_buffer_used,
				buf + used_size, count - used_size);
			pm_data->read_buffer_used += count - used_size;
			used_size += count - used_size;
		}

		pcat_pm_uart_receive_parse(pm_data, pm_data->read_buffer,
			&pm_data->read_buffer_used, pcat_pm_uart_cmd_exec);
	}

	return count;
}

const struct serdev_device_ops pcat_pm_serdev_ops = {
	.receive_buf = pcat_pm_uart_serdev_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

/**
 * pcat_pm_uart_serdev_open - Open and initialize serial port
 * @pm_data: Driver data
 *
 * Opens the serial device, configures baud rate and parity,
 * and creates the worker thread.
 *
 * Return: 0 on success, negative errno on failure
 */
int pcat_pm_uart_serdev_open(struct pcat_pm_data *pm_data)
{
	struct serdev_device *serdev = pm_data->serdev;
	struct device *dev = &serdev->dev;
	int ret;

	ret = devm_serdev_device_open(dev, serdev);
	if (ret < 0)
		return ret;

	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret < 0) {
		dev_err(dev, "set parity failed\n");
		return ret;
	}

	serdev_device_set_baudrate(serdev, pm_data->baudrate);
	serdev_device_set_flow_control(serdev, false);

	pm_data->kworker = kthread_create_worker(0, "pcat-pm-kworker");
	if (IS_ERR(pm_data->kworker)) {
		ret = PTR_ERR(pm_data->kworker);
		dev_err(dev, "Failed to create kworker: %d\n", ret);
		return ret;
	}

	sched_set_fifo(pm_data->kworker->task);

	return 0;
}
