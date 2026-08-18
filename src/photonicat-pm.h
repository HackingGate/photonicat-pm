/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Photonicat Power Manager Driver - Common Header
 *
 * This driver provides an interface to the Photonicat 2 power management
 * unit (PMU) over a serial (UART) connection. It exposes:
 *
 *  - Power Supply: battery status/capacity/voltage/current, charger status
 *  - Real-Time Clock & Scheduled Boot: RTC backed by PMU, alarm-based wake
 *  - Sensors & Fan: motherboard temperature, fan RPM, thermal cooling control
 *  - LEDs & Peripherals: status LED, beeper, network status LED, motion detection
 *  - Power Button: driver poweroff, KEY_POWER input event, or ignore
 *  - PMU Information: hardware/firmware version, power-on event
 *  - Configuration: charger auto-start
 *  - Advanced: raw PMU command interface (/dev/pcat-pm-ctl)
 *
 * Copyright (c) 2025, Kyosuke Nekoyashiki <supercatexpert@gmail.com>
 * Copyright (c) 2026, HackingGate <i@hackinggate.com>
 */

#ifndef _PHOTONICAT_PM_H
#define _PHOTONICAT_PM_H

#include <linux/init.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/property.h>
#include <linux/serdev.h>
#include <linux/pm_wakeup.h>
#include <linux/reboot.h>
#include <linux/power_supply.h>
#include <linux/rtc.h>
#include <linux/miscdevice.h>
#include <linux/thermal.h>
#include <linux/time64.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

/**
 * PCAT_PM_BUFFER_SIZE - Size of UART receive/transmit buffers
 */
#define PCAT_PM_BUFFER_SIZE 4096

/**
 * PCAT_PM_WATCHDOG_DEFAULT_INTERVAL - Default watchdog heartbeat interval (seconds)
 */
#define PCAT_PM_WATCHDOG_DEFAULT_INTERVAL 10

/**
 * PCAT_PM_FAN_MAX_STATE - Maximum fan speed (100%)
 */
#define PCAT_PM_FAN_MAX_STATE 100

/**
 * enum PCatPMCommandType - PMU serial protocol command types
 * @PCAT_PM_COMMAND_HEARTBEAT: Heartbeat ping to PMU
 * @PCAT_PM_COMMAND_HEARTBEAT_ACK: Heartbeat acknowledgment
 * @PCAT_PM_COMMAND_PMU_HW_VERSION_GET: Request PMU hardware version
 * @PCAT_PM_COMMAND_PMU_HW_VERSION_GET_ACK: PMU hardware version response
 * @PCAT_PM_COMMAND_PMU_FW_VERSION_GET: Request PMU firmware version
 * @PCAT_PM_COMMAND_PMU_FW_VERSION_GET_ACK: PMU firmware version response
 * @PCAT_PM_COMMAND_STATUS_REPORT: PMU status report (battery, charger, sensors)
 * @PCAT_PM_COMMAND_STATUS_REPORT_ACK: Status report acknowledgment
 * @PCAT_PM_COMMAND_DATE_TIME_SYNC: Sync RTC time to PMU
 * @PCAT_PM_COMMAND_DATE_TIME_SYNC_ACK: Date/time sync acknowledgment
 * @PCAT_PM_COMMAND_SCHEDULE_STARTUP_TIME_SET: Set scheduled wake-up time
 * @PCAT_PM_COMMAND_SCHEDULE_STARTUP_TIME_SET_ACK: Scheduled startup acknowledgment
 * @PCAT_PM_COMMAND_PMU_REQUEST_SHUTDOWN: PMU requests host shutdown
 * @PCAT_PM_COMMAND_PMU_REQUEST_SHUTDOWN_ACK: Shutdown request acknowledgment
 * @PCAT_PM_COMMAND_HOST_REQUEST_SHUTDOWN: Host requests PMU shutdown
 * @PCAT_PM_COMMAND_HOST_REQUEST_SHUTDOWN_ACK: Host shutdown acknowledgment
 * @PCAT_PM_COMMAND_WATCHDOG_TIMEOUT_SET: Set watchdog timeout values
 * @PCAT_PM_COMMAND_WATCHDOG_TIMEOUT_SET_ACK: Watchdog timeout acknowledgment
 * @PCAT_PM_COMMAND_VOLTAGE_THRESHOLD_SET: Set the nine PMU voltage thresholds
 * @PCAT_PM_COMMAND_VOLTAGE_THRESHOLD_SET_ACK: Voltage threshold set status
 * @PCAT_PM_COMMAND_POWER_ON_MODE_V2_SET: Set or query power-on mode
 * @PCAT_PM_COMMAND_POWER_ON_MODE_V2_SET_ACK: Power-on mode status or state
 * @PCAT_PM_COMMAND_CHARGER_ON_AUTO_START: Configure charger auto-start
 * @PCAT_PM_COMMAND_CHARGER_ON_AUTO_START_ACK: Charger config acknowledgment
 * @PCAT_PM_COMMAND_NET_STATUS_LED_SETUP: Configure network status LED
 * @PCAT_PM_COMMAND_NET_STATUS_LED_SETUP_ACK: LED setup acknowledgment
 * @PCAT_PM_COMMAND_POWER_ON_EVENT_GET: Get last power-on event
 * @PCAT_PM_COMMAND_POWER_ON_EVENT_GET_ACK: Power-on event response
 * @PCAT_PM_COMMAND_FAN_SET: Set fan speed (0x00-0x64 = 0-100%)
 * @PCAT_PM_COMMAND_FAN_SET_ACK: Fan speed acknowledgment
 * @PCAT_PM_COMMAND_DEVICE_MOVEMENT: Device movement detected notification
 * @PCAT_PM_COMMAND_DEVICE_MOVEMENT_ACK: Movement detection acknowledgment
 * @PCAT_PM_COMMAND_STATUS_LED_BEEPER_V2_SET: Set/get status LED and beeper state
 * @PCAT_PM_COMMAND_STATUS_LED_BEEPER_V2_SET_ACK: LED/beeper state response
 * @PCAT_PM_COMMAND_CHARGE_THRESHOLD_SET: Set charge stop threshold (50-100%)
 * @PCAT_PM_COMMAND_CHARGE_THRESHOLD_SET_ACK: Charge threshold set status
 * @PCAT_PM_COMMAND_CHARGE_THRESHOLD_GET: Request charge stop threshold
 * @PCAT_PM_COMMAND_CHARGE_THRESHOLD_GET_ACK: Charge threshold response
 *
 * These commands are used for communication between the host and the PMU
 * over a serial (UART) connection. The protocol uses a framed format with
 * CRC16 checksums.
 */
typedef enum {
	PCAT_PM_COMMAND_HEARTBEAT = 0x01,
	PCAT_PM_COMMAND_HEARTBEAT_ACK = 0x02,
	PCAT_PM_COMMAND_PMU_HW_VERSION_GET = 0x03,
	PCAT_PM_COMMAND_PMU_HW_VERSION_GET_ACK = 0x04,
	PCAT_PM_COMMAND_PMU_FW_VERSION_GET = 0x05,
	PCAT_PM_COMMAND_PMU_FW_VERSION_GET_ACK = 0x06,
	PCAT_PM_COMMAND_STATUS_REPORT = 0x07,
	PCAT_PM_COMMAND_STATUS_REPORT_ACK = 0x08,
	PCAT_PM_COMMAND_DATE_TIME_SYNC = 0x09,
	PCAT_PM_COMMAND_DATE_TIME_SYNC_ACK = 0x0A,
	PCAT_PM_COMMAND_SCHEDULE_STARTUP_TIME_SET = 0x0B,
	PCAT_PM_COMMAND_SCHEDULE_STARTUP_TIME_SET_ACK = 0x0C,
	PCAT_PM_COMMAND_PMU_REQUEST_SHUTDOWN = 0x0D,
	PCAT_PM_COMMAND_PMU_REQUEST_SHUTDOWN_ACK = 0x0E,
	PCAT_PM_COMMAND_HOST_REQUEST_SHUTDOWN = 0x0F,
	PCAT_PM_COMMAND_HOST_REQUEST_SHUTDOWN_ACK = 0x10,
	PCAT_PM_COMMAND_WATCHDOG_TIMEOUT_SET = 0x13,
	PCAT_PM_COMMAND_WATCHDOG_TIMEOUT_SET_ACK = 0x14,
	PCAT_PM_COMMAND_CHARGER_ON_AUTO_START = 0x15,
	PCAT_PM_COMMAND_CHARGER_ON_AUTO_START_ACK = 0x16,
	/* Refused by every tested firmware, so the driver does not send it.
	 * Kept for protocol documentation and raw /dev/pcat-pm-ctl users.
	 */
	PCAT_PM_COMMAND_VOLTAGE_THRESHOLD_SET = 0x17,
	PCAT_PM_COMMAND_VOLTAGE_THRESHOLD_SET_ACK = 0x18,
	PCAT_PM_COMMAND_NET_STATUS_LED_SETUP = 0x19,
	PCAT_PM_COMMAND_NET_STATUS_LED_SETUP_ACK = 0x1A,
	PCAT_PM_COMMAND_POWER_ON_EVENT_GET = 0x1B,
	PCAT_PM_COMMAND_POWER_ON_EVENT_GET_ACK = 0x1C,
	PCAT_PM_COMMAND_FAN_SET = 0x93,
	PCAT_PM_COMMAND_FAN_SET_ACK = 0x94,
	PCAT_PM_COMMAND_STATUS_LED_BEEPER_V2_SET = 0x9B,
	PCAT_PM_COMMAND_STATUS_LED_BEEPER_V2_SET_ACK = 0x9C,
	PCAT_PM_COMMAND_DEVICE_MOVEMENT = 0x95,
	PCAT_PM_COMMAND_DEVICE_MOVEMENT_ACK = 0x96,
	PCAT_PM_COMMAND_POWER_ON_MODE_V2_SET = 0xA1,
	PCAT_PM_COMMAND_POWER_ON_MODE_V2_SET_ACK = 0xA2,
	PCAT_PM_COMMAND_CHARGE_THRESHOLD_SET = 0xA5,
	PCAT_PM_COMMAND_CHARGE_THRESHOLD_SET_ACK = 0xA6,
	PCAT_PM_COMMAND_CHARGE_THRESHOLD_GET = 0xA7,
	PCAT_PM_COMMAND_CHARGE_THRESHOLD_GET_ACK = 0xA8,
} PCatPMCommandType;

/**
 * PCAT_PM_CHARGE_THRESHOLD_MIN - Lowest charge stop threshold accepted by the PMU
 * PCAT_PM_CHARGE_THRESHOLD_MAX - Highest charge stop threshold accepted by the PMU
 *
 * The PMU rejects values outside this range with a non-zero ACK status and
 * keeps the previously stored threshold.
 */
#define PCAT_PM_CHARGE_THRESHOLD_MIN 50
#define PCAT_PM_CHARGE_THRESHOLD_MAX 100

/**
 * PCAT_PM_POWER_ON_MODE_ENABLED - Power-on mode value that enables auto power-on
 * PCAT_PM_POWER_ON_MODE_DISABLED: Power-on mode value that disables auto power-on
 * PCAT_PM_POWER_ON_MODE_QUERY: Payload that requests the current mode
 * PCAT_PM_POWER_ON_MODE_STATE_FLAG: Flag set in ACKs that report a mode
 *
 * The PMU accepts only %PCAT_PM_POWER_ON_MODE_ENABLED and
 * %PCAT_PM_POWER_ON_MODE_DISABLED as set values, and answers a query with
 * %PCAT_PM_POWER_ON_MODE_STATE_FLAG or-ed with the stored mode. Firmware that
 * has never been configured reports the flag alone, a state that cannot be
 * restored once either mode has been set.
 */
#define PCAT_PM_POWER_ON_MODE_ENABLED 0x01
#define PCAT_PM_POWER_ON_MODE_DISABLED 0x02
#define PCAT_PM_POWER_ON_MODE_QUERY 0xFF
#define PCAT_PM_POWER_ON_MODE_STATE_FLAG 0x80

/**
 * PCAT_PM_CMD_ACK_TIMEOUT_MS - Default wait for a PMU command acknowledgment
 */
#define PCAT_PM_CMD_ACK_TIMEOUT_MS 1000

/**
 * enum pcat_pm_probe_capability - Runtime probe state of a PMU feature
 * @PCAT_PM_PROBE_CAP_PENDING: The PMU has not answered for this feature yet
 * @PCAT_PM_PROBE_CAP_ENABLED: The PMU answered, so the feature is usable
 */
enum pcat_pm_probe_capability {
	PCAT_PM_PROBE_CAP_PENDING = 0,
	PCAT_PM_PROBE_CAP_ENABLED,
};

static inline bool pcat_pm_probe_capability_enabled(
	enum pcat_pm_probe_capability capability)
{
	return capability == PCAT_PM_PROBE_CAP_ENABLED;
}

static inline const char *pcat_pm_probe_capability_name(
	enum pcat_pm_probe_capability capability)
{
	switch (capability) {
	case PCAT_PM_PROBE_CAP_PENDING:
		return "pending-probe";
	case PCAT_PM_PROBE_CAP_ENABLED:
		return "enabled-probe";
	default:
		return "unknown";
	}
}

/**
 * struct pcat_pm_cmd_ack - Acknowledgment state of a synchronous PMU command
 * @cmd_mutex: Serializes senders of the command
 * @wait: Wait queue woken when a matching ACK arrives
 * @frame: Frame number of the last recorded ACK
 * @status: Status byte of the last recorded ACK (0 = accepted)
 * @seen: An ACK has been recorded since the last send
 */
struct pcat_pm_cmd_ack {
	struct mutex cmd_mutex;
	wait_queue_head_t wait;
	u16 frame;
	u8 status;
	bool seen;
};

enum pcat_pm_rtc_capability {
	PCAT_PM_RTC_CAP_PENDING_PROBE = 0,
	PCAT_PM_RTC_CAP_ENABLED_PROBE,
};

static inline bool pcat_pm_rtc_capability_enabled(
	enum pcat_pm_rtc_capability capability)
{
	return capability == PCAT_PM_RTC_CAP_ENABLED_PROBE;
}

static inline const char *pcat_pm_rtc_capability_name(
	enum pcat_pm_rtc_capability capability)
{
	switch (capability) {
	case PCAT_PM_RTC_CAP_PENDING_PROBE:
		return "pending-probe";
	case PCAT_PM_RTC_CAP_ENABLED_PROBE:
		return "enabled-probe";
	default:
		return "unknown";
	}
}

/**
 * enum pcat_pm_button_mode - Response to a PMU power button press
 * @PCAT_PM_BUTTON_MODE_POWEROFF: Power the system off from the driver
 * @PCAT_PM_BUTTON_MODE_INPUT: Report KEY_POWER and let userspace decide
 * @PCAT_PM_BUTTON_MODE_IGNORE: Log the press and do nothing else
 *
 * Selected by the pmu-button-mode device tree property. The order matches
 * the property's string values, which pcat-pm-input.c indexes by.
 */
enum pcat_pm_button_mode {
	PCAT_PM_BUTTON_MODE_POWEROFF = 0,
	PCAT_PM_BUTTON_MODE_INPUT,
	PCAT_PM_BUTTON_MODE_IGNORE,
};

struct pcat_pm_fw_caps {
	enum pcat_pm_rtc_capability rtc_capability;
	enum pcat_pm_probe_capability charge_threshold_capability;
	enum pcat_pm_probe_capability power_on_mode_capability;
};

/**
 * struct pcat_pm_data - Main driver state structure
 * @serdev: Serial device handle
 * @power_gpio: GPIO for power control (active low triggers power off)
 * @battery_psy: Battery power supply device
 * @charger_psy: Charger/AC power supply device
 * @kworker: Kernel worker for periodic tasks
 * @check_work: Work item for heartbeat and status checks
 * @check_timer: High-resolution timer for periodic checks
 * @battery_info: Battery information from device tree
 * @rtc: RTC device handle
 * @hwmon_temp_mb_dev: Hwmon device for motherboard temperature
 * @hwmon_speed_fan_dev: Hwmon device for fan speed
 * @tzdev: Thermal zone device for motherboard temperature (NULL if no DT binding)
 * @input: Power button input device (NULL unless @button_mode is input)
 * @ctl_device: Misc device for userspace control
 * @ctl_mutex: Mutex protecting control device output buffer
 * @ctl_read_mutex: Mutex protecting control device input buffer
 * @ctl_wait: Wait queue for control device poll
 * @cdev: Thermal cooling device for fan control
 * @kobject: Kernel object for sysfs attributes
 * @pm_version: PMU protocol version (1 or 2)
 * @work_flag: Worker thread active flag
 * @poweroff_ok: Shutdown acknowledged by PMU
 * @fan_set_ok: Fan set acknowledged by PMU
 * @baudrate: Serial port baud rate (default 115200)
 * @force_poweroff_timeout: Forced power off timeout in seconds, as configured
 *	by the device tree. Sent to the PMU only while @button_mode is
 *	poweroff, and by the shutdown handler; see
 *	pcat_pm_watchdog_timeout_set()
 * @button_mode: Response to a PMU power button press
 * @write_framenum: Incrementing frame number for sent packets
 * @read_buffer: UART receive buffer
 * @read_buffer_used: Bytes used in receive buffer
 * @ctl_write_buffer: Control device output buffer
 * @ctl_write_buffer_used: Bytes in control output buffer
 * @ctl_write_buffer_ready: Data available for userspace read
 * @ctl_read_buffer: Control device input buffer
 * @ctl_read_buffer_used: Bytes in control input buffer
 * @mutex: Main data protection mutex
 * @status_report_timestamp: Last status report time (ns)
 * @status_report_timeout_warn_timestamp: Last timeout warning time (ns)
 * @battery_technology: Battery chemistry type
 * @battery_design_uwh: Full charge capacity in µWh
 * @battery_design_min_uv: Minimum voltage in µV
 * @battery_design_max_uv: Maximum voltage in µV
 * @battery_voltage_now: Current battery voltage in µV
 * @charger_voltage_now: Current charger voltage in µV
 * @battery_current_now: Battery current in µA (polarity depends on PMU firmware)
 * @battery_energy_now: Current energy in µWh
 * @battery_energy_full: Full charge energy in µWh
 * @battery_soc: State of charge (0-100%)
 * @on_battery: True if running on battery power
 * @on_charger: True if charger is connected
 * @ps_initialized: True after first parsed PMU status report
 * @charge_threshold_ack: ACK state of the charge threshold set command
 * @charge_threshold: Charge stop threshold reported by the PMU (50-100%)
 * @power_on_mode_ack: ACK state of the power-on mode set command
 * @power_on_mode_state: Power-on mode reported by the PMU (0x80, 0x81, 0x82)
 * @board_temp: Motherboard temperature in degrees Celsius
 * @gs_x: Accelerometer X-axis value
 * @gs_y: Accelerometer Y-axis value
 * @gs_z: Accelerometer Z-axis value
 * @gs_ready: Accelerometer data valid
 * @fan_rpm: Current fan speed in RPM
 * @rtc_year: RTC year
 * @rtc_month: RTC month (0-11)
 * @rtc_day: RTC day of month
 * @rtc_hour: RTC hour
 * @rtc_min: RTC minute
 * @rtc_sec: RTC second
 * @rtc_wday: RTC day of week (0 = Sunday, 1 = Monday, ..., 6 = Saturday)
 * @rtc_probe_valid_samples: Consecutive valid PMU RTC samples for dynamic enable
 * @rtc_probe_last_time: Last valid PMU RTC sample timestamp for monotonic probe
 * @rtc_register_work: Deferred registration of @rtc, run once the PMU RTC
 *	passes runtime validation or the fallback delay expires
 * @rtc_registered: @rtc has been registered with the RTC core
 * @fan_ctrl_speed: Fan control setting (0-100%)
 * @fan_managed: True once the driver has explicitly set fan speed
 * @movement_timestamp: Last movement detection time (ns)
 * @movement_activated: Movement detection currently active
 * @status_led_enabled: Status LED enabled state
 * @beeper_enabled: Beeper enabled state
 * @pmu_hw_version: PMU hardware version string
 * @pmu_fw_version: PMU firmware version string
 * @pmu_fw_caps: Runtime capability state for PMU-backed features
 * @power_on_event: Last power-on event code
 * @net_status_led_on_time: Network status LED on time (ms)
 * @net_status_led_off_time: Network status LED off time (ms)
 * @net_status_led_repeat: Network status LED repeat count (0 = infinite)
 */
struct pcat_pm_data {
	/* Device handles */
	struct serdev_device *serdev;
	struct gpio_desc *power_gpio;
	struct power_supply *battery_psy;
	struct power_supply *charger_psy;
	struct power_supply_battery_info *battery_info;
	struct rtc_device *rtc;
	struct device *hwmon_temp_mb_dev;
	struct device *hwmon_speed_fan_dev;
	struct thermal_zone_device *tzdev;
	struct thermal_cooling_device *cdev;
	struct input_dev *input;

	/* Worker thread and timer */
	struct kthread_worker *kworker;
	struct kthread_work check_work;
	struct hrtimer check_timer;

	/* Control device */
	struct miscdevice ctl_device;
	struct mutex ctl_mutex;
	struct mutex ctl_read_mutex;
	wait_queue_head_t ctl_wait;
	wait_queue_head_t rtc_cmd_wait;

	/* Sysfs */
	struct kobject kobject;

	/* Driver configuration */
	u32 pm_version;
	u32 baudrate;
	u32 force_poweroff_timeout;
	enum pcat_pm_button_mode button_mode;
	bool work_flag;
	bool poweroff_ok;
	bool fan_set_ok;

	/* UART protocol state */
	u16 write_framenum;
	u8 read_buffer[PCAT_PM_BUFFER_SIZE];
	size_t read_buffer_used;

	/* Control device buffers */
	u8 ctl_write_buffer[PCAT_PM_BUFFER_SIZE];
	size_t ctl_write_buffer_used;
	bool ctl_write_buffer_ready;
	u8 ctl_read_buffer[PCAT_PM_BUFFER_SIZE];
	size_t ctl_read_buffer_used;

	/* Status report state (protected by @mutex) */
	struct mutex mutex;
	struct mutex rtc_cmd_mutex;
	u64 status_report_timestamp;
	u64 status_report_timeout_warn_timestamp;

	/* Battery and charger state */
	unsigned int battery_technology;
	int battery_design_uwh;
	int battery_design_min_uv;
	int battery_design_max_uv;
	int battery_voltage_now;
	int charger_voltage_now;
	int battery_current_now;
	int battery_energy_now;
	int battery_energy_full;
	int battery_soc;
	bool on_battery;
	bool on_charger;
	bool ps_initialized;

	/* Charge stop threshold (value protected by @mutex) */
	struct pcat_pm_cmd_ack charge_threshold_ack;
	u8 charge_threshold;

	/* Power-on mode (state protected by @mutex) */
	struct pcat_pm_cmd_ack power_on_mode_ack;
	u8 power_on_mode_state;

	/* Sensor data */
	int board_temp;
	int gs_x;
	int gs_y;
	int gs_z;
	bool gs_ready;
	u32 fan_rpm;

	/* RTC state */
	u16 rtc_year;
	u8 rtc_month;
	u8 rtc_day;
	u8 rtc_hour;
	u8 rtc_min;
	u8 rtc_sec;
	u8 rtc_wday;
	u8 rtc_probe_valid_samples;
	time64_t rtc_probe_last_time;
	struct delayed_work rtc_register_work;
	bool rtc_registered;
	u16 rtc_sync_ack_frame;
	u16 schedule_boot_ack_frame;
	u8 rtc_sync_ack_status;
	u8 schedule_boot_ack_status;
	bool rtc_sync_ack_seen;
	bool schedule_boot_ack_seen;

	/* RTC alarm (schedule boot) */
	struct rtc_time alarm_time;
	bool alarm_enabled;
	bool schedule_boot_sent;

	/* Fan control */
	u8 fan_ctrl_speed;
	bool fan_managed;

	/* Movement detection */
	u64 movement_timestamp;
	bool movement_activated;

	/* Status LED and beeper */
	bool status_led_enabled;
	bool beeper_enabled;

	/* PMU information */
	char pmu_hw_version[32];
	char pmu_fw_version[32];
	struct pcat_pm_fw_caps pmu_fw_caps;
	u8 power_on_event;

	/* Network status LED */
	u16 net_status_led_on_time;
	u16 net_status_led_off_time;
	u16 net_status_led_repeat;

	/* Charger auto-start */
	bool charger_on_auto_start;

};

/**
 * pcat_pm_get_data_from_dev - Get driver data from a device
 * @dev: Device pointer from callback
 *
 * Helper for hwmon/rtc callbacks to retrieve driver data.
 *
 * Return: Pointer to pcat_pm_data
 */
static inline struct pcat_pm_data *pcat_pm_get_data_from_dev(struct device *dev)
{
	struct serdev_device *serdev = container_of(dev, struct serdev_device, dev);

	return serdev_device_get_drvdata(serdev);
}

/**
 * typedef pcat_pm_cmd_exec_func - Command execution callback type
 * @pm_data: Driver data structure
 * @rawdata: Raw packet data
 * @rawdata_len: Length of raw packet
 * @src: Source address
 * @dst: Destination address
 * @frame_num: Packet frame number
 * @command: Command type
 * @extra_data: Command payload
 * @extra_data_len: Payload length
 * @need_ack: Whether acknowledgment is required
 */
typedef void (*pcat_pm_cmd_exec_func)(struct pcat_pm_data *pm_data,
	const u8 *rawdata, size_t rawdata_len, u8 src, u8 dst,
	u16 frame_num, u16 command, const u8 *extra_data,
	u16 extra_data_len, bool need_ack);

/* ========================================================================
 * UART Protocol Module (pcat-pm-uart.c)
 * ======================================================================== */

/**
 * pcat_pm_compute_crc16 - Compute CRC16 checksum (Modbus style)
 * @data: Data buffer
 * @len: Buffer length
 *
 * Return: CRC16 checksum value
 */
u16 pcat_pm_compute_crc16(const u8 *data, size_t len);

/**
 * pcat_pm_uart_write_data_frame - Send a command to the PMU
 * @pm_data: Driver data
 * @command: Command type to send
 * @extra_data: Optional payload data
 * @extra_data_len: Payload length (max 512)
 * @need_ack: Request acknowledgment from PMU
 * @timeout: Timeout in jiffies (0 for non-blocking)
 * @frame_num: Optional output for the assigned frame number
 *
 * Return: Number of bytes written or negative error
 */
int pcat_pm_uart_write_data_frame(struct pcat_pm_data *pm_data,
	u16 command, const u8 *extra_data, u16 extra_data_len,
	bool need_ack, long timeout, u16 *frame_num);

static inline int pcat_pm_uart_write_data(struct pcat_pm_data *pm_data,
	u16 command, const u8 *extra_data, u16 extra_data_len,
	bool need_ack, long timeout)
{
	return pcat_pm_uart_write_data_frame(pm_data, command, extra_data,
		extra_data_len, need_ack, timeout, NULL);
}

/**
 * pcat_pm_cmd_ack_init - Initialize acknowledgment state of a PMU command
 * @ack: Acknowledgment state
 */
void pcat_pm_cmd_ack_init(struct pcat_pm_cmd_ack *ack);

/**
 * pcat_pm_cmd_ack_record - Record an acknowledgment received from the PMU
 * @pm_data: Driver data
 * @ack: Acknowledgment state to update
 * @frame_num: Frame number of the acknowledgment
 * @status: Status byte reported by the PMU (0 = accepted)
 *
 * Wakes a sender blocked in pcat_pm_cmd_send_wait().
 */
void pcat_pm_cmd_ack_record(struct pcat_pm_data *pm_data,
	struct pcat_pm_cmd_ack *ack, u16 frame_num, u8 status);

/**
 * pcat_pm_cmd_send_wait - Send a PMU command and wait for its acknowledgment
 * @pm_data: Driver data
 * @ack: Acknowledgment state of this command
 * @command: Command type to send
 * @extra_data: Optional payload
 * @extra_data_len: Payload length
 * @timeout_ms: Milliseconds to wait for the acknowledgment
 *
 * Serializes senders of @command, so only one waits for a given ACK at a time.
 *
 * Return: 0 if the PMU accepted the command, -EIO if it refused, -ETIMEDOUT if
 * it did not answer, or a negative error from the serial write.
 */
int pcat_pm_cmd_send_wait(struct pcat_pm_data *pm_data,
	struct pcat_pm_cmd_ack *ack, u16 command, const u8 *extra_data,
	u16 extra_data_len, unsigned int timeout_ms);

/**
 * pcat_pm_uart_receive_parse - Parse received UART data
 * @pm_data: Driver data
 * @buffer: Receive buffer
 * @buffer_used: Pointer to bytes used (updated on return)
 * @cmd_exec_func: Callback for executing parsed commands
 *
 * Return: Number of bytes consumed from buffer
 */
size_t pcat_pm_uart_receive_parse(struct pcat_pm_data *pm_data,
	u8 *buffer, size_t *buffer_used, pcat_pm_cmd_exec_func cmd_exec_func);

/**
 * pcat_pm_uart_serdev_open - Open and configure the serial port
 * @pm_data: Driver data
 *
 * Return: 0 on success, negative error otherwise
 */
int pcat_pm_uart_serdev_open(struct pcat_pm_data *pm_data);

/**
 * pcat_pm_uart_cmd_exec - Default command execution handler
 * @pm_data: Driver data
 * @rawdata: Raw packet data
 * @rawdata_len: Raw packet length
 * @src: Source address
 * @dst: Destination address
 * @frame_num: Frame number
 * @command: Command type
 * @extra_data: Payload
 * @extra_data_len: Payload length
 * @need_ack: Acknowledgment required
 */
void pcat_pm_uart_cmd_exec(struct pcat_pm_data *pm_data,
	const u8 *rawdata, size_t rawdata_len, u8 src, u8 dst, u16 frame_num,
	u16 command, const u8 *extra_data, u16 extra_data_len, bool need_ack);

extern const struct serdev_device_ops pcat_pm_serdev_ops;

/* ========================================================================
 * Power Supply Module (pcat-pm-power.c)
 * ======================================================================== */

/**
 * pcat_pm_charger_probe - Initialize battery and charger power supplies
 * @pm_data: Driver data
 *
 * Registers battery and charger power supply devices. Reads battery
 * configuration from device tree.
 *
 * Return: 0 on success, negative error otherwise
 */
int pcat_pm_charger_probe(struct pcat_pm_data *pm_data);

/**
 * pcat_pm_charger_remove - Unregister battery and charger power supplies
 * @pm_data: Driver data
 */
void pcat_pm_charger_remove(struct pcat_pm_data *pm_data);

/**
 * pcat_pm_charge_threshold_query - Ask the PMU for its charge stop threshold
 * @pm_data: Driver data
 *
 * Sends CHARGE_THRESHOLD_GET. A valid reply caches the threshold and promotes
 * @pmu_fw_caps.charge_threshold_capability to enabled-probe. Firmware without
 * charge threshold support does not answer, so the capability stays pending.
 */
void pcat_pm_charge_threshold_query(struct pcat_pm_data *pm_data);


/**
 * pcat_pm_charge_threshold_report - Record a threshold value read from the PMU
 * @pm_data: Driver data
 * @threshold: Threshold percentage reported by the PMU
 *
 * Out-of-range values are ignored. A valid value promotes the runtime
 * capability to enabled-probe.
 */
void pcat_pm_charge_threshold_report(struct pcat_pm_data *pm_data,
	u8 threshold);

/* ========================================================================
 * RTC Module (pcat-pm-rtc.c)
 * ======================================================================== */

/**
 * pcat_pm_rtc_probe - Initialize RTC device
 * @pm_data: Driver data
 *
 * Allocates the RTC device and schedules its registration; see
 * pcat_pm_rtc_register_now() for why registration is deferred.
 *
 * Return: 0 on success, negative error otherwise
 */
int pcat_pm_rtc_probe(struct pcat_pm_data *pm_data);

/**
 * pcat_pm_rtc_register_now - Register the RTC device without further delay
 * @pm_data: Driver data
 *
 * Registering the RTC makes the RTC core read it once to seed the system
 * clock (hctosys), and that read only succeeds after the PMU RTC has passed
 * runtime validation. Registration is therefore deferred until this is
 * called, or until the fallback delay set up by pcat_pm_rtc_probe() expires.
 *
 * Safe to call with pcat_pm_data.mutex held and from the UART receive path;
 * the registration itself runs from a workqueue.
 */
void pcat_pm_rtc_register_now(struct pcat_pm_data *pm_data);

/**
 * pcat_pm_rtc_remove - Stop pending RTC registration
 * @pm_data: Driver data
 */
void pcat_pm_rtc_remove(struct pcat_pm_data *pm_data);

/* ========================================================================
 * Hardware Monitor Module (pcat-pm-hwmon.c)
 * ======================================================================== */

/**
 * pcat_pm_hwmon_probe - Initialize hwmon and thermal zone devices
 * @pm_data: Driver data
 *
 * Registers hwmon devices for motherboard temperature and fan speed.
 * Also registers a thermal zone for the motherboard temperature sensor
 * if a matching DT thermal-sensors binding is present; otherwise logs
 * a warning and continues.
 *
 * Return: 0 on success, negative error otherwise
 */
int pcat_pm_hwmon_probe(struct pcat_pm_data *pm_data);

/* ========================================================================
 * Fan Control Module (pcat-pm-fan.c)
 * ======================================================================== */

/**
 * pcat_pm_fan_probe - Initialize thermal cooling device
 * @pm_data: Driver data
 *
 * Registers a thermal cooling device for fan speed control.
 * Fan speed range: 0-100 (percentage).
 *
 * WARNING: The PMU uses unmanaged fan speed by default. Once any fan speed is
 * set via this cooling device, DT thermal policy, or other software,
 * the PMU switches to managed fan speed.
 * See pcat-pm-fan.c and the README Fan Control section for PMU auto-speed
 * reset limitations and restore workarounds.
 *
 * Return: 0 on success, negative error otherwise
 */
int pcat_pm_fan_probe(struct pcat_pm_data *pm_data);

/* ========================================================================
 * Power Button Module (pcat-pm-input.c)
 * ======================================================================== */

/**
 * pcat_pm_input_probe - Read the button mode and register the input device
 * @pm_data: Driver data
 *
 * Picks the mode from the button_mode module parameter, else the optional
 * pmu-button-mode device tree property, else poweroff. Registers a
 * KEY_POWER input device in input mode only.
 *
 * Return: 0 on success, negative error otherwise
 */
int pcat_pm_input_probe(struct pcat_pm_data *pm_data);

/**
 * pcat_pm_button_event - Handle a PMU power button press
 * @pm_data: Driver data
 *
 * Powers off, reports KEY_POWER, or does nothing, per @button_mode.
 * Called from the UART receive context, so it must not sleep.
 */
void pcat_pm_button_event(struct pcat_pm_data *pm_data);

/* ========================================================================
 * Control Device Module (pcat-pm-ctl.c)
 * ======================================================================== */

extern const struct file_operations pcat_pm_ctl_dev_ops;

/**
 * pcat_pm_ctl_cmd_exec - Control device command handler
 * @pm_data: Driver data
 * @rawdata: Raw packet
 * @rawdata_len: Packet length
 * @src: Source address
 * @dst: Destination address
 * @frame_num: Frame number
 * @command: Command type
 * @extra_data: Payload
 * @extra_data_len: Payload length
 * @need_ack: Acknowledgment required
 *
 * Filters and forwards commands from userspace to the PMU.
 */
void pcat_pm_ctl_cmd_exec(struct pcat_pm_data *pm_data,
	const u8 *rawdata, size_t rawdata_len, u8 src, u8 dst, u16 frame_num,
	u16 command, const u8 *extra_data, u16 extra_data_len, bool need_ack);

/* ========================================================================
 * Sysfs Module (pcat-pm-sysfs.c)
 * ======================================================================== */

/**
 * pcat_pm_sysfs_init - Initialize sysfs attributes
 * @pm_data: Driver data
 *
 * Creates /sys/kernel/photonicat-pm/ with sysfs attributes for:
 * movement detection, status LED, beeper, PMU hardware/firmware version,
 * PMU RTC and charge threshold capability, power-on event, network status
 * LED, and charger auto-start.
 *
 * Return: 0 on success, negative error otherwise
 */
int pcat_pm_sysfs_init(struct pcat_pm_data *pm_data);

/**
 * pcat_pm_power_on_mode_query - Ask the PMU for its power-on mode
 * @pm_data: Driver data
 *
 * A reply caches the mode and promotes
 * @pmu_fw_caps.power_on_mode_capability to enabled-probe.
 */
void pcat_pm_power_on_mode_query(struct pcat_pm_data *pm_data);

/**
 * pcat_pm_power_on_mode_report - Record a power-on mode reported by the PMU
 * @pm_data: Driver data
 * @state: Mode byte from the PMU (%PCAT_PM_POWER_ON_MODE_STATE_FLAG or-ed
 *         with the stored mode)
 */
void pcat_pm_power_on_mode_report(struct pcat_pm_data *pm_data, u8 state);

/**
 * pcat_pm_sysfs_cleanup - Remove sysfs attributes and kobject
 * @pm_data: Driver data
 */
void pcat_pm_sysfs_cleanup(struct pcat_pm_data *pm_data);

#endif /* _PHOTONICAT_PM_H */
