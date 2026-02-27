/* SPDX-License-Identifier: GPL-3.0-or-later */
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
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/property.h>
#include <linux/serdev.h>
#include <linux/reboot.h>
#include <linux/power_supply.h>
#include <linux/rtc.h>
#include <linux/miscdevice.h>
#include <linux/thermal.h>
#include <linux/time64.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

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
} PCatPMCommandType;

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
 * @ctl_device: Misc device for userspace control
 * @ctl_mutex: Mutex protecting control device buffers
 * @ctl_wait: Wait queue for control device poll
 * @cdev: Thermal cooling device for fan control
 * @kobject: Kernel object for sysfs attributes
 * @pm_version: PMU protocol version (1 or 2)
 * @work_flag: Worker thread active flag
 * @poweroff_ok: Shutdown acknowledged by PMU
 * @fan_set_ok: Fan set acknowledged by PMU
 * @baudrate: Serial port baud rate (default 115200)
 * @force_poweroff_timeout: Forced power off timeout in seconds
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
 * @rtc_status: RTC status (0 = valid, non-zero = error)
 * @fan_ctrl_speed: Fan control setting (0-100%)
 * @fan_managed: True once the driver has explicitly set fan speed
 * @movement_timestamp: Last movement detection time (ns)
 * @movement_activated: Movement detection currently active
 * @status_led_enabled: Status LED enabled state
 * @beeper_enabled: Beeper enabled state
 * @pmu_hw_version: PMU hardware version string
 * @pmu_fw_version: PMU firmware version string
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

	/* Worker thread and timer */
	struct kthread_worker *kworker;
	struct kthread_work check_work;
	struct hrtimer check_timer;

	/* Control device */
	struct miscdevice ctl_device;
	struct mutex ctl_mutex;
	wait_queue_head_t ctl_wait;

	/* Sysfs */
	struct kobject kobject;

	/* Driver configuration */
	u32 pm_version;
	u32 baudrate;
	u32 force_poweroff_timeout;
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
	u8 rtc_status;

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
 * pcat_pm_uart_write_data - Send a command to the PMU
 * @pm_data: Driver data
 * @command: Command type to send
 * @extra_data: Optional payload data
 * @extra_data_len: Payload length (max 512)
 * @need_ack: Request acknowledgment from PMU
 * @timeout: Timeout in jiffies (0 for non-blocking)
 *
 * Return: Number of bytes written or negative error
 */
int pcat_pm_uart_write_data(struct pcat_pm_data *pm_data,
	u16 command, const u8 *extra_data, u16 extra_data_len,
	bool need_ack, long timeout);

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

/* ========================================================================
 * RTC Module (pcat-pm-rtc.c)
 * ======================================================================== */

/**
 * pcat_pm_rtc_probe - Initialize RTC device
 * @pm_data: Driver data
 *
 * Return: 0 on success, negative error otherwise
 */
int pcat_pm_rtc_probe(struct pcat_pm_data *pm_data);

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
 * Return: 0 on success, negative error otherwise
 */
int pcat_pm_fan_probe(struct pcat_pm_data *pm_data);

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
 * power-on event, network status LED, and charger auto-start.
 *
 * Return: 0 on success, negative error otherwise
 */
int pcat_pm_sysfs_init(struct pcat_pm_data *pm_data);

/**
 * pcat_pm_sysfs_cleanup - Remove sysfs attributes and kobject
 * @pm_data: Driver data
 */
void pcat_pm_sysfs_cleanup(struct pcat_pm_data *pm_data);

#endif /* _PHOTONICAT_PM_H */
