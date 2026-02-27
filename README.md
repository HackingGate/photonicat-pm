# Photonicat Power Manager Driver

[![Build DKMS Package](https://github.com/HackingGate/photonicat-pm/actions/workflows/dkms-build.yml/badge.svg)](https://github.com/HackingGate/photonicat-pm/actions/workflows/dkms-build.yml)

Linux kernel driver for the Photonicat 2 power management unit (PMU).

## Features

### Power Supply

| Interface | Description |
|-----------|-------------|
| `/sys/class/power_supply/battery/` | Battery status, capacity (0–100%), voltage, and current (read-only). |
| `/sys/class/power_supply/charger/` | Charger online status and input voltage (read-only). |

### Real-Time Clock & Scheduled Boot

| Interface | Description |
|-----------|-------------|
| `/dev/rtc0` | Real-time clock backed by PMU. Supports RTC alarms for scheduled power-on via `rtcwake(8)`. |

### Sensors & Fan

| Interface | Description |
|-----------|-------------|
| `sensors pcat_pm_hwmon_temp_mb-*` | Motherboard temperature sensor (read-only). |
| `sensors pcat_pm_hwmon_speed_fan-*` | Fan speed in RPM (read-only). |
| `/sys/class/thermal/thermal_zone*/` | Motherboard temperature as a kernel thermal zone (requires `#thermal-sensor-cells = <0>` in the `pcat-pm` DT node and a `thermal-zones` binding referencing it). When present, the kernel thermal governor can automatically drive the fan cooling device based on temperature. |
| `/sys/class/thermal/cooling_device*/` | Fan control via thermal cooling device interface. Values 0–100 set fixed fan speed percentage. |
| `/sys/kernel/photonicat-pm/fan_state` | Fan speed mode (read-only). Returns `unmanaged` if the driver has not sent a SET command since loading (the PMU could be at auto speed or at a previously-set fixed speed from before a reboot; see [CAUTION](#fan-control) for how to restore auto speed), or the fixed speed percentage (0–100) set by the driver. |

### LEDs & Peripherals

| Interface | Description |
|-----------|-------------|
| `/sys/kernel/photonicat-pm/status_led` | Status LED control (read-write). Write 1 to enable, 0 to disable. |
| `/sys/kernel/photonicat-pm/beeper` | Beeper control (read-write). Write 1 to enable, 0 to disable. |
| `/sys/kernel/photonicat-pm/net_status_led_on_time` | Network status LED on time in milliseconds (read-write, 0–65535). |
| `/sys/kernel/photonicat-pm/net_status_led_off_time` | Network status LED off time in milliseconds (read-write, 0–65535). |
| `/sys/kernel/photonicat-pm/net_status_led_repeat` | Network status LED repeat count (read-write, 0–65535). 0 = infinite. |
| `/sys/kernel/photonicat-pm/movement_trigger` | Accelerometer-based motion detection (read-only). Returns 1 if motion detected, 0 otherwise. |

### PMU Information

| Interface | Description |
|-----------|-------------|
| `/sys/kernel/photonicat-pm/pmu_hw_version` | PMU hardware version string (read-only). Queried from PMU on driver load. |
| `/sys/kernel/photonicat-pm/pmu_fw_version` | PMU firmware version string (read-only). Queried from PMU on driver load. |
| `/sys/kernel/photonicat-pm/power_on_event` | Last power-on event code (read-only). Values: 0 = unknown, 1 = power button, 2 = scheduled, 3 = charger connected, 4 = USB. |

### Configuration

| Interface | Description |
|-----------|-------------|
| `/sys/kernel/photonicat-pm/charger_on_auto_start` | Charger auto-start control (read-write). Write 1 to enable automatic startup when charger is connected, 0 to disable. |
### Advanced

| Interface | Description |
|-----------|-------------|
| `/dev/pcat-pm-ctl` | Raw PMU command interface. See `pcat-pm-ctl(4)` man page for frame format and details. |

## Building

### DKMS Package via Docker

```bash
# Build package and run lintian
docker compose up --build

# Build only (skip lintian)
docker compose up deb

# Output: build/photonicat-pm-dkms_*.deb
```

## Installation

```bash
# Install DKMS package
dpkg -i build/photonicat-pm-dkms_*.deb
```

## Device Tree

The driver reads
[Device Tree](https://www.kernel.org/doc/html/latest/devicetree/usage-model.html)
properties during probe. Board developers (e.g. Armbian, OpenWrt) must add a
`photonicat-pm` node to their board DTS. The example below shows a typical
configuration for Photonicat 2.

### Example

```dts
&uart10 {
    pinctrl-0 = <&uart10m2_xfer>;
    status = "okay";

    pcat_pm: pcat-pm {
        compatible = "photonicat-pm";

        /* Hardware: must match PMU firmware baud rate */
        baudrate = <115200>;

        /* Hardware: GPIO wired to PMU power-sense input (from schematic) */
        power-gpio = <&gpio0 RK_PC1 GPIO_ACTIVE_HIGH>;

        /* Hardware: PMU protocol version on this board */
        pm-version = <2>;

        /* Config: force power off if shutdown hangs (seconds, 0 = disabled) */
        force-poweroff-timeout = <60>;

        /* Optional: exposes board temperature to the kernel thermal framework */
        #thermal-sensor-cells = <0>;

        /* Required child: links to battery description for charger subsystem */
        charger {
            monitored-battery = <&battery>;
        };

        /* Required child: registers fan as a thermal cooling device */
        pcat_fan: fan {
            #cooling-cells = <2>;
        };
    };
};

/* Optional: thermal zone wiring board temperature sensor to fan cooling device */
thermal-zones {
    board-thermal {
        polling-delay-passive = <1000>;  /* ms between passive cooling steps */
        polling-delay = <5000>;          /* ms between idle polls */
        thermal-sensors = <&pcat_pm>;

        trips {
            board_alert: board-alert {
                temperature = <55000>;   /* 55 °C — start active cooling */
                hysteresis  = <5000>;
                type = "active";
            };
            board_crit: board-crit {
                temperature = <85000>;   /* 85 °C — critical shutdown */
                hysteresis  = <5000>;
                type = "critical";
            };
        };

        cooling-maps {
            map0 {
                trip = <&board_alert>;
                cooling-device = <&pcat_fan THERMAL_NO_LIMIT THERMAL_NO_LIMIT>;
            };
        };
    };
};

/* Standard simple-battery node — values from battery datasheet */
battery: battery {
    compatible = "simple-battery";
    device-chemistry = "lithium-ion";
    voltage-min-design-microvolt = <6800000>;
    voltage-max-design-microvolt = <8400000>;
    energy-full-design-microwatt-hours = <51800000>;
    ocv-capacity-celsius = <20>;
    ocv-capacity-table-0 =  <8344000 100>, <8184000 95>, <8070000 90>, <7980000 85>,
                            <7878000 80>, <7790000 75>, <7704000 70>, <7614000 65>,
                            <7524000 60>, <7426000 55>, <7344000 50>, <7294000 45>,
                            <7258000 40>, <7226000 35>, <7196000 30>, <7156000 25>,
                            <7100000 20>, <7038000 15>, <6958000 10>, <6876000 5>,
                            <6800000 0>;
};
```

### Required Properties

| Property | Type | Description |
|----------|------|-------------|
| `compatible` | string | Must be `"photonicat-pm"` |

### Optional Properties

| Property | Type | Default | Kind | Description |
|----------|------|---------|------|-------------|
| `power-gpio` | GPIO | (none) | Hardware | GPIO pin wired to PMU power-sense input. Pulled low at shutdown to signal the PMU. Get the pin from the board schematic; omit if no such wire exists. |
| `baudrate` | `<u32>` | 115200 | Hardware | UART baud rate. Must match the PMU firmware's configured speed. |
| `pm-version` | `<u32>` | 1 | Hardware | PMU protocol version (1 or 2). Determined by the PMU firmware on the board. Version 2 adds battery current, energy, and voltage reporting. |
| `force-poweroff-timeout` | `<u32>` | 0 (disabled) | Config | Forced power-off timeout in seconds (0–255). Sent to the PMU via `WATCHDOG_TIMEOUT_SET` command at driver probe. When non-zero, the PMU will force power off after this many seconds following a software shutdown. Practical range is 0–60; values >60 are ineffective because the driver also sends a hardcoded 60s watchdog timeout that triggers first. When set to 0, only the 60s watchdog guards against hangs. Safety net for stuck shutdowns. |
| `#thermal-sensor-cells` | `<0>` | (not set) | Config | Exposes the motherboard temperature to the kernel thermal framework. Must be `<0>` (no per-sensor arguments). Required when a `thermal-zones` binding in the board DTS references this node via `thermal-sensors`. Without this, the driver still registers an hwmon sensor but no thermal zone. |

## Usage Examples

### Battery Status

```bash
cat /sys/class/power_supply/battery/capacity
# 0-100 (battery capacity percentage)

cat /sys/class/power_supply/battery/status
# Charging or Discharging
```

### Fan Control

The PMU has two fan speed modes:

- **Unmanaged fan speed** — the PMU controls the fan based on its own internal logic (default).
- **Managed fan speed** — the driver sends a SET command to lock the fan at a specific percentage (0–100%).

You can achieve automatic-like behavior via software by defining a
[thermal zone](https://www.kernel.org/doc/html/latest/driver-api/thermal/sysfs-api.html)
in the device tree that wires the board temperature sensor (via `#thermal-sensor-cells`)
to the fan cooling device (via `#cooling-cells`). The kernel thermal governor then
repeatedly sends SET commands based on temperature and trip points. See the
[Device Tree example](#example) for a complete binding.
However, when the system shuts down, the software stops and the PMU retains the
last SET value.

> [!CAUTION]
> Unmanaged may refer the last fixed speed sometimes (not the PMU auto), and is dangerous for thermal management.
>
> When in managed fan speed, after shutdown, the fan stays at the last fixed speed. If the device is still charging or thermally active, the fan will not adjust on its own.
>
> To restore PMU auto speed:
>
> **State 1: After a power button shutdown.**
> Do one of:
> 1. Unplug AC power
> 2. Hold to power on again
>
> **State 2: After a software shutdown.**
> Do:
> 1. Unplug AC power

```bash
# Check if the driver has set a fixed speed
cat /sys/kernel/photonicat-pm/fan_state
# "unmanaged" = driver has not sent a SET command since loading
#               (PMU could be at unmanaged fan speed or at a previously-set managed fan speed;
#                see CAUTION above for how to restore auto speed)
# 0-100       = managed fan speed percentage set by the driver

# Set fan to 50% (switches PMU to managed speed)
echo 50 > /sys/class/thermal/cooling_device0/cur_state

# Set fan to maximum
echo 100 > /sys/class/thermal/cooling_device0/cur_state

# Read current setting
cat /sys/class/thermal/cooling_device0/cur_state
```

### Movement Detection

```bash
cat /sys/kernel/photonicat-pm/movement_trigger
# 1 (motion detected) or 0 (idle)
```

### Status LED Control

```bash
# Turn on status LED
echo 1 > /sys/kernel/photonicat-pm/status_led

# Turn off status LED
echo 0 > /sys/kernel/photonicat-pm/status_led

# Read current status LED state
cat /sys/kernel/photonicat-pm/status_led
```

### Beeper Control

```bash
# Turn on beeper
echo 1 > /sys/kernel/photonicat-pm/beeper

# Turn off beeper
echo 0 > /sys/kernel/photonicat-pm/beeper

# Read current beeper state
cat /sys/kernel/photonicat-pm/beeper
```

### PMU Hardware / Firmware Version

```bash
cat /sys/kernel/photonicat-pm/pmu_hw_version
cat /sys/kernel/photonicat-pm/pmu_fw_version
```

### Power-on Event

```bash
cat /sys/kernel/photonicat-pm/power_on_event
# 0 = unknown, 1 = power button, 2 = scheduled, 3 = charger connected, 4 = USB
```

### Schedule Boot

The driver registers an RTC device with alarm support. When an RTC alarm is set, the driver sends the alarm time to the PMU via UART (`SCHEDULE_STARTUP_TIME_SET`). The PMU stores this schedule and will power on the board at the specified time, even when the system is fully powered off. The alarm is one-shot (non-recurring).

This integrates with standard Linux RTC tools such as `rtcwake(8)`:

```bash
# Power off now, automatically power on after 60 seconds
rtcwake -m off -s 60

# Power off now, automatically power on at the specified time
rtcwake -m off -t $(date -d "2026-02-11 08:00:00" +%s)

# Set a wake alarm 120 seconds from now, but do not power off (useful for testing)
rtcwake -m no -s 120

# Read current RTC alarm
cat /proc/driver/rtc
```

### Network Status LED

```bash
# Set LED blink pattern: 50ms on, 50ms off, infinite repeat (wired mode)
echo 50 > /sys/kernel/photonicat-pm/net_status_led_on_time
echo 50 > /sys/kernel/photonicat-pm/net_status_led_off_time
echo 0 > /sys/kernel/photonicat-pm/net_status_led_repeat

# Set LED blink pattern: 20ms on, 380ms off (mobile mode)
echo 20 > /sys/kernel/photonicat-pm/net_status_led_on_time
echo 380 > /sys/kernel/photonicat-pm/net_status_led_off_time

# Solid on
echo 100 > /sys/kernel/photonicat-pm/net_status_led_on_time
echo 0 > /sys/kernel/photonicat-pm/net_status_led_off_time
```

### Charger Auto-Start

```bash
# Enable auto-start when charger is connected
echo 1 > /sys/kernel/photonicat-pm/charger_on_auto_start

# Disable
echo 0 > /sys/kernel/photonicat-pm/charger_on_auto_start

# Read current state
cat /sys/kernel/photonicat-pm/charger_on_auto_start
```

### Control Device (`/dev/pcat-pm-ctl`)

Raw escape hatch for advanced PMU commands using the binary serial protocol.
See `pcat-pm-ctl(4)` man page for frame format and details.

## Protocol

The driver communicates with the PMU over UART using a framed binary protocol:

```
┌──────┬─────┬─────┬─────────┬────────┬─────────┬─────────┬──────────┬─────────┬──────┐
│ 0xA5 │ SRC │ DST │ FRAME#  │ LENGTH │ COMMAND │ PAYLOAD │ NEED_ACK │  CRC16  │ 0x5A │
│      │     │     │ (2B LE) │ (2B LE)│ (2B LE) │  [...]  │   (1B)   │ (2B LE) │      │
└──────┴─────┴─────┴─────────┴────────┴─────────┴─────────┴──────────┴─────────┴──────┘
```

- **0xA5**: Header sync byte
- **SRC**: Source address (0x01 = host, 0x81 = PMU)
- **DST**: Destination address (0x01 = host, 0x81 = PMU, 0x80 = broadcast, 0xFF = all)
- **FRAME#**: Frame number, auto-incremented by driver (little-endian)
- **LENGTH**: Payload length + 3 (includes COMMAND + NEED_ACK, little-endian)
- **COMMAND**: Command type (little-endian, see `photonicat-pm.h`)
- **PAYLOAD**: Optional command-specific data
- **NEED_ACK**: 1 = request acknowledgment, 0 = no ACK needed
- **CRC16**: Modbus CRC16 of bytes from SRC through NEED_ACK (little-endian)
- **0x5A**: Tail marker

See `photonicat-pm.h` for command definitions.

## Debug Logging

PMU command logging uses the kernel `dev_dbg` facility. Messages are
compiled out (or dynamically off) by default, producing no overhead in
production.

### Runtime (Dynamic Debug)

Requires `CONFIG_DYNAMIC_DEBUG` in the kernel (enabled by most distros):

```bash
# Enable PMU command logs
echo 'file pcat-pm-uart.c +p' > /sys/kernel/debug/dynamic_debug/control

# Disable
echo 'file pcat-pm-uart.c -p' > /sys/kernel/debug/dynamic_debug/control

# View logs
dmesg | grep "PMU cmd="
```

### Build Time

Add `-DDEBUG` to the module CFLAGS to enable all `dev_dbg` messages
unconditionally:

```makefile
# In src/Makefile
ccflags-y += -DDEBUG
```
