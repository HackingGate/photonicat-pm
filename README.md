# Photonicat Power Manager Driver

[![Build DKMS Package](https://github.com/HackingGate/photonicat-pm/actions/workflows/dkms-build.yml/badge.svg)](https://github.com/HackingGate/photonicat-pm/actions/workflows/dkms-build.yml)

Linux kernel driver for the Photonicat 2 power management unit (PMU).

See the [Photonicat PM Wiki](https://github.com/HackingGate/photonicat-pm/wiki)
for MCU firmware inspection and flashing workflows.

## MCU Firmware Capability Policy

The driver treats firmware behavior as runtime-observed capability or quirk
detection, not as a static firmware-version allowlist or denylist.

- **RTC and scheduled boot**: start as `pending-probe`. `/dev/rtc0` remains
  registered for ABI stability, but RTC reads, set-time, alarms, and raw
  scheduled-boot commands are blocked until the PMU reports three consecutive
  valid, advancing RTC samples. Passing that probe promotes
  `pmu_rtc_capability` to `enabled-probe`.
- **Battery capacity**: follows the vendor driver policy. PMU protocol v2
  status reports use the PMU-reported SOC byte directly. Shorter status reports
  fall back to voltage-derived OCV SOC from the device-tree battery profile.
- **Energy and fan**: PMU protocol v2 energy fields and fan auto-speed reset are
  not trusted by current driver releases. `energy_full` remains the static
  device-tree design capacity, `energy_now` is not exported, and fan auto-speed
  restoration requires the documented workarounds.

Observed RTC results are evidence for diagnostics, not feature gates:

| Firmware version | Observed RTC result |
|------------------|---------------------|
| `RA2E1250918000` | Promotes to `enabled-probe`; scheduled boot works. |
| `RA2E1260306000` | Remains `pending-probe`; scheduled boot stays blocked by runtime validation. |

## Features

### Power Supply

| Interface | Description |
|-----------|-------------|
| `/sys/class/power_supply/battery/` | Battery status, capacity (0–100%), voltage, and current (read-only). |
| `/sys/class/power_supply/charger/` | Charger online status and input voltage (read-only). |

> [!CAUTION]
> PMU protocol v2 status-report energy values are not validated as live or
> measured battery energy. The driver keeps `energy_full` as the static
> device-tree design capacity and does not export `energy_now`.

### Real-Time Clock & Scheduled Boot

| Interface | Description |
|-----------|-------------|
| `/dev/rtc0` | Real-time clock backed by PMU. Supports RTC alarms for scheduled power-on via `rtcwake(8)`. |

> [!CAUTION]
> Known affected firmware: `RA2E1260306000`.
> The hardware RTC reports broken values. `/dev/rtc0` remains registered for
> ABI stability, but RTC reads report invalid data and alarm programming fails
> until runtime validation promotes `pmu_rtc_capability` to `enabled-probe`.

### Sensors & Fan

| Interface | Description |
|-----------|-------------|
| `sensors pcat_pm_hwmon_temp_mb-*` | Motherboard temperature sensor (read-only). |
| `sensors pcat_pm_hwmon_speed_fan-*` | Fan speed in RPM (read-only). |
| `/sys/class/thermal/thermal_zone*/` | Motherboard temperature as a kernel thermal zone (requires `#thermal-sensor-cells = <0>` in the `pcat-pm` DT node and a `thermal-zones` binding referencing it). When present, the kernel thermal governor can automatically drive the fan cooling device based on temperature. |
| `/sys/class/thermal/cooling_device*/` | Fan control via the thermal cooling device whose `type` is `pcat-pm-fan`. Cooling-device indexes are not stable; discover the device by type before writing `cur_state`. Values 0–100 set fixed fan speed percentage. |
| `/sys/kernel/photonicat-pm/fan_state` | Fan speed mode (read-only). Returns `unmanaged` or the fixed speed percentage (0–100) set by the driver; see [Fan Control](#fan-control) for semantics and caveats. |

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
| `/sys/kernel/photonicat-pm/pmu_rtc_capability` | PMU RTC policy state (read-only). Values: `pending-probe` or `enabled-probe`. |
| `/sys/kernel/photonicat-pm/power_on_event` | Last power-on event code (read-only). Values: 0 = unknown, 1 = power button, 2 = scheduled, 3 = charger connected, 4 = USB. |

### Configuration

| Interface | Description |
|-----------|-------------|
| `/sys/kernel/photonicat-pm/charger_on_auto_start` | Charger auto-start control (read-write). Write 1 to enable automatic startup when charger is connected, 0 to disable. |
### Advanced

| Interface | Description |
|-----------|-------------|
| `/dev/pcat-pm-ctl` | Raw PMU command interface. Userspace can read selected raw PMU responses, including hardware/firmware version ACKs used by `pcat-pmu-updater --pmu-fw-version-get`. See `pcat-pm-ctl(4)` man page for frame format and details. |

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
| `pm-version` | `<u32>` | 1 | Hardware | PMU protocol version (1 or 2). Determined by the PMU firmware on the board. Version 2 adds battery current and PMU-reported capacity. |
| `force-poweroff-timeout` | `<u32>` | 0 (disabled) | Config | Forced power-off timeout in seconds (0–255). Sent to the PMU via `WATCHDOG_TIMEOUT_SET` command at driver probe. When non-zero, the PMU will force power off after this many seconds following a software shutdown. Practical range is 0–60; values >60 are ineffective because the driver also sends a hardcoded 60s watchdog timeout that triggers first. When set to 0, only the 60s watchdog guards against hangs. Safety net for stuck shutdowns. |
| `#thermal-sensor-cells` | `<0>` | (not set) | Config | Exposes the motherboard temperature to the kernel thermal framework. Must be `<0>` (no per-sensor arguments). Required when a `thermal-zones` binding in the board DTS references this node via `thermal-sensors`. Without this, the driver still registers an hwmon sensor but no thermal zone. |

## Usage Examples

### Battery Status

```bash
cat /sys/class/power_supply/battery/capacity
# 0-100 (battery capacity percentage)

cat /sys/class/power_supply/battery/status
# Charging or Discharging

cat /sys/class/power_supply/battery/energy_full
# Static design full charge capacity from device tree, not live/measured capacity

test ! -e /sys/class/power_supply/battery/energy_now
# energy_now is intentionally not exported by current driver releases
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
> Known affected firmware: all tested firmware versions up to and including
> `RA2E1260306000`.
> No trusted API is exposed to reset fan control back to PMU auto speed. The
> steps below are workarounds to restore PMU auto speed.
>
> Due to that firmware limitation, this driver's `unmanaged` state only means the driver has not sent a fan SET command since loading. It may sometimes mean the PMU retained the last fixed speed instead of returning to PMU auto speed.
>
> When in managed fan speed, after shutdown, the fan stays at the last fixed speed and will not adjust on its own. If the retained speed is low and the device is still charging or otherwise thermally active, this can be unsafe for thermal management.
>
> To restore PMU auto speed:
>
> **State 1: After a power button shutdown.**
> Do one of:
> 1. Ensure AC power is unplugged
> 2. Hold to power on again
>
> **State 2: After a software shutdown.**
> Do:
> 1. Ensure AC power is unplugged
>
> The next boot will have PMU auto speed restored.

Check whether the driver has set a fixed speed:

```sh
cat /sys/kernel/photonicat-pm/fan_state
```

`unmanaged` is the driver-local state described in the caution above. A value
from 0 to 100 is the managed fan speed percentage set by the driver.

The following commands can be pasted from `bash`, `zsh`, or `fish`. They run the
write through `sudo sh -c` so the privileged shell performs the `cur_state`
redirection.

Set fan speed (switches PMU to managed speed). Replace the final argument with
the fixed speed percentage to set; use `100` for maximum:

```sh
sudo sh -c '
speed=$1
fan_cdev=
for cdev in /sys/class/thermal/cooling_device*; do
    [ "$(cat "$cdev/type" 2>/dev/null)" = "pcat-pm-fan" ] || continue
    fan_cdev=$cdev
    break
done
[ -n "$fan_cdev" ] || { echo "pcat-pm-fan cooling device not found" >&2; exit 1; }
echo "$speed" > "$fan_cdev/cur_state"
' sh 50
```

Read current setting:

```sh
sh -c '
fan_cdev=
for cdev in /sys/class/thermal/cooling_device*; do
    [ "$(cat "$cdev/type" 2>/dev/null)" = "pcat-pm-fan" ] || continue
    fan_cdev=$cdev
    break
done
[ -n "$fan_cdev" ] || { echo "pcat-pm-fan cooling device not found" >&2; exit 1; }
cat "$fan_cdev/cur_state"
'
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
cat /sys/kernel/photonicat-pm/pmu_rtc_capability
```

The driver also forwards the raw PMU hardware/firmware version ACK frames
to `/dev/pcat-pm-ctl` for tools that query the PMU through the control
device. The sysfs attributes above are still updated internally by the
driver.

### Power-on Event

```bash
cat /sys/kernel/photonicat-pm/power_on_event
# 0 = unknown, 1 = power button, 2 = scheduled, 3 = charger connected, 4 = USB
```

### Schedule Boot

The driver registers an RTC device with alarm support. When an RTC alarm is set, the driver sends the alarm time to the PMU via UART (`SCHEDULE_STARTUP_TIME_SET`). The PMU stores this schedule and will power on the board at the specified time, even when the system is fully powered off. The alarm is one-shot (non-recurring).
All firmware starts in `pending-probe`; `/dev/rtc0` remains registered, but RTC
reads report invalid data and alarm programming fails until the driver observes
three consecutive valid, advancing PMU RTC samples and promotes the capability
to `enabled-probe`. No firmware version string enables RTC or scheduled boot by
itself.
Raw scheduled-boot commands sent through `/dev/pcat-pm-ctl` are gated by the
same RTC capability state.

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
Userspace can read selected raw PMU responses from this device, including
the PMU hardware/firmware version ACK frames used by
`pcat-pmu-updater --pmu-fw-version-get`.
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
