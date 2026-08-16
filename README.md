# Photonicat Power Manager Driver

[![Build DKMS Package](https://github.com/HackingGate/photonicat-pm/actions/workflows/dkms-build.yml/badge.svg)](https://github.com/HackingGate/photonicat-pm/actions/workflows/dkms-build.yml)

Linux kernel driver for the Photonicat 2 power management unit (PMU).

The PMU is a separate microcontroller (MCU) on the board. Its image is called
"MCU firmware" in the wiki and vendor tooling; this README says "PMU firmware"
throughout, matching the `pmu_fw_version` attribute. See the
[Photonicat PM Wiki](https://github.com/HackingGate/photonicat-pm/wiki) for
firmware inspection and flashing workflows.

## Scope

This driver is the host side of the UART link only. It does not build, sign,
package, or distribute PMU firmware, and it cannot change how the PMU behaves
once a command reaches it.

The firmware is closed source and published only as a wrapped binary image. The
UART protocol it speaks is not: the vendor's open-source userspace manager,
[`photonicat/rockchip_rk3568_pcat_manager`](https://github.com/photonicat/rockchip_rk3568_pcat_manager),
carries the command numbers and payload layouts in `src/pmu-manager.c`. That
source, together with observation of the wire, is where this driver's protocol
definitions come from.

No specification of behavior exists: no document states which commands a given
firmware version honors, what it does when it declines one, or which fields are
trustworthy. Everything here is established by testing real hardware;
behavior can change between firmware versions without notice.
Which firmware versions were tested is recorded under
[MCU Firmware Observed Behavior](https://github.com/HackingGate/photonicat-pm/wiki/MCU-Firmware-Observed-Behavior)
in the wiki.

Firmware defects are outside what this driver can fix — a PMU that ignores a
command, reports a broken clock, or rolls back an update behaves that way before
the driver sees the response. Report those to the vendor. Issues in this
repository are for the driver: parsing, sysfs and ABI behavior, kernel
integration, and packaging.

## PMU Firmware Capability Policy

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
- **Charge stop threshold**: starts as `pending-probe`. The driver queries the
  PMU on load; a reply in the 50–100 range caches the value and promotes
  `pmu_charge_threshold_capability` to `enabled-probe`.
  `charge_control_end_threshold` reads `ENODATA` while the capability is
  pending. Writes are not gated on the probe: firmware without support does
  not answer, so the write fails with `ETIMEDOUT`, and a firmware that answers
  promotes the capability.
- **Power-on mode**: starts as `pending-probe`. The driver queries the PMU on
  load, and any answer promotes `pmu_power_on_mode_capability` to
  `enabled-probe`. `power_on_mode` reads `ENODATA` while the capability is
  pending.
- **Status LED and beeper**: the driver reports the state from the PMU's last
  `STATUS_LED_BEEPER_V2_SET_ACK`, so a refused write is visible as a readback
  that reverts. Some firmware ignores the set command entirely and reports a
  constant state, which leaves both attributes uncontrollable.

A capability that stays `pending-probe` means the running firmware did not
answer the driver's probe for it. Which versions were tested against which
feature is recorded in
[MCU Firmware Observed Behavior](https://github.com/HackingGate/photonicat-pm/wiki/MCU-Firmware-Observed-Behavior),
along with the wiki's flashing instructions. Two limitations are not
version-specific:

- **Fan auto-speed reset**: no firmware exposes a trusted API for it, so
  restoring PMU auto speed needs the workarounds under
  [Fan Control](#fan-control).
- **`VOLTAGE_THRESHOLD_SET` (`0x17`)**, the LED, startup, charger limit,
  auto-shutdown and battery-full voltages the vendor manager configures: the
  PMU refuses every payload tested and offers no command to read the thresholds
  back, so the driver never sends it and exposes no attributes for it. The
  command number stays in `photonicat-pm.h` for raw `/dev/pcat-pm-ctl` users.

`pmu_hw_version` is a firmware-reported string, not a stable board revision —
the same board reports different values under different firmware.

## Features

### Power Supply

| Interface | Description |
|-----------|-------------|
| `/sys/class/power_supply/battery/` | Battery status, capacity (0–100%), voltage, current, power, and static design energy (read-only). |
| `/sys/class/power_supply/battery/charge_control_end_threshold` | Charge stop threshold in percent (read-write, 50–100). Stored in the PMU, so it survives driver reload and reboot. |
| `/sys/class/power_supply/charger/` | Charger online status and input voltage (read-only). |

The charge stop threshold is enforced by the PMU, not by the driver: writing
`charge_control_end_threshold` sends the value to the PMU and reports the
result of the PMU's ACK. Values outside 50–100 are rejected with `EINVAL`
before any command is sent, a PMU refusal returns `EIO`, and firmware without
charge threshold support returns `ETIMEDOUT`. Reads return `ENODATA` until the
PMU has answered a threshold query at least once; see
[PMU Firmware Capability Policy](#pmu-firmware-capability-policy).

> [!CAUTION]
> PMU protocol v2 status-report energy values are not validated as live or
> measured battery energy. The driver keeps `energy_full` as the static
> device-tree design capacity and does not export `energy_now`. The
> `power_now` value is computed from PMU voltage and current, not from PMU
> energy fields.

### Real-Time Clock & Scheduled Boot

| Interface | Description |
|-----------|-------------|
| `/dev/rtc0` | Real-time clock backed by PMU. Supports RTC alarms for scheduled power-on via `rtcwake(8)`. |

`/dev/rtc0` is registered before the PMU clock is trusted, so reads report
invalid data and alarm programming fails until `pmu_rtc_capability` reaches
`enabled-probe`. Firmware whose RTC never passes that probe is listed in the
wiki.

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

`status_led` and `beeper` reads report the state from the PMU's last
`STATUS_LED_BEEPER_V2_SET_ACK`, not the value last written. A read issued
immediately after a write returns the requested value because the ACK has not
arrived yet; wait about a second before reading back a confirmed state. A
readback that reverts means the PMU refused the write — older firmware ignores
the set command outright, which the wiki records.

### Power Button

The PMU reports a power button press with a `PMU_REQUEST_SHUTDOWN` frame. The
`pmu-button-mode` device tree property decides what the driver does with it:

| Mode | Behavior |
|------|----------|
| `poweroff` (default) | The driver calls `orderly_poweroff()`, the same as every release before this property existed. |
| `input` | The driver reports `KEY_POWER` on an input device named `photonicat-pm power button` and takes no other action. Userspace owns the policy, so `HandlePowerKey=` in `logind.conf(5)` applies to this button. |
| `ignore` | The driver logs the press and does nothing. |

The button on this board is wired to the PMU, not to the SoC. The `rk805
pwrkey` input device that a Photonicat 2 also exposes belongs to the PMIC and
is a different button, so `HandlePowerKey=` has no effect on the PMU button
unless `pmu-button-mode = "input"` is set.

The PMU sends one frame per press and never announces a release, so the driver
reports a press immediately followed by a release. Userspace sees every press
as a short press; `HandlePowerKeyLongPress=` cannot trigger from this button.

The mode can also be set with the `button_mode` module parameter, which
overrides the device tree property. This avoids a device tree overlay and a
reboot when trying a mode out:

```bash
# One boot only
rmmod photonicat_pm && insmod photonicat-pm.ko button_mode=input

# Persistently
echo 'options photonicat-pm button_mode=input' > /etc/modprobe.d/photonicat-pm.conf
```

The driver logs the mode it settled on and where it came from at probe:
`PMU button mode: input (module parameter)`.

On a desktop, the desktop environment usually takes over the power key from
`systemd-logind` and applies its own policy — GNOME defaults to suspend, not
power off. Whether the PMU button can wake this board from suspend has not
been verified, so set the desktop's power button action deliberately before
relying on `input` mode there.

```bash
# Confirm the input device is present (input mode only)
grep -A4 'photonicat-pm power button' /proc/bus/input/devices

# Watch presses without acting on them (by-path name comes from the UART
# address, so it is board-specific; this is the Photonicat 2 one)
sudo evtest /dev/input/by-path/platform-2afc0000.serial-event
```

The firmware debounces the button itself: on RA2E1260702000 a quick tap or a
hold under about two seconds sends nothing at all, and a hold of about three
seconds sends the request. A stray press in a bag therefore never reaches the
host in the first place.

> [!IMPORTANT]
> The PMU applies [`force-poweroff-timeout`](#optional-properties) to a
> shutdown it announces itself, not only to one the host announces. Once it
> has sent `PMU_REQUEST_SHUTDOWN` it stops reporting status and cuts power
> that many seconds later whatever the host does. On RA2E1260702000 that was
> 62 s with the property set to 60 and 125 s with it set to 120.
>
> So that a press the host declines does not become a power cut, the driver
> sends 0 for that timeout while the system is running in `input` or
> `ignore` mode, and restores the configured value in the shutdown handler
> and across suspend. A suspended host cannot service the button, so a press
> while suspended cuts power after the timeout in every mode — with button
> wake unverified, that is also the only way to recover a suspend that never
> wakes.
> A press then leaves the PMU running normally, and a shutdown that hangs is
> still cut short. A kernel hang is caught by the 60 s heartbeat watchdog in
> every mode. A hung userspace is not: in `input` mode the button then only
> queues an event nobody reads, and the firmware has no hard-cutoff hold, so
> recovery is SysRq, the serial console, or disconnecting power —
> `poweroff` mode forces the shutdown from the kernel instead.
>
> Boards whose device tree leaves `force-poweroff-timeout` unset are
> unaffected either way; the property defaults to 0. Armbian's Photonicat 2
> device tree sets it to 60.

### PMU Information

| Interface | Description |
|-----------|-------------|
| `/sys/kernel/photonicat-pm/pmu_hw_version` | PMU hardware version string (read-only). Queried from PMU on driver load. |
| `/sys/kernel/photonicat-pm/pmu_fw_version` | PMU firmware version string (read-only). Queried from PMU on driver load. |
| `/sys/kernel/photonicat-pm/pmu_rtc_capability` | PMU RTC policy state (read-only). Values: `pending-probe` or `enabled-probe`. |
| `/sys/kernel/photonicat-pm/pmu_charge_threshold_capability` | PMU charge threshold policy state (read-only). Values: `pending-probe` or `enabled-probe`. |
| `/sys/kernel/photonicat-pm/pmu_power_on_mode_capability` | PMU power-on mode policy state (read-only). Values: `pending-probe` or `enabled-probe`. |
| `/sys/kernel/photonicat-pm/power_on_event` | Last power-on event code (read-only). Values: 0 = unknown, 1 = power button, 2 = scheduled, 3 = charger connected, 4 = USB. |

### Configuration

| Interface | Description |
|-----------|-------------|
| `/sys/kernel/photonicat-pm/charger_on_auto_start` | Charger auto-start control (read-write). Write 1 to enable automatic startup when charger is connected, 0 to disable. |
| `/sys/kernel/photonicat-pm/power_on_mode` | PMU power-on mode (read-write). Reads `unconfigured`, `enabled`, or `disabled`; accepts `enabled`/`1` and `disabled`/`0`. Stored in the PMU. |

### Advanced

| Interface | Description |
|-----------|-------------|
| `/dev/pcat-pm-ctl` | Root-only raw PMU command interface. Userspace can read selected raw PMU responses, including hardware/firmware version ACKs used by `pcat-pmu-updater --pmu-fw-version-get`. See [Protocol](#protocol) for the frame format and `pcat-pm-ctl(4)` for details. |

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

        /* Config: what a PMU power button press does */
        pmu-button-mode = "input";

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
| `force-poweroff-timeout` | `<u32>` | 0 (disabled) | Config | Forced power-off timeout in seconds (0–255). Sent to the PMU via `WATCHDOG_TIMEOUT_SET` command at driver probe. When non-zero, the PMU cuts power this many seconds after a shutdown is announced — by the host, and also by the PMU itself when the power button is pressed. The 60s heartbeat watchdog does not cap it: that one only fires when heartbeats stop, and 120 here measured a 125s cut. Outside `pmu-button-mode = "poweroff"` the driver sends 0 while the system is running and the configured value at shutdown and during suspend, so a declined button press is not a power cut; see [Power Button](#power-button). Safety net for stuck shutdowns. |
| `pmu-button-mode` | string | `"poweroff"` | Config | What the driver does when the PMU reports a power button press: `"poweroff"` calls `orderly_poweroff()` from the driver, `"input"` reports `KEY_POWER` on an input device and leaves the decision to userspace, `"ignore"` logs the press and does nothing. An unrecognized value falls back to `"poweroff"` with a warning. Overridden by the `button_mode` module parameter when that is set. See [Power Button](#power-button). |
| `#thermal-sensor-cells` | `<0>` | (not set) | Config | Exposes the motherboard temperature to the kernel thermal framework. Must be `<0>` (no per-sensor arguments). Required when a `thermal-zones` binding in the board DTS references this node via `thermal-sensors`. Without this, the driver still registers an hwmon sensor but no thermal zone. |

## Usage Examples

### Battery Status

```bash
cat /sys/class/power_supply/battery/capacity
# 0-100 (battery capacity percentage)
# v2 PMU status reports use PMU SOC directly; shorter reports use OCV fallback

cat /sys/class/power_supply/battery/status
# Charging or Discharging

cat /sys/class/power_supply/battery/energy_full
# Static design full charge capacity from device tree, not live/measured capacity

cat /sys/class/power_supply/battery/power_now
# Computed from voltage_now and current_now

test ! -e /sys/class/power_supply/battery/energy_now
# energy_now is intentionally not exported by current driver releases
```

### Charge Stop Threshold

```bash
# Stop charging at 80% (accepted range is 50-100)
echo 80 > /sys/class/power_supply/battery/charge_control_end_threshold

# Read the threshold currently stored in the PMU
cat /sys/class/power_supply/battery/charge_control_end_threshold

# Charge to full again
echo 100 > /sys/class/power_supply/battery/charge_control_end_threshold

# Check whether the running firmware answered the threshold query
cat /sys/kernel/photonicat-pm/pmu_charge_threshold_capability
# pending-probe or enabled-probe
```

The PMU stores the threshold, so it survives driver reload, reboot, and power
off. Once the battery is above the threshold, `status` reads `Not charging`
until the threshold is raised or the battery drains below it.

### Fan Control

The PMU has two fan speed modes:

- **Unmanaged fan speed** — the PMU controls the fan based on its own internal logic (default).
- **Managed fan speed** — the driver sends a SET command to lock the fan at a specific percentage (0–100%).

Temperature-driven control is available in software: define a
[thermal zone](https://www.kernel.org/doc/html/latest/driver-api/thermal/sysfs-api.html)
in the device tree wiring the board temperature sensor (via
`#thermal-sensor-cells`) to the fan cooling device (via `#cooling-cells`), and
the kernel thermal governor sends SET commands as the temperature crosses trip
points. See the [Device Tree example](#example). Once the system shuts down the
governor stops and the PMU retains the last SET value.

> [!CAUTION]
> No firmware exposes a trusted API to reset fan control back to PMU auto
> speed; the steps below are workarounds.
>
> Because of that, `unmanaged` only means the driver has not sent a fan SET
> command since loading — the PMU may still be holding a fixed speed set
> earlier.
>
> After a shutdown in managed fan speed, the fan stays at the last fixed speed
> and will not adjust on its own. A low retained speed on a device that is still
> charging or otherwise thermally active is unsafe.
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

A value from 0 to 100 is the managed fan speed percentage set by the driver;
`unmanaged` is the driver-local state described in the caution above.

The snippets below run the write through `sudo sh -c` so the privileged shell
performs the `cur_state` redirection, which makes them safe to paste into
`bash`, `zsh`, or `fish`.

Set fan speed (switches PMU to managed speed). Replace the final argument with
the percentage to set; use `100` for maximum:

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

### Status LED and Beeper

`status_led` and `beeper` take 1 or 0, and read back the state from the PMU's
last ACK, so a read needs about a second to reflect a write:

```bash
echo 0 > /sys/kernel/photonicat-pm/status_led
sleep 1
cat /sys/kernel/photonicat-pm/status_led
```

On firmware that refuses the write, the read reverts to the PMU's own state. See
[LEDs & Peripherals](#leds--peripherals).

### PMU Hardware / Firmware Version

```bash
cat /sys/kernel/photonicat-pm/pmu_hw_version
cat /sys/kernel/photonicat-pm/pmu_fw_version
cat /sys/kernel/photonicat-pm/pmu_rtc_capability
```

The raw version ACK frames are also forwarded to `/dev/pcat-pm-ctl` for tools
that query the PMU through the control device.

### Schedule Boot

The driver registers an RTC device with alarm support. Setting an alarm sends
the time to the PMU (`SCHEDULE_STARTUP_TIME_SET`), which stores it and powers on
the board at that time even from a full power-off. The alarm is one-shot.

All firmware starts in `pending-probe`: `/dev/rtc0` stays registered, but reads
report invalid data and alarm programming fails until the driver observes three
consecutive valid, advancing PMU RTC samples. Raw scheduled-boot commands
through `/dev/pcat-pm-ctl` are gated the same way, and no firmware version
string enables either by itself.

Works with standard Linux RTC tools such as `rtcwake(8)`:

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

### Power-On Mode

```bash
# Read the mode stored in the PMU
cat /sys/kernel/photonicat-pm/power_on_mode
# unconfigured, enabled, or disabled

# Power on automatically when external power is applied
echo enabled > /sys/kernel/photonicat-pm/power_on_mode

# Back to manual power-on
echo disabled > /sys/kernel/photonicat-pm/power_on_mode
```

> [!CAUTION]
> `unconfigured` is the state of a PMU whose power-on mode has never been set,
> and leaving it is one-way. The PMU accepts only `enabled` and `disabled`, so
> the first write to this attribute leaves `unconfigured` permanently.
> `disabled` is the state the vendor manager uses for manual power-on.

### Control Device (`/dev/pcat-pm-ctl`)

Raw escape hatch for advanced PMU commands using the binary serial protocol.
See [Protocol](#protocol) below for the frame format, and the `pcat-pm-ctl(4)`
man page for the commands the driver refuses to forward and the responses that
can be read back.

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

PMU command logging uses the kernel `dev_dbg` facility. Messages are compiled
out, or dynamically off, by default.

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
