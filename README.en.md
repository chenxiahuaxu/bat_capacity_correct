# Battery Capacity Correct / 电池容量校正

[English](README.en.md) | [中文](README.md)

A lightweight Magisk module that corrects battery percentage display errors on Xiaomi devices running third-party ROMs (e.g., LineageOS).

---

## Background

On certain Xiaomi devices (Redmi Note 9 Pro / joyeuse / miatoll) running AOSP-derived ROMs (LineageOS, etc.), the battery percentage displayed by the system UI is severely inaccurate:

- Stuck at 1%~4% while the device runs normally for hours
- Percentage doesn't rise during charging
- Reboot doesn't help

The same device works correctly under MIUI, ruling out hardware failure. The root cause: third-party ROMs lack the proprietary QG (Qualcomm Gauge) calibration config present in Xiaomi's closed-source BSP.

## How It Works

The module runs a background daemon that chooses one of two paths based on whether `capacity_raw` is valid:

```
capacity_raw (when valid) → Coulomb Counting + OCV Calibration → Voltage Lookup Table
```

### Data Source Strategy

| State | Condition | Method |
|-------|-----------|--------|
| `FALLBACK_RAW` | `capacity_raw ≥ 10%` | Direct overwrite with `raw / 100` |
| `FALLBACK_VOLTAGE` | `capacity_raw < 10%` | Coulomb counting + OCV calibration + voltage lookup |

When raw is valid, use it directly — no further processing needed. When raw is corrupted (chip ADC returns garbage), the module enters the voltage estimation state, which internally runs coulomb counting as the primary tracking source, OCV calibration for idle drift correction, and voltage lookup as the ultimate fallback.

### Coulomb Counting + OCV Calibration

Hybrid approach used by TI / ADI / MPS professional fuel gauge ICs:

1. **Coulomb Counting**: `ΔSOC = ∫I·dt / full_capacity`, immune to voltage noise, accurately tracks charge/discharge
2. **OCV Calibration**: When current < 250mA, voltage → SOC via lookup table → nudges coulomb count with 33% weight
3. **Unidirectional Constraint**: SOC only increases during charging, only decreases during discharging

Parameters: `charge_full_design = 4820000 µAh`, `1% SOC change = 173,520,000 µA·s`.

### Anomaly Detection

Each cycle cross-checks:
- `bms/capacity` — BMS driver computed value
- `capacity_raw` — Chip raw ADC (0~10000)
- `voltage_now` — Battery terminal voltage (µV)
- `current_now` — Real-time current (µA, negative = charging)

See `detect_anomaly()` function comments for detection conditions.

### State Machine

```
INIT → NORMAL (no anomaly, patrol every 10s)
     → FALLBACK_RAW (raw valid, overwrite raw/100 every 3s)
     → FALLBACK_VOLTAGE (raw broken, coulomb+OCV+voltage every 3s)
```

## Sysfs Node Mapping

| Node | Path | Purpose |
|------|------|---------|
| Write target | `battery/capacity` | Value read by Android framework |
| Raw source | `bms/capacity_raw` | Chip raw SOC (0~10000) |
| BMS computed | `bms/capacity` | Driver-level SOC (anomaly detection) |
| Voltage | `bms/voltage_now` | Voltage estimation + OCV calibration |
| Current | `bms/current_now` | Coulomb counting integration |
| Full capacity | `bms/charge_full_design` | Coulomb counting reference (4820000 µAh) |

## Installation

1. Ensure device is rooted with Magisk ≥ 20.4
2. Flash the module ZIP in Magisk Manager
3. Reboot

## Verification & Monitoring

```bash
# System battery level
dumpsys battery | grep level

# Live log
tail -f /data/local/tmp/bat_correct.log

# Compare data sources
cat /sys/class/power_supply/bms/capacity_raw
cat /sys/class/power_supply/bms/capacity
cat /sys/class/power_supply/battery/capacity
cat /sys/class/power_supply/bms/voltage_now
cat /sys/class/power_supply/bms/current_now
```

## Log Format

Three lines per cycle, uniform fields across all states:

```
读取: capacity_bms=53%  capacity_bat=7%  capacity_raw=0  voltage_now=4078mV  current_now=-300mA  status=Charging
判断: 检测=类型B(raw损坏)  →  选择 电压估算
执行: 写入 69% → battery/capacity  ✓  [now=4078mV OCV=4063mV 库仑=69% |I|=300mA 中]
```

| Field | Meaning |
|-------|---------|
| `capacity_bms` | BMS driver computed value (for detection only) |
| `capacity_bat` | Current `battery/capacity` node value (Android display) |
| `capacity_raw` | Chip raw / 100 (may be 0 during anomaly) |
| `voltage_now` | Real-time battery terminal voltage |
| `current_now` | Real-time current (negative = charging) |
| `OCV` | EMA-smoothed open-circuit voltage estimate |
| `库仑` (Coulomb) | Coulomb-counter-tracked SOC (primary tracking source) |
| `|I|` | Absolute current magnitude |
| `置信` (Confidence) | 校准/高/中/低 — OCV calibration trigger or current damping level |
| `✓/✗` | Write success / BMS rejected |

## Build

```bash
# Windows (NDK 23+)
source\build.bat

# Linux / macOS
export ANDROID_NDK_HOME=/path/to/ndk
$ANDROID_NDK_HOME/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android24-clang \
    -O2 -fPIE -pie -o bat_capacity_correct source/bat_capacity_correct.c
```

## License

[GNU General Public License v3.0](LICENSE)

## References

- [liyafe1997 / Xiaomi-fix-battery-one-percent](https://github.com/liyafe1997/Xiaomi-fix-battery-one-percent)
- TI MSPM0 Gauge L2 Solution — Coulomb counter + OCV data fusion
- Analog Devices ModelGauge m5 EZ Algorithm — Hybrid fuel gauge
- 《电池容量(%)纠正》v1.3.0 — 嘟嘟ski
