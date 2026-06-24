# 电池容量校正 / Battery Capacity Correct

[中文](README.md) | [English](README.en.md)

一个轻量级 Magisk 模块，用于校正第三方 ROM（如 LineageOS）下 Android 设备的电池电量显示偏差。

---

## 背景

部分 Android 设备（尤其是 xiaomi 系列）在 LineageOS 等 AOSP 衍生 ROM 下，系统电量百分比与真实电量严重不符：

- 电量卡在 1%~4%，但实际可正常使用数小时
- 充电时电量不上升
- 重启无效

同设备在 MIUI 下正常，排除硬件故障。根因是第三方 ROM 缺少 Xiaomi 闭源 BSP 中 QG (Qualcomm Gauge) 电量计的完整校准配置。

## 工作原理

模块运行后台守护进程，根据 `capacity_raw` 是否有效选择两条路径之一：

```
capacity_raw (有效时) → 库仑计数 + OCV 校准 → 电压查表估算
```

### 数据源策略

| 状态 | 条件 | 方法 |
|------|------|------|
| `FALLBACK_RAW` | `capacity_raw ≥ 10%` | 直接覆写 `raw / 100` |
| `FALLBACK_VOLTAGE` | `capacity_raw < 10%` | 库仑计数 + OCV 校准 + 电压查表 |

当 raw 有效时直接使用，无需其他手段。当 raw 损坏（芯片 ADC 返回异常值）时，进入电压估算状态，内部同时运行库仑计数作为主力跟踪源，OCV 校准为空闲时修正，电压查表为终极兜底。

### 库仑计数 + OCV 校准

采用 TI / ADI / MPS 等专业电量计同款混合方案：

1. **库仑计数**：`ΔSOC = ∫I·dt / 满容量`，不受电压噪声影响，准确跟踪充放电变化量
2. **OCV 校准**：电流 < 250mA 时记录电压 → 查表得 SOC → 以 33% 权重拉正库仑计累积误差
3. **单向约束**：充电态容量只升不降，放电态只降不升，物理上保证单调性

容量参数：`charge_full_design = 4820000 µAh`，`SOC 变化 1% = 173,520,000 µA·s`。

### 异常检测

每轮循环交叉比对以下节点：
- `bms/capacity` — BMS 驱动计算值
- `capacity_raw` — 芯片原始 ADC 值 (0~10000)
- `voltage_now` — 电池端电压 (µV)
- `current_now` — 实时电流 (µA，负=充电)

异常判定条件见 `detect_anomaly()` 函数注释。

### 状态机

```
INIT → NORMAL（无异常，每 10s 巡检）
     → FALLBACK_RAW（raw 可信，每 3s 覆写 raw/100）
     → FALLBACK_VOLTAGE（raw 损坏，每 3s 库仑+OCV+电压）
```

## Sysfs 节点映射

| 节点 | 路径 | 用途 |
|------|------|------|
| 写入目标 | `battery/capacity` | Android 读取的电量值 |
| raw 真值源 | `bms/capacity_raw` | 芯片原始 SOC (0~10000) |
| BMS 计算值 | `bms/capacity` | 驱动层 SOC（异常检测用） |
| 电压 | `bms/voltage_now` | 电压估算 + OCV 校准 |
| 电流 | `bms/current_now` | 库仑计数积分 |
| 满容量 | `bms/charge_full_design` | 库仑计数基准 (4820000 µAh) |

## 安装

1. 确保设备已 Root，Magisk ≥ 20.4
2. 在 Magisk Manager 中刷入模块 ZIP
3. 重启设备

## 验证与监控

```bash
# 查看系统电量
dumpsys battery | grep level

# 查看模块运行日志
tail -f /data/local/tmp/bat_correct.log

# 对比各数据源
cat /sys/class/power_supply/bms/capacity_raw
cat /sys/class/power_supply/bms/capacity
cat /sys/class/power_supply/battery/capacity
cat /sys/class/power_supply/bms/voltage_now
cat /sys/class/power_supply/bms/current_now
```

## 日志解读

每轮循环固定三行，格式统一：

```
读取: capacity_bms=53%  capacity_bat=7%  capacity_raw=0  voltage_now=4078mV  current_now=-300mA  status=Charging
判断: 检测=类型B(raw损坏)  →  选择 电压估算
执行: 写入 69% → battery/capacity  ✓  [now=4078mV OCV=4063mV 库仑=69% |I|=300mA 中]
```

| 字段 | 含义 |
|------|------|
| `capacity_bms` | BMS 驱动计算的电量（异常检测用，不直接使用） |
| `capacity_bat` | `battery/capacity` 节点当前值（Android 显示的） |
| `capacity_raw` | 芯片原始值 / 100（异常时可能为 0） |
| `voltage_now` | 电池实时端电压 |
| `current_now` | 实时电流，负=充电正=放电 |
| `OCV` | EMA 平滑后的开路电压估算值 |
| `库仑` | 库仑计数器推算的 SOC（主力跟踪源） |
| `|I|` | 电流绝对值 |
| `置信` | 校准/高/中/低——OCV 校准触发或电流阻尼等级 |
| `✓/✗` | 写入成功/BMS 拒绝 |

## 构建

```bash
# Windows (NDK 23+)
source\build.bat

# Linux / macOS
export ANDROID_NDK_HOME=/path/to/ndk
$ANDROID_NDK_HOME/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android24-clang \
    -O2 -fPIE -pie -o bat_capacity_correct source/bat_capacity_correct.c
```

## 许可

[GNU General Public License v3.0](LICENSE)

## 参考

- [liyafe1997 / Xiaomi-fix-battery-one-percent](https://github.com/liyafe1997/Xiaomi-fix-battery-one-percent)
- TI MSPM0 Gauge L2 Solution — 库仑计 + OCV 数据融合方案
- Analog Devices ModelGauge m5 EZ Algorithm — 混合电量计
- 《电池容量(%)纠正》v1.3.0 — 嘟嘟ski
