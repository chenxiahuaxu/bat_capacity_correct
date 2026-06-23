# 电池容量校正 (Battery Capacity Correct)

一个轻量级的 Magisk 模块，用于校正 Android 设备上电池电量百分比显示与真实电量之间的偏差。

---

## 背景

部分 Android 设备（尤其是 xiaomi 系列）在刷入 LineageOS 等第三方 ROM 后，系统 UI 显示的电量百分比与电池真实电量严重不符。常见现象包括：

- 电量显示卡在 1%，但实际上可正常使用数小时
- 充电时电量不上升
- 重启无效

但同一设备在 MIUI 官方固件下表现正常，说明硬件无故障。

**根本原因**：第三方 ROM 的内核驱动或 Health HAL 层对电量芯片的 SOC（State of Charge）映射链路存在缺陷，而 MIUI 拥有闭源 BSP 的完整校准配置。

**本模块的原理**：直接从芯片原始数据节点 `capacity_raw`（0~10000 范围）读取真实电量，转换为百分比后覆写到系统读取的 `capacity` 节点，绕过可能存在问题的驱动→HAL 映射链路。

## 安装

1. 确保设备已 Root 并安装 Magisk ≥ 20.4
2. 在 Magisk Manager 中刷入模块 ZIP
3. 重启设备

## 验证

```bash
# 查看当前系统报告的电量
dumpsys battery | grep level

# 对比原始值
cat /sys/class/power_supply/bms/capacity_raw   # 除以 100 即真实 %
cat /sys/class/power_supply/bms/capacity       # 系统显示的 %
```

## 许可

[GNU General Public License v3.0](LICENSE)

## 参考

- [liyafe1997 / Xiaomi-fix-battery-one-percent](https://github.com/liyafe1997/Xiaomi-fix-battery-one-percent) — 骁龙 865/870 rapid_soc_dec 问题内核级分析与修复
- 《电池容量(%)纠正》v1.3.0 — 嘟嘟ski

