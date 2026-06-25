/*
 * bat_capacity_correct.c — 多阶段电池电量校正
 *
 * ── 设计原理 ──────────────────────────────────────────────────────────────
 *
 * 核心问题: 部分Android设备在第三方 ROM 下, BMS 的 capacity_raw 恒为 0,
 *          系统电量卡在 1-4%。需要纯软件的方案替代硬件电量计。
 *
 * 两条腿走路:
 *   (1) 库仑计数 (Coulomb counter) — 积分电流, 跟踪相对变化量。
 *       基于电荷守恒定律: SOC(t) = SOC(t0) - (1/C_rated) ∫ I·dt
 *       不受电压噪声影响, 高频准确; 但会随时间和传感器误差缓慢漂移。
 *
 *   (2) OCV 校准 (Open Circuit Voltage) — 用电压查表, 修正绝对位置。
 *       放电低负载时, 端电压 ≈ 开路电压 (OCV), 通过 OCV-SOC 曲线
 *       可独立估算 SOC。低频准确, 用来校准库仑计数累积的漂移。
 *
 *   两者互补: 库仑快而漂, OCV 慢而准。标准电量计 (TI/ADI/MPS) 均为此架构。
 *
 * OCV 校准时机:
 *   放电低电流 (|I|<250mA) — 立刻触发, 端电压 ≈ OCV, 最可靠。
 *   充电 — 端电压被 IR 压升, 不可直接使用。
 *         需先通过内阻 R 计算 IR 压降扣掉, 再校准。但 IR 压降估算有误差,
 *         故充电时改用 60s 定时触发, 并用 discharge_ocv(上次放电极的 OCV 基准)兜底。
 *   放电高电流 — 端电压被 IR 压低, 需通过内阻补回。
 *
 * 内阻 R 的计算:
 *   充放共用同一内阻 (物理上相同)。在 |I|≥500mA 时,
 *   用 stop_lt_smooth 与 discharge_ocv 的差值除以当前电流, EMA 平滑后
 *   作为统一内阻值。充电扣 IR, 放电补 IR。
 *
 * OCV 基准 (discharge_ocv):
 *   放电 |I|<250mA 时持续 EMA 跟踪, 是系统中最接近真实开路电压的值。
 *   充电时用作兜底电压; 放电高电流时用作内阻计算参照。
 *
 * 单向约束:
 *   物理上, 充电 SOC 只升不降, 放电只降不升。防止 KPI 突变。
 *   初始化帧时强制重置约束, 接受首次低负载 OCV 全量写入。
 *
 * 电压表的使用:
 *   OCV→SOC 查表仅在最写入时调用一次, 中间所有运算都在 SOC% 或 mV 空间。
 *
 * ── 状态机 ────────────────────────────────────────────────────────────────
 *
 *   INIT              → 等待 10s BMS 初始化 → 检测进入 NORMAL 或兜底
 *   NORMAL            → 无异常, 后台巡检 (boot 异常时持续同步)
 *   RECOVERING        → 尝试恢复控制器 (暂不启用)
 *   FALLBACK_RAW      → raw/100 覆写 battery/capacity
 *   FALLBACK_VOLTAGE  → 库仑计数 + OCV 校准, 核心路径
 *
 * ── 数据处理 ──────────────────────────────────────────────────────────────
 *
 *   主循环: 每 3s 一轮 FALLBACK_VOLTAGE 流水线
 *     Step 1: 库仑计数  (coulomb_step)  — SOC 空间积分
 *     Step 2: EMA + OCV 基准  (ema_smooth, track_ocv)
 *     Step 3: 首次初始化      (try_init)
 *     Step 4: IR 补偿         (ir_compensate)
 *     Step 5: OCV 校准        (ocv_calibrate)
 *     Step 6: 信任标签 + 单向约束 + 写入
 *     Step 3: 单向约束  (constrain_step)
 *     Step 4: 写入 sysfs  (voltage_to_capacity 查表一次)
 *
 * ──────────────────────────────────────────────────────────────────────────
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/stat.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * sysfs 路径常量
 * ═══════════════════════════════════════════════════════════════════════════ */

#define SYS_CAPACITY_RAW  "/sys/class/power_supply/bms/capacity_raw"
#define SYS_CAPACITY      "/sys/class/power_supply/bms/capacity"
#define SYS_BAT_CAPACITY  "/sys/class/power_supply/battery/capacity"  /* 写入目标 */
#define SYS_VOLTAGE_NOW   "/sys/class/power_supply/bms/voltage_now"
#define SYS_CURRENT_NOW   "/sys/class/power_supply/bms/current_now"
#define SYS_STATUS        "/sys/class/power_supply/battery/status"
#define SYS_BMS_RESET     "/sys/class/power_supply/bms/reset"
#define SYS_CHARGE_FULL   "/sys/class/power_supply/bms/charge_full_design"

/* ═══════════════════════════════════════════════════════════════════════════
 * 阈值
 * ═══════════════════════════════════════════════════════════════════════════ */

#define CAP_MAX_ANOMALY    2     /* capacity ≤ 2% → 可疑                  */
#define VOLT_MIN_HEALTHY   3700  /* 电压 ≥ 3700mV → 电池实际不亏电          */
#define RAW_MIN_OK         10    /* raw/100 ≥ 10% → raw 可信               */
#define RAW_BAD_THRESHOLD  5     /* raw/100 ≤ 5% 且电压高 → raw 已损坏     */

#define CUR_HIGH_TRUST     200   /* 电流 < 此值：高置信，全量更新 EMA       */
#define CUR_MED_TRUST      500   /* 200~500mA：中置信，半量更新             */
                                 /* > 500mA：低置信，不更新，保持上一条     */

#define RECOVERY_WAIT_SEC  10    /* 恢复后等待 PMIC 重算                    */
#define NORMAL_POLL_SEC    10    /* NORMAL 状态巡检间隔                      */
#define FALLBACK_POLL_SEC  3     /* 兜底覆写间隔                            */

/* ═══════════════════════════════════════════════════════════════════════════
 * 状态枚举 / 状态枚举
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    STATE_INIT,
    STATE_NORMAL,
    STATE_RECOVERING,
    STATE_FALLBACK_RAW,
    STATE_FALLBACK_VOLTAGE,
} State;

static const char *state_name(State s)
{
    switch (s) {
    case STATE_INIT:             return "初始化";
    case STATE_NORMAL:           return "正常";
    case STATE_RECOVERING:       return "恢复中";
    case STATE_FALLBACK_RAW:     return "raw兜底";
    case STATE_FALLBACK_VOLTAGE: return "电压估算";
    default:                     return "未知";
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 传感器数据结构
 * ═══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    int  bms_cap_pct;       /* bms/capacity    (0~100) — 驱动原始值，用于异常检测 */
    int  bat_cap_pct;       /* battery/capacity(0~100) — Android 显示值，写入目标   */
    int  raw_pct;           /* capacity_raw    (0~100)                                */
    int  current_ua;        /* current_now     (µA)  — 负=充电, 正=放电              */
    int  volt_mv;           /* voltage_now     (mV)                                   */
    char charging[16];      /* status 字符串                                          */
} SensorData;

/* ═══════════════════════════════════════════════════════════════════════════
 * 日志宏
 * ═══════════════════════════════════════════════════════════════════════════ */

#define LOG(fmt, ...) do { \
    time_t _t = time(NULL); \
    struct tm *_tm = localtime(&_t); \
    fprintf(stderr, "[%02d:%02d:%02d] bat_correct: " fmt "\n", \
            _tm->tm_hour, _tm->tm_min, _tm->tm_sec, ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

/* ═══════════════════════════════════════════════════════════════════════════
 * 模块1：sysfs 读写工具
 * ═══════════════════════════════════════════════════════════════════════════ */

static int sysfs_read_int(const char *path, int *out)
{
    FILE *fp;
    char buf[32] = {0};

    fp = fopen(path, "r");
    if (!fp) return -1;
    if (!fgets(buf, sizeof(buf), fp)) { fclose(fp); return -1; }
    fclose(fp);
    *out = atoi(buf);
    return 0;
}

static int sysfs_read_str(const char *path, char *buf, size_t len)
{
    FILE *fp;
    fp = fopen(path, "r");
    if (!fp) return -1;
    if (!fgets(buf, (int)len, fp)) { fclose(fp); return -1; }
    size_t n = strlen(buf);
    if (n > 0 && buf[n - 1] == '\n') buf[n - 1] = '\0';
    fclose(fp);
    return 0;
}

static int sysfs_write_int(const char *path, int val)
{
    FILE *fp;
    fp = fopen(path, "w");
    if (!fp) return -1;
    fprintf(fp, "%d", val);
    fclose(fp);
    return 0;
}

/* 写字符串到 sysfs（用于覆写 capacity 节点）
 *   1. chmod 确保可写（BMS 驱动可能重置权限）
 *   2. 先不带换行符写（兼容原方案）
 *   3. 立刻回读验证，不匹配则带换行符重试
 */
static int sysfs_write_str(const char *path, const char *val)
{
    FILE *fp;
    int verify;

    /* 确保节点可写 */
    chmod(path, 0666);

    /* 尝试 1：不带换行符 */
    fp = fopen(path, "w");
    if (!fp) return -1;
    fprintf(fp, "%s", val);
    fclose(fp);

    /* 回读验证 */
    if (sysfs_read_int(path, &verify) == 0 && verify == atoi(val))
        return 0;

    /* 尝试 2：带换行符（部分内核驱动要求） */
    chmod(path, 0666);
    fp = fopen(path, "w");
    if (!fp) return -1;
    fprintf(fp, "%s\n", val);
    fclose(fp);

    if (sysfs_read_int(path, &verify) == 0 && verify == atoi(val))
        return 0;

    return -1;  /* 两种方式均被 BMS 覆盖 */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * capacity 节点写入句柄（保持打开，同原始方案）
 * ═══════════════════════════════════════════════════════════════════════════ */

static FILE *g_fp_cap = NULL;
static int     g_boot_abnormal  = 0;
static int64_t g_uas_per_pct    = 173520000LL;  /* 默认 4820000µAh */

/* 写 capacity 节点 — 仿原始方案：保持 fd 打开，fprintf + fflush */
static int write_capacity(const char *val)
{
    if (!g_fp_cap) return -1;
    fprintf(g_fp_cap, "%s", val);
    fflush(g_fp_cap);

    /* 回读验证 */
    int verify;
    if (sysfs_read_int(SYS_BAT_CAPACITY, &verify) == 0 && verify == atoi(val))
        return 0;
    return -1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 模块2：传感器读取
 * ═══════════════════════════════════════════════════════════════════════════ */

static int read_sensors(SensorData *s)
{
    int raw_raw; /* 原始 raw 值 (0~10000) */

    if (sysfs_read_int(SYS_CAPACITY_RAW, &raw_raw) != 0) return -1;
    s->raw_pct = raw_raw / 100;
    /* 钳位：BMS 异常时 raw 可能返回 INT_MIN 等溢出值 */
    if (s->raw_pct < 0)   s->raw_pct = 0;
    if (s->raw_pct > 100) s->raw_pct = 100;

    if (sysfs_read_int(SYS_VOLTAGE_NOW, &s->volt_mv) != 0) return -1;
    s->volt_mv /= 1000; /* µV → mV */

    /* current_now: 电流 (µA)，负=充电，best-effort */
    if (sysfs_read_int(SYS_CURRENT_NOW, &s->current_ua) != 0)
        s->current_ua = 0;

    if (sysfs_read_int(SYS_CAPACITY, &s->bms_cap_pct) != 0) return -1;

    if (sysfs_read_int(SYS_BAT_CAPACITY, &s->bat_cap_pct) != 0) return -1;

    if (sysfs_read_str(SYS_STATUS, s->charging, sizeof(s->charging)) != 0)
        strcpy(s->charging, "Unknown");

    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 模块3：异常检测
 *
 * 返回：
 *   0 — 无异常
 *   1 — 异常类型A（capacity 错，raw 可信）
 *   2 — 异常类型B（capacity 和 raw 都错了，需电压估算）
 * ═══════════════════════════════════════════════════════════════════════════ */

static int detect_anomaly(const SensorData *s)
{
    /* 检查1：系统报极低电量，但电压健康 */
    if (s->bms_cap_pct <= CAP_MAX_ANOMALY && s->volt_mv >= VOLT_MIN_HEALTHY) {
        if (s->raw_pct >= RAW_MIN_OK)
            return 1;   /* 类型A：raw 可信 */
        else
            return 2;   /* 类型B：raw 也坏了 */
    }

    /* 检查2：raw 接近 0 但电压很高 → raw 损坏
     *         前提：bms 读数明显高于 raw（两者不一致才判异常） */
    if (s->raw_pct <= RAW_BAD_THRESHOLD
        && s->volt_mv >= VOLT_MIN_HEALTHY
        && s->bms_cap_pct > RAW_BAD_THRESHOLD)
        return 2;

    /* 检查2b：raw 极低（≤1%）且电压健康 → 无论如何 raw 已损坏 */
    if (s->raw_pct <= 1 && s->volt_mv >= VOLT_MIN_HEALTHY)
        return 2;

    /* 检查3：bms 与 raw 偏差 >3% */
    if (abs(s->bms_cap_pct - s->raw_pct) > 3)
        return 1;

    /* 检查4：系统显示与 raw 偏差 >5%，raw 可信时需覆写 */
    if (s->raw_pct >= RAW_MIN_OK
        && abs(s->bat_cap_pct - s->raw_pct) > 5)
        return 1;

    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 模块4：电压 → 电量 查表 (5020mAh 锂电池放电曲线)
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * 锂电池 OCV-SOC 放电曲线（4.4V 高压锂聚合物, 3.85V nominal）
 *
 * 数据源: Qualcomm QG 燃油计 pc-temp-ocv-lut @ 25°C
 * (ASUS ZE550KL 4.4V 3000mAh 电池, Android 内核设备树官方标定)
 * 端点: 4.372V (100%), 3.400V (cutoff), 3.000V (0%)
 *
 * 锚点 (内核原始数据, 百分比→mV):
 *   100%:4372  95%:4306  90%:4247  85%:4190  80%:4134
 *    75%:4081  70%:4030  65%:3984  60%:3930  55%:3884
 *    50%:3850  45%:3826  40%:3804  35%:3786  30%:3770
 *    25%:3753  20%:3734  10%:3684   5%:3643   0%:3000
 *
 * 锚点间线性插值 → 101 个采样点。
 */
static const int voltage_map[][2] = {
    /* SOC%  V_mV */
    {4400, 100},
    {4372,  99}, {4346,  98}, {4324,  97}, {4306,  96}, {4292,  95},
    {4280,  94}, {4268,  93}, {4257,  92}, {4247,  91}, {4238,  90},
    {4228,  89}, {4219,  88}, {4210,  87}, {4200,  86}, {4190,  85},
    {4179,  84}, {4168,  83}, {4157,  82}, {4146,  81}, {4134,  80},
    {4124,  79}, {4113,  78}, {4103,  77}, {4092,  76}, {4081,  75},
    {4071,  74}, {4061,  73}, {4051,  72}, {4041,  71}, {4030,  70},
    {4021,  69}, {4012,  68}, {4003,  67}, {3994,  66}, {3984,  65},
    {3975,  64}, {3966,  63}, {3957,  62}, {3948,  61}, {3939,  60},
    {3930,  59}, {3921,  58}, {3912,  57}, {3903,  56}, {3894,  55},
    {3884,  54}, {3875,  53}, {3866,  52}, {3857,  51}, {3850,  50},
    {3845,  49}, {3840,  48}, {3835,  47}, {3831,  46}, {3826,  45},
    {3822,  44}, {3817,  43}, {3813,  42}, {3808,  41}, {3804,  40},
    {3800,  39}, {3797,  38}, {3793,  37}, {3790,  36}, {3786,  35},
    {3783,  34}, {3780,  33}, {3776,  32}, {3773,  31}, {3770,  30},
    {3767,  29}, {3763,  28}, {3760,  27}, {3756,  26}, {3753,  25},
    {3749,  24}, {3745,  23}, {3741,  22}, {3738,  21}, {3734,  20},
    {3729,  19}, {3724,  18}, {3719,  17}, {3714,  16}, {3710,  15},
    {3705,  14}, {3700,  13}, {3695,  12}, {3690,  11}, {3684,  10},
    {3676,   9}, {3668,   8}, {3660,   7}, {3652,   6}, {3643,   5},
    {3615,   4}, {3587,   3}, {3558,   2}, {3530,   1}, {3400,   0},
};
#define VMAP_LEN (sizeof(voltage_map) / sizeof(voltage_map[0]))

static int voltage_to_capacity(int volt_mv)
{
    if (volt_mv >= voltage_map[0][0])              return 100;
    if (volt_mv <= voltage_map[VMAP_LEN - 1][0])   return 0;

    for (int i = 0; i < (int)VMAP_LEN - 1; i++) {
        int v_hi = voltage_map[i][0],     v_lo = voltage_map[i + 1][0];
        if (volt_mv <= v_hi && volt_mv >= v_lo) {
            int c_hi = voltage_map[i][1], c_lo = voltage_map[i + 1][1];
            int rv = v_hi - v_lo, rc = c_hi - c_lo, d = volt_mv - v_lo;
            return c_lo + (d * rc + rv / 2) / rv;
        }
    }
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 模块5：控制器恢复（暂不启用 / 暂不启用）
 * ═══════════════════════════════════════════════════════════════════════════ */

static void do_recovery(void)
{
    LOG("  >>> 执行恢复操作 <<<");

    /* a) 清除 Android Framework 层电池缓存 */
    LOG("  [恢复] dumpsys battery reset");
    system("dumpsys battery reset 2>/dev/null");
    usleep(500000);

    /* b) 尝试 BMS 硬件重置 */
    if (access(SYS_BMS_RESET, W_OK) == 0) {
        sysfs_write_int(SYS_BMS_RESET, 1);
        LOG("  [恢复] 已触发 BMS 硬件重置");
    } else {
        LOG("  [恢复] BMS reset 节点不存在，跳过");
    }
    usleep(500000);

    /* c) 写入伪造电量触发内核重算 SOC */
    sysfs_write_int(SYS_CAPACITY, 50);
    LOG("  [恢复] 写入 capacity=50 触发内核重算");
    usleep(500000);

    LOG("  [恢复] 等待 %d 秒让 PMIC 自校准...", RECOVERY_WAIT_SEC);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 状态函数
 *
 * 每个状态函数：
 *   接收传感器数据，执行该状态的逻辑，返回下一个状态。
 *   在函数内部管理自己的 sleep / 计时。
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── STATE_INIT：初始检测 ──────────────────────────────────────────────── */

static State state_init(SensorData *s)
{
    LOG("╔════════════════════════════════════════╗");
    LOG("║  bat_capacity_correct 服务启动         ║");
    LOG("╚════════════════════════════════════════╝");
    LOG("初始读数: capacity_bms=%d%%  capacity_bat=%d%%  capacity_raw=%d%%  voltage_now=%dmV  status=%s",
        s->bms_cap_pct, s->bat_cap_pct, s->raw_pct, s->volt_mv, s->charging);
    LOG("等待 BMS 初始化 (10s)...");
    sleep(10);

    if (read_sensors(s) != 0)
        LOG("警告: BMS 初始化后传感器读取失败");

    LOG("BMS 初始化后: capacity_bms=%d%%  capacity_bat=%d%%  capacity_raw=%d%%  voltage_now=%dmV  status=%s",
        s->bms_cap_pct, s->bat_cap_pct, s->raw_pct, s->volt_mv, s->charging);

    int type = detect_anomaly(s);

    if (type == 0) {
        LOG("检测结果: 无异常 → 进入 NORMAL 状态");
        return STATE_NORMAL;
    } else if (type == 1) {
        LOG("检测结果: 异常类型A (raw可信) → 进入 raw兜底");
        return STATE_FALLBACK_RAW;
    } else {
        LOG("检测结果: 异常类型B (raw已损坏) → 进入 电压估算");
        return STATE_FALLBACK_VOLTAGE;
    }
}

/* ── STATE_NORMAL：同步 BMS → 系统显示 + 巡检 ────────────────────────── */

static State state_normal(const SensorData *s)
{
    int type = detect_anomaly(s);

    if (type != 0) {
        LOG("*** 巡检发现异常！***");
        if (type == 2)
            return STATE_FALLBACK_VOLTAGE;
        else
            return STATE_FALLBACK_RAW;
    }

    /* 无异常 */
    if (g_boot_abnormal) {
        /* boot 时异常 → 即使恢复也持续同步 */
        char sync_val[16];
        sprintf(sync_val, "%d", s->bms_cap_pct);
        write_capacity(sync_val);
        LOG("巡检: capacity_bms=%d%%  capacity_bat=%d%%  capacity_raw=%d%%  voltage_now=%dmV  current_now=%dmA  status=%s  |  检测=正常  |  同步 %s%% → battery/capacity",
            s->bms_cap_pct, s->bat_cap_pct, s->raw_pct, s->volt_mv,
            s->current_ua / 1000, s->charging, sync_val);
    } else {
        LOG("巡检: capacity_bms=%d%%  capacity_bat=%d%%  capacity_raw=%d%%  voltage_now=%dmV  current_now=%dmA  status=%s  |  检测=正常",
            s->bms_cap_pct, s->bat_cap_pct, s->raw_pct, s->volt_mv,
            s->current_ua / 1000, s->charging);
    }
    sleep(NORMAL_POLL_SEC);
    return STATE_NORMAL;
}

/* ── STATE_RECOVERING：执行恢复 + 等待 + 重新评估 ─────────────────────── */

static State state_recovering(SensorData *s)
{
    static int   phase  = 0;       /* 0=未开始, 1=等待中, 2=已评估  */
    static time_t t_start = 0;

    /* 首次进入：执行恢复 */
    if (phase == 0) {
        LOG("");
        LOG("╔════════════════════════════════════════╗");
        LOG("║  进入 RECOVERING 状态                  ║");
        LOG("╚════════════════════════════════════════╝");
        do_recovery();
        t_start = time(NULL);
        phase = 1;
    }

    /* 等待阶段 */
    if (phase == 1) {
        time_t elapsed = time(NULL) - t_start;
        if (elapsed < RECOVERY_WAIT_SEC) {
            if ((int)elapsed % 15 == 0 && elapsed > 0)
                LOG("  等待中... 剩余 %lld 秒",
                    (long long)(RECOVERY_WAIT_SEC - elapsed));
            sleep(5);
            return STATE_RECOVERING;
        }
        phase = 2;  /* 等待结束，进入评估 */
    }

    /* 评估阶段：重新读传感器，判断恢复结果 */
    LOG("恢复等待结束，重新评估...");
    if (read_sensors(s) != 0) {
        LOG("警告: 传感器读取失败，保持 RECOVERING");
        sleep(5);
        return STATE_RECOVERING;
    }

    int type = detect_anomaly(s);
    LOG("恢复后读数: capacity_bms=%d%%  capacity_bat=%d%%  capacity_raw=%d%%  voltage_now=%dmV  status=%s",
        s->bms_cap_pct, s->bat_cap_pct, s->raw_pct, s->volt_mv, s->charging);

    /* 重置 phase 以便下次进入时从 0 开始 */
    phase = 0;

    if (type == 0) {
        LOG(">>> 恢复成功！进入 NORMAL 状态 <<<");
        LOG("");
        return STATE_NORMAL;
    }

    LOG(">>> 恢复失败，进入兜底模式 <<<");
    LOG("");
    if (type == 2)
        return STATE_FALLBACK_VOLTAGE;
    else
        return STATE_FALLBACK_RAW;
}

/* ── STATE_FALLBACK_RAW：raw/100 覆写 capacity ─────────────────────────── */

static State state_fallback_raw(const SensorData *s)
{
    char cur_val[16];

    sprintf(cur_val, "%d", s->raw_pct);

    int ok = (write_capacity(cur_val) == 0);
    LOG("执行: 写入 %s%% → battery/capacity  %s",
        cur_val, ok ? "✓" : "✗");

    sleep(FALLBACK_POLL_SEC);
    return STATE_FALLBACK_RAW;
}

/* ── STATE_FALLBACK_VOLTAGE：电压估算覆写 capacity ──────────────────────── */

/*
 * 电压估算策略 / 电压估算策略：
 *   库仑计数 + OCV 校准（TI/ADI/MPS 专业电量计同款方案）：
 *     1. 库仑计数：ΔSOC = ∫I·dt / 满容量，不受电压噪声影响，准确跟踪变化量
 *     2. OCV 校准：空闲时用开路电压修正库仑计累积误差
 *     3. 电流越大，库仑计数权重越高；电流越小，OCV 校准越活跃
 */
/* ── 库仑计: 每 1% SOC 对应 µA·s = charge_full_design(µAh) × 36 */

/*
 * 库仑计数子步
 *   原理: SOC 变化量 = 累计电流×时间 / 电池满容量
 *     g_uas_per_pct = charge_full_design(µAh) × 36 (每 1% SOC 对应的电荷量)
 *     例: 2000mA 充电, 每 3s 累计 6,000,000 µA·s → 约 29 轮(87s) 才变化 1%
 *         500mA 充电, 每 3s 累计 1,500,000 µA·s → 约 116 轮(348s) 才变化 1%
 */
static void coulomb_step(int current_ua, int volt_mv, int *coulomb_soc)
{
    static int64_t coulomb_acc = 0;

    if (*coulomb_soc < 0) {
        *coulomb_soc = voltage_to_capacity(volt_mv);
        coulomb_acc = 0;
    }
    coulomb_acc += (int64_t)current_ua * FALLBACK_POLL_SEC;
    int soc_delta = (int)(coulomb_acc / g_uas_per_pct);
    if (soc_delta != 0) {
        *coulomb_soc -= soc_delta;  /* 正电流=放电, SOC 递减; 负=充电, 递增 */
        coulomb_acc -= (int64_t)soc_delta * g_uas_per_pct;
    }
    if (*coulomb_soc < 0)   *coulomb_soc = 0;
    if (*coulomb_soc > 100) *coulomb_soc = 100;
}

/*
 * ir_compensate — 统一内阻管理
 *
 * 原理:
 *   锂电池内阻 R 是物理定值, 充电放电相同。
 *   充电时端电压 = OCV + IR, 放电时端电压 = OCV - IR。
 *   已知 R 和 |I|, 即可从端电压恢复出近似 OCV。
 *
 * R 的计算:
 *   |I|≥500mA 时, 用 |volt_smooth - discharge_ocv| / |I| 估算 R,
 *   阈值 20mV 过滤噪声, EMA(α=0.1) 平滑。
 *
 * 两个职责:
 *   1. 充放电切换时重置 EMA, 避免电压滞后
 *   2. 用 R 补偿端电压: 充电扣掉 IR 压降, 放电补回 IR 压降
 *
 * 静态变量:
 *   r_mohm       — EMA 内阻 (mΩ), 充放共用, R=0 时不做补偿(由校准兜底)
 *   was_charging — 上一帧充放电状态 (检测切换)
 */
static void ir_compensate(int volt_mv, int i_ma, int is_charging,
                          int *volt_smooth, int *volt_for_ocv,
                          int discharge_ocv)
{
    static int r_mohm       = 0;
    static int was_charging = -1;

    /* 充放电切换时重置 EMA, 避免滞后 */
    if (was_charging != -1 && was_charging != is_charging)
        *volt_smooth = volt_mv;
    was_charging = is_charging;

    /* 统一内阻 EMA: |I|≥500mA 且电压差 >20mV 时计算 */
    if (i_ma >= 500 && discharge_ocv > 0) {
        int dv = abs(*volt_smooth - discharge_ocv);
        if (dv > 20) {
            int r = dv * 1000 / i_ma;               /* mV→mΩ */
            r_mohm = (r_mohm == 0) ? r : (r_mohm * 9 + r) / 10;
        }
    }

    /* IR 补偿: 充电扣掉, 放电补回 */
    *volt_for_ocv = *volt_smooth;
    if (r_mohm > 0) {
        int ir_mv = r_mohm * i_ma / 1000;           /* mΩ × mA → mV */
        if (is_charging)
            *volt_for_ocv -= ir_mv;
        else
            *volt_for_ocv += ir_mv;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * EMA 平滑
 *
 * 所有后续步骤的基础。α=0.1, 抑制瞬时噪声。
 */
static void ema_smooth(int volt_mv, int *volt_smooth)
{
    if (*volt_smooth == 0)
        *volt_smooth = volt_mv;
    else
        *volt_smooth = (*volt_smooth * 9 + volt_mv) / 10;
}

/*
 * track_ocv — OCV 基准跟踪
 *
 * 放电 |I|<250mA 时持续 EMA 跟踪 volt_smooth, 作为真实开路电压的近似。
 * 充电和高电流放电时用作内阻计算参照。
 */
static void track_ocv(int i_ma, int is_charging, int volt_smooth, int *discharge_ocv)
{
    if (i_ma < 250 && !is_charging) {
        if (*discharge_ocv == 0)
            *discharge_ocv = volt_smooth;
        else
            *discharge_ocv = (*discharge_ocv * 9 + volt_smooth) / 10;
    }
}

/*
 * ocv_calibrate — 用 OCV 估算值校准库仑计
 *
 * 两路:
 *   放电低电流 (|I|<250mA) → 立刻校准, weight=100%
 *   其余 → 60s 定时校准, weight=250*100/|I| (电流衰减)
 *
 * 校准电压: volt_for_ocv (IR 补偿后), 被 IR 压低时兜底 discharge_ocv。
 */
static void ocv_calibrate(int i_ma, int is_charging,
                          int volt_for_ocv, int discharge_ocv,
                          int *coulomb_soc, time_t *last_ocv_cal,
                          const char **trust)
{
    if (i_ma < 250 && !is_charging) {
        /* 放电低电流：立刻校准 */
        int ocv_soc = voltage_to_capacity(volt_for_ocv);
        int delta   = ocv_soc - *coulomb_soc;
        int correction = delta;
        if (correction > 2)  correction = 2;
        if (correction < -2) correction = -2;
        LOG("  校准: cal=%dmV(volt_f_ocv) → ocv=%d%%  coulomb=%d%%  delta=%d  corr=%+d%%",
            volt_for_ocv, ocv_soc, *coulomb_soc, delta, correction);
        *coulomb_soc += correction;
        *last_ocv_cal = time(NULL);
        *trust = "校准";
    } else if (time(NULL) - *last_ocv_cal >= 60) {
        /* 60s 定时校准：端电压被 IR 压低时用 discharge_ocv 兜底 */
        int cal_volt = volt_for_ocv;
        const char *src = "volt_f_ocv";
        if (discharge_ocv > 0 && volt_for_ocv < discharge_ocv) {
            cal_volt = discharge_ocv;
            src = "discharge_ocv";
        }
        int ocv_soc = voltage_to_capacity(cal_volt);
        int delta   = ocv_soc - *coulomb_soc;
        int weight = 250 * 100 / (i_ma > 0 ? i_ma : 1);
        if (weight > 100) weight = 100;
        int correction = delta * weight / 100;
        if (correction > 2)  correction = 2;
        if (correction < -2) correction = -2;
        LOG("  校准: cal=%dmV(%s) → ocv=%d%%  coulomb=%d%%  delta=%d  weight=%d  corr=%+d%%",
            cal_volt, src, ocv_soc, *coulomb_soc, delta, weight, correction);
        *coulomb_soc += correction;
        *last_ocv_cal = time(NULL);
        *trust = "校准";
    }
}

/*
 * 单向约束子步
 *   充电时 SOC 不应下降, 放电时不应上升。
 *   充放电切换时自动重置约束。
 */
static int constrain_step(int est_raw, int is_charging, int force_reset)
{
    static int cap_constrained = -1;
    static int prev_chg        = -1;  /* 用于检测充放电切换 */

    /* 强制重置（首次低负载 OCV 初始化时调用） */
    if (force_reset) {
        cap_constrained = est_raw;
        prev_chg = is_charging;
        return est_raw;
    }

    /* 充放电切换时重置约束 */
    if (prev_chg != -1 && prev_chg != is_charging)
        cap_constrained = -1;
    prev_chg = is_charging;

    int est;
    if (cap_constrained < 0) {
        est = est_raw;
    } else if (is_charging) {
        est = (est_raw > cap_constrained) ? est_raw : cap_constrained;
        if (est_raw < cap_constrained)
            LOG("  约束: 充电中库仑=%d%% < 上限=%d%%, 保持=%d%%",
                est_raw, cap_constrained, est);
    } else {
        est = (est_raw < cap_constrained) ? est_raw : cap_constrained;
        if (est_raw > cap_constrained)
            LOG("  约束: 放电中库仑=%d%% > 上限=%d%%, 保持=%d%%",
                est_raw, cap_constrained, est);
    }
    cap_constrained = est;
    if (est > 100) est = 100;
    if (est < 0)   est = 0;
    return est;
}

/*
 * state_fallback_voltage — 电压估算编排器 (每 3s 一轮)
 *
 * 六步流水线:
 *   1. coulomb_step     → 电荷积分, 更新 coulomb_soc
 *   2. try_init         → 首次低负载初始化 (一次性, 用原始电压)
 *   3. try_init         → 首次低负载初始化 (一次性)
 *   4. ir_compensate    → 统一内阻 IR 补偿
 *   5. ocv_calibrate    → OCV 校准 (可能修改 coulomb_soc)
 *   6. 信任标签 + 约束 + 写入
 *
 * 静态变量:
 *   coulomb_soc  — 库仑计跟踪的 SOC%, 整个模块的核心状态
 *   volt_smooth  — EMA 平滑端电压 (mV), 跨帧保持
 */
static State state_fallback_voltage(const SensorData *s)
{
    static int coulomb_soc       = -1;     /* 库仑计 SOC         */
    static int volt_smooth       = 0;      /* EMA 平滑电压       */
    static int initialized       = 0;      /* 是否已完成首次初始化 */
    static int discharge_ocv     = 0;      /* OCV 基准 (mV)       */
    static time_t last_ocv_cal   = 0;      /* 上次校准时间         */
    char  cur_val[16];
    int   is_charging, i_ma, est;
    const char *trust = NULL;

    /* 充放电判断 */
    is_charging = (strstr(s->charging, "Charging") || strstr(s->charging, "charging"));
    i_ma = abs(s->current_ua) / 1000;

    /* ── 1. 首次初始化 (通过前不执行任何操作) ────────────────── */
    if (!initialized) {
        if (i_ma < 500 && !is_charging) {
            coulomb_soc = voltage_to_capacity(s->volt_mv);
            discharge_ocv = s->volt_mv;
            last_ocv_cal = time(NULL);
            trust = "初始化";
            initialized = 1;
        }
        return STATE_FALLBACK_VOLTAGE;
    }

    /* ── 2. 库仑计数 ────────────────────────────────────────── */
    coulomb_step(s->current_ua, s->volt_mv, &coulomb_soc);

    /* ── 3. EMA + OCV + IR + 校准 ────────────────────────────── */
    ema_smooth(s->volt_mv, &volt_smooth);
    int volt_for_ocv;
    track_ocv(i_ma, is_charging, volt_smooth, &discharge_ocv);
    ir_compensate(s->volt_mv, i_ma, is_charging, &volt_smooth, &volt_for_ocv, discharge_ocv);
    ocv_calibrate(i_ma, is_charging, volt_for_ocv, discharge_ocv,
                  &coulomb_soc, &last_ocv_cal, &trust);

    /* ── 4. 信任标签 ────────────────────────────────────────── */
    if (trust == NULL) {
        if (i_ma < CUR_HIGH_TRUST)      trust = "高";
        else if (i_ma < CUR_MED_TRUST)  trust = "中";
        else                            trust = "低";
    }

    /* ── 7. 单向约束 ────────────────────────────────────────── */
    if (trust && strcmp(trust, "初始化") == 0)
        est = constrain_step(coulomb_soc, is_charging, 1);
    else
        est = constrain_step(coulomb_soc, is_charging, 0);

    /* ── 8. 写入 ────────────────────────────────────────────── */
    sprintf(cur_val, "%d", est);
    int ok = (write_capacity(cur_val) == 0);

    LOG("执行: 写入 %s%% → battery/capacity  %s  [now=%dmV  OCV=%dmV  库仑=%d%%  |I|=%dmA  %s]",
        cur_val, ok ? "✓" : "✗", s->volt_mv, volt_smooth,
        coulomb_soc, i_ma, trust);

    sleep(FALLBACK_POLL_SEC);
    return STATE_FALLBACK_VOLTAGE;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 主循环：状态机调度
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(void)
{
    SensorData sensor;
    State state = STATE_INIT;

    /* 打开 battery/capacity 节点，保持句柄（Android 读取的节点，可写） */
    g_fp_cap = fopen(SYS_BAT_CAPACITY, "w");
    if (!g_fp_cap) {
        LOG("致命错误: 无法打开 %s", SYS_BAT_CAPACITY);
        return 1;
    }
    LOG("已打开 capacity 写入句柄 (%s)", SYS_BAT_CAPACITY);

    /* 动态读取电池满容量 (µAh) → 计算每 1% SOC 对应的 µA·s */
    {
        int full_uah = 0;
        if (sysfs_read_int(SYS_CHARGE_FULL, &full_uah) == 0 && full_uah > 0) {
            g_uas_per_pct = (int64_t)full_uah * 36LL;
            LOG("电池容量: %d µAh → Coulomb: %lld µA·s/%%", full_uah, (long long)g_uas_per_pct);
        }
    }

    /* 首次读取传感器 */
    if (read_sensors(&sensor) != 0) {
        LOG("致命错误: 传感器读取失败");
        return 1;
    }

    /* ── 状态机主循环 ─────────────────────────────────────────── */

    while (1) {

        switch (state) {

        case STATE_INIT:
            state = state_init(&sensor);
            if (state != STATE_NORMAL)
                g_boot_abnormal = 1;
            break;

        case STATE_NORMAL:
            /* state_normal 内部已 sleep + detect，返回后读新数据 */
            state = state_normal(&sensor);
            if (read_sensors(&sensor) != 0) {
                LOG("警告: 读传感器失败, 保持 NORMAL");
                sleep(NORMAL_POLL_SEC);
            }
            break;

        case STATE_RECOVERING:
            /* state_recovering 内部自行管理传感器读写 */
            state = state_recovering(&sensor);
            break;

        case STATE_FALLBACK_RAW:
        case STATE_FALLBACK_VOLTAGE:
            /* 先读最新数据，再每轮独立判断 */
            if (read_sensors(&sensor) != 0) {
                LOG("警告: 读传感器失败, 保持兜底");
                sleep(FALLBACK_POLL_SEC);
                break;
            }
            {
                int t = detect_anomaly(&sensor);
                State new_state;

                /* ── 第1行：原始读取数据 ───────────────────── */
                LOG("读取: capacity_bms=%d%%  capacity_bat=%d%%  capacity_raw=%d  voltage_now=%dmV  current_now=%dmA  status=%s",
                    sensor.bms_cap_pct, sensor.bat_cap_pct, sensor.raw_pct,
                    sensor.volt_mv, sensor.current_ua / 1000, sensor.charging);

                /* ── 第2行：检测结果 + 选择方案 ────────────── */
                if (t == 0) {
                    LOG("判断: 检测=正常  →  选择 NORMAL（同步BMS）");
                    new_state = STATE_NORMAL;
                } else if (t == 1) {
                    LOG("判断: 检测=类型A(raw可信)  →  选择 raw覆写");
                    new_state = STATE_FALLBACK_RAW;
                } else {
                    LOG("判断: 检测=类型B(raw损坏)  →  选择 电压估算");
                    new_state = STATE_FALLBACK_VOLTAGE;
                }

                /* 状态切换日志（在执行之前） */
                if (new_state != state) {
                    LOG("");
                    LOG("═══ 状态切换: %s → %s ═══",
                        state_name(state), state_name(new_state));
                }
                state = new_state;

                /* ── 第3行：执行（由各状态函数输出） ──────── */
                switch (state) {
                case STATE_NORMAL:
                    /* raw 刚恢复，同步 BMS 值到系统显示 */
                    {
                        char sync_val[16];
                        sprintf(sync_val, "%d", sensor.bms_cap_pct);
                        write_capacity(sync_val);
                        LOG("执行: 同步BMS %s%% → battery/capacity",
                            sync_val);
                    }
                    break;
                case STATE_FALLBACK_RAW:
                    state = state_fallback_raw(&sensor);
                    break;
                case STATE_FALLBACK_VOLTAGE:
                    state = state_fallback_voltage(&sensor);
                    break;
                default:
                    break;
                }
            }
            break;
        }
    }

    if (g_fp_cap) fclose(g_fp_cap);
    return 0;
}
