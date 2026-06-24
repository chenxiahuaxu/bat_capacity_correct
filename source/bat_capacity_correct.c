/*
 * bat_capacity_correct.c — 多阶段电池电量校正
 *
 * 状态机 / 状态机：
 *   INIT              → 初始检测，判定进入 NORMAL 或兜底
 *   NORMAL            → 无异常，只后台定时巡检，不写任何 sysfs
 *   RECOVERING        → 尝试恢复控制器（暂不使用）
 *   FALLBACK_RAW      → raw 可信：raw/100 覆写 battery/capacity
 *   FALLBACK_VOLTAGE  → raw 无效：电压查表估算覆写
 *
 * 电压估算策略 / 电压估算策略：
 *   电流置信度指数平滑：
 *     |I| < 200mA  → 高置信，全量更新 EMA
 *     200 ≤ |I| < 500mA → 中置信，半量更新
 *     |I| ≥ 500mA → 低置信，不更新，保持上一条估计值
 *
 * 核心原则：轻载时电压可信度高，重载/快充时电流越大可信度越低。
 * 写入目标：/sys/class/power_supply/battery/capacity（Android 读取的节点）
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

    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 模块4：电压 → 电量 查表 (5020mAh 锂电池放电曲线)
 * ═══════════════════════════════════════════════════════════════════════════ */

/*
 * 锂电池放电曲线 (3.85V 标称, 5020mAh)
 * 每 1% 一个采样点，相邻点间线性插值，估算精度 ±1%。
 *
 * 曲线形态:
 *   4.40V ~ 4.18V  高台陡降区  100%→85%  ~15mV/%
 *   4.18V ~ 3.70V  平台缓降区   85%→20%   ~7mV/%
 *   3.70V ~ 3.50V  尾部急降区   20%→ 0%  ~10mV/%
 */
static const int voltage_map[][2] = {
    /* SOC%  V_mV */
    {4400, 100},
    {4375,  99}, {4350,  98}, {4330,  97}, {4315,  96}, {4300,  95},
    {4286,  94}, {4272,  93}, {4260,  92}, {4248,  91}, {4236,  90},
    {4225,  89}, {4215,  88}, {4206,  87}, {4197,  86}, {4188,  85},
    {4180,  84}, {4172,  83}, {4164,  82}, {4156,  81}, {4148,  80},
    {4140,  79}, {4133,  78}, {4126,  77}, {4119,  76}, {4112,  75},
    {4105,  74}, {4099,  73}, {4093,  72}, {4087,  71}, {4081,  70},
    {4075,  69}, {4069,  68}, {4063,  67}, {4057,  66}, {4051,  65},
    {4045,  64}, {4039,  63}, {4033,  62}, {4027,  61}, {4021,  60},
    {4015,  59}, {4009,  58}, {4003,  57}, {3997,  56}, {3991,  55},
    {3985,  54}, {3979,  53}, {3973,  52}, {3967,  51}, {3961,  50},
    {3955,  49}, {3949,  48}, {3943,  47}, {3937,  46}, {3931,  45},
    {3925,  44}, {3919,  43}, {3913,  42}, {3907,  41}, {3901,  40},
    {3894,  39}, {3887,  38}, {3880,  37}, {3873,  36}, {3866,  35},
    {3859,  34}, {3852,  33}, {3845,  32}, {3838,  31}, {3831,  30},
    {3824,  29}, {3816,  28}, {3808,  27}, {3800,  26}, {3792,  25},
    {3784,  24}, {3776,  23}, {3768,  22}, {3760,  21}, {3752,  20},
    {3744,  19}, {3736,  18}, {3728,  17}, {3720,  16}, {3712,  15},
    {3704,  14}, {3696,  13}, {3688,  12}, {3680,  11}, {3672,  10},
    {3664,   9}, {3656,   8}, {3648,   7}, {3640,   6}, {3632,   5},
    {3624,   4}, {3616,   3}, {3608,   2}, {3600,   1}, {3500,   0},
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

static State state_init(const SensorData *s)
{
    int type = detect_anomaly(s);

    LOG("╔════════════════════════════════════════╗");
    LOG("║  bat_capacity_correct 服务启动         ║");
    LOG("╚════════════════════════════════════════╝");
    LOG("初始读数: capacity_bms=%d%%  capacity_bat=%d%%  capacity_raw=%d%%  voltage_now=%dmV  status=%s",
        s->bms_cap_pct, s->bat_cap_pct, s->raw_pct, s->volt_mv, s->charging);

    if (type == 0) {
        LOG("检测结果: 无异常 → 进入 NORMAL 状态（后台巡检）");
        return STATE_NORMAL;
    } else if (type == 1) {
        LOG("检测结果: 异常类型A (raw可信) → 进入 raw兜底");
        return STATE_FALLBACK_RAW;
    } else {
        LOG("检测结果: 异常类型B (raw已损坏) → 进入 电压估算");
        return STATE_FALLBACK_VOLTAGE;
    }
}

/* ── STATE_NORMAL：后台巡检，不写 sysfs ────────────────────────────────── */

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

    /* 无异常：打印巡检结果，等待下一次 */
    LOG("巡检: capacity_bms=%d%%  capacity_bat=%d%%  capacity_raw=%d%%  voltage_now=%dmV  current_now=%dmA  status=%s  |  检测=正常",
        s->bms_cap_pct, s->bat_cap_pct, s->raw_pct, s->volt_mv,
        s->current_ua / 1000, s->charging);
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
 *     2. OCV 校准：电流 < 100mA 时记录真实开路电压 → 修正库仑计数的累积误差
 *     3. 电流越大，库仑计数权重越高；电流越小，OCV 校准越活跃
 */
static State state_fallback_voltage(const SensorData *s)
{
    /* ── 库仑计参数 ──────────────────────────────────────── */
#define CHARGE_FULL_UAH 4820000LL           /* µAh, 电池满容量         */
#define UAS_PER_PCT     (CHARGE_FULL_UAH * 36LL) /* µA·s per 1% SOC  */

    static int64_t coulomb_acc = 0;         /* 累计 µA·s                  */
    static int     coulomb_soc = -1;        /* 库仑计 SOC (×1%)           */
    static int     volt_smooth  = 0;        /* EMA 平滑后的 OCV (mV)      */
    static int     cap_constrained = -1;    /* 单向约束后的输出容量        */
    static int     prev_charging = -1;      /* 上次充电状态                */
    char cur_val[16];
    int  is_charging, i_ma, est_raw, est;
    const char *trust;

    /* 判断充电状态 */
    is_charging = (s->charging[0] == 'C' || s->charging[0] == 'c');
    i_ma = abs(s->current_ua) / 1000;

    /* 充放电切换时重置约束 */
    if (prev_charging != -1 && prev_charging != is_charging) {
        cap_constrained = -1;
    }
    prev_charging = is_charging;

    /* ── 1. 库仑计数：累积电流 → SOC 变化 ─────────────────── */
    if (coulomb_soc < 0) {
        /* 初始化：从电压查表获取起点 */
        coulomb_soc = voltage_to_capacity(s->volt_mv);
        coulomb_acc = 0;
    }

    coulomb_acc += (int64_t)s->current_ua * FALLBACK_POLL_SEC;
    int soc_delta = (int)(coulomb_acc / UAS_PER_PCT);
    if (soc_delta != 0) {
        coulomb_soc -= soc_delta;  /* 正电流=放电,SOC递减 */
        coulomb_acc -= (int64_t)soc_delta * UAS_PER_PCT;
    }
    if (coulomb_soc < 0)   coulomb_soc = 0;
    if (coulomb_soc > 100) coulomb_soc = 100;

    /* ── 2. OCV 平滑 + 校准 ────────────────────────────────── */
    if (volt_smooth == 0)
        volt_smooth = s->volt_mv;
    else
        volt_smooth = (volt_smooth * 9 + s->volt_mv) / 10;  /* α=0.1 */

    /* 电流 < 250mA 时用 OCV 修正库仑计累积误差 */
    if (i_ma < 250) {
        int ocv_soc = voltage_to_capacity(volt_smooth);
        coulomb_soc = (coulomb_soc * 2 + ocv_soc) / 3;  /* OCV 33% 权重 */
        coulomb_acc = 0;
        trust = "校准";
    } else if (i_ma < CUR_HIGH_TRUST) {
        trust = "高";
    } else if (i_ma < CUR_MED_TRUST) {
        trust = "中";
    } else {
        trust = "低";
    }

    /* ── 3. 单向约束 ────────────────────────────────────────── */
    est_raw = coulomb_soc;

    if (cap_constrained < 0) {
        est = est_raw;
    } else if (is_charging) {
        est = (est_raw > cap_constrained) ? est_raw : cap_constrained;
    } else {
        est = (est_raw < cap_constrained) ? est_raw : cap_constrained;
    }
    cap_constrained = est;
    if (est > 100) est = 100;
    if (est < 0)   est = 0;

    sprintf(cur_val, "%d", est);

    /* ── 4. 写入 ────────────────────────────────────────────── */
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
                    LOG("判断: 检测=正常  →  选择 NORMAL（不写入）");
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
                    LOG("执行: 不写入");
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
