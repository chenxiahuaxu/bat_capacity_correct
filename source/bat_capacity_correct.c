/*
 * bat_capacity_correct.c — 多阶段电池电量校正
 *
 * 状态机 / 状态机：
 *   INIT              → 初始检测，判定进入 NORMAL 还是 RECOVERING
 *   NORMAL            → 无异常，只后台定时巡检，不写任何 sysfs
 *   RECOVERING        → 尝试恢复控制器，等待后重新评估
 *   FALLBACK_RAW      → 恢复失败、raw 可信：raw/100 覆写 capacity
 *   FALLBACK_VOLTAGE  → 恢复失败、raw 也坏：电压查表估算覆写
 *
 * 转换路径：
 *   INIT → NORMAL | RECOVERING
 *   NORMAL → RECOVERING (巡检发现异常)
 *   RECOVERING → NORMAL | FALLBACK_RAW | FALLBACK_VOLTAGE
 *   FALLBACK_RAW → NORMAL (异常自行消失)
 *   FALLBACK_VOLTAGE → FALLBACK_RAW (raw 恢复可信) | NORMAL
 *
 * 核心原则：无异常时零干预，只在必要时介入。
 * 电压是硬件 ADC 直读的物理量，始终作为终极真值源。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * sysfs 路径常量
 * ═══════════════════════════════════════════════════════════════════════════ */

#define SYS_CAPACITY_RAW  "/sys/class/power_supply/bms/capacity_raw"
#define SYS_CAPACITY      "/sys/class/power_supply/bms/capacity"
#define SYS_VOLTAGE_NOW   "/sys/class/power_supply/bms/voltage_now"
#define SYS_STATUS        "/sys/class/power_supply/battery/status"
#define SYS_BMS_RESET     "/sys/class/power_supply/bms/reset"

/* ═══════════════════════════════════════════════════════════════════════════
 * 阈值
 * ═══════════════════════════════════════════════════════════════════════════ */

#define CAP_MAX_ANOMALY    2     /* capacity ≤ 2% → 可疑                  */
#define VOLT_MIN_HEALTHY   3700  /* 电压 ≥ 3700mV → 电池实际不亏电          */
#define RAW_MIN_OK         10    /* raw/100 ≥ 10% → raw 可信               */
#define RAW_BAD_THRESHOLD  5     /* raw/100 ≤ 5% 且电压高 → raw 已损坏     */

#define RECOVERY_WAIT_SEC  60    /* 恢复后等待 PMIC 重算                    */
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
    int  cap_pct;           /* 系统 capacity   (0~100)     */
    int  raw_pct;           /* capacity_raw    (0~100)     */
    int  volt_mv;           /* voltage_now     (mV)        */
    char charging[16];      /* status 字符串               */
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

/* 写字符串到 sysfs（用于覆写 capacity 节点） */
static int sysfs_write_str(const char *path, const char *val)
{
    FILE *fp;
    fp = fopen(path, "w");
    if (!fp) return -1;
    fprintf(fp, "%s", val);
    fclose(fp);
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 模块2：传感器读取
 * ═══════════════════════════════════════════════════════════════════════════ */

static int read_sensors(SensorData *s)
{
    int raw_raw; /* 原始 raw 值 (0~10000) */

    if (sysfs_read_int(SYS_CAPACITY_RAW, &raw_raw) != 0) return -1;
    s->raw_pct = raw_raw / 100;

    if (sysfs_read_int(SYS_VOLTAGE_NOW, &s->volt_mv) != 0) return -1;
    s->volt_mv /= 1000; /* µV → mV */

    if (sysfs_read_int(SYS_CAPACITY, &s->cap_pct) != 0) return -1;

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
    if (s->cap_pct <= CAP_MAX_ANOMALY && s->volt_mv >= VOLT_MIN_HEALTHY) {
        if (s->raw_pct >= RAW_MIN_OK)
            return 1;   /* 类型A：raw 可信 */
        else
            return 2;   /* 类型B：raw 也坏了 */
    }

    /* 检查2：raw 接近 0 但电压很高 → raw 损坏 */
    if (s->raw_pct <= RAW_BAD_THRESHOLD && s->volt_mv >= VOLT_MIN_HEALTHY)
        return 2;

    /* 检查3：系统与 raw 偏差 >30% */
    if (abs(s->cap_pct - s->raw_pct) > 30)
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
 * 模块5：控制器恢复
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
    LOG("║  三步策略: 检测 → 恢复 → 兜底         ║");
    LOG("╚════════════════════════════════════════╝");
    LOG("初始读数: 系统=%d%%  raw=%d%%  电压=%dmV  充电=%s",
        s->cap_pct, s->raw_pct, s->volt_mv, s->charging);

    if (type == 0) {
        LOG("检测结果: 无异常 → 进入 NORMAL 状态（后台巡检）");
        return STATE_NORMAL;
    } else if (type == 1) {
        LOG("检测结果: 异常类型A (raw可信) → 进入 RECOVERING 状态");
    } else {
        LOG("检测结果: 异常类型B (raw已损坏) → 进入 RECOVERING 状态");
    }
    return STATE_RECOVERING;
}

/* ── STATE_NORMAL：后台巡检，不写 sysfs ────────────────────────────────── */

static State state_normal(const SensorData *s)
{
    int type = detect_anomaly(s);

    if (type != 0) {
        LOG("*** 巡检发现异常！系统=%d%%  raw=%d%%  电压=%dmV ***",
            s->cap_pct, s->raw_pct, s->volt_mv);
        LOG("*** 异常类型: %s → 进入 RECOVERING ***",
            type == 2 ? "类型B(raw也坏)" : "类型A(raw可信)");
        return STATE_RECOVERING;
    }

    /* 无异常：打印巡检结果，等待下一次 */
    LOG("巡检: 系统=%d%%  raw=%d%%  电压=%dmV  充电=%s  [正常]",
        s->cap_pct, s->raw_pct, s->volt_mv, s->charging);
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
    LOG("恢复后读数: 系统=%d%%  raw=%d%%  电压=%dmV  充电=%s",
        s->cap_pct, s->raw_pct, s->volt_mv, s->charging);

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
    static char prev_val[16] = {0};
    char cur_val[16];

    sprintf(cur_val, "%d", s->raw_pct);

    /* 写入 (仅在值变化时，每次 fopen/fclose 确保 sysfs 生效) */
    if (strcmp(prev_val, cur_val) != 0) {
        if (sysfs_write_str(SYS_CAPACITY, cur_val) == 0) {
            int verify;
            /* 立即回读验证写入是否被 BMS 驱动覆盖 */
            if (sysfs_read_int(SYS_CAPACITY, &verify) == 0) {
                LOG("覆写: 写入=%s%%  回读=%d%%  raw=%d  电压=%dmV  "
                    "充电=%s  [raw兜底]%s",
                    cur_val, verify, s->raw_pct, s->volt_mv, s->charging,
                    verify == atoi(cur_val) ? "" : " ← BMS已覆盖!");
            } else {
                LOG("覆写: 容量=%s%%  raw=%d  电压=%dmV  "
                    "充电=%s  [raw兜底]",
                    cur_val, s->raw_pct, s->volt_mv, s->charging);
            }
            strcpy(prev_val, cur_val);
        } else {
            LOG("覆写失败: 无法写入 %s", SYS_CAPACITY);
        }
    }

    /* 检查异常是否已自行消失 */
    if (detect_anomaly(s) == 0) {
        LOG("*** 异常已消失 → 返回 NORMAL ***");
        return STATE_NORMAL;
    }

    sleep(FALLBACK_POLL_SEC);
    return STATE_FALLBACK_RAW;
}

/* ── STATE_FALLBACK_VOLTAGE：电压估算覆写 capacity ──────────────────────── */

/*
 * 电压估算策略：
 *   1. EMA 平滑 — 指数移动平均消除短时电压抖动
 *   2. 单向约束 — 充电态容量只升不降，放电态容量只降不升
 *   3. 回读验证 — 写后立刻回读，检测 BMS 驱动是否抢写
 */
static State state_fallback_voltage(const SensorData *s)
{
    static int   volt_ema = 0;           /* 电压 EMA 平滑值               */
    static int   cap_constrained = -1;   /* 约束后的输出容量，-1=未初始化 */
    static int   prev_charging = -1;     /* 上次充电状态，-1=未初始化     */
    static char  prev_val[16] = {0};
    char cur_val[16];
    int  is_charging;
    int  est_raw, est;

    /* 判断充电状态 */
    is_charging = (s->charging[0] == 'C' || s->charging[0] == 'c');

    /* 状态切换时重置约束缓存 */
    if (prev_charging != -1 && prev_charging != is_charging) {
        cap_constrained = -1;
        LOG("  充放电状态切换: %s → %s, 重置约束",
            prev_charging ? "充电" : "放电",
            is_charging  ? "充电" : "放电");
    }
    prev_charging = is_charging;

    /* ── 1. EMA 平滑电压 ────────────────────────────────────── */
    if (volt_ema == 0)
        volt_ema = s->volt_mv * 10;      /* 定点数: ×10 保留精度 */
    else
        volt_ema = (volt_ema * 7 + s->volt_mv * 10 * 3) / 10;  /* α=0.3 */

    int volt_smooth = volt_ema / 10;     /* 恢复为 mV */

    /* ── 2. 电压 → 电量 ──────────────────────────────────────── */
    est_raw = voltage_to_capacity(volt_smooth);

    /* ── 3. 单向约束 ────────────────────────────────────────── */
    if (cap_constrained < 0) {
        est = est_raw;                    /* 首次，直接采用 */
    } else if (is_charging) {
        /* 充电态：容量只升不降 */
        est = (est_raw > cap_constrained) ? est_raw : cap_constrained;
    } else {
        /* 放电态：容量只降不升 */
        est = (est_raw < cap_constrained) ? est_raw : cap_constrained;
    }
    cap_constrained = est;

    /* 边界钳位 */
    if (est > 100) est = 100;
    if (est < 0)   est = 0;

    sprintf(cur_val, "%d", est);

    /* ── 4. 写入 ────────────────────────────────────────────── */
    if (strcmp(prev_val, cur_val) != 0) {
        if (sysfs_write_str(SYS_CAPACITY, cur_val) == 0) {
            int verify;
            if (sysfs_read_int(SYS_CAPACITY, &verify) == 0) {
                LOG("覆写: 写入=%s%%  回读=%d%%  原始电压=%dmV  "
                    "平滑=%dmV  raw=%d  充电=%s  [电压估算]%s",
                    cur_val, verify, s->volt_mv, volt_smooth,
                    s->raw_pct, s->charging,
                    verify == est ? "" : " ←BMS覆盖");
            } else {
                LOG("覆写: 容量=%s%%  原始电压=%dmV  平滑=%dmV  "
                    "充电=%s  [电压估算]",
                    cur_val, s->volt_mv, volt_smooth, s->charging);
            }
            strcpy(prev_val, cur_val);
        } else {
            LOG("覆写失败: 无法写入 %s", SYS_CAPACITY);
        }
    }

    /* 检查是否可升级 */
    int type = detect_anomaly(s);
    if (type == 0) {
        LOG("*** 异常已消失 → 返回 NORMAL ***");
        return STATE_NORMAL;
    }
    if (type == 1) {
        LOG("*** raw 已恢复可信 → 切换到 raw兜底 ***");
        return STATE_FALLBACK_RAW;
    }

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
    State prev_state = STATE_INIT;

    /* 首次读取传感器 */
    if (read_sensors(&sensor) != 0) {
        LOG("致命错误: 传感器读取失败");
        return 1;
    }

    /* ── 状态机主循环 ─────────────────────────────────────────── */

    while (1) {
        /* 状态切换日志 */
        if (state != prev_state) {
            LOG("");
            LOG("═══ 状态切换: %s → %s ═══",
                state_name(prev_state), state_name(state));
            prev_state = state;
        }

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
            /* 先读最新数据，再覆写 */
            if (read_sensors(&sensor) != 0) {
                LOG("警告: 读传感器失败, 保持 raw兜底");
                sleep(FALLBACK_POLL_SEC);
                break;
            }
            state = state_fallback_raw(&sensor);
            break;

        case STATE_FALLBACK_VOLTAGE:
            if (read_sensors(&sensor) != 0) {
                LOG("警告: 读传感器失败, 保持 电压估算");
                sleep(FALLBACK_POLL_SEC);
                break;
            }
            state = state_fallback_voltage(&sensor);
            break;
        }
    }

    return 0;
}
