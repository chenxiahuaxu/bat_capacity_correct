/*
 * bat_capacity_correct.c - correct battery capacity
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define CAPACITY_RAW "/sys/class/power_supply/bms/capacity_raw"
#define CAPACITY "/sys/class/power_supply/bms/capacity"

/*
 * 读取 sysfs 节点内容
 * 返回 malloc 的字符串，调用者负责 free
 * 返回 NULL 表示读取失败
 */
static char *read_sysfs(const char *path)
{
    FILE *fp;
    char buf[32] = {0};
    char *result = NULL;

    fp = fopen(path, "r");
    if (!fp)
        return NULL;

    if (fgets(buf, sizeof(buf), fp))
    {
        size_t len = strlen(buf);
        if (len > 0 && buf[len - 1] == '\n')
            buf[len - 1] = '\0';
        result = strdup(buf);
    }
    fclose(fp);
    return result;
}

int main(void)
{
    FILE *fp_capacity = NULL;
    char prev_level[16] = {0};
    char *raw_str = NULL;
    int raw_val, level;
    char cur_level[16];

    fp_capacity = fopen(CAPACITY, "w");
    if (!fp_capacity)
    {
        fprintf(stderr, "Cannot open %s\n", CAPACITY);
        return 1;
    }

    while (1)
    {
        raw_str = read_sysfs(CAPACITY_RAW);
        if (!raw_str)
        {
            fprintf(stderr, "File not found: %s\n", CAPACITY_RAW);
            sleep(3);
            continue;
        }

        raw_val = atoi(raw_str);
        level = raw_val / 100;

        sprintf(cur_level, "%d", level);

        if (strcmp(prev_level, cur_level) != 0)
        {
            fprintf(fp_capacity, "%s", cur_level);
            fflush(fp_capacity);
            printf("Current Capacity: %s\n", cur_level);
            strcpy(prev_level, cur_level);
        }

        free(raw_str);
        sleep(3);
    }
    fclose(fp_capacity);
    return 0;
}
