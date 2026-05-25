#include "max30102.h"

static const char *TAG = "MAX30102";

static i2c_master_dev_handle_t max30102_dev = NULL;

// 数据缓存
static uint32_t ir_buffer[MAX30102_BUFFER_SIZE] = {0};
static uint32_t red_buffer[MAX30102_BUFFER_SIZE] = {0};
static int buffer_idx = 0;

// 计算结果缓存
static volatile uint32_t last_heart_rate = 0;
static volatile uint32_t last_spo2 = 0;

// 测量状态
static volatile bool measurement_active = false;
static volatile uint32_t measurement_elapsed_seconds = 0;
static volatile bool has_valid_data = false;
#define MEASUREMENT_DURATION_S 20

// 按键相关
#define KEY_GPIO_PIN GPIO_NUM_40
static bool spo2_measurement_requested = false;

// 算法相关常量
#define BUFFER_SIZE 500
#define MA4_SIZE 4
#define HAMMING_SIZE 11

static int32_t an_x[BUFFER_SIZE];
static int32_t an_y[BUFFER_SIZE];
static int32_t an_dx[BUFFER_SIZE];
static int32_t an_dx_peak_locs[15];
static uint32_t auw_hamm[HAMMING_SIZE] = {7, 28, 67, 124, 189, 232, 232, 189, 124, 67, 28};

// SpO2 lookup table (184 elements for ratio 0-183)
static const uint8_t uch_spo2_table[184] = {
    95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 98, 98, 98, 98, 99, 99, 99, 99, 100, 100,
    100, 100, 100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98,
    98, 97, 97, 97, 97, 97, 97, 97, 96, 96, 96, 96, 96, 96, 95, 95, 95, 95, 95, 94,
    94, 94, 94, 94, 93, 93, 93, 93, 93, 92, 92, 92, 92, 91, 91, 91, 91, 90, 90, 90,
    89, 89, 89, 89, 88, 88, 88, 87, 87, 87, 86, 86, 86, 85, 85, 85, 84, 84, 84, 83,
    83, 83, 82, 82, 81, 81, 81, 80, 80, 79, 79, 79, 78, 78, 77, 77, 76, 76, 76, 75,
    75, 74, 74, 73, 73, 72, 72, 71, 71, 70, 70, 69, 69, 68, 68, 67, 67, 66, 66, 65,
    65, 64, 64, 63, 62, 62, 61, 61, 60, 60, 59, 59, 58, 58, 57, 56, 56, 55, 55, 54,
    54, 53, 52, 52, 51, 51, 50, 49, 49, 48, 48, 47, 47, 46, 45, 45, 44, 44, 43, 42};

// 初始化按键GPIO
static esp_err_t Max30102_Key_Init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << KEY_GPIO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,  /* 轮询模式，不需要中断 */
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "按键 GPIO%d 配置失败: %s", KEY_GPIO_PIN, esp_err_to_name(ret));
        return ret;
    }

    /* 验证 GPIO 状态：未按下时应为高电平（内部上拉） */
    vTaskDelay(pdMS_TO_TICKS(10));
    int level = gpio_get_level(KEY_GPIO_PIN);
    ESP_LOGI(TAG, "按键 GPIO%d 初始化完成, 当前电平=%d (期望1=未按下)", KEY_GPIO_PIN, level);
    return ESP_OK;
}

// 检查按键是否按下（去抖处理）
static bool Max30102_Is_Key_Pressed(void)
{
    static TickType_t last_press_time = 0;

    /* 低电平 = 按下（按键接地） */
    if (gpio_get_level(KEY_GPIO_PIN) != 0)
        return false;

    /* 去抖：延时后再确认 */
    vTaskDelay(pdMS_TO_TICKS(20));
    if (gpio_get_level(KEY_GPIO_PIN) != 0)
        return false;

    /* 在 debounce 之后获取时间戳 */
    TickType_t now = xTaskGetTickCount();

    /* 防重复触发：5秒内只响应一次 */
    if ((now - last_press_time) <= pdMS_TO_TICKS(5000))
        return false;

    last_press_time = now;
    ESP_LOGI(TAG, "GPIO%d 按键检测成功! 电平=0", KEY_GPIO_PIN);
    return true;
}

// 初始化
esp_err_t Max30102_Init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "初始化 MAX30102");

    // 添加设备
    ret = myiic_add_device(MAX30102_ADDR, &max30102_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "添加 MAX30102 设备失败");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // 重置设备
    ret = Max30102_Write_Reg(REG_MODE_CONFIG, 0x40);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "重置设备失败");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // 配置 FIFO (16样本深度)
    ret = Max30102_Write_Reg(REG_FIFO_CONFIG, 0x1F);
    if (ret != ESP_OK)
        return ret;

    // 配置模式为 SpO2 
    ret = Max30102_Write_Reg(REG_MODE_CONFIG, 0x03);
    if (ret != ESP_OK)
        return ret;

    // 配置 SpO2 采样率和分辨率: 100Hz, 200us LED pulse, 18-bit resolution
    ret = Max30102_Write_Reg(REG_SPO2_CONFIG, 0x27);
    if (ret != ESP_OK)
        return ret;

    // 设置 LED 电流 (较高电流提高信号质量)
    ret = Max30102_Write_Reg(REG_LED1_PA, 0x1F); // 红色 LED: 50mA
    if (ret != ESP_OK)
        return ret;

    ret = Max30102_Write_Reg(REG_LED2_PA, 0x1F); // IR LED: 50mA
    if (ret != ESP_OK)
        return ret;

    // 初始化按键
    ret = Max30102_Key_Init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "按键初始化失败");
        return ret;
    }

    ESP_LOGI(TAG, "MAX30102 初始化完成");
    return ESP_OK;
}

// 写寄存器
esp_err_t Max30102_Write_Reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return myiic_write(max30102_dev, MAX30102_ADDR, buf, 2);
}

// 读寄存器
esp_err_t Max30102_Read_Reg(uint8_t reg, uint8_t *data)
{
    return myiic_write_read(max30102_dev, MAX30102_ADDR, &reg, 1, data, 1);
}

// 读取FIFO中可用的样本数（一次I2C事务读取3个连续寄存器）
static int Max30102_Get_Fifo_Count(void)
{
    uint8_t regs[3];
    uint8_t reg_addr = REG_FIFO_WR_PTR; // 0x04, 后续为 0x05(OVF) 和 0x06(RD_PTR)

    if (myiic_write_read(max30102_dev, MAX30102_ADDR, &reg_addr, 1, regs, 3) != ESP_OK)
        return 0;

    uint8_t wr_ptr = regs[0];
    uint8_t rd_ptr = regs[2];

    int count = wr_ptr - rd_ptr;
    if (count < 0)
        count += 32; // FIFO深度为32

    return count;
}

// 读 FIFO
esp_err_t Max30102_Read_Fifo(uint8_t *buffer, uint8_t count)
{
    uint8_t reg_addr = REG_FIFO_DATA;
    return myiic_write_read(max30102_dev, MAX30102_ADDR, &reg_addr, 1, buffer, count);
}

// 获取心率
uint32_t Max30102_Get_Heart_Rate(void)
{
    return last_heart_rate;
}

// 获取血氧
uint32_t Max30102_Get_Spo2(void)
{
    return last_spo2;
}

// 获取测量状态
bool Max30102_Is_Measuring(void)
{
    return measurement_active;
}

// 获取测量已用秒数
uint32_t Max30102_Get_Elapsed_Seconds(void)
{
    return measurement_elapsed_seconds;
}

// 是否有有效数据可显示
bool Max30102_Has_Valid_Data(void)
{
    return has_valid_data;
}

// 算法排序函数（升序）
static void maxim_sort_ascend(int32_t *pn_x, int32_t n_size)
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++)
    {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j - 1]; j--)
            pn_x[j] = pn_x[j - 1];
        pn_x[j] = n_temp;
    }
}

// 算法排序函数（降序索引）
static void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size)
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++)
    {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--)
            pn_indx[j] = pn_indx[j - 1];
        pn_indx[j] = n_temp;
    }
}

// 找峰值
static void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height)
{
    int32_t i = 1, n_width;
    *pn_npks = 0;

    while (i < n_size - 1)
    {
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i - 1])
        {
            n_width = 1;
            while (i + n_width < n_size && pn_x[i] == pn_x[i + n_width])
                n_width++;
            if (pn_x[i] > pn_x[i + n_width] && (*pn_npks) < 15)
            {
                pn_locs[(*pn_npks)++] = i;
                i += n_width + 1;
            }
            else
                i += n_width;
        }
        else
            i++;
    }
}

// 移除近距离峰值
static void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
{
    int32_t i, j, n_old_npks, n_dist;

    maxim_sort_indices_descend(pn_x, pn_locs, *pn_npks);

    for (i = -1; i < *pn_npks; i++)
    {
        n_old_npks = *pn_npks;
        *pn_npks = i + 1;
        for (j = i + 1; j < n_old_npks; j++)
        {
            n_dist = pn_locs[j] - (i == -1 ? -1 : pn_locs[i]);
            if (n_dist > n_min_distance || n_dist < -n_min_distance)
                pn_locs[(*pn_npks)++] = pn_locs[j];
        }
    }

    maxim_sort_ascend(pn_locs, *pn_npks);
}

// 找峰值主函数
static void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num)
{
    maxim_peaks_above_min_height(pn_locs, pn_npks, pn_x, n_size, n_min_height);
    maxim_remove_close_peaks(pn_locs, pn_npks, pn_x, n_min_distance);
    *pn_npks = (*pn_npks < n_max_num) ? *pn_npks : n_max_num;
}

// 心率血氧算法计算
void Max30102_Algorithm_Calculate(uint32_t *ir_buffer, int32_t buffer_len, uint32_t *red_buffer,
                                  int32_t *spo2, int8_t *spo2_valid,
                                  int32_t *heart_rate, int8_t *hr_valid)
{
    uint32_t un_ir_mean, un_only_once;
    int32_t k, n_i_ratio_count;
    int32_t i, s, m, n_exact_ir_valley_locs_count, n_middle_idx;
    int32_t n_th1, n_npks, n_c_min;
    int32_t an_ir_valley_locs[15];
    int32_t an_exact_ir_valley_locs[15];
    int32_t n_peak_interval_sum;

    int32_t n_y_ac, n_x_ac;
    int32_t n_spo2_calc;
    int32_t n_y_dc_max, n_x_dc_max;
    int32_t n_y_dc_max_idx = 0, n_x_dc_max_idx = 0;
    int32_t an_ratio[5], n_ratio_average;
    int32_t n_nume, n_denom;

    // 移除 IR 信号的 DC 分量
    un_ir_mean = 0;
    for (k = 0; k < buffer_len; k++)
        un_ir_mean += ir_buffer[k];
    un_ir_mean = un_ir_mean / buffer_len;
    for (k = 0; k < buffer_len; k++)
        an_x[k] = ir_buffer[k] - un_ir_mean;

    // 4点移动平均
    for (k = 0; k < buffer_len - MA4_SIZE; k++)
    {
        n_denom = (an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]);
        an_x[k] = n_denom / 4;
    }

    // 获取平滑后 IR 信号的差分
    for (k = 0; k < buffer_len - MA4_SIZE - 1; k++)
        an_dx[k] = (an_x[k + 1] - an_x[k]);

    // 2点移动平均
    for (k = 0; k < buffer_len - MA4_SIZE - 2; k++)
    {
        an_dx[k] = (an_dx[k] + an_dx[k + 1]) / 2;
    }

    // Hamming 窗口
    for (i = 0; i < buffer_len - HAMMING_SIZE - MA4_SIZE - 2; i++)
    {
        s = 0;
        for (k = i; k < i + HAMMING_SIZE; k++)
        {
            s -= an_dx[k] * auw_hamm[k - i];
        }
        an_dx[i] = s / 1146;
    }

    // 阈值计算
    n_th1 = 0;
    for (k = 0; k < buffer_len - HAMMING_SIZE; k++)
    {
        n_th1 += ((an_dx[k] > 0) ? an_dx[k] : (0 - an_dx[k]));
    }
    n_th1 = n_th1 / (buffer_len - HAMMING_SIZE);

    // 找峰值
    maxim_find_peaks(an_dx_peak_locs, &n_npks, an_dx, buffer_len - HAMMING_SIZE, n_th1, 8, 5);

    // 计算心率
    n_peak_interval_sum = 0;
    if (n_npks >= 2)
    {
        for (k = 1; k < n_npks; k++)
            n_peak_interval_sum += (an_dx_peak_locs[k] - an_dx_peak_locs[k - 1]);
        n_peak_interval_sum = n_peak_interval_sum / (n_npks - 1);
        *heart_rate = (int32_t)(6000 / n_peak_interval_sum);
        *hr_valid = 1;
    }
    else
    {
        *heart_rate = -999;
        *hr_valid = 0;
    }

    // 计算谷底位置
    for (k = 0; k < n_npks; k++)
        an_ir_valley_locs[k] = an_dx_peak_locs[k] + HAMMING_SIZE / 2;

    // 保存原始数据
    for (k = 0; k < buffer_len; k++)
    {
        an_x[k] = ir_buffer[k];
        an_y[k] = red_buffer[k];
    }

    // 精确找到谷底位置
    n_exact_ir_valley_locs_count = 0;
    for (k = 0; k < n_npks; k++)
    {
        un_only_once = 1;
        m = an_ir_valley_locs[k];
        n_c_min = 16777216;
        if (m + 5 < buffer_len - HAMMING_SIZE && m - 5 > 0)
        {
            for (i = m - 5; i < m + 5; i++)
            {
                if (an_x[i] < n_c_min)
                {
                    if (un_only_once > 0)
                    {
                        un_only_once = 0;
                    }
                    n_c_min = an_x[i];
                    an_exact_ir_valley_locs[k] = i;
                }
            }
            if (un_only_once == 0)
                n_exact_ir_valley_locs_count++;
        }
    }

    if (n_exact_ir_valley_locs_count < 2)
    {
        *spo2 = -999;
        *spo2_valid = 0;
        return;
    }

    // 4点移动平均
    for (k = 0; k < buffer_len - MA4_SIZE; k++)
    {
        an_x[k] = (an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]) / 4;
        an_y[k] = (an_y[k] + an_y[k + 1] + an_y[k + 2] + an_y[k + 3]) / 4;
    }

    // 计算比率
    n_ratio_average = 0;
    n_i_ratio_count = 0;

    for (k = 0; k < 5; k++)
        an_ratio[k] = 0;

    for (k = 0; k < n_exact_ir_valley_locs_count - 1; k++)
    {
        n_y_dc_max = -16777216;
        n_x_dc_max = -16777216;
        if (an_exact_ir_valley_locs[k + 1] - an_exact_ir_valley_locs[k] > 10)
        {
            for (i = an_exact_ir_valley_locs[k]; i < an_exact_ir_valley_locs[k + 1]; i++)
            {
                if (an_x[i] > n_x_dc_max)
                {
                    n_x_dc_max = an_x[i];
                    n_x_dc_max_idx = i;
                }
                if (an_y[i] > n_y_dc_max)
                {
                    n_y_dc_max = an_y[i];
                    n_y_dc_max_idx = i;
                }
            }

            n_y_ac = (an_y[an_exact_ir_valley_locs[k + 1]] - an_y[an_exact_ir_valley_locs[k]]) * (n_y_dc_max_idx - an_exact_ir_valley_locs[k]);
            n_y_ac = an_y[an_exact_ir_valley_locs[k]] + n_y_ac / (an_exact_ir_valley_locs[k + 1] - an_exact_ir_valley_locs[k]);
            n_y_ac = an_y[n_y_dc_max_idx] - n_y_ac;

            n_x_ac = (an_x[an_exact_ir_valley_locs[k + 1]] - an_x[an_exact_ir_valley_locs[k]]) * (n_x_dc_max_idx - an_exact_ir_valley_locs[k]);
            n_x_ac = an_x[an_exact_ir_valley_locs[k]] + n_x_ac / (an_exact_ir_valley_locs[k + 1] - an_exact_ir_valley_locs[k]);
            n_x_ac = an_x[n_y_dc_max_idx] - n_x_ac;

            n_nume = (n_y_ac * n_x_dc_max) >> 7;
            n_denom = (n_x_ac * n_y_dc_max) >> 7;

            if (n_denom > 0 && n_i_ratio_count < 5 && n_nume != 0)
            {
                an_ratio[n_i_ratio_count] = (n_nume * 100) / n_denom;
                n_i_ratio_count++;
            }
        }
    }

    maxim_sort_ascend(an_ratio, n_i_ratio_count);
    n_middle_idx = n_i_ratio_count / 2;

    if (n_middle_idx > 1)
        n_ratio_average = (an_ratio[n_middle_idx - 1] + an_ratio[n_middle_idx]) / 2;
    else if (n_middle_idx >= 0 && n_i_ratio_count > 0)
        n_ratio_average = an_ratio[n_middle_idx];
    else
    {
        *spo2 = -999;
        *spo2_valid = 0;
        return;
    }

    if (n_ratio_average > 2 && n_ratio_average < 184)
    {
        n_spo2_calc = uch_spo2_table[n_ratio_average];
        *spo2 = n_spo2_calc;
        *spo2_valid = 1;
    }
    else
    {
        *spo2 = -999;
        *spo2_valid = 0;
    }
}

// 从FIFO读取并解析样本到缓冲区
static int Max30102_Read_And_Store_Samples(void)
{
    int sample_count = Max30102_Get_Fifo_Count();
    if (sample_count <= 0)
        return 0;

    if (sample_count > 16)
        sample_count = 16;

    uint8_t fifo_buffer[96];
    esp_err_t ret = Max30102_Read_Fifo(fifo_buffer, sample_count * 6);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "读取 FIFO 失败: %d", ret);
        return 0;
    }

    for (int s = 0; s < sample_count; s++)
    {
        int offset = s * 6;
        uint32_t ir_value = ((uint32_t)fifo_buffer[offset] << 16) |
                            ((uint32_t)fifo_buffer[offset + 1] << 8) |
                            (uint32_t)fifo_buffer[offset + 2];
        uint32_t red_value = ((uint32_t)fifo_buffer[offset + 3] << 16) |
                             ((uint32_t)fifo_buffer[offset + 4] << 8) |
                             (uint32_t)fifo_buffer[offset + 5];

        ir_buffer[buffer_idx] = ir_value;
        red_buffer[buffer_idx] = red_value;
        buffer_idx = (buffer_idx + 1) % MAX30102_BUFFER_SIZE;
    }

    return sample_count;
}

// 任务函数
void Max30102_Task(void *pvParameters)
{
    esp_log_level_set("gpio", ESP_LOG_ERROR);
    esp_log_level_set("i2c", ESP_LOG_ERROR);

    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t part_id = 0;
    esp_err_t ret = Max30102_Read_Reg(REG_PART_ID, &part_id);
    if (ret == ESP_OK && part_id == 0x15)
    {
        ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (OK)", part_id);
    }
    else
    {
        ESP_LOGE(TAG, "WHO_AM_I 读取失败或错误: 0x%02X, err=%d", part_id, ret);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "MAX30102 任务启动 - 按 GPIO%d 键开始20秒血氧测量", KEY_GPIO_PIN);

    int32_t spo2 = 0, heart_rate = 0;
    int8_t spo2_valid = 0, hr_valid = 0;
    int samples_since_calc = 0;
    int total_samples = 0;
    TickType_t last_second_tick = 0;
    int poll_count = 0;

    while (1)
    {
        if (!measurement_active)
        {
            /* ---- 空闲状态：等待按键 ---- */
            Max30102_Read_And_Store_Samples();

            /* 每5秒打印一次 GPIO 诊断，确认轮询在运行 */
            if (++poll_count >= 100)  /* 100 * 50ms = 5秒 */
            {
                poll_count = 0;
                ESP_LOGI(TAG, "[轮询] GPIO%d 电平=%d (0=按下, 1=未按下)",
                         KEY_GPIO_PIN, gpio_get_level(KEY_GPIO_PIN));
            }

            if (Max30102_Is_Key_Pressed())
            {
                ESP_LOGI(TAG, "========================================");
                ESP_LOGI(TAG, "按键触发! 开始20秒心率血氧测量");
                ESP_LOGI(TAG, "请将手指放置在传感器上，保持不动");
                ESP_LOGI(TAG, "========================================");
                measurement_active = true;
                measurement_elapsed_seconds = 0;
                has_valid_data = false;
                buffer_idx = 0;
                samples_since_calc = 0;
                total_samples = 0;
                last_second_tick = xTaskGetTickCount();
                /* 不清除上次结果，OLED在算法产出新数据前继续显示旧值 */
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }
        else
        {
            /* ---- 测量状态：持续采集 + 周期计算 ---- */
            int count = Max30102_Read_And_Store_Samples();
            samples_since_calc += count;
            total_samples += count;

            /* 每秒更新一次 elapsed_seconds + 打印进度日志 */
            TickType_t now = xTaskGetTickCount();
            if ((now - last_second_tick) >= pdMS_TO_TICKS(1000))
            {
                last_second_tick += pdMS_TO_TICKS(1000);
                measurement_elapsed_seconds++;
                ESP_LOGI(TAG, "[进度] %lu/%d 秒 | 已采样 %d 个 | HR=%" PRIu32 " bpm SpO2=%" PRIu32 "%%",
                         (unsigned long)measurement_elapsed_seconds, MEASUREMENT_DURATION_S,
                         total_samples, last_heart_rate, last_spo2);
            }

            /* 缓冲区满一次就计算一次（~5秒数据） */
            if (samples_since_calc >= MAX30102_BUFFER_SIZE)
            {
                samples_since_calc = 0;
                ESP_LOGI(TAG, "[算法] 缓冲区满 500 样本，开始计算...");

                Max30102_Algorithm_Calculate(ir_buffer, MAX30102_BUFFER_SIZE, red_buffer,
                                             &spo2, &spo2_valid, &heart_rate, &hr_valid);

                ESP_LOGI(TAG, "[算法] 原始结果: hr_valid=%d heart_rate=%ld spo2_valid=%d spo2=%ld",
                         hr_valid, (long)heart_rate, spo2_valid, (long)spo2);

                if (hr_valid && heart_rate >= HEART_RATE_MIN_VALID && heart_rate <= HEART_RATE_MAX_VALID)
                {
                    last_heart_rate = (uint32_t)heart_rate;
                    has_valid_data = true;
                    ESP_LOGI(TAG, "[结果] 心率更新: %" PRIu32 " bpm", last_heart_rate);
                }
                else
                {
                    ESP_LOGW(TAG, "[结果] 心率无效 (raw=%ld, valid=%d)", (long)heart_rate, hr_valid);
                }

                if (spo2_valid && spo2 >= 0)
                {
                    last_spo2 = (uint32_t)spo2;
                    has_valid_data = true;
                    ESP_LOGI(TAG, "[结果] 血氧更新: %" PRIu32 "%%", last_spo2);
                }
                else
                {
                    ESP_LOGW(TAG, "[结果] 血氧无效 (raw=%ld, valid=%d)", (long)spo2, spo2_valid);
                }
            }

            /* 检查20秒是否到期 */
            if (measurement_elapsed_seconds >= MEASUREMENT_DURATION_S)
            {
                measurement_active = false;

                ESP_LOGI(TAG, "========================================");
                ESP_LOGI(TAG, "测量结束! 最终结果:");
                ESP_LOGI(TAG, "  心率: %" PRIu32 " bpm", last_heart_rate);
                ESP_LOGI(TAG, "  血氧: %" PRIu32 "%%", last_spo2);
                ESP_LOGI(TAG, "  总采样数: %d", total_samples);
                ESP_LOGI(TAG, "========================================");

                Message_Queue_Send_Heart_Rate(last_heart_rate, last_spo2, 0, false);
            }

            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}