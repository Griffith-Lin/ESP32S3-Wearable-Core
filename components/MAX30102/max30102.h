#ifndef MAX30102_H
#define MAX30102_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "myiic.h"
#include "MessageQueue.h"

// --- 硬件映射 ---
#define MAX30102_INT_GPIO 6         /* 中断引脚 (当前未使用) */
#define MAX30102_ADDR     0x57      /* I2C 7-bit 地址 */

// --- 寄存器地址 ---
#define REG_INTR_STATUS_1 0x00     /* 中断状态寄存器 1 (读取清除) */
#define REG_INTR_STATUS_2 0x01     /* 中断状态寄存器 2 */
#define REG_INTR_ENABLE_1 0x02     /* 中断使能寄存器 1 */
#define REG_INTR_ENABLE_2 0x03     /* 中断使能寄存器 2 */
#define REG_FIFO_WR_PTR   0x04     /* FIFO 写指针 (0~31) */
#define REG_OVF_COUNTER   0x05     /* FIFO 溢出计数器 */
#define REG_FIFO_RD_PTR   0x06     /* FIFO 读指针 (0~31) */
#define REG_FIFO_DATA     0x07     /* FIFO 数据寄存器 (连续读取) */
#define REG_FIFO_CONFIG   0x08     /* FIFO 配置: 采样平均、满中断阈值 */
#define REG_MODE_CONFIG   0x09     /* 模式配置: HR-only / SpO2 / Multi-LED */
#define REG_SPO2_CONFIG   0x0A     /* SpO2 配置: ADC 范围、采样率、脉宽 */
#define REG_LED1_PA       0x0C     /* 红色 LED 脉冲幅度 (电流) */
#define REG_LED2_PA       0x0D     /* IR LED 脉冲幅度 (电流) */
#define REG_PILOT_PA      0x10     /* pilot LED 脉冲幅度 */
#define REG_PART_ID       0xFF     /* 器件 ID (应返回 0x15) */

// --- 算法参数 ---
#define MAX30102_BUFFER_SIZE 500    /* 心率/血氧算法所需的采样缓冲区大小 */
#define IR_BUF_LEN          500    /* IR 通道缓冲区长度 */
#define RED_BUF_LEN         500    /* Red 通道缓冲区长度 */
#define HEART_RATE_MIN_VALID 40    /* 有效心率下限 (bpm) */
#define HEART_RATE_MAX_VALID 180   /* 有效心率上限 (bpm) */

// --- API 声明 ---

/**
 * @brief  初始化 MAX30102：添加设备到 I2C 总线 → 复位 → 配置 FIFO/模式/SpO2/LED 电流 → 初始化按键 GPIO
 *         I2C 写操作通过 Write_Reg 完成
 */
esp_err_t Max30102_Init(void);

/**
 * @brief  写单个寄存器
 *         I2C 时序: [S][ADDR+W][REG][DATA][P]
 *         实现: 将 reg 和 data 拼成 2 字节缓冲区，调用 myiic_write() 一次性发送
 *
 * @param  reg   目标寄存器地址 (如 0x09 = MODE_CONFIG)
 * @param  data  要写入的字节值
 */
esp_err_t Max30102_Write_Reg(uint8_t reg, uint8_t data);

/**
 * @brief  读单个寄存器
 *         I2C 时序: [S][ADDR+W][REG][Sr][ADDR+R][DATA][P]
 *         实现: 先发送寄存器地址，再读取 1 字节，调用 myiic_write_read()
 *
 * @param  reg   目标寄存器地址 (如 0xFF = PART_ID)
 * @param  data  [out] 读取到的 1 字节存放位置
 */
esp_err_t Max30102_Read_Reg(uint8_t reg, uint8_t *data);

/**
 * @brief  从 FIFO 数据寄存器 (0x07) 连续读取多字节
 *         I2C 时序: [S][ADDR+W][0x07][Sr][ADDR+R][count字节][P]
 *         每个采样 6 字节: [IR_H][IR_M][IR_L][RED_H][RED_M][RED_L]
 *         每个通道 18-bit 有效数据，存储在 3 字节高位列中
 *
 * @param  buffer  [out] 读取数据的存放缓冲区，长度 >= count
 * @param  count   要读取的字节数 (通常 = 样本数 × 6)
 */
esp_err_t Max30102_Read_Fifo(uint8_t *buffer, uint8_t count);

/**
 * @brief  获取最近一次计算的心率值 (单位: bpm)
 * @return 心率值，0 表示尚未产出有效数据
 */
uint32_t Max30102_Get_Heart_Rate(void);

/**
 * @brief  获取最近一次计算的血氧值 (单位: %)
 * @return 血氧百分比，0 表示尚未产出有效数据
 */
uint32_t Max30102_Get_Spo2(void);

/**
 * @brief  查询当前是否正在进行 20 秒测量
 * @return true=测量中, false=空闲
 */
bool Max30102_Is_Measuring(void);

/**
 * @brief  获取当前测量已过去的秒数
 * @return 已用秒数 (0~20)，未测量时返回 0
 */
uint32_t Max30102_Get_Elapsed_Seconds(void);

/**
 * @brief  查询算法是否已产出至少一次有效的心率或血氧数据
 * @return true=有有效数据可显示, false=尚无有效数据
 */
bool Max30102_Has_Valid_Data(void);

/**
 * @brief  心率血氧核心算法 (移植自 Maxim 官方参考实现)
 *         处理流程: 去直流 → 移动平均 → 差分 → Hamming 窗滤波 → 峰值检测 → 计算心率
 *         血氧计算: 在 IR 信号谷底附近计算 AC/DC 比值，查表得到 SpO2
 *
 * @param  ir_buffer    [in]  IR 通道采样数据缓冲区
 * @param  buffer_len   [in]  缓冲区长度 (通常 500)
 * @param  red_buffer   [in]  Red 通道采样数据缓冲区
 * @param  spo2         [out] 计算得到的血氧值 (-999 表示无效)
 * @param  spo2_valid   [out] 1=血氧有效, 0=无效
 * @param  heart_rate   [out] 计算得到的心率值 (-999 表示无效)
 * @param  hr_valid     [out] 1=心率有效, 0=无效
 */
void Max30102_Algorithm_Calculate(uint32_t *ir_buffer, int32_t buffer_len, uint32_t *red_buffer,
                                  int32_t *spo2, int8_t *spo2_valid,
                                  int32_t *heart_rate, int8_t *hr_valid);

/**
 * @brief  FreeRTOS 任务：按键触发 20 秒心率血氧测量
 *         空闲态: 后台读取 FIFO + 每 5 秒打印 GPIO 诊断 + 等待按键
 *         测量态: 持续采集 → 每 500 样本调用算法 → 更新结果 → 到期自动停止
 *         结果通过全局变量缓存，供 Get_Heart_Rate/Get_Spo2 读取
 *
 * @param  pvParameters  未使用，传 NULL
 */
void Max30102_Task(void *pvParameters);

#endif // MAX30102_H