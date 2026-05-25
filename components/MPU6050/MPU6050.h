#ifndef MPU6050_H
#define MPU6050_H

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
#define MPU6050_INT_GPIO      -1      /* 中断引脚, -1 表示未使用 */
#define MPU6050_ADDR          0x68    /* I2C 7-bit 地址 (AD0 接低电平时) */
#define MPU6050_WHO_AM_I_VAL  0x68    /* MPU6050 WHO_AM_I 寄存器返回值 */
#define MPU6500_WHO_AM_I_VAL  0x70    /* MPU6500 WHO_AM_I 寄存器返回值 (寄存器兼容) */

// --- 寄存器地址 ---
#define MPU6050_REG_WHO_AM_I     0x75 /* 器件 ID 寄存器 (只读, 返回 0x68 或 0x70) */
#define MPU6050_REG_PWR_MGMT_1   0x6B /* 电源管理 1: 复位(bit7)、睡眠(bit6)、时钟源选择(bit2:0) */
#define MPU6050_REG_SMPLRT_DIV   0x19 /* 采样率分频器: 采样率 = 1kHz / (1 + DIV), 当前 DIV=19 → 50Hz */
#define MPU6050_REG_CONFIG       0x1A /* DLPF 配置: 数字低通滤波器带宽, 当前值 0x05 → 加速度 10Hz / 陀螺仪 10Hz */
#define MPU6050_REG_GYRO_CONFIG  0x1B /* 陀螺仪配置: 量程选择(bit4:3), 当前值 0x18 → ±2000°/s (16.4 LSB/°/s) */
#define MPU6050_REG_ACCEL_CONFIG 0x1C /* 加速度计配置: 量程选择(bit4:3), 当前值 0x18 → ±16g (2048 LSB/g) */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B /* 数据起始寄存器: 从该地址连续读取 14 字节获取全部 6 轴 + 温度 */

// --- 算法参数 ---
#define MPU6050_BUFFER_SIZE      110   /* 摔倒/抽搐检测的采样缓冲区大小 (110 样本 ÷ 50Hz ≈ 2.2 秒) */
#define MPU6050_SAMPLES_PER_SEC  50    /* 采样率 (Hz), 由 SMPLRT_DIV 寄存器决定 */

// --- API 声明 ---

/**
 * @brief  初始化 MPU6050：添加设备到 I2C 总线 → 软件复位 → 唤醒 → 配置采样率/量程
 *         所有配置通过 Write_Reg 写入对应寄存器完成
 */
esp_err_t Mpu6050_Init(void);

/**
 * @brief  写单个寄存器
 *         I2C 时序: [S][ADDR+W][REG][DATA][P]
 *         实现: 将 reg 和 data 拼成 2 字节缓冲区，调用 myiic_write() 一次性发送
 *
 * @param  reg   目标寄存器地址 (如 0x6B = PWR_MGMT_1)
 * @param  data  要写入的字节值
 */
esp_err_t Mpu6050_Write_Reg(uint8_t reg, uint8_t data);

/**
 * @brief  读单个寄存器
 *         I2C 时序: [S][ADDR+W][REG][Sr][ADDR+R][DATA][P]
 *         实现: 先发送寄存器地址，再读取 1 字节，调用 myiic_write_read()
 *
 * @param  reg   目标寄存器地址 (如 0x75 = WHO_AM_I)
 * @param  data  [out] 读取到的 1 字节存放位置
 */
esp_err_t Mpu6050_Read_Reg(uint8_t reg, uint8_t *data);

/**
 * @brief  从 0x3B 起连续读取 14 字节原始数据，一次性获取全部 6 轴
 *         I2C 时序: [S][ADDR+W][0x3B][Sr][ADDR+R][14字节][P]
 *         寄存器布局 (0x3B~0x48):
 *           [0-1] Accel_X  [2-3] Accel_Y  [4-5] Accel_Z
 *           [6-7] Temp     [8-9] Gyro_X   [10-11] Gyro_Y  [12-13] Gyro_Z
 *         每个轴为 16 位有符号大端值，拼接方式: (high << 8) | low
 *
 * @param  ax  [out] X 轴加速度原始值 (±16g 量程, 2048 LSB/g)
 * @param  ay  [out] Y 轴加速度原始值
 * @param  az  [out] Z 轴加速度原始值
 * @param  gx  [out] X 轴陀螺仪原始值 (±2000°/s 量程, 16.4 LSB/°/s)
 * @param  gy  [out] Y 轴陀螺仪原始值
 * @param  gz  [out] Z 轴陀螺仪原始值
 */
esp_err_t Mpu6050_Read_Raw(int16_t *ax, int16_t *ay, int16_t *az,
                           int16_t *gx, int16_t *gy, int16_t *gz);

/**
 * @brief  基于加速度数据检测摔倒或抽搐事件
 *
 * @param  ax_buf  X 轴加速度缓冲区
 * @param  ay_buf  Y 轴加速度缓冲区
 * @param  az_buf  Z 轴加速度缓冲区
 * @param  len     缓冲区长度 (至少 10)
 * @return true=检测到异常, false=正常
 */
bool Mpu6050_Detect_Fall_Or_Convulsion(int16_t *ax_buf, int16_t *ay_buf, int16_t *az_buf, int len);
bool Get_isFall(void);

// --- 标志位管理函数 ---
bool Mpu6050_Can_Read(void);   /* 检测任务是否已产出新数据 */
void Mpu6050_Clear_Flag(void); /* 外部读取后清除标志 */

/**
 * @brief  获取缓存的加速度数据 (不产生 I2C 通信)
 * @param  ax  [out] X 轴加速度, NULL 表示不获取
 * @param  ay  [out] Y 轴加速度, NULL 表示不获取
 * @param  az  [out] Z 轴加速度, NULL 表示不获取
 */
void Mpu6050_Get_Accel_Data(int16_t *ax, int16_t *ay, int16_t *az);

/**
 * @brief  获取缓存的陀螺仪数据 (不产生 I2C 通信)
 * @param  gx  [out] X 轴角速度, NULL 表示不获取
 * @param  gy  [out] Y 轴角速度, NULL 表示不获取
 * @param  gz  [out] Z 轴角速度, NULL 表示不获取
 */
void Mpu6050_Get_Gyro_Data(int16_t *gx, int16_t *gy, int16_t *gz);

/**
 * @brief  FreeRTOS 任务：以 50Hz 循环读取 MPU6050 数据
 *         内部调用 Mpu6050_Init() → Read_Raw() → 更新缓存 → 摔倒/抽搐检测
 *         数据通过全局变量缓存，供 Get_Accel/Gyro_Data 读取
 * @param  pvParameters  未使用，传 NULL
 */
void Task_Mpu6050_Monitor(void *pvParameters);

#endif // MPU6050_H