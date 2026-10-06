/**
 ****************************************************************************************************
 * @file        app_config.h
 * @brief       应用级硬件/任务配置（引脚、分辨率、LVGL 缓冲、后台任务参数）
 * @note        仅集中放置编译期常量宏，不含任何逻辑；采集分辨率 CAMERA_H/V_RES 见 drivers/camera.h
 ****************************************************************************************************
 */

#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"

// ---------------------------------------------------------------- SPI LCD 引脚
#define LCD_HOST SPI2_HOST

#define PIN_NUM_SCLK GPIO_NUM_12 // GPIO12
#define PIN_NUM_MOSI GPIO_NUM_11 // GPIO11
#define PIN_NUM_MISO GPIO_NUM_13 // GPIO13，SPI LCD 一般不用
#define PIN_NUM_CS GPIO_NUM_21   // GPIO21，SPILCD_CS
#define PIN_NUM_DC GPIO_NUM_40   // GPIO40，SPILCD_DC
#define PIN_NUM_RST                                                            \
  GPIO_NUM_NC // LCD 复位由 XL9555 的 SLCD_RST_IO 控制，不占用 GPIO
#define PIN_NUM_QUADHD GPIO_NUM_NC
#define PIN_NUM_QUADWP GPIO_NUM_NC

// ---------------------------------------------------------------- 显示分辨率
#define LCD_H_RES 320 // 2.4 寸屏横屏分辨率
#define LCD_V_RES 240

// LVGL 绘制缓冲按行数配置：320 * 80 * 2B = 51.2KB/块，双缓冲共 102.4KB
// （按 240 行整屏 3 等分，减少刷屏往返次数；需内部 RAM 有足够余量）
#define LVGL_BUFFER_ROWS 80

// ---------------------------------------------------------------- 摄像头预览任务
#define CAM_TASK_STACK_BYTES 6144
#define CAM_TASK_PRIORITY 5 // 高于 LVGL 任务，保证预览流畅
#define CAM_TASK_CORE 1     // 固定到 CPU1，避免与系统任务争抢 CPU0
