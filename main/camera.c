/**
 ****************************************************************************************************
 * @file        camera.c
 * @author      Satori Eye
 * @version     V1.0
 * @date        2026-10-05
 * @brief       OV2640 摄像头驱动（ATK-MC2640 模块 + 正点原子 ESP32-S3 开发板）
 * @note        引脚布局参考正点原子官方例程（DNESP32S3 摄像头接口）
 ****************************************************************************************************
 */

#include "camera.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "xl9555.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CAMERA";

/* 正点原子 ESP32-S3 开发板摄像头接口（P2）引脚：
 * PWDN/RESET 由 XL9555 控制，XCLK 由模块自带 24MHz 有源晶振提供 */
#define CAM_PIN_PWDN    GPIO_NUM_NC
#define CAM_PIN_RESET   GPIO_NUM_NC
#define CAM_PIN_XCLK    GPIO_NUM_NC
#define CAM_PIN_SIOD    GPIO_NUM_39     /* SCCB SDA */
#define CAM_PIN_SIOC    GPIO_NUM_38     /* SCCB SCL */
#define CAM_PIN_D7      GPIO_NUM_18
#define CAM_PIN_D6      GPIO_NUM_17
#define CAM_PIN_D5      GPIO_NUM_16
#define CAM_PIN_D4      GPIO_NUM_15
#define CAM_PIN_D3      GPIO_NUM_7
#define CAM_PIN_D2      GPIO_NUM_6
#define CAM_PIN_D1      GPIO_NUM_5
#define CAM_PIN_D0      GPIO_NUM_4
#define CAM_PIN_VSYNC   GPIO_NUM_47
#define CAM_PIN_HREF    GPIO_NUM_48
#define CAM_PIN_PCLK    GPIO_NUM_45

esp_err_t camera_init(void)
{
    camera_config_t config = {
        .pin_pwdn       = CAM_PIN_PWDN,
        .pin_reset      = CAM_PIN_RESET,
        .pin_xclk       = CAM_PIN_XCLK,
        .pin_sccb_sda   = CAM_PIN_SIOD,
        .pin_sccb_scl   = CAM_PIN_SIOC,
        .pin_d7         = CAM_PIN_D7,
        .pin_d6         = CAM_PIN_D6,
        .pin_d5         = CAM_PIN_D5,
        .pin_d4         = CAM_PIN_D4,
        .pin_d3         = CAM_PIN_D3,
        .pin_d2         = CAM_PIN_D2,
        .pin_d1         = CAM_PIN_D1,
        .pin_d0         = CAM_PIN_D0,
        .pin_vsync      = CAM_PIN_VSYNC,
        .pin_href       = CAM_PIN_HREF,
        .pin_pclk       = CAM_PIN_PCLK,
        .xclk_freq_hz   = 24000000,                 /* 与模块 24MHz 晶振一致 */
        .ledc_timer     = LEDC_TIMER_0,             /* XCLK 为 NC，不会实际驱动 LEDC */
        .ledc_channel   = LEDC_CHANNEL_0,
        .pixel_format   = PIXFORMAT_RGB565,         /* 与 LCD 直连格式一致，免转换 */
        .frame_size     = FRAMESIZE_QQVGA,          /* 160x120，主循环 2 倍像素展开为 320x240 显示 */
        .jpeg_quality   = 12,                       /* 仅 JPEG 模式有效 */
        .fb_count       = 3,                        /* 三缓冲：驱动流水线预采集，取帧无需等待 */
        .fb_location    = CAMERA_FB_IN_PSRAM,       /* 帧缓冲放 PSRAM */
        .grab_mode      = CAMERA_GRAB_WHEN_EMPTY,   /* 有帧即取，预览延迟低 */
        .sccb_i2c_port  = 1,                        /* SCCB 走 I2C1，避免与 XL9555 的 I2C0 冲突 */
    };

    /* XL9555 控制摄像头电源与复位：
     * OV_PWDN 拉低进入工作模式；复位脚拉低 20ms 再拉高完成硬复位 */
    xl9555_pin_write(OV_PWDN_IO, 0);
    xl9555_pin_write(OV_RESET_IO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    xl9555_pin_write(OV_RESET_IO, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(err));
        return err;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Failed to get camera sensor handle");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OV2640 detected (PID=0x%02X, VER=0x%02X), QQVGA %dx%d RGB565, %d fb in PSRAM",
             sensor->id.PID, sensor->id.VER, CAMERA_H_RES, CAMERA_V_RES, config.fb_count);

    return ESP_OK;
}
