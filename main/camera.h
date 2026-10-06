/**
 ****************************************************************************************************
 * @file        camera.h
 * @author      Satori Eye
 * @version     V1.0
 * @date        2026-10-05
 * @brief       OV2640 摄像头驱动（ATK-MC2640 模块 + 正点原子 ESP32-S3 开发板）
 ****************************************************************************************************
 */

#ifndef __CAMERA_H
#define __CAMERA_H

#include "esp_camera.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 采集分辨率：QQVGA 160x120，显示端由 LVGL 2 倍放大铺满 320x240 屏幕；
 * 读出量为 QVGA 的 1/4，换取更高传感器帧率（为后续人脸跟踪打基础） */
#define CAMERA_H_RES    160
#define CAMERA_V_RES    120

/**
 * @brief  初始化 OV2640 摄像头
 *         （XL9555 控制 PWDN/复位时序 + esp_camera_init，RGB565/QQVGA/三缓冲）
 * @param  无
 * @retval ESP_OK: 初始化成功; 其它: 失败
 */
esp_err_t camera_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_H */
