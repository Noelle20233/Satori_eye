/**
 ****************************************************************************************************
 * @file        camera_preview.h
 * @brief       摄像头预览业务流水线：采集 -> QQVGA 自展开 -> 提交检测 -> LVGL 渲染
 ****************************************************************************************************
 */

#pragma once

// 初始化摄像头（camera_init 内部完成 XL9555 电源/复位时序与 esp_camera_init）
void camera_preview_init(void);

// 预览任务入口（FreeRTOS TaskFunction_t）：作为 camera_preview 任务体运行
void camera_preview_task(void *arg);
