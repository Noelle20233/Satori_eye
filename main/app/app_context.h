/**
 ****************************************************************************************************
 * @file        app_context.h
 * @brief       跨层共享的应用状态（原 main.cpp 中 Application 的成员变量）
 * @note        单一实例 g_app 由 app_context.cpp 定义；各层通过引用全局访问，
 *              并发语义与原类一致（原成员即为多任务共享，volatile 标记跨任务字段）
 ****************************************************************************************************
 */

#pragma once

#include <cstdint>

#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"

#include "face_tracker.h" // face_track_state_t（TrkStateStr 使用）

struct AppContext {
  // -------------------------------------------------------------- 板级句柄
  esp_lcd_panel_io_handle_t io_handle = nullptr;
  esp_lcd_panel_handle_t panel_handle = nullptr;
  lv_display_t *disp_handle = nullptr;

  // -------------------------------------------------------------- 预览 UI
  lv_obj_t *preview_image = nullptr; // 摄像头预览图像对象
  lv_obj_t *fps_label = nullptr;     // 帧率覆盖层
  lv_obj_t *detect_label = nullptr;  // 人脸检测信息覆盖层
  lv_obj_t *thr_label = nullptr;     // 阈值/跟踪状态 OSD（右上角）
  lv_image_dsc_t preview_dsc = {};   // 摄像头帧图像描述符（LVGL 图像源）
  uint16_t *preview_buf = nullptr;   // 320x240 显示缓冲（QQVGA 2 倍展开后）

  // ------------------------------------------------------ 阈值 OSD 显隐控制
  // dirty/常显由按键/串口任务置位，其余仅相机任务访问
  volatile bool osd_thr_dirty = false; // true = 需刷新 OSD 文本并弹出
  volatile bool osd_always_show = false; // BOOT 键切换的常显开关
  int64_t osd_show_until_us = 0;         // 自动隐藏截止时刻（仅相机任务）
  bool thr_label_shown = false;          // OSD 当前是否可见（仅相机任务）
  uint32_t capture_seq = 0;              // 采集帧序号（仅相机任务）

  // ---------------------------------------------- 面板/电源守护与诊断统计
  volatile uint32_t fb_fail_count = 0;        // esp_camera_fb_get 失败次数
  volatile uint32_t heal_count = 0;           // 面板自愈执行次数
  volatile uint32_t xl9555_anomaly_count = 0; // XL9555 输出异常次数
  volatile bool auto_heal_enabled = true;     // 周期自愈开关（串口 4 号命令）
  volatile bool detect_enabled = true;        // 人脸检测开关（串口 5/KEY3 键）
  uint32_t last_fps_x10 = 0;                  // 最近一次帧率（x10）
  volatile int last_face_count = 0;           // 最近一次检测到的人脸数
  lv_mem_monitor_t lv_mem_stat = {};          // 最近一次 LVGL 池监控快照
  bool lv_mem_valid = false;
};

// 全局唯一实例（定义见 app_context.cpp）
extern AppContext g_app;

// 跟踪状态枚举 -> 短字符串（日志/OSD 共用）
const char *TrkStateStr(face_track_state_t st);
