/**
 ****************************************************************************************************
 * @file        app.cpp
 * @brief       应用装配层实现（原 main.cpp 中 Application::init/run/CycleThr 及主监控循环）
 ****************************************************************************************************
 */

#include "app.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "app_config.h"
#include "app_context.h"
#include "board_display.h"
#include "camera_preview.h"
#include "console.h"
#include "face_detect.h"
#include "keys.h"
#include "panel_guard.h"
#include "preview_ui.h"

static constexpr const char *TAG = "MAIN";

void app_init(void) {
  ESP_LOGI(TAG, "Initializing application...");

  board_lcd_init();
  board_lvgl_init();
  camera_preview_init();
  if (face_detect_init() != ESP_OK) {
    ESP_LOGE(TAG, "Face detect init failed.");
  }
  preview_ui_create();

  ESP_LOGI(TAG, "Application initialized.");
}

// 三级阈值循环（idx 0=MSR、1=MNP、2=NMS），档位 0.2~0.8 步进 0.1；
// 取当前值在档位中最接近的位置，切到下一档（三级各自独立循环）
void app_cycle_thr(int idx) {
  static const float kLadder[7] = {0.2f, 0.3f, 0.4f, 0.5f,
                                   0.6f, 0.7f, 0.8f};
  float msr = 0.0f, mnp = 0.0f, nms = 0.0f;
  face_detect_get_thresholds(&msr, &mnp, &nms);
  const float cur = (idx == 0) ? msr : ((idx == 1) ? mnp : nms);
  int pos = 0; // 找不到精确档位时从 0 起步
  for (int i = 0; i < 7; i++) {
    if (std::fabs(cur - kLadder[i]) < 0.005f) {
      pos = i;
      break;
    }
  }
  const float next = kLadder[(pos + 1) % 7];
  if (idx == 0) {
    face_detect_set_score_thr(0, next);
  } else if (idx == 1) {
    face_detect_set_score_thr(1, next);
  } else {
    face_detect_set_nms_thr(next);
  }
  face_detect_get_thresholds(&msr, &mnp, &nms); // 重新读请求值
  ESP_LOGI(TAG, "THR: MSR=%.2f MNP=%.2f NMS=%.2f", msr, mnp, nms);
  g_app.osd_thr_dirty = true;
}

void app_run(void) {
  // 摄像头预览任务：采集 -> LVGL 渲染显示 -> 归还帧缓冲
  BaseType_t ret = xTaskCreatePinnedToCore(
      camera_preview_task, "camera_preview", CAM_TASK_STACK_BYTES, NULL,
      CAM_TASK_PRIORITY, NULL, CAM_TASK_CORE);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create camera task.");
    abort();
  }

  // 面板/电源守护任务：周期重发面板初始化命令（健康面板无感），
  // 面板被干扰进入睡眠/显示关闭/配置漂移时数秒内自动恢复；
  // 同时校验 XL9555 配置寄存器与关键输出电平
  xTaskCreatePinnedToCore(panel_guard_task, "panel_heal", 3072, NULL, 2, NULL,
                          0);
  // 串口控制台任务：1=软自愈 2=硬自愈(复位脉冲) 3=状态 4=开关自动自愈
  xTaskCreatePinnedToCore(console_task, "console", 3072, NULL, 4, NULL, 0);
  // 按键任务：BOOT(GPIO0) + XL9555 KEY0~KEY3 轮询去抖，切换阈值循环 /
  // OSD 常显 / 检测开关（与串口命令互通）
  xTaskCreatePinnedToCore(keys_task, "keys", 3072, NULL, 2, NULL, 1);
  ESP_LOGI(TAG,
           "Console: 1=panel heal, 2=panel hard heal, 3=status, "
           "4=toggle auto-heal, 5=toggle face detect, 6=MSR thr, 7=MNP thr, "
           "8=NMS thr");

  // 主监控循环：周期性堆/LVGL 池体检日志
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10000));

    if (lvgl_port_lock(200)) {
      lv_mem_monitor(&g_app.lv_mem_stat);
      g_app.lv_mem_valid = true;
      lvgl_port_unlock();
    }

    ESP_LOGI(TAG,
             "Heap: internal free %" PRIu32 " B, PSRAM free %" PRIu32
             " B, fb_fail=%" PRIu32 ", heal=%" PRIu32 ", xl9555_bad=%" PRIu32,
             (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (uint32_t)g_app.fb_fail_count, (uint32_t)g_app.heal_count,
             (uint32_t)g_app.xl9555_anomaly_count);
    if (g_app.lv_mem_valid) {
      ESP_LOGI(TAG, "LVGL mem: used %u%%, frag %u%%, free %" PRIu32 " B",
               g_app.lv_mem_stat.used_pct, g_app.lv_mem_stat.frag_pct,
               (uint32_t)g_app.lv_mem_stat.free_size);
    }
  }
}
