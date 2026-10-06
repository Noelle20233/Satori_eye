/**
 ****************************************************************************************************
 * @file        console.cpp
 * @brief       串口控制台实现（原 main.cpp 的 ConsoleLoop + LogStatus）
 ****************************************************************************************************
 */

#include "console.h"

#include <cinttypes>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "app.h"
#include "app_context.h"
#include "face_detect.h"
#include "face_tracker.h"
#include "panel_guard.h"
#include "xl9555.h"

static constexpr const char *TAG = "CONSOLE";

static void LogStatus(void) {
  ESP_LOGI(TAG,
           "STATUS: up=%lld s fps=%u.%u fb_fail=%" PRIu32 " heal=%" PRIu32
           " xl9555_bad=%" PRIu32,
           esp_timer_get_time() / 1000000,
           (unsigned)(g_app.last_fps_x10 / 10),
           (unsigned)(g_app.last_fps_x10 % 10), (uint32_t)g_app.fb_fail_count,
           (uint32_t)g_app.heal_count, (uint32_t)g_app.xl9555_anomaly_count);
  ESP_LOGI(TAG, "STATUS pins: PWR=%d RST=%d PWDN=%d OVRST=%d",
           xl9555_pin_read(SLCD_PWR_IO), xl9555_pin_read(SLCD_RST_IO),
           xl9555_pin_read(OV_PWDN_IO), xl9555_pin_read(OV_RESET_IO));
  if (lvgl_port_lock(500)) {
    lv_mem_monitor(&g_app.lv_mem_stat);
    g_app.lv_mem_valid = true;
    lvgl_port_unlock();
  }
  if (g_app.lv_mem_valid) {
    ESP_LOGI(TAG,
             "STATUS heap: int=%" PRIu32 " B psram=%" PRIu32
             " B lvgl_used=%u%% lvgl_frag=%u%% lvgl_free=%" PRIu32 " B",
             (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             g_app.lv_mem_stat.used_pct, g_app.lv_mem_stat.frag_pct,
             (uint32_t)g_app.lv_mem_stat.free_size);
  }
  uint32_t det_fps_x10 = 0;
  uint32_t det_drops = 0;
  face_detect_stats(&det_fps_x10, &det_drops);
  ESP_LOGI(TAG, "STATUS detect: faces=%d det_fps=%u.%u det_drops=%" PRIu32,
           (int)g_app.last_face_count, (unsigned)(det_fps_x10 / 10),
           (unsigned)(det_fps_x10 % 10), (uint32_t)det_drops);
  float msr = 0.0f, mnp = 0.0f, nms = 0.0f;
  face_detect_get_thresholds(&msr, &mnp, &nms);
  ESP_LOGI(TAG, "STATUS thr: MSR=%.2f MNP=%.2f NMS=%.2f", msr, mnp, nms);
  const face_track_out_t *trk = face_tracker_get(); // 调试视图，允许轻微读取竞态
  ESP_LOGI(TAG,
           "STATUS track: state=%s score=%u miss=%d age=%dms v=(%.0f,%.0f)",
           TrkStateStr(trk->state), (unsigned)trk->score, trk->miss,
           trk->age_ms, trk->vx, trk->vy);
}

// 串口控制台（UART0，与日志共用；不影响日志输出）
void console_task(void *arg) {
  (void)arg;
  if (uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0) != ESP_OK) {
    ESP_LOGE(TAG, "Console: uart driver install failed");
    vTaskDelete(NULL);
    return;
  }
  uint8_t c = 0;
  while (true) {
    if (uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(200)) != 1) {
      continue;
    }
    switch (c) {
    case '1':
      panel_heal(false);
      ESP_LOGI(TAG, "Panel soft heal done (#%" PRIu32 ")",
               (uint32_t)g_app.heal_count);
      break;
    case '2':
      panel_heal(true);
      ESP_LOGI(TAG, "Panel hard heal done (#%" PRIu32 ")",
               (uint32_t)g_app.heal_count);
      break;
    case '3':
      LogStatus();
      break;
    case '4':
      g_app.auto_heal_enabled = !g_app.auto_heal_enabled;
      ESP_LOGI(TAG, "Auto heal %s", g_app.auto_heal_enabled ? "ON" : "OFF");
      break;
    case '5':
      // 调试开关：暂停/恢复人脸检测（用于单变量对比实验）
      g_app.detect_enabled = !g_app.detect_enabled;
      face_detect_set_enabled(g_app.detect_enabled);
      break;
    case '6':
      app_cycle_thr(0); // MSR 阈值循环
      break;
    case '7':
      app_cycle_thr(1); // MNP 阈值循环
      break;
    case '8':
      app_cycle_thr(2); // NMS 阈值循环
      break;
    default:
      break;
    }
  }
}
