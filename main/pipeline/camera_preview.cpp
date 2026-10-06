/**
 ****************************************************************************************************
 * @file        camera_preview.cpp
 * @brief       摄像头预览流水线实现（原 main.cpp 的 InitCamera + CameraLoop）
 ****************************************************************************************************
 */

#include "camera_preview.h"

#include <cinttypes>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "app_config.h"
#include "app_context.h"
#include "camera.h"
#include "face_detect.h"
#include "face_tracker.h"
#include "preview_ui.h"

static constexpr const char *TAG = "CAM";

// ---------------------------------------------------------------- 摄像头
void camera_preview_init(void) {
  // camera_init 内部完成 XL9555 电源/复位时序与 esp_camera_init
  esp_err_t err = camera_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
  }
}

void camera_preview_task(void *arg) {
  (void)arg;
  uint32_t fps_count = 0;
  int64_t fps_window_start_us = esp_timer_get_time();

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL) {
      g_app.fb_fail_count = g_app.fb_fail_count + 1;
      ESP_LOGE(TAG, "Camera capture failed");
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // QQVGA -> 320x240 自展开：每像素水平/垂直各复制一次（约 1ms），
    // 避开 LVGL 缩放 transform 的昂贵路径，走 1:1 快速渲染；
    // 展开后立即归还相机帧，与 LVGL 异步渲染彻底解耦
    const uint16_t *src = (const uint16_t *)fb->buf;
    uint16_t *dst = g_app.preview_buf;
    for (int y = 0; y < CAMERA_V_RES; y++) {
      const uint16_t *s = src + y * CAMERA_H_RES;
      for (int x = 0; x < CAMERA_H_RES; x++) {
        dst[2 * x] = s[x];
        dst[2 * x + 1] = s[x];
      }
      // 第二行 = 第一行（垂直方向 2 倍）
      memcpy(dst + CAMERA_H_RES * 2, dst, CAMERA_H_RES * 4);
      dst += CAMERA_H_RES * 4; // 前进两行
    }
    // 提交帧副本给人脸检测：仅检测空闲时拷贝，忙则丢帧保最新；
    // 检测任务独立运行于 CPU0，不影响本任务采集/渲染时序
    int64_t cap_us = esp_timer_get_time();
    g_app.capture_seq = g_app.capture_seq + 1;
    face_detect_submit(fb->buf, fb->len, g_app.capture_seq, cap_us);
    esp_camera_fb_return(fb);

    // 取最新检测结果喂给跟踪器（~25fps 纯逻辑更新），再在 LVGL 锁外
    // 把跟踪框与其他人脸框画进显示缓冲（坐标 x2 映射）
    face_detect_result_t det = {};
    bool det_ok = face_detect_fetch_result(&det);
    if (det_ok) {
      face_tracker_update(&det, esp_timer_get_time());
      g_app.last_face_count = det.count;
    }
    const face_track_out_t *trk = face_tracker_get();
    preview_ui_draw_track_target(trk);
    if (det_ok) {
      preview_ui_draw_other_faces(&det, trk);
    }

    float fps = -1.0f;
    if (lvgl_port_lock(1000)) {
      // 同步渲染最新一帧（preview_dsc 恒定指向 preview_buf，无需改源）
      lv_obj_invalidate(g_app.preview_image);
      lv_refr_now(g_app.disp_handle);

      // 每秒统计一次帧率并更新屏幕覆盖层
      fps_count++;
      int64_t now_us = esp_timer_get_time();
      int64_t elapsed_us = now_us - fps_window_start_us;
      if (elapsed_us >= 1000000) {
        fps = fps_count * 1000000.0f / (float)elapsed_us;
        /* LVGL 内置 sprintf 未启用浮点支持时 %f 会输出成 "f"，
         * 改用整数拼接显示一位小数 */
        uint32_t fps_x10 = (uint32_t)(fps * 10.0f + 0.5f);
        g_app.last_fps_x10 = fps_x10;
        lv_label_set_text_fmt(g_app.fps_label, "FPS: %u.%u",
                              (unsigned)(fps_x10 / 10),
                              (unsigned)(fps_x10 % 10));
        // 同时刷新检测信息覆盖层（人脸数 + 检测帧率 + 跟踪状态）
        uint32_t det_fps_x10 = 0;
        face_detect_stats(&det_fps_x10, nullptr);
        lv_label_set_text_fmt(g_app.detect_label, "FACE: %d | DET: %u.%u | %s",
                              (int)g_app.last_face_count,
                              (unsigned)(det_fps_x10 / 10),
                              (unsigned)(det_fps_x10 % 10),
                              g_app.detect_enabled ? TrkStateStr(trk->state)
                                                   : "OFF");
        // OSD 可见期间周期刷新（跟踪年龄/状态持续变化）
        if (g_app.thr_label_shown) {
          preview_ui_update_thr_label();
        }
        fps_count = 0;
        fps_window_start_us = now_us;
      }

      // OSD 显隐：dirty 弹出并显示 3s；常显优先；非常显超时自动隐藏
      int64_t osd_now = esp_timer_get_time();
      if (g_app.osd_thr_dirty) {
        g_app.osd_thr_dirty = false;
        preview_ui_update_thr_label();
        lv_obj_set_hidden(g_app.thr_label, false);
        g_app.thr_label_shown = true;
        g_app.osd_show_until_us = osd_now + 3000000;
      }
      if (g_app.osd_always_show && !g_app.thr_label_shown) {
        preview_ui_update_thr_label();
        lv_obj_set_hidden(g_app.thr_label, false);
        g_app.thr_label_shown = true;
      }
      if (g_app.thr_label_shown && !g_app.osd_always_show &&
          osd_now > g_app.osd_show_until_us) {
        lv_obj_set_hidden(g_app.thr_label, true);
        g_app.thr_label_shown = false;
      }
      lvgl_port_unlock();
    }

    if (fps >= 0.0f) {
      ESP_LOGI(TAG,
               "Preview FPS: %.1f TRK=%s score=%u miss=%d bbox=(%d,%d,%d,%d) "
               "center=(%d,%d) age=%d ms det_frame=%u",
               fps, TrkStateStr(trk->state), (unsigned)trk->score, trk->miss,
               (int)trk->x1, (int)trk->y1, (int)trk->x2, (int)trk->y2,
               ((int)trk->x1 + trk->x2) / 2, ((int)trk->y1 + trk->y2) / 2,
               trk->age_ms, (unsigned)det.frame_id);
      // 每秒让出 1 个 tick：采集+展开+渲染忙等使本任务几乎满负荷，
      // 周期性给 CPU1 空闲任务运行窗口，避免 IDLE1 饿死触发 Task WDT
      vTaskDelay(1);
    }
  }
}
