/**
 ****************************************************************************************************
 * @file        preview_ui.cpp
 * @brief       预览显示层实现（原 main.cpp 的 CreatePreviewUI + 绘制辅助 / 阈值 OSD）
 ****************************************************************************************************
 */

#include "preview_ui.h"

#include <vector>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "dl_image_draw.hpp"

#include "app_config.h"
#include "app_context.h"
#include "face_detect.h"
#include "face_tracker.h"

static constexpr const char *TAG = "UI";

// 检测框 160x120 坐标 ×2 映射到 320x240 并夹取到显示范围；
// 映射后退化（x2<=x1 或 y2<=y1）返回 false
static bool MapBoxToScreen(int x1, int y1, int x2, int y2, int *ox1, int *oy1,
                           int *ox2, int *oy2) {
  x1 = x1 * 2;
  y1 = y1 * 2;
  x2 = x2 * 2;
  y2 = y2 * 2;
  // 夹取到显示范围且保证 x2>x1、y2>y1（draw_hollow_rectangle 含断言）
  if (x1 < 0) {
    x1 = 0;
  }
  if (y1 < 0) {
    y1 = 0;
  }
  if (x2 > LCD_H_RES - 1) {
    x2 = LCD_H_RES - 1;
  }
  if (y2 > LCD_V_RES - 1) {
    y2 = LCD_V_RES - 1;
  }
  if (x2 <= x1 || y2 <= y1) {
    return false;
  }
  *ox1 = x1;
  *oy1 = y1;
  *ox2 = x2;
  *oy2 = y2;
  return true;
}

// 标准 IoU（交并比），坐标基准一致即可（此处为 160x120 空间）；含防除零
static float BoxIoU16(int16_t ax1, int16_t ay1, int16_t ax2, int16_t ay2,
                      int16_t bx1, int16_t by1, int16_t bx2, int16_t by2) {
  const int ix1 = (ax1 > bx1) ? ax1 : bx1;
  const int iy1 = (ay1 > by1) ? ay1 : by1;
  const int ix2 = (ax2 < bx2) ? ax2 : bx2;
  const int iy2 = (ay2 < by2) ? ay2 : by2;
  const int iw = ix2 - ix1;
  const int ih = iy2 - iy1;
  if (iw <= 0 || ih <= 0) {
    return 0.0f; // 无相交
  }
  const float inter = (float)iw * (float)ih;
  const float area_a = (float)(ax2 - ax1) * (float)(ay2 - ay1);
  const float area_b = (float)(bx2 - bx1) * (float)(by2 - by1);
  const float uni = area_a + area_b - inter;
  if (uni <= 0.0f) {
    return 0.0f; // 防除零
  }
  return inter / uni;
}

// ---------------------------------------------------------------- 预览 UI
void preview_ui_create(void) {
  // LVGL API 非线程安全：创建/修改 UI 前必须加锁
  if (!lvgl_port_lock(0)) {
    ESP_LOGE(TAG, "Failed to lock LVGL.");
    return;
  }

  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);

  // 显示缓冲 320x240 RGB565（153.6KB，放 PSRAM）：
  // 由相机任务将 QQVGA 帧 2 倍展开填充，dsc 恒定指向该缓冲
  g_app.preview_buf = (uint16_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * 2,
                                                   MALLOC_CAP_SPIRAM);
  if (g_app.preview_buf == NULL) {
    ESP_LOGE(TAG, "Failed to allocate preview buffer.");
    lvgl_port_unlock();
    return;
  }

  // 摄像头 RGB565 帧为大端字节序（高字节在前），声明为 RGB565_SWAPPED：
  // LVGL 渲染时自动交换字节，全程零拷贝、零手动转换
  g_app.preview_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
  g_app.preview_dsc.header.cf = LV_COLOR_FORMAT_RGB565_SWAPPED;
  g_app.preview_dsc.header.w = LCD_H_RES;
  g_app.preview_dsc.header.h = LCD_V_RES;
  g_app.preview_dsc.header.stride = LCD_H_RES * 2;
  g_app.preview_dsc.data_size = LCD_H_RES * LCD_V_RES * 2;
  g_app.preview_dsc.data = (const uint8_t *)g_app.preview_buf;

  g_app.preview_image = lv_image_create(scr);
  lv_obj_align(g_app.preview_image, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_image_set_src(g_app.preview_image,
                   &g_app.preview_dsc); // 1:1 显示，无需缩放

  g_app.fps_label = lv_label_create(scr);
  lv_label_set_text(g_app.fps_label, "FPS: --");
  lv_obj_set_style_text_color(g_app.fps_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_bg_color(g_app.fps_label, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_app.fps_label, LV_OPA_50, LV_PART_MAIN);
  lv_obj_set_style_pad_all(g_app.fps_label, 4, LV_PART_MAIN);
  lv_obj_align(g_app.fps_label, LV_ALIGN_TOP_LEFT, 4, 4);

  // 检测信息覆盖层（FPS 标签下方）：人脸数与检测帧率，随 FPS 每秒刷新
  g_app.detect_label = lv_label_create(scr);
  lv_label_set_text(g_app.detect_label, "FACE: -- | DET: --");
  lv_obj_set_style_text_color(g_app.detect_label, lv_color_white(),
                              LV_PART_MAIN);
  lv_obj_set_style_bg_color(g_app.detect_label, lv_color_black(),
                            LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_app.detect_label, LV_OPA_50, LV_PART_MAIN);
  lv_obj_set_style_pad_all(g_app.detect_label, 4, LV_PART_MAIN);
  lv_obj_align_to(g_app.detect_label, g_app.fps_label,
                  LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);

  // 阈值/跟踪状态 OSD（右上角，样式与 fps_label 一致）：默认隐藏，
  // 按键/串口触发时弹出 3s，或 BOOT 键切换为常显
  g_app.thr_label = lv_label_create(scr);
  lv_label_set_text(g_app.thr_label, "MSR -- MNP -- NMS --\nTRK=IDLE");
  lv_obj_set_style_text_color(g_app.thr_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_bg_color(g_app.thr_label, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_app.thr_label, LV_OPA_50, LV_PART_MAIN);
  lv_obj_set_style_pad_all(g_app.thr_label, 4, LV_PART_MAIN);
  lv_obj_align(g_app.thr_label, LV_ALIGN_TOP_RIGHT, -4, 4);
  lv_obj_set_hidden(g_app.thr_label, true);

  lvgl_port_unlock();
}

// 刷新阈值 OSD 文本。只在 LVGL 锁内调用；LVGL 内建 sprintf 不支持 %f，
// 故阈值用整数 x100 的形式拼出两位小数
void preview_ui_update_thr_label(void) {
  float msr = 0.0f, mnp = 0.0f, nms = 0.0f;
  face_detect_get_thresholds(&msr, &mnp, &nms);
  const face_track_out_t *trk = face_tracker_get();
  const uint32_t m100 = (uint32_t)(msr * 100.0f + 0.5f);
  const uint32_t mnp100 = (uint32_t)(mnp * 100.0f + 0.5f);
  const uint32_t nms100 = (uint32_t)(nms * 100.0f + 0.5f);
  lv_label_set_text_fmt(g_app.thr_label,
                        "MSR %u.%02u MNP %u.%02u NMS %u.%02u\n"
                        "TRK=%s miss=%d age=%dms",
                        (unsigned)(m100 / 100), (unsigned)(m100 % 100),
                        (unsigned)(mnp100 / 100), (unsigned)(mnp100 % 100),
                        (unsigned)(nms100 / 100), (unsigned)(nms100 % 100),
                        TrkStateStr(trk->state), (int)trk->miss,
                        (int)trk->age_ms);
}

// 画跟踪目标框：TRACKING=绿色、PREDICTING=黄色（短暂丢失外推中）。
// 必须在 LVGL 锁外调用：本缓冲唯一写入者就是相机任务
void preview_ui_draw_track_target(const face_track_out_t *trk) {
  if (trk == nullptr || !trk->valid) {
    return;
  }
  int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
  if (!MapBoxToScreen(trk->x1, trk->y1, trk->x2, trk->y2, &x1, &y1, &x2, &y2)) {
    return;
  }
  // 绿色 RGB(0,255,0)=0x07E0 / 黄色=0xFFE0；preview_buf 为大端字节序，
  // 故按字节序 {高,低} 传入（draw_hollow_rectangle 逐字节拷贝）
  std::vector<uint8_t> color = {0x07, 0xE0};
  if (trk->state == FACE_TRACK_PREDICTING) {
    color = {0xFF, 0xE0};
  }
  const dl::image::img_t img = {
      .data = g_app.preview_buf,
      .width = LCD_H_RES,
      .height = LCD_V_RES,
      .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565BE,
  };
  dl::image::draw_hollow_rectangle(img, x1, y1, x2, y2, color, 2);
}

// 画非跟踪目标的其余人脸：1px 绿色细框；与跟踪目标 IoU>0.5 的框跳过
// （已在 draw_track_target 画过，避免重叠双框）。必须在 LVGL 锁外调用
void preview_ui_draw_other_faces(const face_detect_result_t *det,
                                 const face_track_out_t *trk) {
  if (det == nullptr || det->count <= 0) {
    return;
  }
  const std::vector<uint8_t> green = {0x07, 0xE0};
  const dl::image::img_t img = {
      .data = g_app.preview_buf,
      .width = LCD_H_RES,
      .height = LCD_V_RES,
      .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565BE,
  };
  for (int i = 0; i < det->count; i++) {
    const face_box_t &b = det->boxes[i];
    if (trk != nullptr && trk->valid &&
        BoxIoU16(b.x1, b.y1, b.x2, b.y2, trk->x1, trk->y1, trk->x2, trk->y2) >
            0.5f) {
      continue;
    }
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    if (!MapBoxToScreen(b.x1, b.y1, b.x2, b.y2, &x1, &y1, &x2, &y2)) {
      continue;
    }
    dl::image::draw_hollow_rectangle(img, x1, y1, x2, y2, green, 1);
  }
}
