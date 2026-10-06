#include "camera.h"
#include "face_detect.h"
#include "face_tracker.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "xl9555.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "dl_image_draw.hpp"

#include <cinttypes>
#include <cmath>
#include <cstring>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

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

#define LCD_H_RES 320 // 2.4 寸屏横屏分辨率
#define LCD_V_RES 240

// LVGL 绘制缓冲按行数配置：320 * 80 * 2B = 51.2KB/块，双缓冲共 102.4KB
// （按 240 行整屏 3 等分，减少刷屏往返次数；需内部 RAM 有足够余量）
#define LVGL_BUFFER_ROWS 80

// 摄像头预览任务配置
#define CAM_TASK_STACK_BYTES 6144
#define CAM_TASK_PRIORITY 5 // 高于 LVGL 任务，保证预览流畅
#define CAM_TASK_CORE 1     // 固定到 CPU1，避免与系统任务争抢 CPU0

static constexpr const char *TAG = "MAIN";

class Application {
public:
  void init() {
    ESP_LOGI(TAG, "Initializing application...");

    InitLCD();
    InitLVGL();
    InitCamera();
    if (face_detect_init() != ESP_OK) {
      ESP_LOGE(TAG, "Face detect init failed.");
    }
    CreatePreviewUI();

    ESP_LOGI(TAG, "Application initialized.");
  }

  void run() {
    // 摄像头预览任务：采集 -> LVGL 渲染显示 -> 归还帧缓冲
    BaseType_t ret = xTaskCreatePinnedToCore(
        CameraTaskEntry, "camera_preview", CAM_TASK_STACK_BYTES, this,
        CAM_TASK_PRIORITY, NULL, CAM_TASK_CORE);
    if (ret != pdPASS) {
      ESP_LOGE(TAG, "Failed to create camera task.");
      abort();
    }

    // 面板/电源守护任务：每 5s 重发面板初始化命令（健康面板无感），
    // 面板被干扰进入睡眠/显示关闭/配置漂移时数秒内自动恢复；
    // 同时校验 XL9555 配置寄存器与关键输出电平
    xTaskCreatePinnedToCore(HealTaskEntry, "panel_heal", 3072, this, 2, NULL,
                            0);
    // 串口控制台任务：1=软自愈 2=硬自愈(复位脉冲) 3=状态 4=开关自动自愈
    xTaskCreatePinnedToCore(ConsoleTaskEntry, "console", 3072, this, 4, NULL,
                            0);
    // 按键任务：BOOT(GPIO0) + XL9555 KEY0~KEY3 轮询去抖，切换阈值循环 /
    // OSD 常显 / 检测开关（与串口命令互通）
    xTaskCreatePinnedToCore(KeyTaskEntry, "keys", 3072, this, 2, NULL, 1);
    ESP_LOGI(TAG,
             "Console: 1=panel heal, 2=panel hard heal, 3=status, "
             "4=toggle auto-heal, 5=toggle face detect, 6=MSR thr, 7=MNP thr, "
             "8=NMS thr");

    while (true) {
      vTaskDelay(pdMS_TO_TICKS(10000));

      if (lvgl_port_lock(200)) {
        lv_mem_monitor(&lv_mem_stat);
        lv_mem_valid = true;
        lvgl_port_unlock();
      }

      ESP_LOGI(TAG,
               "Heap: internal free %" PRIu32 " B, PSRAM free %" PRIu32
               " B, fb_fail=%" PRIu32 ", heal=%" PRIu32 ", xl9555_bad=%" PRIu32,
               (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
               (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
               (uint32_t)fb_fail_count, (uint32_t)heal_count,
               (uint32_t)xl9555_anomaly_count);
      if (lv_mem_valid) {
        ESP_LOGI(TAG, "LVGL mem: used %u%%, frag %u%%, free %" PRIu32 " B",
                 lv_mem_stat.used_pct, lv_mem_stat.frag_pct,
                 (uint32_t)lv_mem_stat.free_size);
      }
    }
  }

private:
  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_lcd_panel_handle_t panel_handle = NULL;
  lv_display_t *disp_handle = NULL;

  lv_obj_t *preview_image = NULL;  // 摄像头预览图像对象
  lv_obj_t *fps_label = NULL;      // 帧率覆盖层
  lv_obj_t *detect_label = NULL;   // 人脸检测信息覆盖层
  lv_obj_t *thr_label = NULL;      // 阈值/跟踪状态 OSD（右上角）
  lv_image_dsc_t preview_dsc = {}; // 摄像头帧图像描述符（LVGL 图像源）
  uint16_t *preview_buf = NULL; // 320x240 显示缓冲（QQVGA 2 倍展开后）

  // 阈值 OSD 显隐控制：dirty/常显由按键/串口任务置位，其余仅 CameraLoop 访问
  volatile bool osd_thr_dirty = false;   // true = 需刷新 OSD 文本并弹出
  volatile bool osd_always_show = false; // BOOT 键切换的常显开关
  int64_t osd_show_until_us = 0;         // 自动隐藏截止时刻（仅 CameraLoop）
  bool thr_label_shown = false;          // OSD 当前是否可见（仅 CameraLoop）
  uint32_t capture_seq = 0;              // 采集帧序号（仅 CameraLoop）

  // 面板/电源守护与诊断统计
  volatile uint32_t fb_fail_count = 0;        // esp_camera_fb_get 失败次数
  volatile uint32_t heal_count = 0;           // 面板自愈执行次数
  volatile uint32_t xl9555_anomaly_count = 0; // XL9555 输出异常次数
  volatile bool auto_heal_enabled = true;     // 周期自愈开关（串口 4 号命令）
  volatile bool detect_enabled = true;        // 人脸检测开关（串口 5/KEY3 键）
  uint32_t last_fps_x10 = 0;                  // 最近一次帧率（x10）
  volatile int last_face_count = 0;           // 最近一次检测到的人脸数
  lv_mem_monitor_t lv_mem_stat = {};          // 最近一次 LVGL 池监控快照
  bool lv_mem_valid = false;

  // ---------------------------------------------------------------- 摄像头
  void InitCamera() {
    // camera_init 内部完成 XL9555 电源/复位时序与 esp_camera_init
    esp_err_t err = camera_init();
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
    }
  }

  static void CameraTaskEntry(void *arg) {
    static_cast<Application *>(arg)->CameraLoop();
  }

  void CameraLoop() {
    uint32_t fps_count = 0;
    int64_t fps_window_start_us = esp_timer_get_time();

    while (true) {
      camera_fb_t *fb = esp_camera_fb_get();
      if (fb == NULL) {
        fb_fail_count = fb_fail_count + 1;
        ESP_LOGE(TAG, "Camera capture failed");
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }

      // QQVGA -> 320x240 自展开：每像素水平/垂直各复制一次（约 1ms），
      // 避开 LVGL 缩放 transform 的昂贵路径，走 1:1 快速渲染；
      // 展开后立即归还相机帧，与 LVGL 异步渲染彻底解耦
      const uint16_t *src = (const uint16_t *)fb->buf;
      uint16_t *dst = preview_buf;
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
      capture_seq = capture_seq + 1;
      face_detect_submit(fb->buf, fb->len, capture_seq, cap_us);
      esp_camera_fb_return(fb);

      // 取最新检测结果喂给跟踪器（~25fps 纯逻辑更新），再在 LVGL 锁外
      // 把跟踪框与其他人脸框画进显示缓冲（坐标 x2 映射）
      face_detect_result_t det = {};
      bool det_ok = face_detect_fetch_result(&det);
      if (det_ok) {
        face_tracker_update(&det, esp_timer_get_time());
        last_face_count = det.count;
      }
      const face_track_out_t *trk = face_tracker_get();
      DrawTrackTarget(trk);
      if (det_ok) {
        DrawOtherFaces(&det, trk);
      }

      float fps = -1.0f;
      if (lvgl_port_lock(1000)) {
        // 同步渲染最新一帧（preview_dsc 恒定指向 preview_buf，无需改源）
        lv_obj_invalidate(preview_image);
        lv_refr_now(disp_handle);

        // 每秒统计一次帧率并更新屏幕覆盖层
        fps_count++;
        int64_t now_us = esp_timer_get_time();
        int64_t elapsed_us = now_us - fps_window_start_us;
        if (elapsed_us >= 1000000) {
          fps = fps_count * 1000000.0f / (float)elapsed_us;
          /* LVGL 内置 sprintf 未启用浮点支持时 %f 会输出成 "f"，
           * 改用整数拼接显示一位小数 */
          uint32_t fps_x10 = (uint32_t)(fps * 10.0f + 0.5f);
          last_fps_x10 = fps_x10;
          lv_label_set_text_fmt(fps_label, "FPS: %u.%u",
                                (unsigned)(fps_x10 / 10),
                                (unsigned)(fps_x10 % 10));
          // 同时刷新检测信息覆盖层（人脸数 + 检测帧率 + 跟踪状态）
          uint32_t det_fps_x10 = 0;
          face_detect_stats(&det_fps_x10, nullptr);
          lv_label_set_text_fmt(detect_label, "FACE: %d | DET: %u.%u | %s",
                                (int)last_face_count,
                                (unsigned)(det_fps_x10 / 10),
                                (unsigned)(det_fps_x10 % 10),
                                detect_enabled ? TrkStateStr(trk->state)
                                               : "OFF");
          // OSD 可见期间周期刷新（跟踪年龄/状态持续变化）
          if (thr_label_shown) {
            UpdateThrLabelText();
          }
          fps_count = 0;
          fps_window_start_us = now_us;
        }

        // OSD 显隐：dirty 弹出并显示 3s；常显优先；非常显超时自动隐藏
        int64_t osd_now = esp_timer_get_time();
        if (osd_thr_dirty) {
          osd_thr_dirty = false;
          UpdateThrLabelText();
          lv_obj_set_hidden(thr_label, false);
          thr_label_shown = true;
          osd_show_until_us = osd_now + 3000000;
        }
        if (osd_always_show && !thr_label_shown) {
          UpdateThrLabelText();
          lv_obj_set_hidden(thr_label, false);
          thr_label_shown = true;
        }
        if (thr_label_shown && !osd_always_show &&
            osd_now > osd_show_until_us) {
          lv_obj_set_hidden(thr_label, true);
          thr_label_shown = false;
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

  // ---------------------------------------------------- 绘制辅助 / 阈值 OSD
  // 跟踪状态枚举 -> 短字符串（日志/OSD 共用）
  static const char *TrkStateStr(face_track_state_t st) {
    switch (st) {
    case FACE_TRACK_ACQUIRING:
      return "ACQ";
    case FACE_TRACK_TRACKING:
      return "TRACK";
    case FACE_TRACK_PREDICTING:
      return "PRED";
    default:
      return "IDLE";
    }
  }

  // 三级阈值循环（idx 0=MSR、1=MNP、2=NMS），档位 0.2~0.8 步进 0.1；
  // 取当前值在档位中最接近的位置，切到下一档（三级各自独立循环）
  void CycleThr(int idx) {
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
    osd_thr_dirty = true;
  }

  // 刷新阈值 OSD 文本。只在 LVGL 锁内调用；LVGL 内建 sprintf 不支持 %f，
  // 故阈值用整数 x100 的形式拼出两位小数
  void UpdateThrLabelText() {
    float msr = 0.0f, mnp = 0.0f, nms = 0.0f;
    face_detect_get_thresholds(&msr, &mnp, &nms);
    const face_track_out_t *trk = face_tracker_get();
    const uint32_t m100 = (uint32_t)(msr * 100.0f + 0.5f);
    const uint32_t mnp100 = (uint32_t)(mnp * 100.0f + 0.5f);
    const uint32_t nms100 = (uint32_t)(nms * 100.0f + 0.5f);
    lv_label_set_text_fmt(thr_label,
                          "MSR %u.%02u MNP %u.%02u NMS %u.%02u\n"
                          "TRK=%s miss=%d age=%dms",
                          (unsigned)(m100 / 100), (unsigned)(m100 % 100),
                          (unsigned)(mnp100 / 100), (unsigned)(mnp100 % 100),
                          (unsigned)(nms100 / 100), (unsigned)(nms100 % 100),
                          TrkStateStr(trk->state), (int)trk->miss,
                          (int)trk->age_ms);
  }

  // 检测框 160x120 坐标 ×2 映射到 320x240 并夹取到显示范围；
  // 映射后退化（x2<=x1 或 y2<=y1）返回 false
  static bool MapBoxToScreen(int x1, int y1, int x2, int y2, int *ox1,
                             int *oy1, int *ox2, int *oy2) {
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

  // 画跟踪目标框：TRACKING=绿色、PREDICTING=黄色（短暂丢失外推中）。
  // 必须在 LVGL 锁外调用：本缓冲唯一写入者就是本相机任务
  void DrawTrackTarget(const face_track_out_t *trk) {
    if (trk == nullptr || !trk->valid) {
      return;
    }
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    if (!MapBoxToScreen(trk->x1, trk->y1, trk->x2, trk->y2, &x1, &y1, &x2,
                        &y2)) {
      return;
    }
    // 绿色 RGB(0,255,0)=0x07E0 / 黄色=0xFFE0；preview_buf 为大端字节序，
    // 故按字节序 {高,低} 传入（draw_hollow_rectangle 逐字节拷贝）
    std::vector<uint8_t> color = {0x07, 0xE0};
    if (trk->state == FACE_TRACK_PREDICTING) {
      color = {0xFF, 0xE0};
    }
    const dl::image::img_t img = {
        .data = preview_buf,
        .width = LCD_H_RES,
        .height = LCD_V_RES,
        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565BE,
    };
    dl::image::draw_hollow_rectangle(img, x1, y1, x2, y2, color, 2);
  }

  // 画非跟踪目标的其余人脸：1px 绿色细框；与跟踪目标 IoU>0.5 的框跳过
  // （已在 DrawTrackTarget 画过，避免重叠双框）。必须在 LVGL 锁外调用
  void DrawOtherFaces(const face_detect_result_t *det,
                      const face_track_out_t *trk) {
    if (det == nullptr || det->count <= 0) {
      return;
    }
    const std::vector<uint8_t> green = {0x07, 0xE0};
    const dl::image::img_t img = {
        .data = preview_buf,
        .width = LCD_H_RES,
        .height = LCD_V_RES,
        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565BE,
    };
    for (int i = 0; i < det->count; i++) {
      const face_box_t &b = det->boxes[i];
      if (trk != nullptr && trk->valid &&
          BoxIoU16(b.x1, b.y1, b.x2, b.y2, trk->x1, trk->y1, trk->x2,
                   trk->y2) > 0.5f) {
        continue;
      }
      int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
      if (!MapBoxToScreen(b.x1, b.y1, b.x2, b.y2, &x1, &y1, &x2, &y2)) {
        continue;
      }
      dl::image::draw_hollow_rectangle(img, x1, y1, x2, y2, green, 1);
    }
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

  // ------------------------------------------------------------ 面板/电源守护
  static void HealTaskEntry(void *arg) {
    static_cast<Application *>(arg)->HealLoop();
  }

  static void ConsoleTaskEntry(void *arg) {
    static_cast<Application *>(arg)->ConsoleLoop();
  }

  void HealLoop() {
    ESP_LOGI(TAG, "Panel guard started (soft heal every 30s).");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(30000));
      GuardXl9555();
      if (auto_heal_enabled) {
        PanelHeal(false);
      }
    }
  }

  // XL9555 守护：回读方向配置寄存器与关键输出电平（仅读，无副作用）。
  // 芯片受扰/掉电复位会导致输出寄存器丢失或整体复位成输入态，
  // 此时重新配置并恢复电平；I2C 瞬时读失败不当作芯片异常，避免误写。
  void GuardXl9555() {
    uint8_t cfg[2] = {0xFF, 0xFF};
    esp_err_t err = xl9555_read_reg(XL9555_CONFIG_PORT0_REG, cfg, 2);
    if (err != ESP_OK) {
      return;
    }
    bool bad = (cfg[0] != 0x03 || cfg[1] != 0xF0) ||
               (xl9555_pin_read(SLCD_PWR_IO) != 1) ||
               (xl9555_pin_read(SLCD_RST_IO) != 1) ||
               (xl9555_pin_read(OV_PWDN_IO) != 0) ||
               (xl9555_pin_read(OV_RESET_IO) != 1);
    if (bad) {
      xl9555_anomaly_count = xl9555_anomaly_count + 1;
      ESP_LOGW(TAG, "XL9555 anomaly #%" PRIu32 " (cfg=%02X,%02X), restoring",
               (uint32_t)xl9555_anomaly_count, cfg[0], cfg[1]);
      // 直接写配置寄存器 {P0=0x03,P1=0xF0}（等价 xl9555_ioconfig(0xF003)，
      // 但不带其重试死循环，避免 I2C 永久故障时守护任务被卡死）
      uint8_t cfg_fix[2] = {0x03, 0xF0};
      if (xl9555_write_byte(XL9555_CONFIG_PORT0_REG, cfg_fix, 2) != ESP_OK) {
        ESP_LOGE(TAG, "XL9555 config restore failed");
      }
      xl9555_pin_write(SLCD_PWR_IO, 1);
      xl9555_pin_write(SLCD_RST_IO, 1);
      xl9555_pin_write(OV_PWDN_IO, 0);
      xl9555_pin_write(OV_RESET_IO, 1);
    }
  }

  // ST7789 面板状态自愈：重发初始化命令序列（SLPOUT/MADCTL/COLMOD/RAMCTRL
  // + 旋转 + 颜色反转 + DISPON）。所有命令幂等且不触碰 GRAM，正常状态下
  // 无感；面板若被误置为睡眠/显示关闭/配置漂移可在周期内恢复。
  // 注意：必须在 LVGL 锁内执行——esp_lcd 的 SPI 事务不可并发，否则
  // acquire bus 失败会丢掉 flush 完成回调，使刷新等待永久卡死。
  void PanelHeal(bool hard_reset) {
    if (!lvgl_port_lock(2000)) {
      ESP_LOGW(TAG, "PanelHeal: LVGL lock timeout, skipped");
      return;
    }
    if (hard_reset) {
      // 与 InitLCD 一致的硬件复位时序：RST 低 10ms -> 高 -> 120ms
      xl9555_pin_write(SLCD_RST_IO, 0);
      vTaskDelay(pdMS_TO_TICKS(10));
      xl9555_pin_write(SLCD_RST_IO, 1);
      vTaskDelay(pdMS_TO_TICKS(120));
    }
    esp_lcd_panel_init(panel_handle);          // SLPOUT + MADCTL(含旋转位) + COLMOD + RAMCTRL
    esp_lcd_panel_swap_xy(panel_handle, true); // 与 InitLVGL 的 rotation 配置一致
    esp_lcd_panel_mirror(panel_handle, true, false);
    esp_lcd_panel_invert_color(panel_handle, true);
    esp_lcd_panel_disp_on_off(panel_handle, true);
    heal_count = heal_count + 1;
    lvgl_port_unlock();
  }

  // 串口控制台（UART0，与日志共用；不影响日志输出）
  void ConsoleLoop() {
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
        PanelHeal(false);
        ESP_LOGI(TAG, "Panel soft heal done (#%" PRIu32 ")",
                 (uint32_t)heal_count);
        break;
      case '2':
        PanelHeal(true);
        ESP_LOGI(TAG, "Panel hard heal done (#%" PRIu32 ")",
                 (uint32_t)heal_count);
        break;
      case '3':
        LogStatus();
        break;
      case '4':
        auto_heal_enabled = !auto_heal_enabled;
        ESP_LOGI(TAG, "Auto heal %s", auto_heal_enabled ? "ON" : "OFF");
        break;
      case '5':
        // 调试开关：暂停/恢复人脸检测（用于单变量对比实验）
        detect_enabled = !detect_enabled;
        face_detect_set_enabled(detect_enabled);
        break;
      case '6':
        CycleThr(0); // MSR 阈值循环
        break;
      case '7':
        CycleThr(1); // MNP 阈值循环
        break;
      case '8':
        CycleThr(2); // NMS 阈值循环
        break;
      default:
        break;
      }
    }
  }

  // 按键任务入口
  static void KeyTaskEntry(void *arg) {
    static_cast<Application *>(arg)->KeyLoop();
  }

  // 按键扫描（20ms 周期）：BOOT(GPIO0) + XL9555 KEY0~KEY3（均低有效）。
  // 映射：BOOT=OSD 常显开关、KEY0=MSR、KEY1=MNP、KEY2=NMS、KEY3=检测开关。
  // 去抖：每键一个 -3..3 计数（按下 +1、松开 -1），>=3 触发并按锁存标志
  // 防重复；松开到 <=-3 后重新武装。I2C 读失败时跳过 XL9555 4 键本 tick
  // 的去抖更新（不伪造松开，避免假边缘），BOOT 照常处理。
  void KeyLoop() {
    // BOOT 键：GPIO0 上拉输入，仅轮询不加中断
    gpio_config_t boot_cfg = {};
    boot_cfg.pin_bit_mask = 1ULL << GPIO_NUM_0;
    boot_cfg.mode = GPIO_MODE_INPUT;
    boot_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    boot_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    boot_cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&boot_cfg);
    ESP_LOGI(TAG,
             "Keys: BOOT=OSD always-show, KEY0=MSR thr, KEY1=MNP thr, "
             "KEY2=NMS thr, KEY3=detect on/off");

    int stable[5] = {};   // 每键去抖计数，钳位在 [-3,3]
    bool latched[5] = {}; // true = 已触发等待松开（防连发）
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(20));

      // raw[0]=BOOT（GPIO0 低有效）；raw[1..4]=KEY0~KEY3（XL9555 输入寄存器 P1）
      bool raw[5] = {};
      raw[0] = (gpio_get_level(GPIO_NUM_0) == 0);
      uint8_t p1 = 0;
      const bool keys_ok =
          (xl9555_read_reg(XL9555_INPUT_PORT1_REG, &p1, 1) == ESP_OK);
      if (keys_ok) {
        raw[1] = ((p1 >> 7) & 1) == 0; // KEY0
        raw[2] = ((p1 >> 6) & 1) == 0; // KEY1
        raw[3] = ((p1 >> 5) & 1) == 0; // KEY2
        raw[4] = ((p1 >> 4) & 1) == 0; // KEY3
      }

      for (int i = 0; i < 5; i++) {
        if (i > 0 && !keys_ok) {
          continue; // I2C 失败：不更新 KEY0~KEY3 去抖，避免伪松动
        }
        if (raw[i]) {
          if (stable[i] < 3) {
            stable[i] = stable[i] + 1;
          }
        } else if (stable[i] > -3) {
          stable[i] = stable[i] - 1;
        }
        if (!latched[i] && stable[i] >= 3) {
          latched[i] = true;
          if (i == 0) {
            osd_always_show = !osd_always_show;
            ESP_LOGI(TAG, "Thr OSD %s", osd_always_show ? "ON" : "OFF");
          } else if (i == 1) {
            CycleThr(0); // KEY0: MSR
          } else if (i == 2) {
            CycleThr(1); // KEY1: MNP
          } else if (i == 3) {
            CycleThr(2); // KEY2: NMS
          } else {
            // KEY3：检测开关，与串口 '5' 命令互通
            detect_enabled = !detect_enabled;
            face_detect_set_enabled(detect_enabled);
          }
        } else if (latched[i] && stable[i] <= -3) {
          latched[i] = false; // 键已松开：重新武装
        }
      }
    }
  }

  void LogStatus() {
    ESP_LOGI(TAG,
             "STATUS: up=%lld s fps=%u.%u fb_fail=%" PRIu32 " heal=%" PRIu32
             " xl9555_bad=%" PRIu32,
             esp_timer_get_time() / 1000000,
             (unsigned)(last_fps_x10 / 10), (unsigned)(last_fps_x10 % 10),
             (uint32_t)fb_fail_count, (uint32_t)heal_count,
             (uint32_t)xl9555_anomaly_count);
    ESP_LOGI(TAG, "STATUS pins: PWR=%d RST=%d PWDN=%d OVRST=%d",
             xl9555_pin_read(SLCD_PWR_IO), xl9555_pin_read(SLCD_RST_IO),
             xl9555_pin_read(OV_PWDN_IO), xl9555_pin_read(OV_RESET_IO));
    if (lvgl_port_lock(500)) {
      lv_mem_monitor(&lv_mem_stat);
      lv_mem_valid = true;
      lvgl_port_unlock();
    }
    if (lv_mem_valid) {
      ESP_LOGI(TAG,
               "STATUS heap: int=%" PRIu32 " B psram=%" PRIu32
               " B lvgl_used=%u%% lvgl_frag=%u%% lvgl_free=%" PRIu32 " B",
               (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
               (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
               lv_mem_stat.used_pct, lv_mem_stat.frag_pct,
               (uint32_t)lv_mem_stat.free_size);
    }
    uint32_t det_fps_x10 = 0;
    uint32_t det_drops = 0;
    face_detect_stats(&det_fps_x10, &det_drops);
    ESP_LOGI(TAG, "STATUS detect: faces=%d det_fps=%u.%u det_drops=%" PRIu32,
             (int)last_face_count, (unsigned)(det_fps_x10 / 10),
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

  // ---------------------------------------------------------------- 预览 UI
  void CreatePreviewUI() {
    // LVGL API 非线程安全：创建/修改 UI 前必须加锁
    if (!lvgl_port_lock(0)) {
      ESP_LOGE(TAG, "Failed to lock LVGL.");
      return;
    }

    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);

    // 显示缓冲 320x240 RGB565（153.6KB，放 PSRAM）：
    // 由 CameraLoop 将 QQVGA 帧 2 倍展开填充，dsc 恒定指向该缓冲
    preview_buf = (uint16_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * 2,
                                               MALLOC_CAP_SPIRAM);
    if (preview_buf == NULL) {
      ESP_LOGE(TAG, "Failed to allocate preview buffer.");
      lvgl_port_unlock();
      return;
    }

    // 摄像头 RGB565 帧为大端字节序（高字节在前），声明为 RGB565_SWAPPED：
    // LVGL 渲染时自动交换字节，全程零拷贝、零手动转换
    preview_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    preview_dsc.header.cf = LV_COLOR_FORMAT_RGB565_SWAPPED;
    preview_dsc.header.w = LCD_H_RES;
    preview_dsc.header.h = LCD_V_RES;
    preview_dsc.header.stride = LCD_H_RES * 2;
    preview_dsc.data_size = LCD_H_RES * LCD_V_RES * 2;
    preview_dsc.data = (const uint8_t *)preview_buf;

    preview_image = lv_image_create(scr);
    lv_obj_align(preview_image, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_image_set_src(preview_image, &preview_dsc); // 1:1 显示，无需缩放

    fps_label = lv_label_create(scr);
    lv_label_set_text(fps_label, "FPS: --");
    lv_obj_set_style_text_color(fps_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_bg_color(fps_label, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(fps_label, LV_OPA_50, LV_PART_MAIN);
    lv_obj_set_style_pad_all(fps_label, 4, LV_PART_MAIN);
    lv_obj_align(fps_label, LV_ALIGN_TOP_LEFT, 4, 4);

    // 检测信息覆盖层（FPS 标签下方）：人脸数与检测帧率，随 FPS 每秒刷新
    detect_label = lv_label_create(scr);
    lv_label_set_text(detect_label, "FACE: -- | DET: --");
    lv_obj_set_style_text_color(detect_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_bg_color(detect_label, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(detect_label, LV_OPA_50, LV_PART_MAIN);
    lv_obj_set_style_pad_all(detect_label, 4, LV_PART_MAIN);
    lv_obj_align_to(detect_label, fps_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);

    // 阈值/跟踪状态 OSD（右上角，样式与 fps_label 一致）：默认隐藏，
    // 按键/串口触发时弹出 3s，或 BOOT 键切换为常显
    thr_label = lv_label_create(scr);
    lv_label_set_text(thr_label, "MSR -- MNP -- NMS --\nTRK=IDLE");
    lv_obj_set_style_text_color(thr_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_bg_color(thr_label, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(thr_label, LV_OPA_50, LV_PART_MAIN);
    lv_obj_set_style_pad_all(thr_label, 4, LV_PART_MAIN);
    lv_obj_align(thr_label, LV_ALIGN_TOP_RIGHT, -4, 4);
    lv_obj_set_hidden(thr_label, true);

    lvgl_port_unlock();
  }

  // ---------------------------------------------------------------- LCD
  void InitLCD() {
    // 初始化 I2C 与 XL9555（LCD 电源/复位/背光均由 XL9555 控制）
    xl9555_init();

    // 打开 LCD 电源/背光（PWR 拉高，模块背光点亮）
    xl9555_pin_write(SLCD_PWR_IO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    // LCD 硬件复位：RST 拉低(≥10us) -> 拉高 -> 等待 120ms 复位完成
    xl9555_pin_write(SLCD_RST_IO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    xl9555_pin_write(SLCD_RST_IO, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    // spi初始化
    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = PIN_NUM_SCLK;
    buscfg.mosi_io_num = PIN_NUM_MOSI;
    buscfg.miso_io_num = PIN_NUM_MISO;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.data4_io_num = -1;
    buscfg.data5_io_num = -1;
    buscfg.data6_io_num = -1;
    buscfg.data7_io_num = -1;
    buscfg.flags = 0;
    buscfg.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;
    buscfg.max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t);

    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI Initialized.");

    // panel io初始化
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.dc_gpio_num = PIN_NUM_DC; // 40
    io_config.cs_gpio_num = PIN_NUM_CS; // 21
    // 80MHz（最终配置）：写屏窗口约 15ms（≈1 个面板扫描场），
    // 撕裂单次错位幅度最小、运动最流畅；超出 ST7789V 标称上限（62.5MHz）但实测稳定
    io_config.pclk_hz = 80 * 1000 * 1000;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.spi_mode = 0;
    io_config.trans_queue_depth = 10;

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                             &io_config, &io_handle));
    ESP_LOGI(TAG, "Panel IO Initialized.");

    // 面板初始化
    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num =
        PIN_NUM_RST; // NC：复位由 XL9555 控制，不占用 GPIO
    // 正点原子 2.4 寸屏实测：需 RGB 顺序（BGR 顺序会导致红蓝通道互换）
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = 16;

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    ESP_LOGI(TAG, "Panel Initialized.");

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);

    // 正点原子 2.4 寸屏：RGB 颜色顺序 + 开启颜色反转，两者配合颜色才正确
    esp_lcd_panel_invert_color(panel_handle, true);

    // 注意：横屏方向（swap_xy/mirror）统一在 InitLVGL 的 rotation 配置中设置
    // （lvgl_port_add_disp 会把该配置应用到面板），避免两处配置不一致。

    esp_lcd_panel_disp_on_off(panel_handle, true);
    return;
  }

  void InitLVGL() {
    // 1. 初始化 LVGL 移植层：负责 lv_tick 时钟、lv_timer_handler 任务与互斥锁
    const lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&port_cfg));

    // 2. 把 esp_lcd 面板注册为 LVGL 显示设备
    lvgl_port_display_cfg_t disp_cfg = {};
    disp_cfg.io_handle = io_handle;
    disp_cfg.panel_handle = panel_handle;
    disp_cfg.buffer_size = LCD_H_RES * LVGL_BUFFER_ROWS; // 单位：像素
    disp_cfg.double_buffer = true;
    disp_cfg.hres = LCD_H_RES;
    disp_cfg.vres = LCD_V_RES;
    disp_cfg.monochrome = false;
    disp_cfg.color_format = LV_COLOR_FORMAT_RGB565;
    // 与 esp_lcd 初始方向一致：2.4 寸屏横屏（MADCTL：MX=1、MV=1）
    disp_cfg.rotation.swap_xy = true;
    disp_cfg.rotation.mirror_x = true;
    disp_cfg.rotation.mirror_y = false;
    disp_cfg.flags.buff_dma = true; // 缓冲分配在内部 SRAM，可直接 DMA
    // SPI 屏必须交换 RGB565 字节序：否则颜色错乱
    // （例如深色背景 0x1A1A2E 会显示成亮紫色）
    disp_cfg.flags.swap_bytes = true;
    disp_cfg.flags.full_refresh = false;

    disp_handle = lvgl_port_add_disp(&disp_cfg);
    if (disp_handle == NULL) {
      ESP_LOGE(TAG, "Failed to add LVGL display.");
      abort();
    }

    ESP_LOGI(TAG, "LVGL initialized (%d x %d).", LCD_H_RES, LCD_V_RES);
  }
};

extern "C" void app_main() {
  Application app;

  app.init();
  app.run();
}
