// 人脸检测任务模块：数据层独立运行（CPU0），消费相机帧副本输出人脸框。
// 与显示层完全解耦：不触碰 LVGL、不持有相机帧缓冲。
#include "face_detect.h"

#include <cstring>
#include <list>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "human_face_detect.hpp"

namespace {
constexpr const char *TAG = "face_detect";

constexpr int kWidth = 160; // QQVGA
constexpr int kHeight = 120;
constexpr size_t kFrameBytes = (size_t)kWidth * kHeight * 2; // 38400 B

// 检测任务固定到 CPU0（采集/渲染任务在 CPU1）；栈 8KB，prio 4
constexpr BaseType_t kTaskCore = 0;
constexpr UBaseType_t kTaskPriority = 4;
constexpr uint32_t kTaskStackBytes = 8192;

HumanFaceDetect *s_detect = nullptr;

// 双缓冲：摄像头侧只写 s_cur，检测任务只读 s_pending，靠 busy 标志串行化
uint8_t *s_buf[2] = {nullptr, nullptr};
volatile int s_cur = 0;      // 摄像头侧下一个写入索引
volatile int s_pending = 0;  // 待推理帧所在索引
volatile bool s_busy = false; // true = 检测任务持有帧进行推理中

SemaphoreHandle_t s_frame_sem = nullptr; // 提交帧 -> 检测任务
SemaphoreHandle_t s_res_mutex = nullptr; // 保护结果发布/读取

face_box_t s_results[FACE_DETECT_MAX_FACES];
int s_result_count = 0;

volatile uint32_t s_drops = 0;       // 检测忙期间被丢弃的提交次数
volatile uint32_t s_det_fps_x10 = 0; // 最近一秒检测帧率（x10）

// 调试插桩：暂停开关 + 待应用的 MSR 阈值（由检测任务在推理前消费）
volatile bool s_enabled = true;        // false = 暂停帧提交
volatile float s_msr_thr_req = 0.5f;   // 目标 MSR 置信度阈值
volatile bool s_msr_thr_dirty = false; // true = 有待应用的新阈值

// 检测任务主循环：take(sem) -> 推理最新帧副本 -> 发布结果 -> 释放 busy
void DetectTask(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "detect task running on core %d", (int)xPortGetCoreID());

  // 默认 MSRMNP_S8_V1（懒加载：首帧推理时载入模型，避免阻塞启动）
  s_detect = new HumanFaceDetect();

  uint32_t det_count = 0;
  uint64_t infer_sum_us = 0;  // 本统计窗口内推理总耗时
  uint32_t infer_max_us = 0;  // 本统计窗口内单次推理最大耗时
  uint32_t infer_cnt = 0;     // 本统计窗口内推理次数
  int64_t window_start_us = esp_timer_get_time();

  while (true) {
    if (xSemaphoreTake(s_frame_sem, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    // 应用新的 MSR 置信度阈值（仅在本任务上下文调用，避免与推理竞态）
    if (s_msr_thr_dirty) {
      s_msr_thr_dirty = false;
      float thr = s_msr_thr_req;
      s_detect->set_score_thr(thr, 0); // idx 0 = MSR 阶段
      ESP_LOGI(TAG, "MSR score thr applied: %.2f", thr);
    }

    const dl::image::img_t img = {
        .data = s_buf[s_pending],
        .width = kWidth,
        .height = kHeight,
        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565BE, // 与摄像头大端字节序一致
    };

    int64_t infer_t0_us = esp_timer_get_time();
    std::list<dl::detect::result_t> &res = s_detect->run(img);
    int64_t infer_us = esp_timer_get_time() - infer_t0_us;
    infer_sum_us = infer_sum_us + (uint64_t)infer_us;
    infer_cnt = infer_cnt + 1;
    if (infer_us > (int64_t)infer_max_us) {
      infer_max_us = (uint32_t)infer_us;
    }

    // 发布结果：拷贝到共享区后立即释放锁，不长期持有
    if (xSemaphoreTake(s_res_mutex, portMAX_DELAY) == pdTRUE) {
      int n = 0;
      for (const auto &r : res) {
        if (n >= FACE_DETECT_MAX_FACES) {
          break;
        }
        s_results[n].x1 = (int16_t)r.box[0];
        s_results[n].y1 = (int16_t)r.box[1];
        s_results[n].x2 = (int16_t)r.box[2];
        s_results[n].y2 = (int16_t)r.box[3];
        float score = r.score * 100.0f + 0.5f;
        if (score > 100.0f) {
          score = 100.0f;
        }
        if (score < 0.0f) {
          score = 0.0f;
        }
        s_results[n].score = (uint8_t)score;
        n++;
      }
      s_result_count = n;
      xSemaphoreGive(s_res_mutex);
    }

    // 结果已拷贝出：本帧处理完成，允许摄像头侧提交下一帧
    s_busy = false;

    // 每秒统计一次检测帧率
    det_count = det_count + 1;
    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_us = now_us - window_start_us;
    if (elapsed_us >= 1000000) {
      s_det_fps_x10 =
          (uint32_t)((int64_t)det_count * 10 * 1000000 / elapsed_us);
      // 每秒输出推理耗时（avg/max）便于性能调优
      uint32_t avg_us = infer_cnt ? (uint32_t)(infer_sum_us / infer_cnt) : 0;
      ESP_LOGI(TAG, "det fps %u.%u infer avg %u.%u ms max %u.%u ms faces %d",
               (unsigned)(s_det_fps_x10 / 10), (unsigned)(s_det_fps_x10 % 10),
               (unsigned)(avg_us / 1000), (unsigned)((avg_us % 1000) / 100),
               (unsigned)(infer_max_us / 1000),
               (unsigned)((infer_max_us % 1000) / 100), s_result_count);
      infer_sum_us = 0;
      infer_max_us = 0;
      infer_cnt = 0;
      det_count = 0;
      window_start_us = now_us;
    }

    // 让出 1 个 tick：防止连续满负荷推理饿死 IDLE0 触发 Task WDT
    vTaskDelay(1);
  }
}
} // namespace

esp_err_t face_detect_init(void) {
  if (s_frame_sem != nullptr) {
    return ESP_OK; // 已初始化
  }

  // 帧副本放 PSRAM（内部 RAM 紧张，且与相机帧同域，拷贝走 PSRAM 带宽）
  for (int i = 0; i < 2; i++) {
    s_buf[i] = (uint8_t *)heap_caps_malloc(kFrameBytes,
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_buf[i] == nullptr) {
      ESP_LOGE(TAG, "frame buffer alloc failed");
      return ESP_ERR_NO_MEM;
    }
  }

  s_frame_sem = xSemaphoreCreateBinary();
  s_res_mutex = xSemaphoreCreateMutex();
  if (s_frame_sem == nullptr || s_res_mutex == nullptr) {
    ESP_LOGE(TAG, "semaphore create failed");
    return ESP_ERR_NO_MEM;
  }

  BaseType_t ok = xTaskCreatePinnedToCore(DetectTask, "face_detect",
                                          kTaskStackBytes, nullptr,
                                          kTaskPriority, nullptr, kTaskCore);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "detect task create failed");
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG, "init done (dual %uB buffers in PSRAM)",
           (unsigned)kFrameBytes);
  return ESP_OK;
}

bool face_detect_submit(const void *qrgb565be, size_t len) {
  if (qrgb565be == nullptr || len != kFrameBytes) {
    return false;
  }
  if (!s_enabled) {
    return false; // 已暂停：静默忽略，不计入丢帧
  }
  if (s_busy) {
    s_drops = s_drops + 1; // 检测中：丢帧保最新
    return false;
  }

  int idx = s_cur;
  memcpy(s_buf[idx], qrgb565be, kFrameBytes);

  // 先写好数据再发布索引并唤醒检测任务（sem 提供内存屏障）
  s_pending = idx;
  s_cur = idx ^ 1;
  s_busy = true;
  xSemaphoreGive(s_frame_sem);
  return true;
}

int face_detect_fetch(face_box_t *out, int max) {
  if (out == nullptr || max <= 0 || s_res_mutex == nullptr) {
    return 0;
  }
  if (xSemaphoreTake(s_res_mutex, portMAX_DELAY) != pdTRUE) {
    return 0;
  }
  int n = s_result_count < max ? s_result_count : max;
  memcpy(out, s_results, (size_t)n * sizeof(face_box_t));
  xSemaphoreGive(s_res_mutex);
  return n;
}

void face_detect_stats(uint32_t *det_fps_x10, uint32_t *drops) {
  if (det_fps_x10 != nullptr) {
    *det_fps_x10 = s_det_fps_x10;
  }
  if (drops != nullptr) {
    *drops = s_drops;
  }
}

void face_detect_set_enabled(bool enabled) {
  if (!enabled && s_res_mutex != nullptr &&
      xSemaphoreTake(s_res_mutex, portMAX_DELAY) == pdTRUE) {
    s_result_count = 0; // 暂停时清空结果，避免显示陈旧人脸框
    xSemaphoreGive(s_res_mutex);
  }
  s_det_fps_x10 = 0;
  s_enabled = enabled;
  ESP_LOGI(TAG, "detection %s", enabled ? "enabled" : "paused");
}

void face_detect_set_msr_thr(float thr) {
  s_msr_thr_req = thr; // 实际应用由检测任务在下一推理前完成
  s_msr_thr_dirty = true;
}
