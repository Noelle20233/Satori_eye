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
volatile uint32_t s_pending_frame_id = 0; // 待推理帧编号（submit 在发布前写入）
volatile int64_t s_pending_ts_us = 0; // 待推理帧采集时刻（us，submit 发布前写入）

SemaphoreHandle_t s_frame_sem = nullptr; // 提交帧 -> 检测任务
SemaphoreHandle_t s_res_mutex = nullptr; // 保护结果发布/读取

// 结果快照：整体由 s_res_mutex 保护；seq 每发布一帧 +1（1 起步），
// 上层据此判断是否有新结果（暂停时仅清 count，seq 保持不变）
face_detect_result_t s_snapshot;

volatile uint32_t s_drops = 0;       // 检测忙期间被丢弃的提交次数
volatile uint32_t s_det_fps_x10 = 0; // 最近一秒检测帧率（x10）

// 调试插桩：暂停开关 + 待应用的阈值（由检测任务在推理前消费）
volatile bool s_enabled = true; // false = 暂停帧提交

// 阈值请求值：MSR(0)/MNP(1) 置信度 + 两级 NMS；dirty 表示有待应用的新值。
// 初始 dirty = true：检测任务首帧推理前统一应用并打印日志
volatile float s_score_req[2] = {0.5f, 0.4f}; // 默认 MSR=0.5、MNP=0.4
volatile float s_nms_req = 0.5f;              // 默认 NMS=0.5
volatile bool s_thr_dirty = true;

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
  uint32_t last_frame_id = 0; // 最近处理帧的编号（供每秒日志）
  int64_t last_ts_us = 0;     // 最近处理帧的采集时刻（供 age 统计）
  int last_faces = 0;         // 最近处理帧的人脸数
  uint32_t last_score = 0;    // 最近处理帧的最高分（0~100）

  while (true) {
    if (xSemaphoreTake(s_frame_sem, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    // 一次性应用三个阈值（仅在本任务上下文调用，避免与推理竞态）
    if (s_thr_dirty) {
      s_thr_dirty = false;
      float msr = s_score_req[0], mnp = s_score_req[1], nms = s_nms_req;
      s_detect->set_score_thr(msr, 0); // idx 0 = MSR 阶段
      s_detect->set_score_thr(mnp, 1); // idx 1 = MNP 阶段
      s_detect->set_nms_thr(nms, 0);
      s_detect->set_nms_thr(nms, 1);
      ESP_LOGI(TAG, "thr applied MSR=%.2f MNP=%.2f NMS=%.2f", msr, mnp, nms);
    }

    // 读取本帧新鲜度信息（s_busy 挡住后续提交，推理期间保持稳定）
    uint32_t frame_id = s_pending_frame_id;
    int64_t ts_us = s_pending_ts_us;

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

    // 发布结果快照：拷贝到共享区后立即释放锁，不长期持有
    int pub_faces = 0;      // 本帧人脸数（供每秒日志）
    uint32_t pub_score = 0; // 本帧最高分 0~100（供每秒日志）
    if (xSemaphoreTake(s_res_mutex, portMAX_DELAY) == pdTRUE) {
      int n = 0;
      uint32_t max_score = 0;
      for (const auto &r : res) {
        if (n >= FACE_DETECT_MAX_FACES) {
          break;
        }
        face_box_t &b = s_snapshot.boxes[n];
        b.x1 = (int16_t)r.box[0];
        b.y1 = (int16_t)r.box[1];
        b.x2 = (int16_t)r.box[2];
        b.y2 = (int16_t)r.box[3];
        float score = r.score * 100.0f + 0.5f;
        if (score > 100.0f) {
          score = 100.0f;
        }
        if (score < 0.0f) {
          score = 0.0f;
        }
        b.score = (uint8_t)score;
        if ((uint32_t)b.score > max_score) {
          max_score = (uint32_t)b.score;
        }
        n++;
      }
      s_snapshot.count = n;
      s_snapshot.frame_id = frame_id;
      s_snapshot.ts_us = ts_us;
      s_snapshot.infer_us = (uint32_t)infer_us; // 本帧推理耗时（us）
      s_snapshot.seq = s_snapshot.seq + 1;      // 发布序号：1 起步
      pub_faces = n;
      pub_score = max_score;
      xSemaphoreGive(s_res_mutex);
    }

    // 记录最近处理帧信息（供每秒日志统计新鲜度）
    last_frame_id = frame_id;
    last_ts_us = ts_us;
    last_faces = pub_faces;
    last_score = pub_score;

    // 结果已拷贝出：本帧处理完成，允许摄像头侧提交下一帧
    s_busy = false;

    // 每秒统计一次检测帧率
    det_count = det_count + 1;
    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_us = now_us - window_start_us;
    if (elapsed_us >= 1000000) {
      s_det_fps_x10 =
          (uint32_t)((int64_t)det_count * 10 * 1000000 / elapsed_us);
      // 每秒输出推理耗时（avg/max）与最近帧新鲜度，便于性能调优
      uint32_t avg_us = infer_cnt ? (uint32_t)(infer_sum_us / infer_cnt) : 0;
      uint32_t age_ms = 0; // 日志时刻距最近处理帧采集时刻的毫秒数
      if (last_ts_us > 0 && now_us > last_ts_us) {
        age_ms = (uint32_t)((now_us - last_ts_us) / 1000);
      }
      ESP_LOGI(TAG,
               "det fps %u.%u infer avg %u.%u ms max %u.%u ms faces %d "
               "frame=%u age=%u ms score=%u",
               (unsigned)(s_det_fps_x10 / 10), (unsigned)(s_det_fps_x10 % 10),
               (unsigned)(avg_us / 1000), (unsigned)((avg_us % 1000) / 100),
               (unsigned)(infer_max_us / 1000),
               (unsigned)((infer_max_us % 1000) / 100), last_faces,
               (unsigned)last_frame_id, (unsigned)age_ms,
               (unsigned)last_score);
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

bool face_detect_submit(const void *qrgb565be, size_t len, uint32_t frame_id,
                        int64_t ts_us) {
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

  // 先写好数据与新鲜度信息，再发布索引并唤醒检测任务（sem 提供内存屏障）
  s_pending_frame_id = frame_id;
  s_pending_ts_us = ts_us;
  s_pending = idx;
  s_cur = idx ^ 1;
  s_busy = true;
  xSemaphoreGive(s_frame_sem);
  return true;
}

bool face_detect_fetch_result(face_detect_result_t *out) {
  if (out == nullptr || s_res_mutex == nullptr) {
    return false;
  }
  if (xSemaphoreTake(s_res_mutex, portMAX_DELAY) != pdTRUE) {
    return false;
  }
  *out = s_snapshot; // 整体拷贝快照（含新鲜度字段）
  uint32_t seq = s_snapshot.seq;
  xSemaphoreGive(s_res_mutex);
  return seq != 0;
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
    s_snapshot.count = 0; // 暂停时清空结果（seq 不变，上层可据序号不变感知无新结果）
    xSemaphoreGive(s_res_mutex);
  }
  s_det_fps_x10 = 0;
  s_enabled = enabled;
  ESP_LOGI(TAG, "detection %s", enabled ? "enabled" : "paused");
}

void face_detect_set_score_thr(int idx, float thr) {
  if (idx != 0 && idx != 1) {
    return; // 非法索引：忽略
  }
  s_score_req[idx] = thr; // 实际应用由检测任务在下一推理前完成
  s_thr_dirty = true;
}

void face_detect_set_nms_thr(float thr) {
  s_nms_req = thr; // 实际应用由检测任务在下一推理前完成
  s_thr_dirty = true;
}

void face_detect_get_thresholds(float *msr, float *mnp, float *nms) {
  // volatile 先拷贝到局部变量再写出，避免多次读取不一致
  float m0 = s_score_req[0];
  float m1 = s_score_req[1];
  float n = s_nms_req;
  if (msr != nullptr) {
    *msr = m0;
  }
  if (mnp != nullptr) {
    *mnp = m1;
  }
  if (nms != nullptr) {
    *nms = n;
  }
}
