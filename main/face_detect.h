#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 单张人脸检测框（坐标基准：160x120 数据层帧）
typedef struct {
  int16_t x1, y1, x2, y2;
  uint8_t score; // 置信度（0~100）
} face_box_t;

// 单帧最多返回的人脸数
#define FACE_DETECT_MAX_FACES 8

// 检测结果快照（含新鲜度信息，供跟踪/显示层消费）
typedef struct {
  face_box_t boxes[FACE_DETECT_MAX_FACES];
  int count;          // 人脸数量
  uint32_t frame_id;  // 对应提交帧的 frame_id
  int64_t ts_us;      // 该帧采集时刻（esp_timer_get_time 微秒）
  uint32_t infer_us;  // 本次推理耗时（微秒）
  uint32_t seq;       // 结果发布序号，每发布一帧 +1；0 表示尚无结果
} face_detect_result_t;

// 初始化检测任务（CPU0，prio 4，栈 8KB）与双缓冲；幂等
esp_err_t face_detect_init(void);

// 提交一帧 QQVGA RGB565（大端）数据：仅当检测空闲时拷贝副本，
// 检测忙时直接丢帧（返回 false），保证检测始终消费最新帧；
// frame_id/ts_us 为该帧在采集侧的编号与采集时刻，随结果快照一并发布
bool face_detect_submit(const void *qrgb565be, size_t len, uint32_t frame_id, int64_t ts_us);

// 拷贝最新结果快照；返回 false = out 为空或尚无任何结果（seq==0）
bool face_detect_fetch_result(face_detect_result_t *out);

// 统计：检测帧率（x10）、累计丢帧数
void face_detect_stats(uint32_t *det_fps_x10, uint32_t *drops);

// 调试开关：暂停/恢复检测帧提交（暂停时 submit 静默忽略并清空结果）
void face_detect_set_enabled(bool enabled);

// 运行时设置置信度阈值（0~1，下一推理周期生效，线程安全）；idx 0=MSR、1=MNP
void face_detect_set_score_thr(int idx, float thr);

// 运行时设置 NMS 阈值（0~1，下一推理周期生效，线程安全）；两级 NMS 阈值同时设置
void face_detect_set_nms_thr(float thr);

// 取回最近一次请求的阈值（请求值，尚未生效的也会返回）；指针可为 NULL
void face_detect_get_thresholds(float *msr, float *mnp, float *nms);

#ifdef __cplusplus
}
#endif
