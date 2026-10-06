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

// 初始化检测任务（CPU0，prio 4，栈 8KB）与双缓冲；幂等
esp_err_t face_detect_init(void);

// 提交一帧 QQVGA RGB565（大端）数据：仅当检测空闲时拷贝副本，
// 检测忙时直接丢帧（返回 false），保证检测始终消费最新帧
bool face_detect_submit(const void *qrgb565be, size_t len);

// 取最新检测结果（拷贝出），返回人脸数
int face_detect_fetch(face_box_t *out, int max);

// 统计：检测帧率（x10）、累计丢帧数
void face_detect_stats(uint32_t *det_fps_x10, uint32_t *drops);

// 调试开关：暂停/恢复检测帧提交（暂停时 submit 静默忽略并清空结果）
void face_detect_set_enabled(bool enabled);

// 运行时设置 MSR 阶段置信度阈值（0~1，下一推理周期生效，线程安全）
void face_detect_set_msr_thr(float thr);

#ifdef __cplusplus
}
#endif
