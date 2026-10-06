#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "face_detect.h"

#ifdef __cplusplus
extern "C" {
#endif

// 跟踪状态机
typedef enum {
  FACE_TRACK_IDLE = 0,   // 无目标
  FACE_TRACK_ACQUIRING,  // 候选确认中（连续命中未达 2 次，此阶段不上屏）
  FACE_TRACK_TRACKING,   // 正常跟踪（绿框）
  FACE_TRACK_PREDICTING, // 短暂丢失，凭速度预测外推（黄框）
} face_track_state_t;

// 跟踪输出快照（坐标基准 160x120，与 face_detect 数据层一致）
typedef struct {
  bool valid;               // true = 应绘制（TRACKING/PREDICTING）
  face_track_state_t state; // 当前状态
  int16_t x1, y1, x2, y2;   // 160x120 坐标（平滑/预测后）
  uint8_t score;            // 最近一次命中的检测分数（0~100）
  int miss;                 // 连续未命中次数（以新检测结果为单位的累计）
  float vx, vy;             // 速度估计（px/s，调试/未来舵机用）
  int age_ms;               // 最近检测结果年龄（none 时为 -1）
} face_track_out_t;

// 复位全部内部状态回 IDLE（s_out 也清零）
void face_tracker_reset(void);

// 喂入最新检测结果与本地时刻（esp_timer_get_time 微秒），
// 每个相机帧（~25fps）调用一次；res 可为 NULL 或旧快照，内部自行去重/老化
void face_tracker_update(const face_detect_result_t *res, int64_t now_us);

// 取内部输出快照（常量指针，随 update 刷新；仅单线程读）
const face_track_out_t *face_tracker_get(void);

#ifdef __cplusplus
}
#endif
