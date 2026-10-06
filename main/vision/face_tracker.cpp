// 人脸跟踪模块：把约 8fps 的检测结果经 α-β 滤波与速度预测输出 25fps 稳定框。
// 纯计算模块：无任务/无锁/无 LVGL/硬件依赖，仅由相机任务单线程调用 update/get。
#include "face_tracker.h"

#include <cmath>
#include <cstring>

#include "esp_log.h"

namespace {
constexpr const char *TAG = "face_tracker";

// 数据层帧坐标基准（与 face_detect 一致：160x120）
constexpr float kFrameW = 160.0f;
constexpr float kFrameH = 120.0f;

// 状态机阈值
constexpr int kAcquireScore = 55; // IDLE 进入 ACQUIRING 所需的最低分
constexpr int kTrackScore = 40;   // 跟踪/预测中维持命中所需的最低分
constexpr int kAcquireHits = 2;   // ACQUIRING 连续命中次数门槛
constexpr int kMissToPredict = 3; // 连续未命中转 PREDICTING 的门槛
constexpr int kLostMiss = 8;      // 连续未命中彻底丢失（回 IDLE）的门槛

// 老化时间阈值（us）：最近一次“目标证据”距今超过阈值即降级。
// 实测检测周期 ~160ms、结果消费年龄 140~200ms，阈值按实机数据放宽，
// 避免单次漏配/结果晚到触发误降级（真机日志曾出现 miss=2 即被降级）
constexpr int64_t kStaleToPredictUs = 500000; // ACQ->IDLE / TRACK->PREDICT
constexpr int64_t kLostStaleUs = 750000;      // PREDICTING->IDLE
constexpr int64_t kMaxMeasAgeUs = 350000;     // 量测新鲜度上限，超龄不做滤波更新

// α-β 滤波器参数（6.3fps 量测下适当加大增益，减小运动滞后导致的匹配残差）
constexpr float kAlpha = 0.6f;      // 位置修正增益
constexpr float kBeta = 0.2f;       // 速度修正增益
constexpr float kSizeAlpha = 0.3f;  // 尺寸 EMA 增益
constexpr float kMaxSpeed = 400.0f; // 速度限幅（px/s）

// 候选框与当前框的匹配门限/组合权重
constexpr float kMatchIoU = 0.10f;       // IoU 门限
constexpr float kMatchDistRatio = 0.5f;  // 中心距门限（相对当前框对角线）
constexpr float kMatchDistMin = 20.0f;   // 中心距门限（绝对下限，px）
constexpr float kWScore = 0.4f;          // 组合权重：检测分数
constexpr float kWIoU = 0.4f;            // 组合权重：IoU
constexpr float kWDist = 0.2f;           // 组合权重：中心距

// 内部状态：float 中心-尺寸表示
face_track_state_t s_state = FACE_TRACK_IDLE;
float s_cx = 0.0f;               // 框中心 x
float s_cy = 0.0f;               // 框中心 y
float s_w = 0.0f;                // 框宽
float s_h = 0.0f;                // 框高
float s_vx = 0.0f;               // 速度 vx（px/s）
float s_vy = 0.0f;               // 速度 vy（px/s）
uint8_t s_score = 0;             // 最近一次命中的检测分数（0~100）
int s_miss = 0;                  // 连续未命中次数（以新检测结果为单位的累计）
int s_acq_hits = 0;              // ACQUIRING 阶段的连续命中次数
uint32_t s_last_seq = 0;         // 最近消费的检测结果序号
int64_t s_last_seen_us = 0;      // 最近一次“目标证据”的本地时刻
int64_t s_last_meas_ts_us = 0;   // α-β dt 基准：最近一次量测帧的采集时刻
int64_t s_last_tick_us = 0;      // 上一 tick 的本地时刻
int64_t s_last_result_ts_us = 0; // 最近收到的任何新结果的采集时刻
face_track_out_t s_out;          // 对外输出快照（face_tracker_get 返回其指针）

// 状态名（仅用于状态转换日志）
const char *StateName(face_track_state_t st) {
  switch (st) {
    case FACE_TRACK_ACQUIRING:
      return "ACQUIRING";
    case FACE_TRACK_TRACKING:
      return "TRACKING";
    case FACE_TRACK_PREDICTING:
      return "PREDICTING";
    default:
      return "IDLE";
  }
}

// 跟踪复位（丢失语义）：几何/速度/命中计数清零回 IDLE；
// 保留结果序号与结果年龄基准，避免同一结果被重复消费
void ResetTrack(void) {
  s_state = FACE_TRACK_IDLE;
  s_cx = 0.0f;
  s_cy = 0.0f;
  s_w = 0.0f;
  s_h = 0.0f;
  s_vx = 0.0f;
  s_vy = 0.0f;
  s_score = 0;
  s_miss = 0;
  s_acq_hits = 0;
  s_last_seen_us = 0;
  s_last_meas_ts_us = 0;
}

// 放弃确认（ACQ 阶段未达标）：清残留几何/分数回 IDLE，
// 避免 IDLE 状态下日志/输出仍带最后一次候选框
void AbandonAcquire(void) {
  s_cx = 0.0f;
  s_cy = 0.0f;
  s_w = 0.0f;
  s_h = 0.0f;
  s_vx = 0.0f;
  s_vy = 0.0f;
  s_score = 0;
  s_acq_hits = 0;
  s_state = FACE_TRACK_IDLE;
}

// 交并比：两个框均为 (cx, cy, w, h) 中心-尺寸表示
float IoU(float acx, float acy, float aw, float ah, float bcx, float bcy,
          float bw, float bh) {
  const float ax1 = acx - aw * 0.5f;
  const float ay1 = acy - ah * 0.5f;
  const float ax2 = acx + aw * 0.5f;
  const float ay2 = acy + ah * 0.5f;
  const float bx1 = bcx - bw * 0.5f;
  const float by1 = bcy - bh * 0.5f;
  const float bx2 = bcx + bw * 0.5f;
  const float by2 = bcy + bh * 0.5f;
  const float ix = fminf(ax2, bx2) - fmaxf(ax1, bx1); // 交集宽
  const float iy = fminf(ay2, by2) - fmaxf(ay1, by1); // 交集高
  if (ix <= 0.0f || iy <= 0.0f) {
    return 0.0f;
  }
  const float inter = ix * iy;
  const float uni = aw * ah + bw * bh - inter;
  return uni > 0.0f ? inter / uni : 0.0f; // 防除零
}

// α-β 量测更新：以量测时刻为基准先预测再按残差修正位置/速度，尺寸做慢速 EMA
void MeasureUpdate(float cxm, float cym, float wm, float hm, int64_t ts_us) {
  const float dt = (float)(ts_us - s_last_meas_ts_us) / 1e6f;
  if (dt <= 0.001f) {
    // 首次量测或时间异常：直接采用量测值，速度归零
    s_cx = cxm;
    s_cy = cym;
    s_vx = 0.0f;
    s_vy = 0.0f;
    s_w = wm;
    s_h = hm;
  } else {
    const float px = s_cx + s_vx * dt; // 预测：上次量测时刻 -> 本次量测时刻
    const float py = s_cy + s_vy * dt;
    const float rx = cxm - px; // 位置残差
    const float ry = cym - py;
    s_cx = px + kAlpha * rx;
    s_cy = py + kAlpha * ry;
    s_vx = s_vx + kBeta * rx / dt;
    s_vy = s_vy + kBeta * ry / dt;
    s_vx = fmaxf(-kMaxSpeed, fminf(kMaxSpeed, s_vx)); // 速度限幅防发散
    s_vy = fmaxf(-kMaxSpeed, fminf(kMaxSpeed, s_vy));
    s_w = s_w + kSizeAlpha * (wm - s_w); // 尺寸慢速平滑
    s_h = s_h + kSizeAlpha * (hm - s_h);
  }
  s_last_meas_ts_us = ts_us;
}
} // namespace

void face_tracker_reset(void) {
  ResetTrack();
  s_last_seq = 0;
  s_last_tick_us = 0;
  s_last_result_ts_us = 0;
  memset(&s_out, 0, sizeof(s_out));
  ESP_LOGI(TAG, "tracker reset");
}

void face_tracker_update(const face_detect_result_t *res, int64_t now_us) {
  const face_track_state_t prev_state = s_state;
  int cs = -1; // 本帧命中分数（-1 = 本帧无命中），供状态转换日志使用

  // 1) 时间步进：按上一 tick 到本 tick 的时间差外推中心；PREDICTING 时衰减速度
  float dt_tick = 0.0f;
  if (s_last_tick_us != 0) {
    dt_tick = (float)(now_us - s_last_tick_us) / 1e6f;
    if (dt_tick < 0.0f) {
      dt_tick = 0.0f; // 时间乱序防御
    }
  }
  s_last_tick_us = now_us;

  if (s_state == FACE_TRACK_TRACKING || s_state == FACE_TRACK_PREDICTING) {
    s_cx = s_cx + s_vx * dt_tick;
    s_cy = s_cy + s_vy * dt_tick;
  }
  if (s_state == FACE_TRACK_PREDICTING) {
    s_vx = s_vx * 0.85f; // 每次 tick 衰减，抑制预测发散
    s_vy = s_vy * 0.85f;
  }
  if (s_state != FACE_TRACK_IDLE) {
    s_cx = fmaxf(0.0f, fminf(kFrameW, s_cx)); // 中心夹取到帧内
    s_cy = fmaxf(0.0f, fminf(kFrameH, s_cy));
  }

  // 2) 新结果判定：seq 递增视为新结果，刷新结果年龄基准
  const bool new_res =
      (res != nullptr && res->seq != 0 && res->seq != s_last_seq);
  if (new_res) {
    s_last_seq = res->seq;
    s_last_result_ts_us = res->ts_us;
  }

  // 3) 仅“新鲜”的新结果参与量测（超龄结果只影响年龄显示，不做滤波更新）
  if (new_res && (now_us - res->ts_us) <= kMaxMeasAgeUs) {
    // 3a) 选候选：已有框按 IoU/距离门限 + 组合权重择优；IDLE 取最高分
    bool has_best = false;
    float bm_cx = 0.0f;
    float bm_cy = 0.0f;
    float bm_w = 0.0f;
    float bm_h = 0.0f;
    int bm_score = 0;
    float bm_combo = -1.0f;
    const int count = (res->count < FACE_DETECT_MAX_FACES)
                          ? res->count
                          : FACE_DETECT_MAX_FACES;

    for (int i = 0; i < count; i++) {
      const face_box_t &b = res->boxes[i];
      const float w = (float)b.x2 - (float)b.x1;
      const float h = (float)b.y2 - (float)b.y1;
      if (w <= 0.0f || h <= 0.0f) {
        continue; // 非法框防御
      }
      const float cx = ((float)b.x1 + (float)b.x2) * 0.5f;
      const float cy = ((float)b.y1 + (float)b.y2) * 0.5f;

      if (s_state == FACE_TRACK_IDLE) {
        // 无既有框：直接取检测分数最高者
        if (!has_best || (int)b.score > bm_score) {
          has_best = true;
          bm_cx = cx;
          bm_cy = cy;
          bm_w = w;
          bm_h = h;
          bm_score = (int)b.score;
        }
        continue;
      }

      // 已有框：IoU/中心距双门限，再按加权组合分择优
      const float iou = IoU(cx, cy, w, h, s_cx, s_cy, s_w, s_h);
      const float dx = cx - s_cx;
      const float dy = cy - s_cy;
      const float dist = sqrtf(dx * dx + dy * dy);
      const float diag = sqrtf(s_w * s_w + s_h * s_h);
      const float dist_gate = fmaxf(kMatchDistMin, kMatchDistRatio * diag);
      if (iou < kMatchIoU && dist > dist_gate) {
        continue; // 不合格候选
      }
      const float inv_dist = 1.0f - fminf(1.0f, dist / (diag + 1e-3f));
      const float combo = kWScore * ((float)b.score / 100.0f) +
                          kWIoU * iou + kWDist * inv_dist;
      if (!has_best || combo > bm_combo) {
        has_best = true;
        bm_cx = cx;
        bm_cy = cy;
        bm_w = w;
        bm_h = h;
        bm_score = (int)b.score;
        bm_combo = combo;
      }
    }
    if (has_best) {
      cs = bm_score;
    }

    // 3b) 状态/命中更新
    if (s_state == FACE_TRACK_IDLE) {
      if (has_best && cs >= kAcquireScore) {
        // 首次 acquire：直接采用量测几何，速度清零，进入候选确认
        s_cx = bm_cx;
        s_cy = bm_cy;
        s_w = bm_w;
        s_h = bm_h;
        s_vx = 0.0f;
        s_vy = 0.0f;
        s_score = (uint8_t)cs;
        s_acq_hits = 1;
        s_state = FACE_TRACK_ACQUIRING;
        s_last_seen_us = now_us;
        s_last_meas_ts_us = res->ts_us;
      }
      // 否则维持 IDLE
    } else if (s_state == FACE_TRACK_ACQUIRING) {
      if (has_best && cs >= kAcquireScore) {
        MeasureUpdate(bm_cx, bm_cy, bm_w, bm_h, res->ts_us);
        s_score = (uint8_t)cs;
        s_acq_hits = s_acq_hits + 1;
        s_last_seen_us = now_us;
        if (s_acq_hits >= kAcquireHits) {
          s_state = FACE_TRACK_TRACKING; // 连续命中达标：正式跟踪
          s_miss = 0;
        }
      } else {
        AbandonAcquire(); // 确认失败：清候选框，等待下次重新 acquire
      }
    } else { // TRACKING / PREDICTING
      if (has_best && cs >= kTrackScore) {
        MeasureUpdate(bm_cx, bm_cy, bm_w, bm_h, res->ts_us);
        s_score = (uint8_t)cs;
        s_state = FACE_TRACK_TRACKING; // 命中即恢复跟踪
        s_miss = 0;
        s_last_seen_us = now_us;
      } else {
        s_miss = s_miss + 1;
        if (s_miss >= kLostMiss) {
          ResetTrack(); // 彻底丢失：整体回 IDLE
        } else if (s_miss >= kMissToPredict &&
                   s_state == FACE_TRACK_TRACKING) {
          s_state = FACE_TRACK_PREDICTING; // 短暂丢失：凭速度外推
        }
      }
    }
  }

  // 4) 老化降级：每 tick 判断（无论本帧是否有新结果）
  const int64_t seen_age_us = now_us - s_last_seen_us;
  if (s_state == FACE_TRACK_ACQUIRING && seen_age_us > kStaleToPredictUs) {
    AbandonAcquire(); // 确认期长时间无新证据：放弃
  } else if (s_state == FACE_TRACK_TRACKING &&
             seen_age_us > kStaleToPredictUs) {
    s_state = FACE_TRACK_PREDICTING; // 跟踪超时未更新：转预测
  } else if (s_state == FACE_TRACK_PREDICTING &&
             seen_age_us > kLostStaleUs) {
    ResetTrack(); // 预测超时：彻底丢失
  }

  // 5) 输出装配：坐标 = round(中心 ± 尺寸/2)，int16 输出
  s_out.state = s_state;
  s_out.valid =
      (s_state == FACE_TRACK_TRACKING || s_state == FACE_TRACK_PREDICTING);
  s_out.x1 = (int16_t)roundf(s_cx - s_w * 0.5f);
  s_out.y1 = (int16_t)roundf(s_cy - s_h * 0.5f);
  s_out.x2 = (int16_t)roundf(s_cx + s_w * 0.5f);
  s_out.y2 = (int16_t)roundf(s_cy + s_h * 0.5f);
  s_out.score = s_score;
  s_out.miss = s_miss;
  s_out.vx = s_vx;
  s_out.vy = s_vy;
  if (s_last_result_ts_us == 0) {
    s_out.age_ms = -1; // 尚无任何结果
  } else {
    const int64_t age_ms = (now_us - s_last_result_ts_us) / 1000;
    s_out.age_ms = (age_ms < 0) ? 0 : (int)age_ms; // 乱序负数取 0
  }

  // 6) 状态转换日志：仅在切换时打印（含 miss/cs 便于调参）
  if (s_state != prev_state) {
    switch (s_state) {
      case FACE_TRACK_ACQUIRING:
        ESP_LOGI(TAG, "track %s -> ACQUIRING (score=%d)", StateName(prev_state),
                 cs);
        break;
      case FACE_TRACK_TRACKING:
        ESP_LOGI(TAG, "track %s -> TRACKING (score=%d)", StateName(prev_state),
                 cs);
        break;
      case FACE_TRACK_PREDICTING:
        ESP_LOGI(TAG, "track %s -> PREDICTING (miss=%d)",
                 StateName(prev_state), s_miss);
        break;
      default: // FACE_TRACK_IDLE
        ESP_LOGI(TAG, "track %s -> IDLE (lost)", StateName(prev_state));
        break;
    }
  }
}

const face_track_out_t *face_tracker_get(void) { return &s_out; }
