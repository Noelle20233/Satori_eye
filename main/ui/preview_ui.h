/**
 ****************************************************************************************************
 * @file        preview_ui.h
 * @brief       预览显示层：LVGL 预览控件创建 + 人脸框叠加绘制 + 阈值 OSD 文本
 * @note        UI 对象与显示缓冲存于共享上下文 g_app；绘制函数直接写 preview_buf
 ****************************************************************************************************
 */

#pragma once

#include "face_detect.h"
#include "face_tracker.h"

// 创建预览图像与 FPS/检测/阈值 OSD 覆盖层，并分配 320x240 显示缓冲。
// 内部自行加 LVGL 锁（依赖 board_lvgl_init 已完成）
void preview_ui_create(void);

// 刷新阈值/跟踪 OSD 文本 —— 必须在 LVGL 锁内调用
void preview_ui_update_thr_label(void);

// 画跟踪目标框（TRACKING 绿 / PREDICTING 黄）—— 必须在 LVGL 锁外调用
void preview_ui_draw_track_target(const face_track_out_t *trk);

// 画非跟踪目标的其余人脸（1px 绿框）—— 必须在 LVGL 锁外调用
void preview_ui_draw_other_faces(const face_detect_result_t *det,
                                 const face_track_out_t *trk);
