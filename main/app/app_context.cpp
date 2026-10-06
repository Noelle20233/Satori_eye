/**
 ****************************************************************************************************
 * @file        app_context.cpp
 * @brief       全局应用状态实例定义与共享的枚举->字符串辅助
 ****************************************************************************************************
 */

#include "app_context.h"

AppContext g_app;

const char *TrkStateStr(face_track_state_t st) {
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
