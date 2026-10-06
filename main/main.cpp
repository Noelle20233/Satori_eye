/**
 ****************************************************************************************************
 * @file        main.cpp
 * @brief       应用入口：仅做装配调用，具体逻辑按层级拆分至 app/ board/ ui/ pipeline/ service/
 * @note        分层一览：
 *                app/       配置宏、共享上下文、初始化编排与后台任务创建、主监控循环
 *                board/     板级显示 bring-up（SPI/ST7789 + esp_lvgl_port）
 *                ui/        预览 UI 与人脸框叠加绘制、阈值 OSD
 *                pipeline/  摄像头预览流水线（采集->展开->检测提交->渲染）
 *                service/   面板守护、串口控制台、按键交互
 *                drivers/   外设驱动（camera / xl9555 / myiic）
 *                vision/    视觉算法（face_detect / face_tracker）
 ****************************************************************************************************
 */

#include "app.h"

extern "C" void app_main() {
  app_init();
  app_run();
}
