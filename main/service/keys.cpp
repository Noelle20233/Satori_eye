/**
 ****************************************************************************************************
 * @file        keys.cpp
 * @brief       按键扫描与去抖实现（原 main.cpp 的 KeyLoop）
 ****************************************************************************************************
 */

#include "keys.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"

#include "app.h"
#include "app_context.h"
#include "face_detect.h"
#include "xl9555.h"

static constexpr const char *TAG = "KEYS";

// 按键扫描（20ms 周期）：BOOT(GPIO0) + XL9555 KEY0~KEY3（均低有效）。
// 映射：BOOT=OSD 常显开关、KEY0=MSR、KEY1=MNP、KEY2=NMS、KEY3=检测开关。
// 去抖：每键一个 -3..3 计数（按下 +1、松开 -1），>=3 触发并按锁存标志
// 防重复；松开到 <=-3 后重新武装。I2C 读失败时跳过 XL9555 4 键本 tick
// 的去抖更新（不伪造松开，避免假边缘），BOOT 照常处理。
void keys_task(void *arg) {
  (void)arg;
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
          g_app.osd_always_show = !g_app.osd_always_show;
          ESP_LOGI(TAG, "Thr OSD %s", g_app.osd_always_show ? "ON" : "OFF");
        } else if (i == 1) {
          app_cycle_thr(0); // KEY0: MSR
        } else if (i == 2) {
          app_cycle_thr(1); // KEY1: MNP
        } else if (i == 3) {
          app_cycle_thr(2); // KEY2: NMS
        } else {
          // KEY3：检测开关，与串口 '5' 命令互通
          g_app.detect_enabled = !g_app.detect_enabled;
          face_detect_set_enabled(g_app.detect_enabled);
        }
      } else if (latched[i] && stable[i] <= -3) {
        latched[i] = false; // 键已松开：重新武装
      }
    }
  }
}
