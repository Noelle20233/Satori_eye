/**
 ****************************************************************************************************
 * @file        panel_guard.cpp
 * @brief       面板/电源守护实现（原 main.cpp 的 HealLoop + GuardXl9555 + PanelHeal）
 ****************************************************************************************************
 */

#include "panel_guard.h"

#include <cinttypes>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_lcd_panel_ops.h"
#include "esp_log.h"

#include "esp_lvgl_port.h"

#include "app_context.h"
#include "xl9555.h"

static constexpr const char *TAG = "GUARD";

// XL9555 守护：回读方向配置寄存器与关键输出电平（仅读，无副作用）。
// 芯片受扰/掉电复位会导致输出寄存器丢失或整体复位成输入态，
// 此时重新配置并恢复电平；I2C 瞬时读失败不当作芯片异常，避免误写。
static void GuardXl9555(void) {
  uint8_t cfg[2] = {0xFF, 0xFF};
  esp_err_t err = xl9555_read_reg(XL9555_CONFIG_PORT0_REG, cfg, 2);
  if (err != ESP_OK) {
    return;
  }
  bool bad = (cfg[0] != 0x03 || cfg[1] != 0xF0) ||
             (xl9555_pin_read(SLCD_PWR_IO) != 1) ||
             (xl9555_pin_read(SLCD_RST_IO) != 1) ||
             (xl9555_pin_read(OV_PWDN_IO) != 0) ||
             (xl9555_pin_read(OV_RESET_IO) != 1);
  if (bad) {
    g_app.xl9555_anomaly_count = g_app.xl9555_anomaly_count + 1;
    ESP_LOGW(TAG, "XL9555 anomaly #%" PRIu32 " (cfg=%02X,%02X), restoring",
             (uint32_t)g_app.xl9555_anomaly_count, cfg[0], cfg[1]);
    // 直接写配置寄存器 {P0=0x03,P1=0xF0}（等价 xl9555_ioconfig(0xF003)，
    // 但不带其重试死循环，避免 I2C 永久故障时守护任务被卡死）
    uint8_t cfg_fix[2] = {0x03, 0xF0};
    if (xl9555_write_byte(XL9555_CONFIG_PORT0_REG, cfg_fix, 2) != ESP_OK) {
      ESP_LOGE(TAG, "XL9555 config restore failed");
    }
    xl9555_pin_write(SLCD_PWR_IO, 1);
    xl9555_pin_write(SLCD_RST_IO, 1);
    xl9555_pin_write(OV_PWDN_IO, 0);
    xl9555_pin_write(OV_RESET_IO, 1);
  }
}

// ST7789 面板状态自愈：重发初始化命令序列（SLPOUT/MADCTL/COLMOD/RAMCTRL
// + 旋转 + 颜色反转 + DISPON）。所有命令幂等且不触碰 GRAM，正常状态下
// 无感；面板若被误置为睡眠/显示关闭/配置漂移可在周期内恢复。
// 注意：必须在 LVGL 锁内执行——esp_lcd 的 SPI 事务不可并发，否则
// acquire bus 失败会丢掉 flush 完成回调，使刷新等待永久卡死。
void panel_heal(bool hard_reset) {
  if (!lvgl_port_lock(2000)) {
    ESP_LOGW(TAG, "PanelHeal: LVGL lock timeout, skipped");
    return;
  }
  if (hard_reset) {
    // 与 board_lcd_init 一致的硬件复位时序：RST 低 10ms -> 高 -> 120ms
    xl9555_pin_write(SLCD_RST_IO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    xl9555_pin_write(SLCD_RST_IO, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
  }
  esp_lcd_panel_init(g_app.panel_handle);          // SLPOUT + MADCTL(含旋转位) + COLMOD + RAMCTRL
  esp_lcd_panel_swap_xy(g_app.panel_handle, true); // 与 board_lvgl_init 的 rotation 配置一致
  esp_lcd_panel_mirror(g_app.panel_handle, true, false);
  esp_lcd_panel_invert_color(g_app.panel_handle, true);
  esp_lcd_panel_disp_on_off(g_app.panel_handle, true);
  g_app.heal_count = g_app.heal_count + 1;
  lvgl_port_unlock();
}

void panel_guard_task(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "Panel guard started (soft heal every 30s).");
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(30000));
    GuardXl9555();
    if (g_app.auto_heal_enabled) {
      panel_heal(false);
    }
  }
}
