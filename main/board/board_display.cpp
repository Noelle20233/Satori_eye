/**
 ****************************************************************************************************
 * @file        board_display.cpp
 * @brief       板级显示 bring-up（原 main.cpp 的 InitLCD / InitLVGL）
 ****************************************************************************************************
 */

#include "board_display.h"

#include <cstdlib>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_log.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "app_config.h"
#include "app_context.h"
#include "xl9555.h"

static constexpr const char *TAG = "BOARD";

// ---------------------------------------------------------------- LCD
void board_lcd_init(void) {
  // 初始化 I2C 与 XL9555（LCD 电源/复位/背光均由 XL9555 控制）
  xl9555_init();

  // 打开 LCD 电源/背光（PWR 拉高，模块背光点亮）
  xl9555_pin_write(SLCD_PWR_IO, 1);
  vTaskDelay(pdMS_TO_TICKS(50));

  // LCD 硬件复位：RST 拉低(≥10us) -> 拉高 -> 等待 120ms 复位完成
  xl9555_pin_write(SLCD_RST_IO, 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  xl9555_pin_write(SLCD_RST_IO, 1);
  vTaskDelay(pdMS_TO_TICKS(120));

  // spi初始化
  spi_bus_config_t buscfg = {};
  buscfg.sclk_io_num = PIN_NUM_SCLK;
  buscfg.mosi_io_num = PIN_NUM_MOSI;
  buscfg.miso_io_num = PIN_NUM_MISO;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.data4_io_num = -1;
  buscfg.data5_io_num = -1;
  buscfg.data6_io_num = -1;
  buscfg.data7_io_num = -1;
  buscfg.flags = 0;
  buscfg.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;
  buscfg.max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t);

  ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
  ESP_LOGI(TAG, "SPI Initialized.");

  // panel io初始化
  esp_lcd_panel_io_spi_config_t io_config = {};
  io_config.dc_gpio_num = PIN_NUM_DC; // 40
  io_config.cs_gpio_num = PIN_NUM_CS; // 21
  // 80MHz（最终配置）：写屏窗口约 15ms（≈1 个面板扫描场），
  // 撕裂单次错位幅度最小、运动最流畅；超出 ST7789V 标称上限（62.5MHz）但实测稳定
  io_config.pclk_hz = 80 * 1000 * 1000;
  io_config.lcd_cmd_bits = 8;
  io_config.lcd_param_bits = 8;
  io_config.spi_mode = 0;
  io_config.trans_queue_depth = 10;

  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                           &io_config, &g_app.io_handle));
  ESP_LOGI(TAG, "Panel IO Initialized.");

  // 面板初始化
  esp_lcd_panel_dev_config_t panel_config = {};
  panel_config.reset_gpio_num =
      PIN_NUM_RST; // NC：复位由 XL9555 控制，不占用 GPIO
  // 正点原子 2.4 寸屏实测：需 RGB 顺序（BGR 顺序会导致红蓝通道互换）
  panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
  panel_config.bits_per_pixel = 16;

  ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(g_app.io_handle, &panel_config,
                                           &g_app.panel_handle));
  ESP_LOGI(TAG, "Panel Initialized.");

  esp_lcd_panel_reset(g_app.panel_handle);
  esp_lcd_panel_init(g_app.panel_handle);

  // 正点原子 2.4 寸屏：RGB 颜色顺序 + 开启颜色反转，两者配合颜色才正确
  esp_lcd_panel_invert_color(g_app.panel_handle, true);

  // 注意：横屏方向（swap_xy/mirror）统一在 board_lvgl_init 的 rotation 配置中设置
  // （lvgl_port_add_disp 会把该配置应用到面板），避免两处配置不一致。

  esp_lcd_panel_disp_on_off(g_app.panel_handle, true);
  return;
}

void board_lvgl_init(void) {
  // 1. 初始化 LVGL 移植层：负责 lv_tick 时钟、lv_timer_handler 任务与互斥锁
  const lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
  ESP_ERROR_CHECK(lvgl_port_init(&port_cfg));

  // 2. 把 esp_lcd 面板注册为 LVGL 显示设备
  lvgl_port_display_cfg_t disp_cfg = {};
  disp_cfg.io_handle = g_app.io_handle;
  disp_cfg.panel_handle = g_app.panel_handle;
  disp_cfg.buffer_size = LCD_H_RES * LVGL_BUFFER_ROWS; // 单位：像素
  disp_cfg.double_buffer = true;
  disp_cfg.hres = LCD_H_RES;
  disp_cfg.vres = LCD_V_RES;
  disp_cfg.monochrome = false;
  disp_cfg.color_format = LV_COLOR_FORMAT_RGB565;
  // 与 esp_lcd 初始方向一致：2.4 寸屏横屏（MADCTL：MX=1、MV=1）
  disp_cfg.rotation.swap_xy = true;
  disp_cfg.rotation.mirror_x = true;
  disp_cfg.rotation.mirror_y = false;
  disp_cfg.flags.buff_dma = true; // 缓冲分配在内部 SRAM，可直接 DMA
  // SPI 屏必须交换 RGB565 字节序：否则颜色错乱
  // （例如深色背景 0x1A1A2E 会显示成亮紫色）
  disp_cfg.flags.swap_bytes = true;
  disp_cfg.flags.full_refresh = false;

  g_app.disp_handle = lvgl_port_add_disp(&disp_cfg);
  if (g_app.disp_handle == NULL) {
    ESP_LOGE(TAG, "Failed to add LVGL display.");
    abort();
  }

  ESP_LOGI(TAG, "LVGL initialized (%d x %d).", LCD_H_RES, LCD_V_RES);
}
