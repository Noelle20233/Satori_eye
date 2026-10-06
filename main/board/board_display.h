/**
 ****************************************************************************************************
 * @file        board_display.h
 * @brief       板级显示bring-up：SPI/ST7789 面板初始化 与 esp_lvgl_port 显示注册
 * @note        初始化结果写入共享上下文 g_app（io_handle/panel_handle/disp_handle）
 ****************************************************************************************************
 */

#pragma once

// ST7789 SPI 面板 bring-up：XL9555 电源/复位时序 + SPI 总线 + panel IO + 面板
void board_lcd_init(void);

// 初始化 LVGL 移植层并把面板注册为 LVGL 显示设备（依赖 board_lcd_init 的句柄）
void board_lvgl_init(void);
