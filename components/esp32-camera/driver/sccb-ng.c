/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver with GPIO bit-banging.
 *
 * This is a drop-in replacement for the ESP-IDF hardware I2C master
 * implementation. On the ATK-MC2640 module the hardware I2C master driver
 * produced persistent NACKs (ESP_ERR_INVALID_RESPONSE) on every read
 * transaction, which wedged esp_camera_init(). The OV2640 datasheet defines
 * reads as a two-phase sequence, each phase terminated by a STOP:
 *   phase 1: START + ID(W) + sub-address + X + STOP
 *   phase 2: START + ID(R) + data + NA + STOP
 * Driving SDA/SCL manually keeps full control of the bus timing and issues
 * exactly this sequence.
 */
#include <stdbool.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sccb.h"
#include "sensor.h"
#include <stdio.h>
#include "sdkconfig.h"
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char *TAG = "sccb-ng";
#endif

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

#if CONFIG_SCCB_HARDWARE_I2C_PORT1
const int SCCB_I2C_PORT_DEFAULT = 1;
#else
const int SCCB_I2C_PORT_DEFAULT = 0;
#endif

#define MAX_DEVICES UINT8_MAX-1

/* Half-bit delay. 2us per half period -> ~200-250kHz, well below the
 * 400kHz SCCB limit while staying robust on jumper wires. */
#define SCCB_DELAY_US 2

/*
 The legacy I2C driver used addresses to differentiate between devices, and
 the hardware driver kept a bus-registered handle per device. The bit-bang
 driver talks to the bus directly, so only the address list is kept to
 preserve the original "device must be probed before use" semantics.
*/
typedef struct
{
    uint16_t address;
} device_t;

static device_t devices[MAX_DEVICES];
static uint8_t device_count = 0;
static int sccb_i2c_port;
static bool sccb_owns_i2c_port;

static int sccb_sda_pin = -1;
static int sccb_scl_pin = -1;

static int get_device_index(uint8_t slv_addr)
{
    for (uint8_t i = 0; i < device_count; i++)
    {
        if (slv_addr == devices[i].address)
        {
            return i;
        }
    }

    ESP_LOGE(TAG, "Device with address %02x not found", slv_addr);
    return -1;
}

int SCCB_Install_Device(uint8_t slv_addr)
{
    if (device_count >= MAX_DEVICES)
    {
        ESP_LOGE(TAG, "cannot add more than %d devices", MAX_DEVICES);
        return ESP_FAIL;
    }

    devices[device_count].address = slv_addr;
    device_count++;
    return ESP_OK;
}

static inline void sccb_delay(void)
{
    esp_rom_delay_us(SCCB_DELAY_US);
}

/* Open-drain pins: driving 1 releases the line, 0 pulls it low. */
static inline void sccb_sda_out(int level)
{
    gpio_set_level(sccb_sda_pin, level);
}

static inline void sccb_scl_out(int level)
{
    gpio_set_level(sccb_scl_pin, level);
}

static inline int sccb_sda_in(void)
{
    return gpio_get_level(sccb_sda_pin);
}

static void sccb_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << sccb_sda_pin) | (1ULL << sccb_scl_pin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    sccb_sda_out(1);
    sccb_scl_out(1);
    sccb_delay();

    /* If a previous transaction was interrupted the sensor may still hold
     * SDA low; clock it out so the bus returns to idle. */
    if (sccb_sda_in() == 0)
    {
        for (int i = 0; i < 9; i++)
        {
            sccb_scl_out(0);
            sccb_delay();
            sccb_scl_out(1);
            sccb_delay();
        }
        sccb_scl_out(0);
        sccb_delay();
        sccb_scl_out(1);
        sccb_delay();
    }
}

static void sccb_start(void)
{
    sccb_sda_out(1);
    sccb_scl_out(1);
    sccb_delay();
    sccb_sda_out(0); /* SDA falls while SCL is high */
    sccb_delay();
    sccb_scl_out(0);
    sccb_delay();
}

static void sccb_stop(void)
{
    sccb_sda_out(0);
    sccb_delay();
    sccb_scl_out(1);
    sccb_delay();
    sccb_sda_out(1); /* SDA rises while SCL is high */
    sccb_delay();
}

/* Returns 0 when the slave acknowledged, -1 on NACK. */
static int sccb_write_byte(uint8_t data)
{
    for (int i = 7; i >= 0; i--)
    {
        sccb_sda_out((data >> i) & 0x1);
        sccb_delay();
        sccb_scl_out(1);
        sccb_delay();
        sccb_scl_out(0);
        sccb_delay();
    }

    /* Release SDA and sample the acknowledge bit driven by the slave. */
    sccb_sda_out(1);
    sccb_delay();
    sccb_scl_out(1);
    sccb_delay();
    int ack = sccb_sda_in();
    sccb_scl_out(0);
    sccb_delay();

    return ack == 0 ? 0 : -1;
}

/* ack != 0 drives ACK (low) for the ninth clock, else NAK (high). */
static uint8_t sccb_read_byte(int ack)
{
    uint8_t data = 0;

    sccb_sda_out(1); /* host releases SDA */
    for (int i = 7; i >= 0; i--)
    {
        sccb_delay();
        sccb_scl_out(1);
        sccb_delay();
        if (sccb_sda_in())
        {
            data |= (1 << i);
        }
        sccb_scl_out(0);
        sccb_delay();
    }

    sccb_sda_out(ack ? 0 : 1);
    sccb_delay();
    sccb_scl_out(1);
    sccb_delay();
    sccb_scl_out(0);
    sccb_delay();
    sccb_sda_out(1); /* release again */

    return data;
}

/* Best-effort bus recovery: if the slave holds SDA low (aborted transfer,
 * desynchronized state machine), clock it out and issue a STOP so both
 * sides return to idle. */
static void sccb_bus_recover(void)
{
    sccb_sda_out(1);
    sccb_scl_out(0);
    sccb_delay();

    int pulses = 0;
    while (sccb_sda_in() == 0 && pulses < 9)
    {
        sccb_scl_out(1);
        sccb_delay();
        sccb_scl_out(0);
        sccb_delay();
        pulses++;
    }

    /* STOP: SDA low -> SCL high -> SDA high */
    sccb_sda_out(0);
    sccb_delay();
    sccb_scl_out(1);
    sccb_delay();
    sccb_sda_out(1);
    sccb_delay();

    ESP_LOGW(TAG, "sccb bus recover: %d pulses, sda idle level=%d", pulses, sccb_sda_in());
}

int SCCB_Init(int pin_sda, int pin_scl)
{
    ESP_LOGI(TAG, "pin_sda %d pin_scl %d (bit-bang)", pin_sda, pin_scl);

    if (pin_sda < 0 || pin_scl < 0)
    {
        ESP_LOGE(TAG, "invalid SCCB pins sda=%d scl=%d", pin_sda, pin_scl);
        return ESP_ERR_INVALID_ARG;
    }

    sccb_sda_pin = pin_sda;
    sccb_scl_pin = pin_scl;
    sccb_i2c_port = SCCB_I2C_PORT_DEFAULT;
    sccb_owns_i2c_port = true;
    ESP_LOGI(TAG, "sccb_i2c_port=%d", sccb_i2c_port);

    sccb_gpio_init();

    return ESP_OK;
}

int SCCB_Use_Port(int i2c_num)
{ /* the bit-bang driver has no hardware port to reuse */
    ESP_LOGW(TAG, "SCCB_Use_Port(%d) ignored, bit-bang bus uses the SCCB_Init pins", i2c_num);

    if (sccb_sda_pin < 0 || sccb_scl_pin < 0)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (sccb_owns_i2c_port)
    {
        SCCB_Deinit();
    }
    sccb_i2c_port = i2c_num;
    sccb_owns_i2c_port = false;
    return ESP_OK;
}

int SCCB_Deinit(void)
{
    for (uint8_t i = 0; i < device_count; i++)
    {
        devices[i].address = 0;
    }
    device_count = 0;

    if (sccb_sda_pin >= 0 && sccb_scl_pin >= 0)
    {
        sccb_sda_out(1);
        sccb_scl_out(1);
    }
    sccb_owns_i2c_port = false;

    return ESP_OK;
}

int SCCB_Probe(uint8_t slv_addr)
{
    sccb_start();
    int ret = sccb_write_byte((slv_addr << 1) | 0);
    sccb_stop();

    if (ret != 0)
    {
        return ESP_ERR_NOT_FOUND;
    }

    return SCCB_Install_Device(slv_addr);
}

/* Single SCCB read transaction. *ok is set to 0 on failure and *phase
 * points to "addr" or "data" depending on which phase failed. */
static uint8_t sccb_read_transaction(uint8_t slv_addr, uint8_t reg, int *ok, const char **phase)
{
    /* Phase 1: START + ID(W) + sub-address + X + STOP */
    sccb_start();
    if (sccb_write_byte((slv_addr << 1) | 0) != 0 || sccb_write_byte(reg) != 0)
    {
        sccb_stop();
        *ok = 0;
        *phase = "addr";
        return 0;
    }
    sccb_stop();

    /* Phase 2: START + ID(R) + data + NA + STOP */
    sccb_start();
    if (sccb_write_byte((slv_addr << 1) | 1) != 0)
    {
        sccb_stop();
        *ok = 0;
        *phase = "data";
        return 0;
    }
    uint8_t data = sccb_read_byte(0 /* NA */);
    sccb_stop();

    *ok = 1;
    *phase = "ok";
    return data;
}

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    if (get_device_index(slv_addr) < 0)
    {
        return 0;
    }

    int ok = 0;
    const char *phase = "ok";
    uint8_t data = sccb_read_transaction(slv_addr, reg, &ok, &phase);
    if (ok)
    {
        return data;
    }

    /* The transaction failed (slave NACK or bus stuck). Try to bring the
     * bus back to idle and retry once before giving up. */
    ESP_LOGD(TAG, "SCCB_Read %s phase failed addr:0x%02x, reg:0x%02x; recovering bus",
             phase, slv_addr, reg);
    sccb_bus_recover();

    data = sccb_read_transaction(slv_addr, reg, &ok, &phase);
    if (!ok)
    {
        ESP_LOGE(TAG, "SCCB_Read %s phase failed addr:0x%02x, reg:0x%02x (after retry)",
                 phase, slv_addr, reg);
        return 0;
    }

    ESP_LOGW(TAG, "SCCB_Read addr:0x%02x, reg:0x%02x recovered on retry", slv_addr, reg);
    return data;
}

int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    if (get_device_index(slv_addr) < 0)
    {
        return -1;
    }

    sccb_start();
    int ret = sccb_write_byte((slv_addr << 1) | 0);
    if (ret == 0)
    {
        ret = sccb_write_byte(reg);
    }
    if (ret == 0)
    {
        ret = sccb_write_byte(data);
    }
    sccb_stop();

    if (ret != 0)
    {
        ESP_LOGE(TAG, "SCCB_Write Failed addr:0x%02x, reg:0x%02x, data:0x%02x", slv_addr, reg, data);
    }

    return ret;
}

uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg)
{
    if (get_device_index(slv_addr) < 0)
    {
        return 0;
    }

    sccb_start();
    int ret = sccb_write_byte((slv_addr << 1) | 0);
    if (ret == 0)
    {
        ret = sccb_write_byte(reg >> 8);
    }
    if (ret == 0)
    {
        ret = sccb_write_byte(reg & 0x00FF);
    }
    sccb_stop();

    if (ret != 0)
    {
        ESP_LOGE(TAG, "W [%04x] fail\n", reg);
        return 0;
    }

    sccb_start();
    if (sccb_write_byte((slv_addr << 1) | 1) != 0)
    {
        sccb_stop();
        ESP_LOGE(TAG, "W [%04x] fail\n", reg);
        return 0;
    }
    uint8_t data = sccb_read_byte(0 /* NA */);
    sccb_stop();

    return data;
}

int SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data)
{
    if (get_device_index(slv_addr) < 0)
    {
        return -1;
    }

    sccb_start();
    int ret = sccb_write_byte((slv_addr << 1) | 0);
    if (ret == 0)
    {
        ret = sccb_write_byte(reg >> 8);
    }
    if (ret == 0)
    {
        ret = sccb_write_byte(reg & 0x00FF);
    }
    if (ret == 0)
    {
        ret = sccb_write_byte(data);
    }
    sccb_stop();

    if (ret != 0)
    {
        ESP_LOGE(TAG, "W [%04x]=%02x fail\n", reg, data);
    }

    return ret;
}

uint16_t SCCB_Read_Addr16_Val16(uint8_t slv_addr, uint16_t reg)
{
    if (get_device_index(slv_addr) < 0)
    {
        return 0;
    }

    sccb_start();
    int ret = sccb_write_byte((slv_addr << 1) | 0);
    if (ret == 0)
    {
        ret = sccb_write_byte(reg >> 8);
    }
    if (ret == 0)
    {
        ret = sccb_write_byte(reg & 0x00FF);
    }
    sccb_stop();

    if (ret != 0)
    {
        ESP_LOGE(TAG, "W [%04x] fail\n", reg);
        return 0;
    }

    sccb_start();
    if (sccb_write_byte((slv_addr << 1) | 1) != 0)
    {
        sccb_stop();
        ESP_LOGE(TAG, "W [%04x] fail\n", reg);
        return 0;
    }
    uint8_t rx_buffer[2];
    rx_buffer[0] = sccb_read_byte(1 /* ACK */);
    rx_buffer[1] = sccb_read_byte(0 /* NA */);
    sccb_stop();

    return ((uint16_t)rx_buffer[0] << 8) | (uint16_t)rx_buffer[1];
}

int SCCB_Write_Addr16_Val16(uint8_t slv_addr, uint16_t reg, uint16_t data)
{
    if (get_device_index(slv_addr) < 0)
    {
        return -1;
    }

    sccb_start();
    int ret = sccb_write_byte((slv_addr << 1) | 0);
    if (ret == 0)
    {
        ret = sccb_write_byte(reg >> 8);
    }
    if (ret == 0)
    {
        ret = sccb_write_byte(reg & 0x00FF);
    }
    if (ret == 0)
    {
        ret = sccb_write_byte(data >> 8);
    }
    if (ret == 0)
    {
        ret = sccb_write_byte(data & 0x00FF);
    }
    sccb_stop();

    if (ret != 0)
    {
        ESP_LOGE(TAG, "W [%04x]=%02x fail\n", reg, data);
    }

    return ret;
}
