#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "bmp3.h"
#include "bmp3_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO           18
#define I2C_MASTER_SDA_IO           23
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define BMP388_I2C_ADDR             0x76

esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

int8_t bmp388_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    return i2c_master_write_read_device(I2C_MASTER_NUM, dev_addr, &reg_addr, 1, reg_data, len, 1000 / portTICK_PERIOD_MS) == ESP_OK ? BMP3_OK : BMP3_E_COMM_FAIL;
}

int8_t bmp388_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    uint8_t data[len + 1];
    data[0] = reg_addr;
    memcpy(&data[1], reg_data, len);
    return i2c_master_write_to_device(I2C_MASTER_NUM, dev_addr, data, len + 1, 1000 / portTICK_PERIOD_MS) == ESP_OK ? BMP3_OK : BMP3_E_COMM_FAIL;
}

void bmp388_delay_us(uint32_t period, void *intf_ptr)
{
    vTaskDelay(period / 1000 / portTICK_PERIOD_MS);
}

void barometric_pressure_task(void *pvParameters)
{
    ESP_ERROR_CHECK(i2c_master_init());

    struct bmp3_dev dev;
    uint8_t i2c_addr = BMP388_I2C_ADDR;

    dev.intf = BMP3_I2C_INTF;
    dev.read = bmp388_i2c_read;
    dev.write = bmp388_i2c_write;
    dev.delay_us = bmp388_delay_us;
    dev.intf_ptr = &i2c_addr;

    if (bmp3_init(&dev) != BMP3_OK) {
        printf("Something went wrong\n");
        vTaskDelete(NULL);
    }

    struct bmp3_settings settings = { 0 };
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
    settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;

    uint32_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN |
                        BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS |
                        BMP3_SEL_ODR;

    bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    bmp3_set_op_mode(&settings, &dev);

    struct bmp3_data data;

    while (1) {
        if (bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev) == BMP3_OK) {
            printf("🌡Temperatura: %.2f °C | ⬇Ciśnienie: %.2f hPa\n",
                   data.temperature, data.pressure / 100.0);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // odczyt co 1 sek
    }
}

void enable_barometric_pressure_service(void)
{
    xTaskCreate(barometric_pressure_task, "barometric_pressure_task", 4096, NULL, 5, NULL);
}