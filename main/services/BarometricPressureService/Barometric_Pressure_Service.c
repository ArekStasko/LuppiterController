#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "bmp3.h"
#include "bmp3_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#define I2C_MASTER_SCL_IO           18
#define I2C_MASTER_SDA_IO           23
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define BMP388_I2C_ADDR             0x76

static struct bmp3_dev dev;
static uint8_t i2c_addr = BMP388_I2C_ADDR;

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

void enable_barometric_pressure_service(void)
{
    dev.intf = BMP3_I2C_INTF;
    dev.read = bmp388_i2c_read;
    dev.write = bmp388_i2c_write;
    dev.delay_us = bmp388_delay_us;
    dev.intf_ptr = &i2c_addr;

    if (bmp3_init(&dev) != BMP3_OK) {
        printf("Something went wrong\n");
    }

    struct bmp3_settings settings = { 0 };
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
    settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
 	settings.op_mode = BMP3_MODE_NORMAL;

    uint32_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN |
                            BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS |
                            BMP3_SEL_IIR_FILTER | BMP3_SEL_FIFO_MODE;

    bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    bmp3_set_op_mode(&settings, &dev);
}


void init_barometric_pressure_service(void)
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
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    enable_barometric_pressure_service();
}

float pressure_to_altitude(float pressure_hPa, float sea_level_pressure_hPa)
{
    return 44330.0 * (1.0 - pow(pressure_hPa / sea_level_pressure_hPa, 0.1903));
}

struct bmp3_data get_barometric_pressure_data(void)
{
    struct bmp3_data data;
	bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
    return data;
}

double get_temperature(void)
{
    struct bmp3_data data = get_barometric_pressure_data();
    return data.temperature;
}

double get_barometric_pressure(void)
{
    struct bmp3_data data = get_barometric_pressure_data();
    return data.pressure / 100.0;
}

float get_altitude(void)
{
    struct bmp3_data data = get_barometric_pressure_data();
    float altitude = pressure_to_altitude(data.pressure / 100.0, 1013.25);
    return altitude;
}