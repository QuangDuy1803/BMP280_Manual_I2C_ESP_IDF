#include "driver/i2c.h"
#include <cstring>

#define I2C_MASTER_SCL_IO 12
#define I2C_MASTER_SDA_IO 13
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define BMP280_ADDRESS 0x77
#define BMP280_DATASHEET "https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf"

void setupI2C();
void WriteReg8(uint8_t reg, uint8_t val);
uint8_t ReadReg8(uint8_t reg);
uint16_t ReadReg16_LE(uint8_t reg);
uint32_t ReadReg20_BE(uint8_t reg);

void MeasureTemp(void *pvParam);

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("I2C Starting..\n");

    setupI2C();

    xTaskCreatePinnedToCore(MeasureTemp, "MeasureTemp", 2048, NULL, 1, NULL, 1);
}

void MeasureTemp(void *pvParam)
{
    for (;;)
    {
        uint16_t dig_T1 = ReadReg16_LE(0x88);
        int16_t dig_T2 = ReadReg16_LE(0x8A);
        int16_t dig_T3 = ReadReg16_LE(0x8C);

        // Write to take oversampling
        WriteReg8(0xF4, 0b10010011);
        vTaskDelay(pdMS_TO_TICKS(10));

        int32_t adc_T = ReadReg20_BE(0xFA);
        int32_t var1 = ((double)adc_T / 16384 - (double)dig_T1 / 1024) * (double)dig_T2;
        int32_t var2 = (((double)adc_T / 131072 - (double)dig_T1 / 8912) * ((double)adc_T / 131072 - (double)dig_T1 / 8912)) * (double)dig_T3;
        double temp = ((var1 + var2) / 5120.0);

        printf("TEMP: %.2f\n", (temp));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setupI2C()
{
    i2c_config_t config = {};
    memset(&config, 0, sizeof(config));

    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = I2C_MASTER_SDA_IO;
    config.scl_io_num = I2C_MASTER_SCL_IO;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(I2C_MASTER_NUM, &config);
    i2c_driver_install(I2C_MASTER_NUM, config.mode, 0, 0, 0);
}

void WriteReg8(uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

uint8_t ReadReg8(uint8_t reg)
{
    uint8_t val = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_ADDRESS << 1 | I2C_MASTER_READ), true);
    i2c_master_read_byte(cmd, &val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
        return 0xFF;
    i2c_cmd_link_delete(cmd);

    return val;
}

uint16_t ReadReg16_LE(uint8_t reg)
{
    uint8_t lsb = ReadReg8(reg);
    uint8_t msb = ReadReg8(reg + 1);
    return (msb << 8) | lsb;
}

uint32_t ReadReg20_BE(uint8_t reg)
{
    uint8_t msb = ReadReg8(reg);
    uint8_t lsb = ReadReg8(reg + 1);
    uint8_t xlsb = ReadReg8(reg + 2);
    return ((uint32_t)msb << 12) | ((uint32_t)lsb << 4) | (xlsb >> 4);
}