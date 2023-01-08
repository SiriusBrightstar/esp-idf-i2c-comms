#include <stdio.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

static const char *TAG = "i2c-slave";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO 14                    /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 12                    /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(0)            /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH) /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH) /*!< I2C slave rx buffer size */
#define I2C_SLAVE_ADDR 0x45                    /*!< I2C slave address */

typedef struct tankData
{
    int tankID;
    int waterLevel;
} tankData;

tankData tankOne;

/**
 * @brief i2c slave initialization
 */
static esp_err_t i2c_slave_init(void)
{
    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_SLAVE_ADDR,
        .slave.maximum_speed = 100000,
    };
    esp_err_t err = i2c_param_config(i2c_slave_port, &conf_slave);
    if (err != ESP_OK)
    {
        return err;
    }
    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}

void app_main(void)
{
    int counter = 0;
    int i2c_data = 0;

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_slave_init());

    while (true)
    {
        tankOne.tankID = counter++;
        tankOne.waterLevel = (counter % 100);
        ESP_LOGI(TAG, "Tank ID: %d | Water Level: %d", tankOne.tankID, tankOne.waterLevel);
        i2c_data = i2c_slave_write_buffer(I2C_SLAVE_NUM, (uint8_t *)&tankOne, sizeof(tankOne), (1000 / portTICK_RATE_MS));

        if (i2c_data != ESP_FAIL)
        {
            ESP_LOGI(TAG, "I2C Data Length: %d Bytes", i2c_data);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to generate data");
        }

        vTaskDelay(10000 / portTICK_RATE_MS);
    }
}
