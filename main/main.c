#include <sys/cdefs.h>
/* I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"
#include "ads1115.h"
//#include "ads111x.h"

static const char *TAG = "main";

/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by running two tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a MPU6050 sensor for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP8266 chip.
 *
 * Pin assignment:
 *
 * - master:
 *    GPIO14 is assigned as the data signal of i2c master port
 *    GPIO2 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect sda/scl of sensor with GPIO14/GPIO2
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 */

#define ADS111X_ADDR_GND 0x48 //!< I2C device address with ADDR pin connected to ground
#define ADS111X_ADDR_VCC 0x49 //!< I2C device address with ADDR pin connected to VCC
#define ADS111X_ADDR_SDA 0x4a //!< I2C device address with ADDR pin connected to SDA
#define ADS111X_ADDR_SCL 0x4b //!< I2C device address with ADDR pin connected to SCL

#define I2C_EXAMPLE_MASTER_SCL_IO           5                /*!< gpio number for I2C master clock D1 Wemos */
#define I2C_EXAMPLE_MASTER_SDA_IO           4                /*!< gpio number for I2C master data  D2 Wemos  */
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_BUS_MS_TO_WAIT                  200
#define I2C_BUS_TICKS_TO_WAIT (I2C_BUS_MS_TO_WAIT/portTICK_RATE_MS)
#define I2C_ACK_CHECK_EN                    0x1     /*!< I2C master will check ack from slave*/

#define MPU6050_SENSOR_ADDR                 ADS111X_ADDR_GND             /*!< slave address for MPU6050 sensor */
#define MPU6050_CMD_START                   0x41             /*!< Command to set measure mode */
#define MPU6050_WHO_AM_I                    0x75             /*!< Command to read WHO_AM_I reg */
#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

/**
 * Define the mpu6050 register address:
 */
#define SMPLRT_DIV      0x19
#define CONFIG          0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48
#define PWR_MGMT_1      0x6B
#define WHO_AM_I        0x75  /*!< Command to read WHO_AM_I reg */

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

_Noreturn static void i2c_bus_scan_task (void *arg) {
    i2c_master_init();

    while (1) {
        uint8_t device_count = 0;
        for (uint8_t dev_address = 1; dev_address < 127; dev_address++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (dev_address << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, I2C_BUS_TICKS_TO_WAIT);

            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "found i2c device address = 0x%02x", dev_address);
                device_count++;
            }

            i2c_cmd_link_delete(cmd);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
}

static void ads1115_test_task(void *arg) {
    i2c_master_init();
    ads1115_t adc = ads1115_config(I2C_NUM_0, ADS111X_ADDR_GND);
    int16_t adc_raw = 0;
    double volts = 0;
    static const char *TASK_TAG = "ads1115_test_task";

    while (1) {
        adc_raw = ads1115_get_raw(&adc);
        volts = ads1115_raw_to_voltage(&adc, adc_raw);

        ESP_LOGI(TASK_TAG, "ADC AINP0 = %d | Volts = %f |", adc_raw, volts);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
}

void app_main(void)
{
    //start i2c task
//    xTaskCreate(i2c_bus_scan_task, "i2c_bus_scan_task", 2048, NULL, 10, NULL);
    xTaskCreate(ads1115_test_task, "ads1115_test_task", 2048, NULL, 10, NULL);
}