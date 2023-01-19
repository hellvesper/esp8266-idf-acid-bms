#include <sys/cdefs.h>

//#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "ads1115.h"

//#include "screen_driver.h"


static const char *TAG = "main";


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
#define I2C_BUS_TICKS_TO_WAIT               (I2C_BUS_MS_TO_WAIT/portTICK_RATE_MS)
#define I2C_ACK_CHECK_EN                    0x1     /*!< I2C master will check ack from slave*/

#define HOUR_MILLIS            3.6e6
#define MEASURE_INTERVAL       500 	 // millis
#define PRINT_INTERVAL         1000  // millis
#define SHUNT_RESISTOR_mOhm    1.0F  // 0.001 Ohm resistor value in milliohms
#define SHUNT_RESISTOR_Ohm     0.001 // 0.001 Ohm resistor value in milliohms
#define VOLTAGE_DIVIDER_RATIO  11    // Vsource / Vout


#define CONFIG_BROKER_URL CONFIG_MQTT_BROKER_URL


#define GPIO_OUTPUT_IO_0     GPIO_NUM_2 // D4 Wemos
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)

/** If you use a spi interface screen, set the USE_SPI_SCREEN to 1, otherwise use 8080 interface. */
#define USE_SPI_SCREEN 1

#define MAX_ZOOM  2500
#define ITERATION 128


typedef struct {
    double volts;
    double amps;
    double power;
    double amphour;
    double watts;
    bool lock;
} power_t;

power_t power;
esp_mqtt_client_handle_t mqtt_client;
/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = 1U;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = 1U;
    conf.clk_stretch_tick = 300U; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

static void i2c_bus_scan_task (void *arg) {
    i2c_master_init();

    while (1) {
        for (uint8_t dev_address = 1U; dev_address < 127U; dev_address++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (dev_address << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, I2C_BUS_TICKS_TO_WAIT);

            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "found i2c device address = 0x%02x", dev_address);
            }

            i2c_cmd_link_delete(cmd);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
}

static void ads1115_task(void *arg) {
    static const char *TASK_TAG = "ads1115_task";
    i2c_master_init();
    ads1115_t adc = ads1115_config(I2C_NUM_0, ADS111X_ADDR_GND);
    int16_t adc_raw = 0;
    double volts = 0;
    double amps = 0;
    static bool VA = true;

    ads1115_set_mux(&adc, ADS1115_MUX_0_1);
    ads1115_set_pga(&adc, ADS1115_FSR_2_048);
    ads1115_set_mode(&adc, ADS1115_MODE_SINGLE);
    ads1115_set_sps(&adc, ADS1115_SPS_64); // than lower rate than more noise will be filtered with ads filter
    while (1) {
        if (VA) {
            power.lock = true;
            adc_raw = ads1115_get_raw(&adc);
            power.volts = ads1115_raw_to_voltage(&adc, adc_raw) * (double )VOLTAGE_DIVIDER_RATIO;
            ads1115_set_mux(&adc, ADS1115_MUX_2_3);
            VA = !VA;
            ESP_LOGI(TASK_TAG, "ADC AINP0N1 = %d | Volts = %f | Amp = %f | Free: %d", adc_raw, power.volts, power.amps, uxTaskGetStackHighWaterMark(NULL));
        } else {
            power.lock = true;
            adc_raw = ads1115_get_raw(&adc);
            volts = ads1115_raw_to_voltage(&adc, adc_raw);
            power.amps = volts / SHUNT_RESISTOR_Ohm;
            ads1115_set_mux(&adc, ADS1115_MUX_0_1);
            VA = !VA;
            ESP_LOGI(TASK_TAG, "ADC AINP2N3 = %d | Volts = %f | Amp = %f | Free: %d", adc_raw, power.volts, power.amps, uxTaskGetStackHighWaterMark(NULL));
        }

        power.power = power.volts * power.amps;
        power.watts += power.power / (double )HOUR_MILLIS * (double)MEASURE_INTERVAL/2;
        power.amphour += power.amps / (double )HOUR_MILLIS * (double)MEASURE_INTERVAL/2;
        power.lock = false;

        vTaskDelay(pdMS_TO_TICKS(MEASURE_INTERVAL)/2U);
    }

    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
}

/*
 * MQTT Section
 */
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
//    esp_mqtt_client_handle_t client = event->client;
//    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
//            msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
//            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
//
//            msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
//            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
//
//            msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
//            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
//
//            msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
//            ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
//            msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
//            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
            .uri = CONFIG_BROKER_URL,
    };

//    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    esp_mqtt_client_start(mqtt_client);

}

static void mqtt_task(void *arg) {
    static char buffer [10];
    while (1) {
        if (!power.lock) {
            snprintf(buffer, 10, "%.4f", power.volts);
            esp_mqtt_client_publish(mqtt_client, "battery/voltage", buffer, 0, 0, 0);
            snprintf(buffer, 10, "%.4f", power.amps);
            esp_mqtt_client_publish(mqtt_client, "battery/current", buffer, 0, 0, 0);
            snprintf(buffer, 10, "%.4f", power.power);
            esp_mqtt_client_publish(mqtt_client, "battery/power", buffer, 0, 0, 0);
            snprintf(buffer, 10, "%.4f", power.watts);
            esp_mqtt_client_publish(mqtt_client, "battery/watts", buffer, 0, 0, 1);
            snprintf(buffer, 10, "%.4f", power.amphour);
            esp_mqtt_client_publish(mqtt_client, "battery/amps", buffer, 0, 0, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

    }
}

static void charging_control_task(void *arg) {
//    gpio_set_level(GPIO_OUTPUT_IO_0, 0);
}



void app_main(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO15/16
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

//    xTaskCreate(charging_control_task, "charging_control_task", 512, NULL, 5, NULL);

    //start i2c task
    //xTaskCreate(i2c_bus_scan_task, "i2c_bus_scan_task", 2048, NULL, 10, NULL);
    xTaskCreate(ads1115_task, "ads1115_task", 2048, NULL, 10, NULL);

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();
    xTaskCreate(mqtt_task, "mqtt_task", 2048, NULL, 10, NULL);
}