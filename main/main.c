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

#include "screen_driver.h"
#include "img_array.h"

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

static uint16_t g_color_table[ITERATION];
static scr_driver_t g_lcd;
static scr_info_t g_lcd_info;

static void generate_mandelbrot(scr_driver_t *lcd, uint16_t size_x, uint16_t size_y, int32_t offset_x, int32_t offset_y, uint16_t zoom, uint16_t *line_buffer)
{
    uint8_t i;
    uint16_t x, y;
    float tmp1, tmp2;
    float p_r, p_i;
    float num_real, num_img;
    float radius;
    ESP_LOGI(TAG, "zoom = %d", zoom);

    for (y = 0; y < size_y; y++) {
        for (x = 0; x < size_x; x++) {
            num_real = x - offset_x;
            p_r = num_real = num_real / zoom;
            num_img = y - offset_y;
            p_i = num_img = num_img / zoom;
            i = 0;
            radius = 0;
            if (0 == x && 0 == y) {
                ESP_LOGI(TAG, "start(%f, %f)", num_real, num_img);
            }
            if (size_x - 1 == x && size_y - 1 == y) {
                ESP_LOGI(TAG, "end  (%f, %f)", num_real, num_img);
            }

            while ((i < ITERATION - 1) && (radius < 16)) {
                // z = z^2 + c
                tmp1 = num_real * num_real;
                tmp2 = num_img * num_img;
                num_img = 2 * num_real * num_img + p_i;
                num_real = tmp1 - tmp2 + p_r;
                radius = tmp1 + tmp2; // Modulus^2
                i++;
            }
            if (NULL != line_buffer) {
                line_buffer[x] = g_color_table[i];
            } else {
                lcd->draw_pixel(x, y, g_color_table[i]);
            }
        }
        if (NULL != line_buffer) {
            lcd->draw_bitmap(0, y, g_lcd_info.width, 1, line_buffer);
        }
    }
}

static uint16_t rgb888_to_rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static void init_CLUT(uint16_t *clut)
{
    uint32_t i = 0x00;
    uint16_t red = 0, green = 0, blue = 0;

    for (i = 0; i < ITERATION; i++) {
        red = (i * 6 * 256 / ITERATION) % 256;
        green = (i * 4 * 256 / ITERATION) % 256;
        blue = (i * 8 * 256 / ITERATION) % 256;
        clut[i] = rgb888_to_rgb565(red, green, blue);
    }
}

static void screen_clear(scr_driver_t *lcd, int color)
{
    scr_info_t lcd_info;
    lcd->get_info(&lcd_info);
    uint16_t *buffer = malloc(lcd_info.width * sizeof(uint16_t));
    if (NULL == buffer) {
        for (size_t y = 0; y < lcd_info.height; y++) {
            for (size_t x = 0; x < lcd_info.width; x++) {
                lcd->draw_pixel(x, y, color);
            }
        }
    } else {
        for (size_t i = 0; i < lcd_info.width; i++) {
            buffer[i] = color;
        }

        for (int y = 0; y < lcd_info.height; y++) {
            lcd->draw_bitmap(0, y, lcd_info.width, 1, buffer);
        }

        free(buffer);
    }
}

static void lcd_bitmap_test(scr_driver_t *lcd)
{
    scr_info_t lcd_info;
    lcd->get_info(&lcd_info);

    uint16_t *pixels = heap_caps_malloc((img_width * img_height) * sizeof(uint16_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (NULL == pixels) {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
        return;
    }
    memcpy(pixels, img_array, (img_width * img_height) * sizeof(uint16_t));
    lcd->draw_bitmap(0, 0, img_width, img_height, (uint16_t *)pixels);
    heap_caps_free(pixels);
}

//static void lcd_speed_test(scr_driver_t *lcd)
//{
//    scr_info_t lcd_info;
//    lcd->get_info(&lcd_info);
//
//#if defined(CONFIG_BOARD_ESP32_M5STACK)
//    uint32_t w = 320, h = 240;
//#else
//    uint32_t w = 240, h = 240;
//#endif
//
//    w = lcd_info.width < w ? lcd_info.width : w;
//    h = lcd_info.height < h ? lcd_info.height : h;
//
//    const uint32_t buffer_size = w * h;
//    uint16_t *pixels = heap_caps_malloc(buffer_size * sizeof(uint16_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
//    if (NULL == pixels) {
//        ESP_LOGE(TAG, "Memory for bitmap is not enough");
//        return;
//    }
//
//    esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(xPortGetCoreID()));
//    const uint16_t color_table[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW};
//    uint32_t times = 256;
//    uint64_t s;
//    uint64_t t1 = 0;
//    for (int i = 0; i < times; i++) {
//        for (int j = 0; j < buffer_size; j++) {
//            pixels[j] = color_table[i % 4];
//        }
//        s = esp_timer_get_time();
//        lcd->draw_bitmap(0, 0, w, h, pixels);
//        t1 += (esp_timer_get_time() - s);
//    }
//    t1 = t1 / 1000;
//    float time_per_frame = (float)t1 / (float)times;
//    float fps = (float)times * 1000.f / (float)t1;
//    float write_speed = sizeof(uint16_t) * buffer_size * times / 1024.0f / 1.0240f / (float)t1;
//    float factor = ((float)w * (float)h) / ((float)lcd_info.width * (float)lcd_info.height);
//    ESP_LOGI(TAG, "-------%s Test Result------", lcd_info.name);
//    if (w != lcd_info.width || h != lcd_info.height) {
//        ESP_LOGI(TAG, "@resolution 240x240          [time per frame=%.2fMS, fps=%.2f, speed=%.2fMB/s]", time_per_frame, fps, write_speed);
//        ESP_LOGI(TAG, "@resolution %ux%u infer to [time per frame=%.2fMS, fps=%.2f]",
//                 lcd_info.width, lcd_info.height,
//                 time_per_frame / factor, factor * fps);
//    } else {
//        ESP_LOGI(TAG, "@resolution %ux%u          [time per frame=%.2fMS, fps=%.2f, speed=%.2fMB/s]", lcd_info.width, lcd_info.height, time_per_frame, fps, write_speed);
//    }
//    ESP_LOGI(TAG, "-------------------------------------");
//
//    esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(xPortGetCoreID()));
//    heap_caps_free(pixels);
//}

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

    esp_err_t ret = ESP_OK;
#if USE_SPI_SCREEN
    iot_board_init();

#if defined(CONFIG_BOARD_ESP32_M5STACK)
    spi_bus_handle_t spi_bus = iot_board_get_handle(BOARD_SPI3_ID);
#else
    spi_bus_handle_t spi_bus = iot_board_get_handle(BOARD_SPI2_ID);
#endif

    scr_interface_spi_config_t spi_lcd_cfg = {
            .spi_bus = spi_bus,
            .pin_num_cs = -1,
            .pin_num_dc = GPIO_NUM_15, // D8 Wemos
            .clk_freq = 20000000,
            .swap_data = true,
    };

    scr_interface_driver_t *iface_drv;
    scr_interface_create(SCREEN_IFACE_SPI, &spi_lcd_cfg, &iface_drv);
#if defined(CONFIG_BOARD_ESP32_M5STACK)
    ret = scr_find_driver(SCREEN_CONTROLLER_ILI9342, &g_lcd);
#else
    ret = scr_find_driver(SCREEN_CONTROLLER_ILI9341, &g_lcd);
#endif
    if (ESP_OK != ret) {
        return;
        ESP_LOGE(TAG, "screen find failed");
    }

    scr_controller_config_t lcd_cfg = {
            .interface_drv = iface_drv,
            .pin_num_rst = GPIO_NUM_0,// D3 Wemos BOARD_LCD_SPI_RESET_PIN,
            .pin_num_bckl = -1,//BOARD_LCD_SPI_BL_PIN,
            .rst_active_level = 0,
            .bckl_active_level = 1,
            .offset_hor = 0,
            .offset_ver = 0,
#if defined(CONFIG_BOARD_ESP32_M5STACK)
            .width = 320,
        .height = 240,
        .rotate = SCR_DIR_LRTB,
#else
            .width = 240,
            .height = 320,
            .rotate = SCR_DIR_BTRL,
#endif
    };
    ret = g_lcd.init(&lcd_cfg);
#else

    i2s_lcd_config_t i2s_lcd_cfg = {
        .data_width  = BOARD_LCD_I2S_BITWIDTH,
        .pin_data_num = {
            BOARD_LCD_I2S_D0_PIN,
            BOARD_LCD_I2S_D1_PIN,
            BOARD_LCD_I2S_D2_PIN,
            BOARD_LCD_I2S_D3_PIN,
            BOARD_LCD_I2S_D4_PIN,
            BOARD_LCD_I2S_D5_PIN,
            BOARD_LCD_I2S_D6_PIN,
            BOARD_LCD_I2S_D7_PIN,
            BOARD_LCD_I2S_D8_PIN,
            BOARD_LCD_I2S_D9_PIN,
            BOARD_LCD_I2S_D10_PIN,
            BOARD_LCD_I2S_D11_PIN,
            BOARD_LCD_I2S_D12_PIN,
            BOARD_LCD_I2S_D13_PIN,
            BOARD_LCD_I2S_D14_PIN,
            BOARD_LCD_I2S_D15_PIN,
        },
        .pin_num_cs = BOARD_LCD_I2S_CS_PIN,
        .pin_num_wr = BOARD_LCD_I2S_WR_PIN,
        .pin_num_rs = BOARD_LCD_I2S_RS_PIN,

        .clk_freq = 20000000,
        .i2s_port = I2S_NUM_0,
        .buffer_size = 32000,
        .swap_data = true,
    };

    scr_interface_driver_t *iface_drv;
    scr_interface_create(SCREEN_IFACE_8080, &i2s_lcd_cfg, &iface_drv);

    ret = scr_find_driver(SCREEN_CONTROLLER_ILI9806, &g_lcd);
    if (ESP_OK != ret) {
        return;
        ESP_LOGE(TAG, "screen find failed");
    }
    scr_controller_config_t lcd_cfg = {
        .interface_drv = iface_drv,
        .pin_num_rst = BOARD_LCD_I2S_RESET_PIN,
        .pin_num_bckl = BOARD_LCD_I2S_BL_PIN,
        .rst_active_level = 0,
        .bckl_active_level = 1,
        .offset_hor = 0,
        .offset_ver = 0,
        .width = 480,
        .height = 854,
        .rotate = SCR_SWAP_XY | SCR_MIRROR_Y, /** equal to SCR_DIR_BTLR */
    };
    ret = g_lcd.init(&lcd_cfg);
#endif
    if (ESP_OK != ret) {
        return;
        ESP_LOGE(TAG, "screen initialize failed");
    }

    g_lcd.get_info(&g_lcd_info);
    ESP_LOGI(TAG, "Screen name:%s | width:%d | height:%d", g_lcd_info.name, g_lcd_info.width, g_lcd_info.height);

    screen_clear(&g_lcd, COLOR_ESP_BKGD);
    vTaskDelay(pdMS_TO_TICKS(500));

    /**  Run test */
    lcd_bitmap_test(&g_lcd);
    vTaskDelay(pdMS_TO_TICKS(2000));
//    lcd_speed_test(&g_lcd);
    vTaskDelay(pdMS_TO_TICKS(2000));

    init_CLUT(g_color_table); /** Initialize color look up table */

    /** Define an interesting point on the complex plane */
    float real = -0.68481 + (g_lcd_info.width / 2 / (float)MAX_ZOOM);
    float img = 0.380584 + (g_lcd_info.height / 2 / (float)MAX_ZOOM);

    uint16_t *pixels = heap_caps_malloc(g_lcd_info.width * sizeof(uint16_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (NULL == pixels) {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
        return;
    }

    float zoom;

    while (1) {
        for (zoom = 50; zoom <= MAX_ZOOM; zoom *= 1.1f) {
            int32_t off_x = (real) * zoom;
            int32_t off_y = (img) * zoom;
            generate_mandelbrot(&g_lcd, g_lcd_info.width, g_lcd_info.height, g_lcd_info.width / 2 - off_x,  g_lcd_info.height / 2 - off_y, zoom, pixels);
            vTaskDelay(1); /** Delay one tick for feed task watchdog */
        }
        for (; zoom > 50; zoom /= 1.1f) {
            int32_t off_x = (real) * zoom;
            int32_t off_y = (img) * zoom;
            generate_mandelbrot(&g_lcd, g_lcd_info.width, g_lcd_info.height, g_lcd_info.width / 2 - off_x,  g_lcd_info.height / 2 - off_y, zoom, pixels);
            vTaskDelay(1);
        }
    }
}