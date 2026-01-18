#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_client.h"

#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/adc.h"

#include "lwip/inet.h"
#include "esp_sleep.h"
#include "LCD.h"

/* ===================== GPIO DEFINITIONS ===================== */

#define LED_PIN        2
#define LED_UP         16
#define LED_DOWN       17
#define SLEEP_BUTTON   32

/* ===================== RTC WIFI FAST CONNECT ===================== */

typedef struct {
    uint8_t bssid[6];                 // Access Point MAC address
    uint8_t channel;                  // WiFi channel
    esp_netif_ip_info_t ip_info;       // Static IP configuration
    bool valid;                       // Validity flag
} rtc_wifi_config_t;

/* Persisted across deep sleep */
RTC_DATA_ATTR rtc_wifi_config_t fast_wifi_cfg;

/* ===================== SENSOR DATA STRUCT ===================== */

typedef struct {
    double temp;
    double pressure;
} data_sensors;

data_sensors sensors_value;

/* ===================== GLOBAL STATE ===================== */

// Sensor enable flag (protected by mutex)
volatile bool stop_senzor = true;

// ISR routing selector:
// 0 = server heartbeat task
// 1 = power management task
volatile uint8_t task_selection_isr = 0;

// Wakeup cause (timer / external / power-on)
esp_sleep_wakeup_cause_t wakeup_reason;

/* ===================== RTOS HANDLES ===================== */

TaskHandle_t xSenzorHandle        = NULL;
TaskHandle_t xVerificare_server   = NULL;
TaskHandle_t Power_Managment      = NULL;

SemaphoreHandle_t xStop_Senzor_Mutex = NULL;
SemaphoreHandle_t date_sensor_mutex  = NULL;

/* ===================== HTTP FUNCTIONS ===================== */

/* Sends sensor data to the remote server */
void trimite_date_la_server_extern(void)
{
    char url[128];
    sprintf(url,
            "http://10.162.8.221:5000/update?temp=%.2lf&pres=%.2lf",
            sensors_value.temp,
            sensors_value.pressure);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

/* Checks if the server is reachable */
bool server_verification(void)
{
    esp_http_client_config_t config = {
        .url = "http://10.162.8.221:5000/heartbeat",
        .method = HTTP_METHOD_GET,
        .timeout_ms = 3000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    int status = 0;
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(client);
    }

    esp_http_client_cleanup(client);
    return (err == ESP_OK && status == 200);
}

/* ===================== SERVER HEARTBEAT TASK ===================== */

void Task_online_server(void *param)
{
    bool server_started = false;
    bool server_stopped = false;

    while (1) {
        /* Wait for periodic notification from hardware timer */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (server_verification()) {
            if (!server_started) {
                xSemaphoreTake(xStop_Senzor_Mutex, portMAX_DELAY);
                stop_senzor = false;
                xSemaphoreGive(xStop_Senzor_Mutex);

                gpio_set_level(LED_DOWN, 0);
                gpio_set_level(LED_UP, 1);

                if (xSenzorHandle != NULL) {
                    xTaskNotifyGive(xSenzorHandle);
                }

                server_started = true;
                server_stopped = false;
            }
        } else {
            if (!server_stopped) {
                xSemaphoreTake(xStop_Senzor_Mutex, portMAX_DELAY);
                stop_senzor = true;
                xSemaphoreGive(xStop_Senzor_Mutex);

                gpio_set_level(LED_DOWN, 1);
                gpio_set_level(LED_UP, 0);

                server_started = false;
                server_stopped = true;
            }
        }
    }
}

/* ===================== TIMER ISR ===================== */

void IRAM_ATTR timer_isr(void *param)
{
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_1, TIMER_0);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_1, TIMER_0);

    BaseType_t higher_priority_woken = pdFALSE;

    if (task_selection_isr) {
        if (Power_Managment != NULL) {
            vTaskNotifyGiveFromISR(Power_Managment, &higher_priority_woken);
        }
    } else {
        if (xVerificare_server != NULL) {
            vTaskNotifyGiveFromISR(xVerificare_server, &higher_priority_woken);
        }
    }

    if (higher_priority_woken) {
        portYIELD_FROM_ISR();
    }
}

/* ===================== SENSOR TASK (CORE 1) ===================== */

void Task_Senzor_Core1(void *param)
{
    while (1) {
        bool local_stop;

        xSemaphoreTake(xStop_Senzor_Mutex, portMAX_DELAY);
        local_stop = stop_senzor;
        xSemaphoreGive(xStop_Senzor_Mutex);

        if (!local_stop) {
            if (xSemaphoreTake(date_sensor_mutex, pdMS_TO_TICKS(1000))) {
                bme280_read_raw();
                sensors_value.temp = real_temp();
                sensors_value.pressure = get_real_pressure();
                xSemaphoreGive(date_sensor_mutex);

                if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
                    xTaskNotifyGive(Power_Managment);
                    vTaskDelete(NULL);
                } else {
                    trimite_date_la_server_extern();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ===================== WIFI EVENT HANDLER ===================== */

void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    static bool disconnected_once = false;

    if (base == WIFI_EVENT) {

        if (id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        }

        else if (id == WIFI_EVENT_STA_DISCONNECTED) {

            if (!disconnected_once) {
                task_selection_isr = 1;
                disconnected_once = true;

                timer_pause(TIMER_GROUP_1, TIMER_0);
                timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 120000000);
                timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
                timer_start(TIMER_GROUP_1, TIMER_0);

                xSemaphoreTake(xStop_Senzor_Mutex, portMAX_DELAY);
                stop_senzor = true;
                xSemaphoreGive(xStop_Senzor_Mutex);

                lcd_clear();
                lcd_set_cursor(0, 0);
                lcd_print("Network down!");
            }

            esp_wifi_connect();
        }
    }

    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {

        task_selection_isr = 0;
        disconnected_once = false;

        timer_pause(TIMER_GROUP_1, TIMER_0);
        timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 30000000);
        timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
        timer_start(TIMER_GROUP_1, TIMER_0);

        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print("Network up!");

        if (xVerificare_server != NULL) {
            xTaskNotifyGive(xVerificare_server);
        }

        if (!fast_wifi_cfg.valid) {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;

            fast_wifi_cfg.ip_info = event->ip_info;

            wifi_ap_record_t ap_info;
            esp_wifi_sta_get_ap_info(&ap_info);

            fast_wifi_cfg.channel = ap_info.primary;
            memcpy(fast_wifi_cfg.bssid, ap_info.bssid, 6);
            fast_wifi_cfg.valid = true;
        }
    }
}

/* ===================== HARDWARE INIT TASK ===================== */

void Task_Hardware_init(void *param)
{
    SemaphoreHandle_t sem = (SemaphoreHandle_t)param;

    timer_config_t timer_cfg = {
        .divider = 80,
        .counter_dir = TIMER_COUNT_UP,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };

    timer_init(TIMER_GROUP_1, TIMER_0, &timer_cfg);
    timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 30000000);
    timer_enable_intr(TIMER_GROUP_1, TIMER_0);
    timer_isr_register(TIMER_GROUP_1, TIMER_0, timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);

    i2c_master_init();
    i2c_data_init();
    lcd_init();
    bme280_init();

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);

    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_UP, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_DOWN, GPIO_MODE_OUTPUT);
    gpio_set_direction(SLEEP_BUTTON, GPIO_MODE_INPUT);

    xSemaphoreGive(sem);
    vTaskDelete(NULL);
}

/* ===================== POWER MANAGEMENT TASK ===================== */

void Task_Power_Management(void *param)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        char buffer[64];

        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print("Sleep Mode...");

        esp_sleep_enable_ext0_wakeup(SLEEP_BUTTON, 1);
        esp_sleep_enable_timer_wakeup(120 * 1000000);

        vTaskDelay(pdMS_TO_TICKS(500));

        snprintf(buffer, sizeof(buffer),
                 "T: %.2fC, P: %.2f",
                 sensors_value.temp,
                 sensors_value.pressure);

        lcd_set_cursor(0, 1);
        lcd_print(buffer);

        esp_deep_sleep_start();
    }
}

/* ===================== APP MAIN ===================== */

void app_main(void)
{
    SemaphoreHandle_t hw_ready = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(
        Task_Hardware_init,
        "hardware_init",
        4096,
        hw_ready,
        18,
        NULL,
        1
    );

    xSemaphoreTake(hw_ready, portMAX_DELAY);
    vSemaphoreDelete(hw_ready);

    wakeup_reason = esp_sleep_get_wakeup_cause();

    if (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER &&
        wakeup_reason != ESP_SLEEP_WAKEUP_EXT0) {
        cal_t_var_read();
        cal_p_var_read();
    }

    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    xStop_Senzor_Mutex = xSemaphoreCreateMutex();
    date_sensor_mutex  = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(Task_online_server, "HTTP_Check", 8192, NULL, 3, &xVerificare_server, 0);
    xTaskCreatePinnedToCore(Task_Senzor_Core1, "SensorTask", 4096, NULL, 2, &xSenzorHandle, 1);
    xTaskCreatePinnedToCore(Task_Power_Management, "DeepSleep", 4096, NULL, 4, &Power_Managment, 1);

    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        stop_senzor = false;
    }

    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0 ||
        wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {

        esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_wifi_init(&cfg);

        wifi_config_t wifi_cfg = {
            .sta = {
                .ssid = "honor",
                .password = "123456789",
                .scan_method = WIFI_FAST_SCAN,
            },
        };

        esp_wifi_set_mode(WIFI_MODE_STA);

        if (!fast_wifi_cfg.valid) {
            esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
            esp_wifi_start();
        } else {
            wifi_cfg.sta.channel = fast_wifi_cfg.channel;
            memcpy(wifi_cfg.sta.bssid, fast_wifi_cfg.bssid, 6);
            wifi_cfg.sta.bssid_set = 1;

            esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            esp_netif_dhcpc_stop(netif);
            esp_netif_set_ip_info(netif, &fast_wifi_cfg.ip_info);

            esp_wifi_set_storage(WIFI_STORAGE_RAM);
            esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
            esp_wifi_start();
        }

        esp_wifi_set_ps(WIFI_PS_NONE);
    }
}
