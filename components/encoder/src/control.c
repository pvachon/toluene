#include "esp_log.h"
#include "esp_gap_ble_api.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "uploader.h"
#include "device.h"

#define TAG "CONTROL"

static
xTaskHandle _control_task_hdl;

static
EventGroupHandle_t _control_task_events;

#define CONTROL_TASK_NAME                   "control"
#define CONTROL_TASK_STACK_WORDS            4096
#define CONTROL_TASK_PRIORITY               9

#define CONTROL_TASK_STATUS_BLE_STARTED             (1 << 1)
#define CONTROL_TASK_STATUS_BLE_FINISHED            (1 << 2)
#define CONTROL_TASK_STATUS_WIFI_UP                 (1 << 3)
#define CONTROL_TASK_STATUS_WIFI_DONE               (1 << 4)
#define CONTROL_TASK_STATUS_BLE_SCAN_TIMEOUT        (1 << 5)
#define CONTROL_TASK_STATUS_BLE_READY               (1 << 6)
#define CONTROL_TASK_STATUS_BLE_STOPPED             (1 << 7)
#define CONTROL_TASK_STATUS_BLE_SCAN_START_REQUEST  (1 << 8)
#define CONTROL_TASK_STATUS_WIFI_FAILURE            (1 << 9)
#define CONTROL_TASK_STATUS_WIFI_DOWN               (1 << 10)

#define CONFIG_TRACKER_MAX_MEM                      (100 * 1024)

enum control_state {
    CONTROL_STATE_IDLE,
    CONTROL_STATE_RUNNING_BLE_SCAN,
    CONTROL_STATE_PAUSED_BLE_SCAN,
    CONTROL_STATE_SHUTTING_DOWN_BLE_SCAN,
    CONTROL_STATE_WIFI_CONNECTING,
    CONTROL_STATE_WIFI_CONNECTED,
    CONTROL_STATE_WIFI_SHUTTING_DOWN,
    CONTROL_STATE_WIFI_BACKOFF,
    CONTROL_STATE_WAIT_WIFI_DOWN,
};

static
enum control_state _control_state;

static
esp_timer_handle_t _control_timer;

static
esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

static
void start_scan(void)
{
    esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    if (scan_ret){
        ESP_LOGE(TAG, "set scan params error, error code = %x", scan_ret);
    }
}

static
void _control_task_timer_fire(void *p)
{
    xEventGroupSetBits(_control_task_events, CONTROL_TASK_STATUS_BLE_SCAN_TIMEOUT);
}

static
esp_timer_create_args_t _control_timer_args = {
    .callback = _control_task_timer_fire,
    .name = "control-timer",
};

static
void _control_task_thread(void *p)
{
    const TickType_t ticks_to_wait = 10000 / portTICK_PERIOD_MS;

#define CONTROL_TIMER_INTERVAL      (300ull * 1000ull * 1000ull)

    size_t nr_backoffs = 0;

    ESP_LOGI(TAG, "---- Control task started ---");

    if (ESP_OK != esp_timer_create(&_control_timer_args, &_control_timer)) {
        ESP_LOGE(TAG, "Failed to create timer, aborting.");
        abort();
    }

    do {
        EventBits_t bits = xEventGroupWaitBits(_control_task_events,
                CONTROL_TASK_STATUS_BLE_STARTED |
                CONTROL_TASK_STATUS_BLE_FINISHED |
                CONTROL_TASK_STATUS_WIFI_UP |
                CONTROL_TASK_STATUS_WIFI_DONE |
                CONTROL_TASK_STATUS_BLE_SCAN_TIMEOUT |
                CONTROL_TASK_STATUS_BLE_READY |
                CONTROL_TASK_STATUS_BLE_STOPPED |
                CONTROL_TASK_STATUS_BLE_SCAN_START_REQUEST |
                CONTROL_TASK_STATUS_WIFI_FAILURE |
                CONTROL_TASK_STATUS_WIFI_DOWN,
                pdTRUE,
                pdFALSE,
                ticks_to_wait);

        if (bits & CONTROL_TASK_STATUS_BLE_READY) {
            if (_control_state == CONTROL_STATE_SHUTTING_DOWN_BLE_SCAN) {
                ESP_LOGI(TAG, "====> STATUS: BLE scan shutdown has been requested");
                _control_state = CONTROL_STATE_WIFI_CONNECTING;
                uploader_connect();
            } else {
                ESP_LOGI(TAG, "====> STATUS: BLE ready, initiating scan");
                _control_state = CONTROL_STATE_RUNNING_BLE_SCAN;
                esp_timer_start_once(_control_timer, CONTROL_TIMER_INTERVAL);
                start_scan();
            }
        }

        if (bits & CONTROL_TASK_STATUS_BLE_SCAN_TIMEOUT) {
            if (_control_state == CONTROL_STATE_PAUSED_BLE_SCAN) {
                /* Mark that the scan should not be resumed the next time the control thread is awoken by
                 * the worker thread.
                 */
                ESP_LOGI(TAG, "====> STATUS: Scan timeout fired, request the pending scan is terminated");
                _control_state = CONTROL_STATE_SHUTTING_DOWN_BLE_SCAN;
            } else {
                /* Request we terminate the scan */
                ESP_LOGI(TAG, "====> STATUS: Shutting down BLE scan, time limit reached");
                _control_state = CONTROL_STATE_SHUTTING_DOWN_BLE_SCAN;
                esp_ble_gap_stop_scanning();
            }
        }

        if (bits & CONTROL_TASK_STATUS_BLE_STOPPED) {
            if (_control_state != CONTROL_STATE_SHUTTING_DOWN_BLE_SCAN) {
                ESP_LOGI(TAG, "====> STATUS: BLE Scan Stopped for Interrogation");
                _control_state = CONTROL_STATE_PAUSED_BLE_SCAN;
            } else if (device_tracker_nr_devs(NULL) != 0) {
                ESP_LOGI(TAG, "====> STATUS: BLE Scan has terminated, waking wifi");
                _control_state = CONTROL_STATE_WIFI_CONNECTING;
                uploader_connect();
            } else {
                ESP_LOGI(TAG, "====> STATUS: No devices found, skipping upload phase");
                _control_state = CONTROL_STATE_RUNNING_BLE_SCAN;
                esp_timer_start_once(_control_timer, CONTROL_TIMER_INTERVAL);
                start_scan();
            }
        }

        if (bits & CONTROL_TASK_STATUS_BLE_SCAN_START_REQUEST) {
            if (_control_state == CONTROL_STATE_IDLE ||
                    _control_state == CONTROL_STATE_PAUSED_BLE_SCAN)
            {
                ESP_LOGD(TAG, "====> STATUS: Scan start requested, starting");
                start_scan();
            } else {
                ESP_LOGE(TAG, "====> STATUS: Scan start requested, but the state isn't right, aborting.");
            }
        }

        if (bits & CONTROL_TASK_STATUS_BLE_STARTED) {
            ESP_LOGI(TAG, "====> STATUS: BLE Scan Started");
            _control_state = CONTROL_STATE_RUNNING_BLE_SCAN;
        }

        if (bits & CONTROL_TASK_STATUS_BLE_FINISHED) {
            ESP_LOGI(TAG, "====> STATUS: BLE scan is finished, turning up the wifi");
            _control_state = CONTROL_STATE_WIFI_CONNECTING;
            uploader_connect();
        }

        if (bits & CONTROL_TASK_STATUS_WIFI_UP) {
            ESP_LOGI(TAG, "====> STATUS: Wifi is up");
            if (nr_backoffs != 0) {
                ESP_LOGI(TAG, "Successfully connected after %zu backoffs", nr_backoffs);
            }
            nr_backoffs = 0;
            _control_state = CONTROL_STATE_WIFI_CONNECTED;
        }

        if (bits & CONTROL_TASK_STATUS_WIFI_DONE) {
            ESP_LOGI(TAG, "====> STATUS: Wifi is done");
            _control_state = CONTROL_STATE_WAIT_WIFI_DOWN;
        }

        if (bits & CONTROL_TASK_STATUS_WIFI_DOWN) {
            if (_control_state == CONTROL_STATE_WAIT_WIFI_DOWN) {
                ESP_LOGI(TAG, "===> STATUS: Got signal wifi is down");
                _control_state = CONTROL_STATE_RUNNING_BLE_SCAN;
                esp_timer_start_once(_control_timer, CONTROL_TIMER_INTERVAL);
                start_scan();
            } else {
                ESP_LOGI(TAG, "===> STATUS: Wifi is now down, but something is awry");
            }
        };

        if (bits & CONTROL_TASK_STATUS_WIFI_FAILURE) {
            ESP_LOGE(TAG, "====> STATUS: Wifi connectivity failure");
            uploader_shutdown();
            size_t mem_used = device_tracker_nr_bytes_used(NULL);

            _control_state = CONTROL_STATE_WIFI_BACKOFF;

            if (mem_used < CONFIG_TRACKER_MAX_MEM) {
                /* Just start BLE scanning again */
                ESP_LOGI(TAG, "====> Resuming BLE scan to give the pipes time to clear");
                _control_state = CONTROL_STATE_RUNNING_BLE_SCAN;
                esp_timer_start_once(_control_timer, CONTROL_TIMER_INTERVAL);
                start_scan();
            }
        } else if (_control_state == CONTROL_STATE_WIFI_BACKOFF) {
            /* We're in backoff, and not status bit was specified -- initiate connection */
            nr_backoffs++;
            uploader_connect();
        }
    } while (1);
}

void control_task_init(void)
{
    _control_task_events = xEventGroupCreate();

    int ret = xTaskCreate(_control_task_thread,
                          CONTROL_TASK_NAME,
                          CONTROL_TASK_STACK_WORDS,
                          NULL,
                          CONTROL_TASK_PRIORITY,
                          &_control_task_hdl);

    if (pdPASS != ret) {
        ESP_LOGE(TAG, "Failed to create uploader worker task (result=%d)", ret);
    }
}

void control_task_signal_ble_ready(void)
{
    xEventGroupSetBits(_control_task_events, CONTROL_TASK_STATUS_BLE_READY);
}

void control_task_signal_ble_gatt_disconnect(void)
{
    xEventGroupSetBits(_control_task_events, CONTROL_TASK_STATUS_BLE_READY);
}

void control_task_signal_ble_scan_started(void)
{
    xEventGroupSetBits(_control_task_events, CONTROL_TASK_STATUS_BLE_STARTED);
}

void control_task_signal_ble_scan_complete(void)
{
    xEventGroupSetBits(_control_task_events, CONTROL_TASK_STATUS_BLE_FINISHED);
}

void control_task_signal_ble_scan_paused(void)
{
    xEventGroupSetBits(_control_task_events, CONTROL_TASK_STATUS_BLE_STOPPED);
}

void control_task_signal_wifi_up(void)
{
    xEventGroupSetBits(_control_task_events, CONTROL_TASK_STATUS_WIFI_UP);
}

void control_task_signal_wifi_done(void)
{
    xEventGroupSetBits(_control_task_events, CONTROL_TASK_STATUS_WIFI_DONE);
}

void control_task_signal_wifi_down(void)
{
    xEventGroupSetBits(_control_task_events, CONTROL_TASK_STATUS_WIFI_DOWN);
}

void control_task_signal_wifi_failure(void)
{
    xEventGroupSetBits(_control_task_events, CONTROL_TASK_STATUS_WIFI_FAILURE);
}

