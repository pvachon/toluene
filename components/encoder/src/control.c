#include "esp_log.h"
#include "esp_gap_ble_api.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "uploader.h"
#include "device.h"

#include "identity.h"

#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include <string.h>

#define TAG "CONTROL"

static
xTaskHandle _control_task_hdl;

static
EventGroupHandle_t _control_task_events;

#define CONTROL_TASK_NAME                           "control"
#define CONTROL_TASK_STACK_WORDS                    8192
#define CONTROL_TASK_PRIORITY                       9

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
#define CONTROL_TASK_STATUS_NTP_ACQUIRED            (1 << 11)

#define CONFIG_TRACKER_MAX_MEM                      (100 * 1024)
#define CONFIG_CONTROL_TIMER_INTERVAL               (300ull * 1000ull * 1000ull)

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
    CONTROL_STATE_WAIT_GET_NTP,
};

static
enum control_state _control_state;

static
esp_timer_handle_t _control_timer;

static
struct identity device_ident;

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

    size_t nr_backoffs = 0,
           ntp_failures = 0;

    ESP_LOGI(TAG, "---- Control task started ---");

    if (ESP_OK != esp_timer_create(&_control_timer_args, &_control_timer)) {
        ESP_LOGE(TAG, "Failed to create timer, aborting.");
        abort();
    }

    /* TODO: need to make this more robust. This is so we trigger the SNTP initial
     * time retrieval.
     */
    _control_state = CONTROL_STATE_WAIT_GET_NTP;

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
                CONTROL_TASK_STATUS_WIFI_DOWN |
                CONTROL_TASK_STATUS_NTP_ACQUIRED
                ,
                pdTRUE,
                pdFALSE,
                ticks_to_wait);

        if (bits & CONTROL_TASK_STATUS_BLE_READY) {
            if (_control_state == CONTROL_STATE_SHUTTING_DOWN_BLE_SCAN) {
                ESP_LOGI(TAG, "====> STATUS: BLE scan shutdown has been requested");
                _control_state = CONTROL_STATE_WIFI_CONNECTING;
                uploader_connect();
            } else if (_control_state == CONTROL_STATE_WAIT_GET_NTP) {
                ESP_LOGI(TAG, "====> STATUS: Setting NTP time");
                uploader_connect();
            } else {
                ESP_LOGI(TAG, "====> STATUS: BLE ready, initiating scan");
                _control_state = CONTROL_STATE_RUNNING_BLE_SCAN;
                esp_timer_start_once(_control_timer, CONFIG_CONTROL_TIMER_INTERVAL);
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
                esp_timer_start_once(_control_timer, CONFIG_CONTROL_TIMER_INTERVAL);
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
            /* If we're waiting for NTP, don't adjust our state */
            if (_control_state == CONTROL_STATE_WIFI_CONNECTING) {
                _control_state = CONTROL_STATE_WIFI_CONNECTED;
            }
        }

        if (bits & CONTROL_TASK_STATUS_WIFI_DONE) {
            ESP_LOGI(TAG, "====> STATUS: Wifi is done");
            _control_state = CONTROL_STATE_WAIT_WIFI_DOWN;
        }

        if (bits & CONTROL_TASK_STATUS_WIFI_DOWN) {
            if (_control_state == CONTROL_STATE_WAIT_WIFI_DOWN ||
                    _control_state == CONTROL_STATE_WIFI_CONNECTING)
            {
                ESP_LOGI(TAG, "===> STATUS: Got signal wifi is down");
                _control_state = CONTROL_STATE_RUNNING_BLE_SCAN;
                esp_timer_start_once(_control_timer, CONFIG_CONTROL_TIMER_INTERVAL);
                start_scan();
            } else if (_control_state == CONTROL_STATE_WAIT_GET_NTP) {
                if (++ntp_failures <= 10) {
                    ESP_LOGW(TAG, "======> STATUS: Failed to get NTP time, we will try again in 10 seconds.");
                    uploader_connect();
                } else {
                    ESP_LOGW(TAG, "======> STATUS: Giving up on setting NTP time, I'm just going to start scanning.");
                    _control_state = CONTROL_STATE_RUNNING_BLE_SCAN;
                    esp_timer_start_once(_control_timer, CONFIG_CONTROL_TIMER_INTERVAL);
                    start_scan();
                }
            } else {
                ESP_LOGI(TAG, "===> STATUS: Wifi is now down, but something is awry (state = %d)", _control_state);
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
                esp_timer_start_once(_control_timer, CONFIG_CONTROL_TIMER_INTERVAL);
                start_scan();
            }
        } else if (_control_state == CONTROL_STATE_WIFI_BACKOFF) {
            /* We're in backoff, and not status bit was specified -- initiate connection */
            nr_backoffs++;
            uploader_connect();
        }

        if (bits & CONTROL_TASK_STATUS_NTP_ACQUIRED) {
            ESP_LOGI(TAG, "====> STATUS: We have NTP time!");
            if (_control_state == CONTROL_STATE_WAIT_GET_NTP) {
                _control_state = CONTROL_STATE_WIFI_CONNECTED;
            }
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

static
void _control_config_init(struct identity *ident)
{
    nvs_handle hdl = NULL;

    uint32_t header = 0;
    uint16_t length = 0;
    uint8_t *body = NULL;

    esp_err_t err = ESP_OK;

    if (NULL == ident) {
        ESP_LOGE(TAG, "Programmer error, identity pointer is NULL");
        abort();
    }

    if (ESP_OK != (err = nvs_open("huffer", NVS_READWRITE, &hdl))) {
        ESP_LOGE(TAG, "Fatal error while opening NVS partition for read-write, aborting (%d)", err);
        abort();
    }

    if (ESP_OK != (err = nvs_erase_all(hdl))) {
        ESP_LOGE(TAG, "Fatal error while erasing NVS partition, aborting (%d)", err);
        abort();
    }

    ESP_LOGI(TAG, "Waiting for identity blob.");

#define CONTROL_CONFIG_HEADER           0xbebafeca

    int fd = -1;

    if (0 > (fd = open("/dev/uart/0", O_RDWR))) {
        int errnum = errno;
        ESP_LOGE(TAG, "Failed to open uart, aborting (errnum = %d, %s)", errnum, strerror(errnum));
        abort();
    }

    do {
        /* Read 4 bytes of header */
        if (4 != read(fd, &header, 4)) {
            int errnum = errno;
            ESP_LOGE(TAG, "An error occurred while reading the header from the console, aborting (errnum = %d, %s)", errnum, strerror(errnum));
            abort();
        }

        if (header != CONTROL_CONFIG_HEADER) {
            ESP_LOGE(TAG, "Malformed header, got %08x, expected %08x", header, CONTROL_CONFIG_HEADER);
            continue;
        }

        /* Wait for 2 bytes on the console indicating the length of the blob */
        if (2 != read(fd, &length, 2)) {
            int errnum = errno;
            ESP_LOGE(TAG, "An error occurred while reading the length from the console, aborting (errnum = %d, %s)", errnum, strerror(errnum));
            abort();
        }

        if (length > IDENTITY_BLOB_LENGTH_MAX || length < 128) {
            ESP_LOGE(TAG, "Identity blob's size doesn't make sense, restarting protocol");
            continue;
        }

        /* Allocate memory to receive the blob */
        if (NULL == (body = realloc(body, length))) {
            ESP_LOGE(TAG, "Unable to reallocate memory for the body, restarting protocol");
            continue;
        }

        long bytes = 0;

        while (bytes < length) {
            long ret = 0;

            ret = read(fd, body + bytes, length - bytes);
            if (ret < 0) {
                int errnum = errno;
                ESP_LOGE(TAG, "An error occurred while reading the configuration body, aborting (errnum =  %d, %s) - read %ld bytes of %zu.", errnum, strerror(errnum), bytes, length);
                abort();
            }

            bytes += ret;
        }

        /* Validate the blob's contents, per our normal path */
        if (identity_read(ident, body, length)) {
            ESP_LOGE(TAG, "Identity bundle is invalid, restarting protocol.");
            continue;
        }

        /* Store the blob to flash on success and break */
        if (ESP_OK != nvs_set_blob(hdl, "identity", body, length)){
            ESP_LOGE(TAG, "Failed to write identity blob to NVS, aborting.");
            abort();
        }

        if (ESP_OK != nvs_commit(hdl)) {
            ESP_LOGE(TAG, "Failed to commit identity to NVS, aborting.");
            abort();
        }

        /* And we're done */
        break;
    } while (1);

    if (NULL != body) {
        free(body);
        body = NULL;
    }

    nvs_close(hdl);
    close(fd);
}

static
int _control_config_load(struct identity *ident)
{
    int ret = -1;

    size_t length = 0;
    uint8_t *blob = NULL;
    nvs_handle hdl;

    if (ESP_OK != nvs_open("huffer", NVS_READONLY, &hdl)) {
        ESP_LOGE(TAG, "Configuration partition does not exist, aborting.");
        goto done_empty;
    }

    if (NULL == ident) {
        ESP_LOGE(TAG, "Programmer error, identity is NULL, aborting.");
        abort();
    }

    if (ESP_OK != nvs_get_blob(hdl, "identity", NULL, &length)) {
        ESP_LOGE(TAG, "Failed to get identity blob, aborting.");
        goto done;
    }

    if (0 == length || length > IDENTITY_BLOB_LENGTH_MAX) {
        ESP_LOGE(TAG, "Malformed identity blob, aborting.");
        goto done;
    }

    if (NULL == (blob = malloc(length))) {
        ESP_LOGE(TAG, "Out of memory for identity blob, aborting.");
        abort();
    }

    if (ESP_OK != nvs_get_blob(hdl, "identity", blob, &length))  {
        ESP_LOGE(TAG, "Unable to read identity blob, aborting.");
        goto done;
    }

    if (identity_read(ident, blob, length)) {
        ESP_LOGE(TAG, "Failed to validate and decode identity blob, aborting.");
        goto done;
    }

    ESP_LOGI(TAG, "Successfully loaded identity from NVS, continuing!");

    ret = 0;
done:
    nvs_close(hdl);

    if (0 != ret) {
        if (ESP_OK != nvs_open("huffer", NVS_READWRITE, &hdl)) {
            ESP_LOGE(TAG, "Failed to reopen partition as read/write");
            abort();
        }

        if (ESP_OK != nvs_erase_all(hdl)) {
            ESP_LOGE(TAG, "Failed to clear NVS, aborting.");
            abort();
        }

        if (ESP_OK != nvs_commit(hdl)) {
            ESP_LOGE(TAG, "Failed to commit NVS, aborting.");
            abort();
        }

        nvs_close(hdl);
    }

done_empty:
    return ret;
}

static
void _control_config_init_subsys(struct identity *ident)
{
    /* Try loading the configuration */
    if (_control_config_load(ident)) {
        /* There is no configuration, so wait for a blob to be loaded via the UART */
        ESP_LOGE(TAG, "There is no configuration available, waiting for an identity to load");
        _control_config_init(ident);
    }
}

void control_load_config(void)
{
    ESP_LOGI(TAG, "Initializing device identity");
    _control_config_init_subsys(&device_ident);
}

void control_get_config_wifi(char *essid, char *password)
{
    strncpy(essid, device_ident.wifi_essid, IDENTITY_ESSID_LEN_MAX);
    strncpy(password, device_ident.wifi_password, IDENTITY_PASSWORD_LEN_MAX);
}

void control_get_config_device_info(char const **phostname, uint16_t *pport, uint32_t *pdev_id)
{
    if (NULL != phostname) {
        *phostname = device_ident.target_host;
    }

    if (NULL != pport) {
        *pport = device_ident.target_port;
    }

    if (NULL != pdev_id) {
        *pdev_id = device_ident.device_id;
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

void control_task_signal_ntp_done(void)
{
    xEventGroupSetBits(_control_task_events, CONTROL_TASK_STATUS_NTP_ACQUIRED);
}

