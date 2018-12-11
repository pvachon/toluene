#include "uploader.h"
#include "control.h"
#include "device.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/apps/sntp.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"

#include <string.h>
#include <time.h>
#include <sys/time.h>

#define TAG                                     "UPLOADER"
#define CONFIG_MAX_SNTP_RETRIES                 20
#define CONFIG_SNTP_REFRESH_COUNT               10

/**
 * Event group to control the uploader task
 */
static
EventGroupHandle_t _uploader_control;

static
xTaskHandle _uploader_task_hdl;

#define UPLOADER_TASK_NAME                  "uploader"
#define UPLOADER_TASK_STACK_WORDS           10240
#define UPLOADER_TASK_PRIORITY              8

/**
 * Wake up, we've successfully connected to the wifi
 */
#define STATUS_BIT_WIFI_CONNECTED           (1 << 2)

/**
 * We've disconnected from the wifi, likely spuriously 
 */
#define STATUS_BIT_WIFI_DISCONNECT          (1 << 1)

/**
 * Terminate the uploader and free up its memory
 */
#define STATUS_BIT_TERMINATE                (1 << 0)

struct connect_hello {
    uint16_t magic;
    uint16_t nr_records;
    uint16_t device_id;
} __attribute__((packed));

static
bool _uploader_set_time_initial = false;

static
void _sntp_set_system_time(void)
{
    time_t now = 0;
    struct tm timeinfo = { 0 };
    bool time_set = false;

    ESP_LOGI(TAG, "Initializing SNTP");

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    ESP_LOGI(TAG, "Setting system time...");
    for (int i = 0; i < CONFIG_MAX_SNTP_RETRIES; i++) {
        ESP_LOGI(TAG, "Setting system time... (%d/%d)", i, CONFIG_MAX_SNTP_RETRIES);
        time(&now);
        localtime_r(&now, &timeinfo);

        /* Check if we have a sensible year */
        if (timeinfo.tm_year >= (2018 - 1900)) {
            time_set = true;
            break;
        }

        /* Sleep for a bit to let sntp do its thing */
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    if (false == time_set) {
        struct timeval tv = {
            .tv_sec = 1543578442,
            .tv_usec = 0,
        };

        ESP_LOGE(TAG, "Failed to set system time, setting to default.");

        if (0 > settimeofday(&tv, NULL)) {
            ESP_LOGE(TAG, "Failed to set system time, at all. Aborting.");
            abort();
        }

        time(&now);
        localtime_r(&now, &timeinfo);
    }

    ESP_LOGI(TAG, "It is %04d-%02d-%02d at %02d:%02d:%02d",
            timeinfo.tm_year + 1900,
            timeinfo.tm_mon + 1,
            timeinfo.tm_mday,
            timeinfo.tm_hour,
            timeinfo.tm_min,
            timeinfo.tm_sec);

    /* We don't need to keep sntp running in the background */
    sntp_stop();

    control_task_signal_ntp_done();
}

static
unsigned _wifi_up_count = 0;

static
esp_err_t _uploader_wifi_evt_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        /* Actually initiate the connection to the stored AP */
        ESP_LOGI(TAG, "UPLOADER: -> Connecting to access point, please wait...");
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        /* Once we have an IP address, wake up the uploader task */
        ESP_LOGI(TAG, "UPLOADER: -> We have an IP address...");

        if (false == _uploader_set_time_initial || _wifi_up_count % CONFIG_SNTP_REFRESH_COUNT == 0) {
            ESP_LOGI(TAG, "We are setting the system time using SNTP, please wait");
            _sntp_set_system_time();
            _uploader_set_time_initial = true;
        }

        _wifi_up_count++;

        control_task_signal_wifi_up();
        xEventGroupSetBits(_uploader_control, STATUS_BIT_WIFI_CONNECTED);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "UPLOADER: -> We have been disconnected, notifying our state machine");
        esp_wifi_disconnect();
        xEventGroupSetBits(_uploader_control, STATUS_BIT_WIFI_DISCONNECT);
        break;
    default:
        ;
    }
    return ESP_OK;
}

static
int _uploader_resolve_host(char const *hostname, struct ip4_addr *tgt_addr)
{
    int ret = -1;

    struct hostent *he = NULL;

    he = gethostbyname(hostname);
    if (NULL == he) {
        ESP_LOGE(TAG, "Could not resolve address %s, failing.", hostname);
        goto done;
    }

    *tgt_addr = *(struct ip4_addr *)he->h_addr;

    ret = 0;
done:
    return ret;
}

static
int _uploader_connect(struct ip4_addr addr, uint16_t port, int *pfd)
{
    int ret = -1;

    int fd = -1,
        r = 0;
    struct sockaddr_in sin;

    if (NULL == pfd) {
        ESP_LOGE(TAG, "Must specify a non-null pointer to receive the file descriptor");
        abort();
    }

    *pfd = -1;

    ESP_LOGI(TAG, IPSTR, IP2STR(&addr));

    if (0 > (fd = socket(AF_INET, SOCK_STREAM, 0))) {
        ESP_LOGE(TAG, "Failed to create socket, aborting.");
        goto done;
    }

    memset(&sin, 0, sizeof(sin));

    ESP_LOGI(TAG, "Connecting...");

    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = addr.addr;
    sin.sin_port = htons(port);
    if (0 > (r = connect(fd, (struct sockaddr *)&sin, sizeof(sin)))) {
        int errnum = errno;
        ESP_LOGE(TAG, "Failed to connect to specified service, reason %d (%s) aborting.", errnum, strerror(errnum));
        goto done;
    }

    ESP_LOGI(TAG, "Connected!");

    *pfd = fd;

    ret = 0;
done:
    if (0 != ret) {
        if (-1 != fd) {
            close(fd);
            fd = -1;
        }
    }
    return ret;
}

static
int _uploader_deliver_device_info(int fd)
{
    int ret = -1;

    struct device *dev = NULL,
                  *dev_tmp = NULL;
    size_t record_id = 0,
           nr_records;

    nr_records = device_tracker_nr_devs(NULL);

    struct connect_hello helo = {
        .magic = 0xDEAD,
        .nr_records = nr_records,
        .device_id = 24601,
    };

    if (6 != write(fd, &helo, sizeof(helo))) {
        int errnum = errno;
        ESP_LOGE(TAG, "Failed to send HELO: %s (%d)", strerror(errnum), errnum);
        goto done;
    }

    list_for_each_type_safe(dev, dev_tmp, &tracker.device_list, d_node) {
        record_id++;
        uint16_t len = dev->encoded_obj_len;

        assert(0 != len);

        if (2 != write(fd, &len, 2)) {
            int errnum = errno;
            ESP_LOGE(TAG, "Failed to send device record header, aborting: %s (%d)", strerror(errnum), errnum);
            goto done;
        }

        if (dev->encoded_obj_len != write(fd, dev->encoded_obj, dev->encoded_obj_len)) {
            int errnum = errno;
            ESP_LOGE(TAG, "Failed to send device record, aborting: %s (%d)", strerror(errnum), errnum);
            goto done;
        }

        /* Remove the device record, free any deeply-used memory */
        if (device_tracker_remove(&tracker, dev)) {
            ESP_LOGE(TAG, "Unexpected failure while removing object, aborting.");
            abort();
        }

        ESP_LOGI(TAG, "Sent device %zu to the backend (length = %u)", record_id, (unsigned)len);

        /* Free the device itself */
        free(dev);
        dev = NULL;
    }

    if (!list_empty(&tracker.device_list)) {
        ESP_LOGI(TAG, "The list is not empty");
        abort();
    }

    ESP_LOGI(TAG, "Sent a total of %zu device records to backend", record_id);

    ret = 0;
done:
    return ret;
}

static
void _uploader_terminate(int fd)
{
    uint16_t confirm = 0;

    if (2 != read(fd, &confirm, sizeof(confirm))) {
        ESP_LOGE(TAG, "End of stream while waiting for confirmation, aborting.");
    }

    ESP_LOGI(TAG, "Now closing the socket for real");

    lwip_shutdown(fd, SHUT_RDWR);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    if (0 != close(fd)) {
        int errnum = errno;
        ESP_LOGE(TAG, "Failed to close (we probably didn't flush the queue successfully): %s (%d)", strerror(errnum), errnum);
    }
}

static
void _uploader_task(void *p)
{
    const TickType_t ticks_to_wait = 10000 / portTICK_PERIOD_MS;
    int conn_fd = -1;
    struct ip4_addr addr;
    bool resolved = false;
    char const *host_name = NULL;
    uint16_t port = 0;

    control_get_config_device_info(&host_name, &port, NULL);

    ESP_LOGI(TAG, "Worker task is starting");

    do {
        EventBits_t bits = xEventGroupWaitBits(_uploader_control,
                STATUS_BIT_WIFI_CONNECTED | STATUS_BIT_WIFI_DISCONNECT | STATUS_BIT_TERMINATE,
                pdTRUE,
                pdFALSE,
                ticks_to_wait);

        if (bits & STATUS_BIT_WIFI_CONNECTED) {
            /* We've been woken up */
            if (false == resolved) {
                ESP_LOGI(TAG, "Resolving host %s...", host_name);
                if (_uploader_resolve_host(host_name, &addr)) {
                    ESP_LOGE(TAG, "Failed to resolve host %s, aborting", host_name);
                    abort();
                }
                resolved = true;
            }

            if (conn_fd != -1) {
                ESP_LOGW(TAG, "We already have a valid file descriptor, closing it.");
                close(conn_fd);
                conn_fd = -1;
            }

            size_t nr_devs = 0;
            if (0 != (nr_devs = device_tracker_nr_devs(NULL))) {
                if (_uploader_connect(addr, port, &conn_fd)) {
                    ESP_LOGE(TAG, "Failed to connect to uploader target, aborting.");
                    control_task_signal_wifi_failure();
                    esp_wifi_disconnect();
                    continue;
                }

                if (_uploader_deliver_device_info(conn_fd)) {
                    ESP_LOGE(TAG, "Failed to deliver device information to backend, aborting.");
                    control_task_signal_wifi_failure();
                    continue;
                }

                _uploader_terminate(conn_fd);
                conn_fd = -1;
            }

            tcpip_adapter_stop(TCPIP_ADAPTER_IF_STA);
            esp_wifi_disconnect();
            control_task_signal_wifi_done();
        } else if (bits & STATUS_BIT_WIFI_DISCONNECT) {
            /* We've been instructed to stop posting results */
            ESP_LOGI(TAG, "Wifi connection terminated, ensuring we clean up LwIP resources");
            if (-1 != conn_fd) {
                _uploader_terminate(conn_fd);
                conn_fd = -1;
            }
            uploader_shutdown();
            control_task_signal_wifi_down();

        } else if (bits & STATUS_BIT_TERMINATE) {
            /* We've been instructed to shut down */
        } else {
            /* We timed out. Do any housekeeping we need and go back to sleep. */
        }
    } while (1);
}

static
void _uploader_create_task(void)
{
    int ret = xTaskCreate(_uploader_task,
                          UPLOADER_TASK_NAME,
                          UPLOADER_TASK_STACK_WORDS,
                          NULL,
                          UPLOADER_TASK_PRIORITY,
                          &_uploader_task_hdl);

    if (pdPASS != ret) {
        ESP_LOGE(TAG, "Failed to create uploader worker task (result=%d)", ret);
    }
}

/**
 * Initialize the uploader. Sets up the worker thread, initializes the basic
 * wifi functionality, etc.
 */
void uploader_init(void)
{
    wifi_config_t wifi_cfg;

    memset(&wifi_cfg, 0, sizeof(wifi_cfg));

    tcpip_adapter_init();

    _uploader_control = xEventGroupCreate();

    /* Register the wifi event listener */
    ESP_ERROR_CHECK(esp_event_loop_init(_uploader_wifi_evt_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    control_get_config_wifi((char *)wifi_cfg.sta.ssid, (char *)wifi_cfg.sta.password);

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg));

    /* Create the worker thread */
    _uploader_create_task();
}

void uploader_shutdown(void)
{
    ESP_LOGI(TAG, "Shutting down the wireless interface.");
    ESP_ERROR_CHECK(esp_wifi_stop());
}

void uploader_connect(void)
{
    ESP_ERROR_CHECK(esp_wifi_start());
}

