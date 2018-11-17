#include "uploader.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"

#define TAG                                 "UPLOADER"

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

#define CONFIG_TARGET_HOST                  "51.15.228.192"

#define CONFIG_ESSID                        ""
#define CONIFG_WPA2_PSK_PASSWORD            "russians"

static
esp_err_t _uploader_wifi_evt_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        /* Actually initiate the connection to the stored AP */
        ESP_LOGI(TAG, "Connecting to access point, please wait...");
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        /* Once we have an IP address, wake up the uploader task */
        xEventGroupSetBits(_uploader_control, STATUS_BIT_WIFI_CONNECTED);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
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

done:
    return ret;
}

static
void _uploader_task(void *p)
{
    const TickType_t ticks_to_wait = 10000 / portTICK_PERIOD_MS;
    int conn_fd = -1;
    struct ip4_addr addr;
    bool resolved = false;

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
                ESP_LOGI(TAG, "Resolving host " CONFIG_TARGET_HOST "...");
                if (_uploader_resolve_host(CONFIG_TARGET_HOST, &addr)) {
                    ESP_LOGE(TAG, "Failed to resolve host " CONFIG_TARGET_HOST ", aborting");
                    abort();
                }
                resolved = true;
            }

            if (conn_fd != -1) {
                ESP_LOGW(TAG, "We already have a valid file descriptor, closing it.");
                close(conn_fd);
                conn_fd = -1;
            }
        } else if (bits & STATUS_BIT_WIFI_DISCONNECT) {
            /* We've been instructed to stop posting results */
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
    tcpip_adapter_init();

    _uploader_control = xEventGroupCreate();

    /* Register the wifi event listener */
    ESP_ERROR_CHECK(esp_event_loop_init(_uploader_wifi_evt_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    /* TODO: read the wifi config from NVS */
    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = "Something",
            .password = "Something Else",
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg));

    /* Create the worker thread */
    _uploader_create_task();
}

/**
 * Shut down the uploader. Terminates the worker thread, cleans up the wifi functionality
 */
void uploader_shutdown(void)
{
    ESP_LOGI(TAG, "Shutting down the wireless interface, for good.");
    ESP_ERROR_CHECK(esp_wifi_disconnect());
}

/**
 * Enqueue a device record to be uploaded. The uploader worker thread will
 * free this memory.
 */
int uploader_enqueue_record(void *record, size_t length)
{
    int ret = -1;

    return ret;
}

