#include "uploader.h"
#include "control.h"
#include "device.h"

#include "root_cert.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/apps/sntp.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_pm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include <mbedtls/config.h>
#include <mbedtls/net_sockets.h>
#include <mbedtls/ssl.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>

#ifdef MBEDTLS_DEBUG_C
#include <mbedtls/debug.h>
#endif /* defined(MBEDTLS_DEBUG_C) */

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
int _uploader_tls_write(mbedtls_ssl_context *ssl, void *data, size_t len)
{
    int ret = -1;
    size_t data_written = 0;

    if (0 == len || NULL == data) {
        ESP_LOGW(TAG, "Was asked to write %p of length %zu, probably a programming error.", data, len);
        ret = 0;
        goto done;
    }

    do {
        int wret = mbedtls_ssl_write(ssl, data + data_written, len - data_written);
        if (wret < 0) {
            int errnum = errno;
            if (wret == MBEDTLS_ERR_SSL_WANT_READ || wret == MBEDTLS_ERR_SSL_WANT_WRITE) {
                /* We need to feed the socket, so continue */
                continue;
            }

            ESP_LOGE(TAG, "Failed to send data, aborting: %s (%d) (%d)", strerror(errnum), errnum, wret);
            goto done;
        } else {
            data_written += wret;
        }
    } while (len < data_written);

    ret = 0;
done:
    return ret;
}

static
int _uploader_tls_read(mbedtls_ssl_context *ssl, void *data, size_t len)
{
    int ret = -1;
    size_t data_read = 0;

    do {
        int rret = mbedtls_ssl_read(ssl, data + data_read, len - data_read);

        if (rret < 0) {
            int errnum = errno;
            if (rret == MBEDTLS_ERR_SSL_WANT_READ || rret == MBEDTLS_ERR_SSL_WANT_WRITE) {
                /* We need to feed the socket, so continue */
                continue;
            }

            ESP_LOGE(TAG, "Failed to receive data, aborting %s (%d) (%d)",  strerror(errnum), errnum, rret);
            goto done;
        } else {
            data_read += rret;
        }
    } while (len < data_read);

    ret = 0;
done:
    return ret;
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
int _uploader_connect(struct ip4_addr addr, uint16_t port, int *pfd, mbedtls_ssl_config *conf, mbedtls_ssl_context *ctx, mbedtls_net_context *server_ctx)
{
    int ret = -1;

    int fd = -1,
        r = 0,
        hret = 0;
    struct sockaddr_in sin;

    if (NULL == pfd) {
        ESP_LOGE(TAG, "Must specify a non-null pointer to receive the file descriptor");
        abort();
    }

    if (NULL == ctx || NULL == ctx) {
        ESP_LOGE(TAG, "Programmer error: must specify pointers to recieve TLS contexts, aborting");
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

    server_ctx->fd =fd;

    ESP_LOGI(TAG, "Connected, initiating TLS");

    mbedtls_ssl_set_bio(ctx, server_ctx, mbedtls_net_send, mbedtls_net_recv, NULL);

    do {
        if (0 > (hret = mbedtls_ssl_handshake(ctx))) {
            int errnum = errno;
            if (hret == MBEDTLS_ERR_SSL_WANT_WRITE || hret == MBEDTLS_ERR_SSL_WANT_READ) {
                hret = 0;
                continue;
            }
            ESP_LOGE(TAG, "Handshake failed (%d), aborting. (%d) - %s", hret, errnum, strerror(errnum));
            goto done;
        }
    } while (0 != hret);

    *pfd = fd;

    ret = 0;
done:
    if (0 != ret) {
        mbedtls_ssl_free(ctx);

        if (-1 != fd) {
            close(fd);
            fd = -1;
        }
    }
    return ret;
}

static
int _uploader_deliver_device_info(mbedtls_ssl_context *ssl)
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

    if (NULL == ssl) {
        ESP_LOGE(TAG, "Programmer error: SSL == NULL");
        abort();
    }

    if (_uploader_tls_write(ssl, &helo, sizeof(helo))) {
        int errnum = errno;
        ESP_LOGE(TAG, "Failed to send HELO: %s (%d)", strerror(errnum), errnum);
        goto done;
    }

    list_for_each_type_safe(dev, dev_tmp, &tracker.device_list, d_node) {
        record_id++;
        assert(2 <= dev->encoded_obj_len);

        /* Write the packed device record; this includes the header */
        if (_uploader_tls_write(ssl, dev->encoded_obj, dev->encoded_obj_len)) {
            int errnum = errno;
            ESP_LOGE(TAG, "Failed to send device record, aborting: %s (%d)", strerror(errnum), errnum);
            goto done;
        }

        ESP_LOGI(TAG, "Sent device %zu to the backend (length = %u)", record_id, (unsigned)dev->encoded_obj_len);

        /* Remove the device record, free any deeply-used memory */
        if (device_tracker_remove(&tracker, dev)) {
            ESP_LOGE(TAG, "Unexpected failure while removing object, aborting.");
            abort();
        }

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
void _uploader_terminate(int fd, mbedtls_ssl_context *ssl)
{
    uint16_t confirm = 0;
    int shdn = 0;
    unsigned shdn_count = 0;

    if (_uploader_tls_read(ssl, &confirm, sizeof(confirm))) {
        ESP_LOGE(TAG, "End of stream while waiting for confirmation, aborting.");
    }

    ESP_LOGI(TAG, "Terminating the TLS connection");

    do {
        shdn = mbedtls_ssl_close_notify(ssl);
        if (0 > shdn) {
            if (shdn == MBEDTLS_ERR_SSL_WANT_READ || shdn == MBEDTLS_ERR_SSL_WANT_WRITE) {
                /* Keep on truckin' */
                shdn = 0;
                continue;
            }
            ESP_LOGE(TAG, "A fatal error %d occurred during SSL shutdown, SSL shutdown will be unclean", shdn);
        }
        ESP_LOGI(TAG, "Waiting for remote end to shut down -- trying again");
    } while (0 == shdn && shdn_count++ < 10);

    if (shdn_count >= 10) {
        ESP_LOGE(TAG, "Shutdown was likely not clean, we tried %d times over.", shdn_count);
    }

    mbedtls_ssl_free(ssl);

    ESP_LOGI(TAG, "Now closing the socket for real");

    lwip_shutdown(fd, SHUT_RDWR);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    if (0 != close(fd)) {
        int errnum = errno;
        ESP_LOGE(TAG, "Failed to close (we probably didn't flush the queue successfully): %s (%d)", strerror(errnum), errnum);
    }
    fd = -1;
}

#ifdef MBEDTLS_DEBUG_C
static
void _uploader_tls_debug(void *ctx, int level, const char *file, int line, const char *str)
{
    printf("TLS(%d): %s (%s:%d)", level, str, file, line);
}
#endif /* defined(MBEDTLS_DEBUG_C) */

static
void _uploader_task(void *p)
{
    const TickType_t ticks_to_wait = 10000 / portTICK_PERIOD_MS;
    struct ip4_addr addr;
    bool resolved = false;
    char const *host_name = NULL;
    uint16_t port = 0;
    int mret = 0;
    uint32_t dev_id = 0;

    mbedtls_ssl_config tls_conf;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_x509_crt ca_cert;

    mbedtls_ctr_drbg_init(&ctr_drbg);
    mbedtls_ssl_config_init(&tls_conf);
    mbedtls_entropy_init(&entropy);
    mbedtls_x509_crt_init(&ca_cert);

    control_get_config_device_info(&host_name, &port, &dev_id);

    /* Prepare the DRBG to be seeded from the given entropy context. Integrate the
     * device ID into the stream.
     */
    ESP_LOGI(TAG, "Preparing the DRBG");

    if (0 != (mret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy, (unsigned char *)&dev_id, sizeof(dev_id)))) {
        ESP_LOGE(TAG, "Failed to seed DRBG, aborting.");
        abort();
    }

    /* Set up the TLS session configuration */
    if (0 != (mret = mbedtls_ssl_config_defaults(&tls_conf, MBEDTLS_SSL_IS_CLIENT, MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT))) {
        ESP_LOGE(TAG, "Failed to set config defaults, aborting.");
        abort();
    }

    /* Attach the CTR DRBG we've initialized */
    mbedtls_ssl_conf_rng(&tls_conf, mbedtls_ctr_drbg_random, &ctr_drbg);

	/* Load the DER encoded certificate embedded in the firmware image */
    if(0 != mbedtls_x509_crt_parse_der(&ca_cert, ca_cert_der, sizeof(ca_cert_der))) {
		ESP_LOGE(TAG, "Failed to load trusted certificate store, aborting.");
		abort();
    }

    mbedtls_ssl_conf_ca_chain(&tls_conf, &ca_cert, NULL);
    mbedtls_ssl_conf_authmode(&tls_conf, MBEDTLS_SSL_VERIFY_REQUIRED);

#ifdef MBEDTLS_DEBUG_C
    mbedtls_debug_set_threshold(2);
    mbedtls_ssl_conf_dbg(&tls_conf, _uploader_tls_debug, NULL);
    ESP_LOGI(TAG, "Enabled debugging output from mbedtls");
#endif /* defined(MBEDTLS_DEBUG_C) */

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

            size_t nr_devs = 0;
            if (0 != (nr_devs = device_tracker_nr_devs(NULL))) {
                int conn_fd = -1;
                mbedtls_ssl_context ssl;
                mbedtls_net_context server_ctx;

                /* Bump clock to maximum speed while negotiating TLS session */
                esp_pm_lock_acquire(ESP_PM_CPU_FREQ_MAX);

                mbedtls_ssl_init(&ssl);
                mbedtls_net_init(&server_ctx);

                if (0 != mbedtls_ssl_setup(&ssl, &tls_conf)) {
                    /* Back to normal clock */
                    esp_pm_lock_release(ESP_PM_CPU_FREQ_MAX);
                    ESP_LOGE(TAG, "Failed to set up SSL context, aborting.");
                    mbedtls_net_free(&server_ctx);
                    control_task_signal_wifi_failure();
                    esp_wifi_disconnect();
                    continue;
                }

                if (_uploader_connect(addr, port, &conn_fd, &tls_conf, &ssl, &server_ctx)) {
                    /* Back to normal clock */
                    esp_pm_lock_release(ESP_PM_CPU_FREQ_MAX);
                    ESP_LOGE(TAG, "Failed to connect to uploader target, aborting.");
                    mbedtls_net_free(&server_ctx);
                    control_task_signal_wifi_failure();
                    esp_wifi_disconnect();
                    continue;
                }
                /* Back to normal clock, now that we're done with setting up the TLS session */
                esp_pm_lock_release(ESP_PM_CPU_FREQ_MAX);

                if (_uploader_deliver_device_info(&ssl)) {
                    ESP_LOGE(TAG, "Failed to deliver device information to backend, aborting.");
                    control_task_signal_wifi_failure();
                }

                _uploader_terminate(conn_fd, &ssl);
                mbedtls_net_free(&server_ctx);
                conn_fd = -1;
            }

            tcpip_adapter_stop(TCPIP_ADAPTER_IF_STA);
            esp_wifi_disconnect();
            control_task_signal_wifi_done();
        } else if (bits & STATUS_BIT_WIFI_DISCONNECT) {
            /* We've been instructed to stop posting results */
            ESP_LOGI(TAG, "Wifi connection terminated, ensuring we clean up LwIP resources");
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

