#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "device.h"
#include "control.h"
#include "uploader.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "esp_pm.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#define TAG                                         "HUFFER"

#define CONFIG_BLE_SCAN_DURATION                    10

#define HOOVER_TASK_NAME                           "hoover"
#define HOOVER_TASK_STACK_WORDS                    8192
#define HOOVER_TASK_PRIORITY                       9

extern void control_gpio_setup(void);

struct gattc_profile_inst {
    struct device *dev_info;
};

struct device *active_dev = NULL;
esp_gatt_if_t active_gattc_if = ESP_GATT_IF_NONE;

/**
 * Event group for pending hoover thread events
 */
static
EventGroupHandle_t _hoover_start;

/**
 * Hoover task thread handle
 */
static
xTaskHandle _hoover_task_hdl;

#define HOOVER_START_BIT_GO                 (1 << 0)
#define HOOVER_DEVICE_DONE                  (1 << 1)

void hoover_wake(void)
{
    xEventGroupSetBits(_hoover_start, HOOVER_START_BIT_GO);
}

void hoover_device_done(void)
{
    xEventGroupSetBits(_hoover_start, HOOVER_DEVICE_DONE);
}

/**
 * Record for a device pending a BLE scan. This is usually contained in a ring buffer.
 */
struct device_pending_scan {
    esp_bd_addr_t bda;
    esp_ble_addr_type_t remote_addr_type;
};

#define CONFIG_BLE_NUM_PENDING_DEVICES              32 /* Size of the pending ring */
#define CONFIG_BLE_CANCEL_SCAN_DEVICES              16 /* Cancel the scan when we're "half full" due to potential late arrivals */
#define BLE_PENDING_DEVICES_MASK                    (CONFIG_BLE_NUM_PENDING_DEVICES - 1)

static
struct device_pending_scan _pending[CONFIG_BLE_NUM_PENDING_DEVICES];

static
uint32_t _pending_head = 0;

static
uint32_t _pending_tail = 0;

static
bool _pending_pop_head(esp_bd_addr_t *bda, esp_ble_addr_type_t *ptype)
{
    if (_pending_head == _pending_tail) {
        return false;
    }

    memcpy(bda, _pending[_pending_tail].bda, 6);
    *ptype = _pending[_pending_tail].remote_addr_type;

    _pending_tail = (_pending_tail + 1) & BLE_PENDING_DEVICES_MASK;

    return true;
}

static
bool _pending_push_head(esp_bd_addr_t bda, esp_ble_addr_type_t type)
{
    if (_pending_tail == ((_pending_head + 1) & BLE_PENDING_DEVICES_MASK)) {
        return false;
    }

    /* Copy in the parameters to be saved */
    memcpy(_pending[_pending_head].bda, bda, 6);
    _pending[_pending_head].remote_addr_type = type;

    _pending_head = (_pending_head + 1) & BLE_PENDING_DEVICES_MASK;

    return true;
}

static
bool _pending_has_items(void)
{
    return _pending_head != _pending_tail;
}

static
unsigned _pending_nr_items(void)
{
    return (_pending_head - _pending_tail) & BLE_PENDING_DEVICES_MASK;
}

/**
 * State machine hooks for hoovering up all readable attributes and data for a particular device.
 *
 * This hooks into the ESP32's BLE state machine and fills in the relevant information as the
 * server is successfully scanned.
 */
static
void hoover_gatt_client_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;
    struct device *dev = active_dev;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(TAG, "ESP_GATTC_REG_EVT");
        active_gattc_if = gattc_if;
        break;

    case ESP_GATTC_CONNECT_EVT: {
        //ESP_LOGI(TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(TAG, "Error during connection, error code = %x", mtu_ret);
            esp_ble_gattc_close(gattc_if, p_data->connect.conn_id);
        }
        break;
    }

    case ESP_GATTC_OPEN_EVT:
        //ESP_LOGI(TAG, "ESP_GATTC_OPEN_EVT");
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGI(TAG, "open failed, status %d", p_data->open.status);
            esp_ble_gattc_close(gattc_if, p_data->open.conn_id);
            break;
        }
        active_dev->interrogated = true;
        break;

    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGW(TAG,"Setting large MTU failed, status code:%x", param->cfg_mtu.status);
        }
        //ESP_LOGI(TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        if (device_on_open(dev, gattc_if, p_data->cfg_mtu.conn_id)) {
            esp_ble_gattc_close(gattc_if, p_data->cfg_mtu.conn_id);
        }
        break;

    case ESP_GATTC_SEARCH_RES_EVT: {
        //ESP_LOGI(TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        //ESP_LOGI(TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (device_on_found_service(dev, gattc_if, p_data->open.conn_id, &p_data->search_res.srvc_id.uuid, p_data->search_res.start_handle, p_data->search_res.end_handle)) {
            ESP_LOGI(TAG, "Did not find services, aborting.");
            esp_ble_gattc_close(gattc_if, p_data->search_res.conn_id);
        }
        break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }

        if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGE(TAG, "Get service information from flash");
        }

        if (device_on_search_finished(dev, gattc_if, p_data->search_cmpl.conn_id)) {
            esp_ble_gattc_close(gattc_if, p_data->search_cmpl.conn_id);
        }
        break;

    case ESP_GATTC_READ_CHAR_EVT:
    case ESP_GATTC_READ_MULTIPLE_EVT:
    {
        if (device_on_read_characteristic(dev, gattc_if, p_data->read.conn_id, p_data->read.value, p_data->read.value_len, p_data->read.handle)) {
            ESP_LOGE(TAG, "Failure during characteristic read, disconnecting.");
            esp_ble_gattc_close(gattc_if, p_data->read.conn_id);
        }

        break;
    }

    case ESP_GATTC_DISCONNECT_EVT:
        if (device_on_disconnect(dev, p_data->disconnect.reason)) {
            ESP_LOGE(TAG, "Error: failure while serializing this device, aborting");
            abort();
        }
        active_dev = NULL;
        hoover_device_done();
        break;
    default:
        break;
    }
}

/**
 * Thread that manages pending device queries
 */
static
void _hoover_task_thread(void *p)
{
    /* We want this thread to only wake up every 10 seconds for housekeeping */
    const TickType_t ticks_to_wait = 10000 / portTICK_PERIOD_MS;
    unsigned idle_wakes = 0;
    size_t last_nr_items = 0;

    ESP_LOGE(TAG, "Starting the Hoover Thread");

    do {
        EventBits_t bits = xEventGroupWaitBits(_hoover_start,
                HOOVER_START_BIT_GO |
                HOOVER_DEVICE_DONE
                ,
                pdTRUE,
                pdFALSE,
                ticks_to_wait);

        if (bits & HOOVER_START_BIT_GO) {
            size_t nr_pending_items = 0;
            idle_wakes = 0;
            if (active_gattc_if == ESP_GATT_IF_NONE) {
                ESP_LOGE(TAG, "Fatal -- GATT interface is not set, something is awry.");
                abort();
            }

            control_task_signal_ble_hoover_start();

            nr_pending_items = device_tracker_nr_devs(NULL);

            if (false == _pending_has_items() && nr_pending_items == last_nr_items) {
                /* Sleep for a while, take a load off your feet */
                ESP_LOGI(TAG, "Nothing new; sleeping for 10 seconds");
                vTaskDelay((10 * 1000)/portTICK_PERIOD_MS);
            }

            last_nr_items = nr_pending_items;

            /* Clear the queue of pending items to scan */
            while (_pending_has_items()) {
                esp_bd_addr_t bda;
                esp_ble_addr_type_t type;
                struct device *dev = NULL;

                if (false == _pending_pop_head(&bda, &type)) {
                    ESP_LOGE(TAG, "Fatal error while trying to pop the head (but we think we have items left) (head=%u, tail=%u)", _pending_head, _pending_tail);
                    abort();
                }

                if (device_tracker_find(&tracker, &dev, bda)) {
                    ESP_LOGE(TAG, "ERROR: Got BDA %02x:%02x:%02x-%02x:%02x:%02x, failed to find in tracker",
                            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
                    /* Skip the rest of processing */
                    continue;
                }

                ESP_LOGI(TAG, "Scanning device %02x:%02x:%02x-%02x:%02x:%02x",
                            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
                active_dev = dev;

                /* Signal to open the GATT server and let the GATT client state machine take over */
                if (ESP_OK != esp_ble_gattc_open(active_gattc_if, bda, type, true)) {
                    ESP_LOGE(TAG, "Fatal error while trying to initiate connection, marking as failed");
                    if (device_on_disconnect(dev, 0)) {
                        ESP_LOGE(TAG, "FATAL: Could not finalize device state, aborting.");
                        abort();
                    }
                }

                /* Sleep until we're notified that the device interrogation is done */
                size_t nr_iters = 0;
                do {
                    EventBits_t bits = xEventGroupWaitBits(_hoover_start,
                            HOOVER_DEVICE_DONE,
                            pdTRUE,
                            pdFALSE,
                            ticks_to_wait);
                    if (bits & HOOVER_DEVICE_DONE) {
                        ESP_LOGI(TAG, "Finished hoovering device attributes, continuing");
                        break;
                    } else {
                        ESP_LOGI(TAG, "Still waiting...");
                    }
                } while (nr_iters++ < 30);

                if (nr_iters >= 30) {
                    ESP_LOGE(TAG, "Fatal error while connecting to device to hoover, and we timed out.");
                    /* TODO: we need to clean up a bit more elegantly from this happening */
                    abort();
                }
            }

            control_task_signal_ble_hoover_finished();
        } else {
            idle_wakes++;
        }
    } while (1);
}

void hoover_task_init(void)
{
    _hoover_start = xEventGroupCreate();

    int ret = xTaskCreate(_hoover_task_thread,
                          HOOVER_TASK_NAME,
                          HOOVER_TASK_STACK_WORDS,
                          NULL,
                          HOOVER_TASK_PRIORITY,
                          &_hoover_task_hdl);

    if (pdPASS != ret) {
        ESP_LOGE(TAG, "Failed to create hoover thread, aborting. (result=%d)", ret);
        abort();
    }
}

static
void hoover_esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        /* Signal that we want to scan for 300 seconds */
        esp_ble_gap_start_scanning(CONFIG_BLE_SCAN_DURATION);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        control_task_signal_ble_scan_started();
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            struct device *dev = NULL;
            uint8_t *bda = scan_result->scan_rst.bda;

            if (_pending_nr_items() == CONFIG_BLE_NUM_PENDING_DEVICES - 1) {
                ESP_LOGW(TAG, "Out of space for device information arrival; dropping on the floor.");
                break;
            }

            /* TODO: need to cope with advertising data changing over time */
            if (device_tracker_find(&tracker, &dev, scan_result->scan_rst.bda)) {
                ESP_LOGI(TAG, "%02x:%02x:%02x-%02x:%02x:%02x -> addr_type: %s (%d) evt_type: %d searchee Adv Data Len %d, Scan Response Len %d",
                        bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
                        scan_result->scan_rst.ble_addr_type == BLE_ADDR_TYPE_PUBLIC ? "public" : "private",
                        scan_result->scan_rst.ble_addr_type,
                        scan_result->scan_rst.ble_evt_type,
                        scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);

                dev = device_new(control_get_config_sensor_id(), bda, scan_result->scan_rst.ble_addr_type == BLE_ADDR_TYPE_PUBLIC,
                        scan_result->scan_rst.ble_evt_type,
                        scan_result->scan_rst.ble_evt_type == ESP_BLE_EVT_CONN_ADV,
                        scan_result->scan_rst.adv_data_len,
                        scan_result->scan_rst.scan_rsp_len,
                        scan_result->scan_rst.ble_adv);

                if (device_tracker_insert(&tracker, dev)) {
                    ESP_LOGE(TAG, "Failed to insert device, aieeee!");
                }

                if (false == dev->connectable) {
                    /* Finalize this device as well */
                    if (device_on_disconnect(dev, 0)) {
                        ESP_LOGE(TAG, "Failed to finalize unconnectable device, aborting.");
                    }
                } else {
                    ESP_LOGI(TAG, "Scheduling to interrogate device %02x:%02x:%02x-%02x:%02x:%02x (pending = %u)",
                        bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
                        _pending_nr_items());
                    _pending_push_head(scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type);

                    if (_pending_nr_items() >= CONFIG_BLE_CANCEL_SCAN_DEVICES) {
                        esp_ble_gap_stop_scanning();
                    }
                }
            }

            assert(NULL != dev);
            break;
        }
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGI(TAG, "ALL DONE! Signaling control thread. I found %zu devices (%zu bytes)", device_tracker_nr_devs(NULL), device_tracker_nr_bytes_used(NULL));
            control_task_signal_ble_scan_complete();
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        control_task_signal_ble_scan_complete();
        ESP_LOGI(TAG, "Scan Stopped due to other condition (have found %zu devices, using %zu bytes of memory)", device_tracker_nr_devs(NULL), device_tracker_nr_bytes_used(NULL));
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void setup_uart(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);

    const uart_config_t uart_config = {
        .baud_rate = CONFIG_CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .use_ref_tick = true
    };

	ESP_ERROR_CHECK(uart_param_config(CONFIG_CONSOLE_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_CONSOLE_UART_NUM, 512, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(CONFIG_CONSOLE_UART_NUM);
}

static
void ble_scanning_setup(void)
{
    esp_err_t ret = ESP_OK;

    esp_power_level_t power_level;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(hoover_esp_gap_cb);
    if (ret) {
        ESP_LOGE(TAG, "%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(hoover_gatt_client_event_handler);
    if(ret){
        ESP_LOGE(TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(0);
    if (ret){
        ESP_LOGE(TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

#if 0
    power_level = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT);
    ESP_LOGI(TAG, "Default power level: %d", power_level);
    power_level = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_SCAN);
    ESP_LOGI(TAG, "Scan power level: %d", power_level);
    power_level = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_CONN_HDL0);
    ESP_LOGI(TAG, "Connection handle 0 power level: %d", power_level);

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_N3);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_N3);
#endif
}

void setup_power_management(void)
{
    esp_pm_config_esp32_t pm = {
        .max_freq_mhz = 240,
        .min_freq_mhz = 80,
        .light_sleep_enable = true,
    };

    if (ESP_OK != esp_pm_configure((void *)&pm)) {
        ESP_LOGW(TAG, "Failed to enable power management.");
    }
}

void app_main(void)
{
    /* Before we do anything else, set the boot status LED */
    control_gpio_setup();

    //setup_power_management();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    setup_uart();
    control_load_config();

    device_tracker_init(&tracker);

    uploader_init();
    hoover_task_init();
    control_task_init();

    ble_scanning_setup();

    control_task_signal_ble_ready();

    ESP_LOGI(TAG, "We are on the air!");
}

