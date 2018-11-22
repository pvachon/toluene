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

#include "freertos/FreeRTOS.h"

#define TAG                         "HUFFER"

#define CONFIG_BLE_SCAN_DURATION    300

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static
bool connecting = false;

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
    bool found_service;
    bool connected;
    struct device *dev_info;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static
struct gattc_profile_inst active = {
    .gattc_cb = gattc_profile_event_handler,
    .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
};

static
void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;
    struct device *dev = active.dev_info;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(TAG, "ESP_GATTC_REG_EVT");
        break;

    case ESP_GATTC_CONNECT_EVT: {
        //ESP_LOGI(TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        active.conn_id = p_data->connect.conn_id;
        memcpy(active.remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(TAG, "config MTU error, error code = %x", mtu_ret);
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
        break;

    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
            esp_ble_gattc_close(gattc_if, p_data->cfg_mtu.conn_id);
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

#if 0
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }else{
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         active.conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,
                                                                         active.service_start_handle,
                                                                         active.service_end_handle,
                                                                         active.char_handle,
                                                                         &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(TAG, "esp_ble_gattc_get_attr_count error");
            }
            if (count > 0){
                esp_gattc_descr_elem_t *descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(TAG, "malloc error, gattc no mem");
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         active.conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    }
                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                     active.conn_id,
                                                                     descr_elem_result[0].handle,
                                                                     sizeof(notify_en),
                                                                     (uint8_t *)&notify_en,
                                                                     ESP_GATT_WRITE_TYPE_RSP,
                                                                     ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(TAG, "esp_ble_gattc_write_char_descr error");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(TAG, "decsr not found");
            }

        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }else{
            ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        esp_log_buffer_hex(TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(TAG, "write descr success ");
        uint8_t write_char_data[35];
        for (int i = 0; i < sizeof(write_char_data); ++i)
        {
            write_char_data[i] = i % 256;
        }
        esp_ble_gattc_write_char( gattc_if,
                                  active.conn_id,
                                  active.char_handle,
                                  sizeof(write_char_data),
                                  write_char_data,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(TAG, "write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(TAG, "write char success ");
        break;
#endif
    case ESP_GATTC_DISCONNECT_EVT:
        active.connected = false;
        active.found_service = false;
        active.dev_info = NULL;
        connecting = false;
        device_on_disconnect(dev, p_data->disconnect.reason);
        //ESP_LOGI(TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        control_task_signal_ble_gatt_disconnect();
        break;
    default:
        break;
    }
}

static
void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
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
            /* TODO: need to cope with advertising data changing over time */
            if (device_tracker_find(&tracker, &dev, scan_result->scan_rst.bda)) {
                ESP_LOGI(TAG, "%02x:%02x:%02x-%02x:%02x:%02x -> addr_type: %s (%d) evt_type: %d searchee Adv Data Len %d, Scan Response Len %d",
                        bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
                        scan_result->scan_rst.ble_addr_type == BLE_ADDR_TYPE_PUBLIC ? "public" : "private",
                        scan_result->scan_rst.ble_addr_type,
                        scan_result->scan_rst.ble_evt_type,
                        scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);

                dev = device_new(bda, scan_result->scan_rst.ble_addr_type == BLE_ADDR_TYPE_PUBLIC,
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
                }
            }

            if (NULL == dev) {
                ESP_LOGE(TAG, "Device is NULL, incoming panic...");
            }

            if (false == connecting &&
                    true == dev->connectable &&
                    false == dev->interrogated &&
                    dev->nr_interrogations < DEV_MAX_INTERROGATIONS)
            {
                ESP_LOGI(TAG, "Connecting to device %02x:%02x:%02x-%02x:%02x:%02x to interrogate it...",
                        bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
                connecting = true;
                active.dev_info = dev;
                dev->nr_interrogations++;
                esp_ble_gap_stop_scanning();
                esp_ble_gattc_open(active.gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
            }

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
        control_task_signal_ble_scan_paused();
        ESP_LOGI(TAG, "Scan Stopped (have found %zu devices, using %zu bytes of memory)", device_tracker_nr_devs(NULL), device_tracker_nr_bytes_used(NULL));
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

static
void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* Grab the gattc_if handle for our generic profile connector */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            active.gattc_if = gattc_if;
        } else {
            ESP_LOGI(TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            gattc_if == active.gattc_if) {
        if (active.gattc_cb) {
            active.gattc_cb(event, gattc_if, param);
        }
    }
}

void app_main()
{
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    device_tracker_init(&tracker);

    uploader_init();
    control_task_init();

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
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(TAG, "%s gap register failed, error code = %x\n", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
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

    control_task_signal_ble_ready();

    ESP_LOGI(TAG, "We are on the air!");
}

