#include "device.h"
#include "rbtree.h"
#include "list.h"

#include "rom/crc.h"

#include "esp_log.h"

#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "encoder.h"

#include <string.h>
#include <sys/time.h>

#define TAG                                             "DEVICE"
#define HOOVER                                          "HOOVER"

#define BLE_SERVICE_GENERIC_ACCESS                      0x1800
#define BLE_SERVICE_GENERIC_ACCESS_ATTR_NAME            0x2a00
#define BLE_SERVICE_GENERIC_ACCESS_ATTR_APPEARANCE      0x2a01

#define BLE_SERVICE_DEVICE_INFO                         0x180a
#define BLE_SERVICE_DEVICE_INFO_ATTR_MANUF_NAME         0x2a29
#define BLE_SERVICE_DEVICE_INFO_ATTR_MODEL_NUMBER       0x2a24
#define BLE_SERVICE_DEVICE_INFO_ATTR_SERIAL_NUMBER      0x2a25
#define BLE_SERVICE_DEVICE_INFO_ATTR_FIRMWARE_REV       0x2a26
#define BLE_SERVICE_DEVICE_INFO_ATTR_SOFTWARE_REV       0x2a28
#define BLE_SERVICE_DEVICE_INFO_ATTR_SYSTEM_ID          0x2a23
#define BLE_SERVICE_DEVICE_INFO_ATTR_PNP_ID             0x2a50

#define INVALID_HANDLE                                  0xffff

struct device_tracker tracker;

size_t device_tracker_nr_devs(struct device_tracker *trk)
{
    if (NULL != trk) {
        return trk->nr_devices;
    } else {
        return tracker.nr_devices;
    }
}

size_t device_tracker_nr_bytes_used(struct device_tracker *trk)
{
    if (NULL != trk) {
        return trk->mem_bytes_used;
    } else {
        return tracker.mem_bytes_used;
    }
}

static
int32_t _hash_mac(uint8_t const *mac)
{
    return (int32_t)(crc32_le(0xfffffffful, mac, DEVICE_MAC_ADDR_LEN) & 0x7fffffff);
}

static
int _device_rb_tree_cmp(const void *lhs, const void *rhs)
{
    int32_t l_crc = (int32_t)lhs;
    int32_t r_crc = (int32_t)rhs;
    return l_crc - r_crc;
}

int device_on_open(struct device *dev, esp_gatt_if_t gattc_if, uint16_t conn_id)
{
    int ret = 1;

    if (dev->state != DEVICE_NOT_CONNECTED) {
        ESP_LOGE(TAG, "Unknown device state at open time, aborting");
        goto done;
    }

    /* Start things off by searching for all the services exposed */
    esp_ble_gattc_search_service(gattc_if, conn_id, NULL);
    dev->state = DEVICE_SEARCH_SERVICE;

    ret = 0;

done:
    return ret;
}

/**
 * Send a request to read a single GATT attribute.
 */
static
int _device_request_read_attrib(struct device *dev, esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t handle)
{
    int ret = 1;

    int status = ESP_GATT_OK;

    /* Enqueue a read request for attribute */
    if (ESP_GATT_OK != (status = esp_ble_gattc_read_char(gattc_if, conn_id, handle, ESP_GATT_AUTH_REQ_NONE))) {
        ESP_LOGE(TAG, "Failed to enqueue request for handle %04x for attribute.", handle);
        goto done;
    }

    dev->state = DEVICE_READ_ATTR;

    ret = 0;
done:
    return ret;
}

static
int _device_capture_all_service_attribs(struct device *dev, struct ble_service *svc, esp_gatt_if_t gattc_if, uint16_t conn_id, uint16_t start_hdl, uint16_t end_hdl)
{
    int ret = -1;

    uint16_t count = 0;
    int esp_err = 0;

    esp_gattc_char_elem_t *chars = NULL;

    ESP_LOGD(HOOVER, "Capturing attributes for conn %u %04x to %04x", conn_id, start_hdl, end_hdl);

    if (ESP_GATT_OK != esp_ble_gattc_get_attr_count(gattc_if, conn_id,
                ESP_GATT_DB_CHARACTERISTIC, start_hdl, end_hdl, INVALID_HANDLE, &count))
    {
        ESP_LOGE(TAG, "Failed to get service attribute count, aborting");
        goto done;
    }

    if (NULL == (chars = malloc(count * sizeof(*chars)))) {
        ESP_LOGE(TAG, "Failed to allocate memory for %u attributes", (unsigned)count);
        goto done;
    }

    /* Get the service characteristics */
    if (ESP_GATT_OK != (esp_err = esp_ble_gattc_get_all_char(gattc_if, conn_id, start_hdl, end_hdl, chars, &count, 0))) {
        ESP_LOGE(TAG, "Failed to get characteristics for this service, aborting (reason = %d, expected %u chars, if=%u).", esp_err, count, gattc_if);
        goto done;
    }

    /* Now add all the attributes */
    for (size_t i = 0; i < count; i++) {
        if (ble_service_add_attribute(svc, &chars[i].uuid, chars[i].char_handle, !!(chars[i].properties & ESP_GATT_CHAR_PROP_BIT_READ))) {
            ESP_LOGE(TAG, "Failed to add attribute descriptor for conn %u handle %04u to service, skipping", conn_id, chars[i].char_handle);
        }
    }

    ret = 0;
done:
    free(chars);
    chars = NULL;

    return ret;
}

static
int _device_hoover_service_attribs(struct device *dev, esp_gatt_if_t gattc_if, uint16_t conn_id)
{
    int ret = 1;

    struct ble_service *svc = NULL;

    ESP_LOGD(HOOVER, "Hoovering attributes starting at characteristic %zu (of %zu) (service %zu)", dev->cur_char, dev->nr_chars, dev->cur_svc);

    if (0 == dev->nr_chars) {
        if (ble_object_get_service(dev->obj, dev->cur_svc, &svc)) {
            ESP_LOGE(TAG, "Failed to get service %zu, aborting", dev->cur_svc);
            goto done;
        }

        if (ble_service_get_nr_attributes(svc, &dev->nr_chars)) {
            ESP_LOGE(TAG, "Failed to get number of attributes for active service");
            goto done;
        }
    }

    for (size_t i = dev->cur_char; i < dev->nr_chars; i++) {
        /* Check if this is one we want to read */
        bool is_read = false;
        uint16_t handle = 0;
        struct ble_service *svc = NULL;

        if (ble_object_get_service(dev->obj, dev->cur_svc, &svc)) {
            ESP_LOGE(TAG, "FATAL: failed to get service ID %zu, aborting.", dev->cur_svc);
            goto done;
        }

        if (ble_service_get_attribute(svc, dev->cur_char, &handle, &is_read)) {
            ESP_LOGE(TAG, "FATAL: failed to get char %zu on service %zu on conn %u", dev->cur_char, dev->cur_svc, conn_id);
            goto done;
        }

        if (true == is_read) {
            ESP_LOGI(TAG, "Reading characteristic %u", handle);
            if (_device_request_read_attrib(dev, gattc_if, conn_id, handle)) {
                goto done;
            }

            /* We've enqueued the read, so let the event loop take over */
            ret = 0;
            goto done;
        }

        /* Otherwise, just keep on going */
        dev->cur_char++;
    }

    if (dev->cur_char >= dev->nr_chars) {
        dev->nr_chars = 0;
        dev->cur_char = 0;
        dev->cur_svc++;
    }

    ret = 0;
done:
    return ret;
}

int device_on_found_service(struct device *dev, esp_gatt_if_t gattc_if, uint16_t conn_id, esp_bt_uuid_t const *service_uuid, uint16_t start_hdl, uint16_t end_hdl)
{
    int ret = -1;

    struct ble_service *svc = NULL;

    if (ble_object_add_service(dev->obj, &svc, service_uuid, start_hdl, end_hdl)) {
        ESP_LOGE(TAG, "Failed to add service record; proceeding with caution.");
        goto done;
    }

    ret = 0;
done:
    return ret;
}

static
int _device_schedule_close(struct device *dev)
{
    int ret = -1;

    if (NULL == dev) {
        ESP_LOGE(TAG, "Device is NULL, aborting attempt to schedule a close");
        goto done;
    }

    if (ESP_OK != esp_ble_gap_disconnect(dev->bda_addr)) {
        ESP_LOGE(TAG, "Failed to schedule GAP disconnect for device, failing.");
        goto done;
    }

    ret = 0;
done:
    return ret;
}

int device_on_search_finished(struct device *dev, esp_gatt_if_t gattc_if, uint16_t conn_id)
{
    int ret = 1;

    uint16_t start_hdl = 0,
             end_hdl = 0;

    dev->state = DEVICE_FIND_SERVICES;
    dev->nr_chars = 0;
    dev->cur_char = 0;
    dev->nr_svcs = 0;

    if (ble_object_get_service_count(dev->obj, &dev->nr_svcs)) {
        ESP_LOGE(TAG, "We have no services, we're all done.");
        goto done;
    }

    if (0 == dev->nr_svcs) {
        ESP_LOGE(TAG, "Device has no services, aborting.");
        goto done;
    }

    for (size_t i = 0; i < dev->nr_svcs; i++) {
        struct ble_service *svc = NULL;

        if (ble_object_get_service_info(dev->obj, i, &start_hdl, &end_hdl)) {
            ESP_LOGE(TAG, "Failed to get service info (service id = %zu), aborting.", i);
            goto done;
        }

        if (ble_object_get_service(dev->obj, i, &svc)) {
            ESP_LOGE(TAG, "Failed to get service %zu, aborting", i);
            goto done;
        }

        if (_device_capture_all_service_attribs(dev, svc, gattc_if, conn_id, start_hdl, end_hdl)) {
            ESP_LOGE(TAG, "Failed to capture service attributes, skipping");
        }
    }

    ESP_LOGD(TAG, "--------> Attributes are all read, now trying to hoover (%zu services)", dev->nr_svcs);

    /* Now try to enter the next state to read attributes */
    for (size_t i = 0; i < dev->nr_svcs; i++) {

        if (_device_hoover_service_attribs(dev, gattc_if, conn_id)) {
            ESP_LOGE(TAG, "Failed to kick off service hoover, aborting.");
            goto done;
        }

        if (dev->state == DEVICE_READ_ATTR) {
            /* We're now waiting on a read to complete, so let's break */
            break;
        }
    }

    if (dev->state != DEVICE_READ_ATTR) {
        /* Schedule a disconnect */
        ESP_LOGI(TAG, "No attributes to query, scheduling a disconnect (gattc_if=%d, conn_id=%u)", gattc_if, conn_id);
        if (ESP_OK != esp_ble_gattc_close(gattc_if, conn_id)) {
            ESP_LOGW(TAG, "Warning: failure while closing gattc_if=%d, conn_id=%u", gattc_if, conn_id);
        }
    }

    ret = 0;
done:
    return ret;
}

int device_on_disconnect(struct device *dev, unsigned reason)
{
    int ret = 1;

    if (NULL == dev) {
        ESP_LOGE(TAG, "Device reference is NULL, aborting.");
        abort();
    }

    dev->state = DEVICE_NOT_CONNECTED;
    dev->interrogated = true;

    ESP_LOGI(TAG, "Device disconnected (reason: %u)", reason);

    /* This disconnect is likely spurious, so kill the iteration state */
    dev->nr_chars = 0;
    dev->cur_char = 0;
    dev->cur_svc = 0;
    dev->nr_svcs = 0;

    if (NULL != dev->obj) {
        if (ble_object_serialize(dev->obj, &dev->encoded_obj, &dev->encoded_obj_len, dev->interrogated)) {
            ESP_LOGE(TAG, "Failed to serialize information about this device, aborting.");
            goto done;
        }
        assert(dev->encoded_obj_len != 0);

        tracker.mem_bytes_used += dev->encoded_obj_len;
    } else {
        ESP_LOGE(TAG, "No device object exists for this entry, aborting.");
    }

    ble_object_delete(&dev->obj);

    ret = 0;
done:
    return ret;
}

int device_on_read_characteristic(struct device *dev, esp_gatt_if_t gattc_if, uint16_t conn_id, void *data, size_t data_len, uint16_t handle)
{
    int ret = 1;

    struct ble_service *svc = NULL;

    ESP_LOGI(TAG, "Read %zu bytes from handle %u", data_len, handle);
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, data_len, ESP_LOG_INFO);

    if (ble_object_get_service(dev->obj, dev->cur_svc, &svc)) {
        ESP_LOGE(TAG, "Failed to get service ID %zu, aborting", dev->cur_svc);
        goto done;
    }

    if (ble_service_set_attribute_data(svc, dev->cur_char, data, data_len)) {
        ESP_LOGE(TAG, "Failed to log the fact that we just read an attribute.");
    }

    /* Increment past this characteristic we just read */
    dev->cur_char++;

    if (dev->cur_char >= dev->nr_chars) {
        dev->cur_svc++;
        dev->cur_char = 0;
        dev->nr_chars = 0;
    }

    /* Let's look for services we're interested in */
    dev->state = DEVICE_FIND_SERVICES;

    /* Resume iteration */
    for (size_t i = dev->cur_svc; i < dev->nr_svcs; i++) {
        if (_device_hoover_service_attribs(dev, gattc_if, conn_id)) {
            ESP_LOGE(TAG, "Failed to hoover attributes, aborting.");
            goto done;
        }

        if (dev->state == DEVICE_READ_ATTR) {
            /* A read is pending */
            break;
        }
    }

    if (dev->state != DEVICE_READ_ATTR) {
        /* Schedule a disconnect */
        ESP_LOGI(HOOVER, "All done, scheduling a disconnect. (gattc_if=%d conn_id=%u)", gattc_if, conn_id);

        if (ESP_OK != esp_ble_gattc_close(gattc_if, conn_id)) {
            ESP_LOGW(HOOVER, "Failed to schedule a disconnect for gattc_if=%d, conn_id=%u", gattc_if, conn_id);
        }
    }

    ret = 0;
done:
    return ret;
}

int device_tracker_insert(struct device_tracker *trk, struct device *dev)
{
    int ret = 1;

    if (RB_OK != rb_tree_insert(&trk->devices, (void *)dev->crc, &dev->r_node)) {
        ESP_LOGE(TAG, "Failed to insert device. Likely duplicate BDA.");
        goto done;
    }

    list_append(&trk->device_list, &dev->d_node);

    trk->nr_devices++;
    trk->mem_bytes_used += sizeof(struct device);

    ret = 0;

done:
    return ret;
}

int device_tracker_find(struct device_tracker *trk, struct device **pdev, uint8_t const *mac)
{
    int ret = 1;
    struct rb_tree_node *node = NULL;
    int32_t dev_crc = _hash_mac(mac);

    if (NULL != pdev) {
        *pdev = NULL;
    }

    if (RB_OK != rb_tree_find(&trk->devices, (void *)dev_crc, &node)) {
        ESP_LOGD(TAG, "Did not find MAC %02x:%02x:%02x-%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        goto done;
    }

    if (NULL != pdev) {
        *pdev = DEV_RBTREE(node);
    }

    ret = 0;

done:
    return ret;
}

void device_tracker_init(struct device_tracker *trk)
{
    rb_tree_new(&trk->devices, _device_rb_tree_cmp);
    list_init(&trk->device_list);
    trk->mem_bytes_used = 0;
    trk->nr_devices = 0;
}

static inline
int64_t _device_get_timestamp(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000LL + (tv.tv_usec/1000000LL);
}

struct device *device_new(uint32_t sensor_id, uint8_t const * mac, esp_ble_evt_type_t evt_type, bool is_public, bool connectable, size_t adv_data_len, size_t scan_rsp_len, uint8_t const * adv_data)
{
    struct device *ndev = calloc(1, sizeof(struct device));

    if (NULL == ndev) {
        ESP_LOGE(TAG, "Failed to allocate device node, aborting");
        goto done;
    }
    ndev->connectable = connectable;
    ndev->is_public = is_public;

    ndev->interrogated = false;
    ndev->nr_interrogations = 0;

    memcpy(ndev->bda_addr, mac, 6);

    ndev->crc = _hash_mac(mac);

    ndev->time_us = _device_get_timestamp();

    /* Create the object */
    if (ble_object_new(&ndev->obj, sensor_id, ndev->connectable, evt_type, mac, is_public, adv_data, adv_data_len, scan_rsp_len, ndev->time_us)) {
        ESP_LOGE(TAG, "Failed to create BLE object, aborting.");
        goto done;
    }

done:
    return ndev;
}

int device_tracker_remove(struct device_tracker *trk, struct device *dev)
{
    int ret = -1;

    if (RB_OK != rb_tree_remove(&trk->devices, &dev->r_node)) {
        ESP_LOGE(TAG, "Failed to remove object from tree, aborting");
        goto done;
    }

    list_del(&dev->d_node);

    if (NULL != dev->obj) {
        ESP_LOGE(TAG, "An unencoded BLE object was hanging around, cleaning it up.");
        ble_object_delete(&dev->obj);
    }

    trk->nr_devices--;
    trk->mem_bytes_used -= dev->encoded_obj_len + sizeof(struct device);

    free(dev->encoded_obj);
    dev->encoded_obj = NULL;
    dev->encoded_obj_len = 0;

    ret = 0;
done:
    return ret;
}

