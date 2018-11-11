#include "device.h"
#include "rbtree.h"
#include "list.h"

#include "rom/crc.h"

#include "esp_log.h"

#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include <string.h>

#define TAG                 "DEVICE"

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

struct ble_service {
    uint16_t service_id;
    uint16_t nr_attributes;
    uint16_t handle_lo;
    uint16_t handle_hi;
    uint16_t *attributes;
};

static
uint16_t _generic_access_attrs[] = {
    BLE_SERVICE_GENERIC_ACCESS_ATTR_NAME,
    BLE_SERVICE_GENERIC_ACCESS_ATTR_APPEARANCE,
};

static
uint16_t _device_info_access_attrs[] = {
    BLE_SERVICE_DEVICE_INFO_ATTR_MANUF_NAME,
    BLE_SERVICE_DEVICE_INFO_ATTR_MODEL_NUMBER,
    BLE_SERVICE_DEVICE_INFO_ATTR_SERIAL_NUMBER,
    BLE_SERVICE_DEVICE_INFO_ATTR_FIRMWARE_REV,
    BLE_SERVICE_DEVICE_INFO_ATTR_SOFTWARE_REV,
    BLE_SERVICE_DEVICE_INFO_ATTR_SYSTEM_ID,
    BLE_SERVICE_DEVICE_INFO_ATTR_PNP_ID,
};

#define BLE_ARRAY_LEN(x)    (sizeof((x))/sizeof((x)[0]))

static
struct ble_service services[] = {
    {
        .service_id = BLE_SERVICE_GENERIC_ACCESS,
        .attributes = _generic_access_attrs,
        .nr_attributes = BLE_ARRAY_LEN(_generic_access_attrs),
    },
    {
        .service_id = BLE_SERVICE_DEVICE_INFO,
        .attributes = _device_info_access_attrs,
        .nr_attributes = BLE_ARRAY_LEN(_device_info_access_attrs),
    },
};

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

    ESP_LOGI(TAG, "Device is open");

    if (dev->state != DEVICE_NOT_CONNECTED) {
        ESP_LOGE(TAG, "Unknown device state at open time, aborting");
        goto done;
    }

    /* Start things off by searching for all the services exposed */
    esp_ble_gattc_search_service(gattc_if, conn_id, NULL);
    dev->state = DEVICE_SEARCH_SERVICE;

    for (size_t i = 0; i < BLE_ARRAY_LEN(services); i++) {
        services[i].handle_hi = INVALID_HANDLE;
        services[i].handle_lo = INVALID_HANDLE;
    }

    ret = 0;

done:
    return ret;
}

int device_on_found_service(struct device *dev, esp_bt_uuid_t const *service_uuid, uint16_t start_hdl, uint16_t end_hdl)
{
    int ret = 1;

    if (service_uuid->len != ESP_UUID_LEN_16) {
        ESP_LOGI(TAG, "UUID is not 16-bits long");
        /* Strictly speaking, this is not an error */
        ret = 0;
        goto done;
    }

    ESP_LOGI(TAG, "UUID16: %x", service_uuid->uuid.uuid16);

    for (size_t i = 0; i < BLE_ARRAY_LEN(services); i++) {
        if (services[i].service_id == service_uuid->uuid.uuid16) {
            services[i].handle_lo = start_hdl;
            services[i].handle_hi = end_hdl;
        }
    }

    ret = 0;
done:
    return ret;
}

/**
 * Send a request to read a single GATT attribute.
 */
static
int _device_request_read_attrib(struct device *dev, esp_gatt_if_t gattc_if, uint16_t conn_id, struct ble_service const *svc)
{
    int ret = 1;

    uint16_t count = 1;
    esp_gattc_char_elem_t char_elem;

    esp_bt_uuid_t uuid = {
        .len = ESP_UUID_LEN_16,
        .uuid = {
            .uuid16 = svc->attributes[dev->attr_id],
        },
    };

    /* Get the number of real attributes */
    esp_gatt_status_t status = esp_ble_gattc_get_char_by_uuid(gattc_if, conn_id,
                                                              svc->handle_lo, svc->handle_hi,
                                                              uuid, &char_elem,
                                                              &count);
    if (status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "Failed to get characteristic handle: %04x", svc->attributes[dev->attr_id]);
        goto done;
    }

    if (0 == count) {
        ESP_LOGE(TAG, "No attributes found (handles: 0x%04x-0x%04x) -- skipping", svc->handle_lo, svc->handle_hi);
        goto done;
    }

    /* Enqueue a read request for attribute */
    if (ESP_GATT_OK != (status = esp_ble_gattc_read_char(gattc_if, conn_id, char_elem.char_handle, ESP_GATT_AUTH_REQ_NONE))) {
        ESP_LOGE(TAG, "Failed to enqueue request for handle %04x for attribute %04x; aborting.", char_elem.char_handle, svc->attributes[dev->attr_id]);
        goto done;
    }

    ESP_LOGI(TAG, "Requested read for handle %04x", char_elem.char_handle);

    ret = 0;
done:
    return ret;
}

/**
 * Walk the list of attributes we want to retrieve for this device.
 */
static
int _device_next_attrib(struct device *dev)
{
    int ret = 1;
    struct ble_service const *svc = &services[dev->service_id];

    if (++dev->attr_id == svc->nr_attributes) {
        dev->service_id++;
        for (size_t i = dev->service_id; i < BLE_ARRAY_LEN(services); i++) {
            svc = &services[i];
            dev->service_id = i;
            if (svc->handle_lo != INVALID_HANDLE || svc->handle_hi != INVALID_HANDLE) {
                dev->attr_id = 0;
                break;
            }
        }
    }

    /* Check if we have no more services to check. If this is the case, end our misery */
    if (dev->service_id >= BLE_ARRAY_LEN(services)) {
        goto done;
    }

    ret = 0;
done:
    return ret;
}

int device_on_search_finished(struct device *dev, esp_gatt_if_t gattc_if, uint16_t conn_id)
{
    int ret = 1;

    dev->state = DEVICE_FIND_SERVICES;
    dev->service_id = 0;
    dev->attr_id = 0;

    /* Kick off the first attribute lookup */
    for (size_t i = 0; i < BLE_ARRAY_LEN(services); i++) {
        struct ble_service *sv = &services[i];

        if (sv->handle_hi != INVALID_HANDLE && sv->handle_lo != INVALID_HANDLE) {
            dev->service_id = i;
            break;
        }
    }

    while (_device_request_read_attrib(dev, gattc_if, conn_id, &services[dev->service_id])) {
        /* Failed to find the requested attribute, so bump to the next one */
        if (_device_next_attrib(dev)) {
            /* This device doesn't expose any interesting attributes */
            goto done;
        }
    }

    ret = 0;
done:
    return ret;
}

int device_on_disconnect(struct device *dev, unsigned reason)
{
    int ret = 1;

    dev->state = DEVICE_NOT_CONNECTED;
    dev->interrogated = true;

    ESP_LOGI(TAG, "Device disconnected (reason: %u)", reason);

    return ret;
}

int device_on_read_characteristic(struct device *dev, esp_gatt_if_t gattc_if, uint16_t conn_id, void *data, size_t data_len, uint16_t handle)
{
    int ret = 1;

    ESP_LOGI(TAG, "Read %zu bytes from handle %u", data_len, handle);
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, data_len, ESP_LOG_INFO);

    if (_device_next_attrib(dev)) {
        goto done;
    }

    /* Try to enqueue a request to read the next attribute */
    while (_device_request_read_attrib(dev, gattc_if, conn_id, &services[dev->service_id])) {
        if (_device_next_attrib(dev)) {
            /* This device doesn't expose any other interesting attributes */
            ESP_LOGW(TAG, "No more interesting attributes, too bad; scheduling a disconnect");
            goto done;
        }
    }

    ret = 0;
done:
    return ret;
}

int device_tracker_insert(struct device_tracker *trk, struct device *dev)
{
    int ret = 1;

    dev->crc = _hash_mac(dev->mac_addr);

    if (RB_OK != rb_tree_insert(&trk->devices, (void *)dev->crc, &dev->r_node)) {
        const uint8_t *mac = dev->mac_addr;
        ESP_LOGE(TAG, "MAC address %02x:%02x:%02x-%02x:%02x:%02x could not be inserted!",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        goto done;
    }

    trk->nr_devices++;

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
}

struct device *device_new(uint8_t const * mac)
{
    struct device *ndev = calloc(1, sizeof(struct device));

    if (NULL == ndev) {
        ESP_LOGE(TAG, "Failed to allocate device node, aborting");
        goto done;
    }
    memcpy(ndev->mac_addr, mac, DEVICE_MAC_ADDR_LEN);

done:
    return ndev;
}

