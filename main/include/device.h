#pragma once

#include "rbtree.h"
#include "list.h"

#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"

#define DEVICE_MAC_ADDR_LEN     6

enum device_interrogation_state {
    DEVICE_NOT_CONNECTED,
    DEVICE_SEARCH_SERVICE,
    DEVICE_FIND_SERVICES,
    DEVICE_READ_ATTR,
    DEVICE_INTERROGATION_DONE,
};

struct device_tracker {
    struct rb_tree devices;
    struct list_entry device_list;
    unsigned nr_devices;
};

struct ble_object;

struct device {
    bool connectable;
    bool interrogated;
    unsigned nr_interrogations;
    enum device_interrogation_state state;
    uint8_t service_id;
    uint8_t attr_id;
    int32_t crc;
    struct list_entry d_node;
    struct rb_tree_node r_node;
    uint8_t mac_addr[DEVICE_MAC_ADDR_LEN];
    uint8_t scan_rsp_len;
    uint8_t adv_data_len;
    uint8_t is_public;
    uint8_t raw[ESP_BLE_ADV_DATA_LEN_MAX + ESP_BLE_SCAN_RSP_DATA_LEN_MAX];
    struct ble_object *obj;
    uint8_t *encoded_obj;
    size_t encoded_obj_len;
};

#define DEV_RBTREE(x)   RB_CONTAINER_OF((x), struct device, r_node)
#define DEV_LIST(x)     BL_CONTAINER_OF((x), struct device, d_node)

#define DEV_MAX_INTERROGATIONS          10

struct device *device_new(uint8_t const * mac);
void device_tracker_init(struct device_tracker *trk);
int device_tracker_insert(struct device_tracker *trk, struct device *dev);
int device_tracker_find(struct device_tracker *trk, struct device **pdev, uint8_t const *mac);

int device_on_open(struct device *dev, esp_gatt_if_t gattc_if, uint16_t conn_id);
int device_on_found_service(struct device *dev, esp_bt_uuid_t const *service_uuid, uint16_t start_hdl, uint16_t end_hdl);
int device_on_disconnect(struct device *dev, unsigned reason);
int device_on_read_characteristic(struct device *dev, esp_gatt_if_t gattc_if, uint16_t conn_id, void *data, size_t data_len, uint16_t handle);
int device_on_search_finished(struct device *dev, esp_gatt_if_t gattc_if, uint16_t conn_id);

