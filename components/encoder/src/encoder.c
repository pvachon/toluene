#include "encoder.h"
#include "ble_device.pb-c.h"

#include "esp_gap_ble_api.h"
#include "esp_heap_caps.h"

#include "esp_log.h"

#include "list.h"

#include <string.h>

#define TAG							"ENCODER"

struct ble_object {
    BleDevice dev;
    uint8_t bda_addr[6];
    uint8_t adv_data[ESP_BLE_ADV_DATA_LEN_MAX + ESP_BLE_SCAN_RSP_DATA_LEN_MAX];
    size_t nr_services;
    BleDevice__BleGattService **svc_ptrs;
};

struct ble_service {
    BleDevice__BleGattService svc;

    BleDevice__BleUuid uuid;

    /** If this service has a long UUID, we'll use this memory to represent it */
    uint8_t uuid_long[ESP_UUID_LEN_128];

    /** Number of encoded attributes */
    size_t nr_attribs;

    /** Start attribute handle */
    uint16_t start_hdl;

    /** End attribute handle */
    uint16_t end_hdl;

    /** Array of attribute pointers */
    BleDevice__BleGattService__BleGattAttr **attr_ptrs;
};

struct ble_service_attribute {
    BleDevice__BleGattService__BleGattAttr attr;
    BleDevice__BleUuid uuid;

    /** If this attribute has a long UUID, we'll use this memory to represent it */
    uint8_t uuid_long[ESP_UUID_LEN_128];

    /** The raw data read back from the service */
    uint8_t data[];
};

static
void *proto_malloc(size_t len)
{
    return malloc(len);
}

static
void *proto_realloc(void *ptr, size_t len)
{
    return realloc(ptr, len);
}

static
void proto_free(void *ptr)
{
    free(ptr);
}

static
void _ble_obj_encode_uuid(esp_bt_uuid_t const *esp_uuid, uint8_t *long_mem, BleDevice__BleUuid *dst_uuid)
{
    *dst_uuid = (BleDevice__BleUuid)BLE_DEVICE__BLE_UUID__INIT;

    switch (esp_uuid->len) {
    case ESP_UUID_LEN_16:
        dst_uuid->uuid_short = esp_uuid->uuid.uuid16;
        dst_uuid->length = BLE_DEVICE__BLE_UUID__BLE_UUID_LENGTH__UUID16;
        dst_uuid->uuid_case = BLE_DEVICE__BLE_UUID__UUID_UUID_SHORT;
        break;
    case ESP_UUID_LEN_32:
        dst_uuid->uuid_short = esp_uuid->uuid.uuid32;
        dst_uuid->length = BLE_DEVICE__BLE_UUID__BLE_UUID_LENGTH__UUID32;
        dst_uuid->uuid_case = BLE_DEVICE__BLE_UUID__UUID_UUID_SHORT;
        break;
    case ESP_UUID_LEN_128:
        memcpy(long_mem, esp_uuid->uuid.uuid128, ESP_UUID_LEN_128);
        dst_uuid->uuid_long.len = ESP_UUID_LEN_128;
        dst_uuid->uuid_long.data = long_mem;
        dst_uuid->length = BLE_DEVICE__BLE_UUID__BLE_UUID_LENGTH__UUID128;
        dst_uuid->uuid_case = BLE_DEVICE__BLE_UUID__UUID_UUID_LONG;
        break;
    default:
        ESP_LOGE(TAG, "FATAL: Malformed UUID object provided from ESP");
    }
}

int ble_object_new(struct ble_object **pobj, unsigned sensor_id, bool connectable, esp_ble_evt_type_t evt_type, uint8_t const *bda_addr, bool is_public, void const *adv_data, uint32_t adv_len, uint32_t scan_rsp_len, int64_t timestamp)
{
    int ret = -1;

    struct ble_object *obj = NULL;

    *pobj = NULL;

    if (adv_len + scan_rsp_len > ESP_BLE_SCAN_RSP_DATA_LEN_MAX + ESP_BLE_ADV_DATA_LEN_MAX) {
        ESP_LOGE(TAG, "FATAL: Advertising data is too long, aborting");
        goto done;
    }

    if (NULL == (obj = proto_malloc(sizeof(struct ble_object)))) {
        ESP_LOGE(TAG, "FATAL: out of memory.");
        goto done;
    }

    obj->dev = (BleDevice)BLE_DEVICE__INIT;

    /* Fill in the Sensor ID */
    obj->dev.has_sensor_id = 1;
    obj->dev.sensor_id = sensor_id;

    /* Fill in the Bluetooth Device Address */
    memcpy(obj->bda_addr, bda_addr, 6);
    obj->dev.bda_address.data = obj->bda_addr;
    obj->dev.bda_address.len = 6;
    obj->dev.has_bda_address = 1;

    /* Fill in the public vs. private state of the BDA */
    obj->dev.has_public_address = 1;
    obj->dev.public_address = is_public ? 1 : 0;

    /* Fill in the event type */
    obj->dev.has_pdu_type = 1;
    switch (evt_type) {
    case ESP_BLE_EVT_CONN_ADV:
        obj->dev.pdu_type = BLE_DEVICE__BLE_ADV_PDU_TYPE__ADV_IND;
        break;
    case ESP_BLE_EVT_CONN_DIR_ADV:
        obj->dev.pdu_type = BLE_DEVICE__BLE_ADV_PDU_TYPE__ADV_INDIRECT_IND;
        break;
    case ESP_BLE_EVT_DISC_ADV:
        obj->dev.pdu_type = BLE_DEVICE__BLE_ADV_PDU_TYPE__ADV_SCAN_IND;
        break;
    case ESP_BLE_EVT_NON_CONN_ADV:
        obj->dev.pdu_type = BLE_DEVICE__BLE_ADV_PDU_TYPE__ADV_NONCONN_IND;
        break;
    default:
        ESP_LOGW(TAG, "Unknown event type, marking as non-connectable");
        obj->dev.pdu_type = BLE_DEVICE__BLE_ADV_PDU_TYPE__ADV_NONCONN_IND;
    }

    /* Fill in the advertising data, if present */
    if (0 != (adv_len + scan_rsp_len)) {
        memcpy(obj->adv_data, adv_data, adv_len + scan_rsp_len);
        obj->dev.adv_data.len = adv_len + scan_rsp_len;
        obj->dev.adv_data.data = obj->adv_data;
        obj->dev.has_adv_data = 1;

        if (0 != adv_len) {
            obj->dev.has_adv_rsp_len = 1;
            obj->dev.adv_rsp_len = adv_len;
        }

        if (0 != scan_rsp_len) {
            obj->dev.has_scan_rsp_len = 1;
            obj->dev.scan_rsp_len = scan_rsp_len;
        }
    }

    obj->dev.has_timestamp = 1;
    obj->dev.timestamp = timestamp;

    /* Mark if this object was connectable */
    obj->dev.has_connectable = 1;
    obj->dev.connectable = connectable ? 1 : 0;

    obj->nr_services = 0;
    obj->svc_ptrs = NULL;

    *pobj = obj;
    ret = 0;
done:
    if (ret != 0) {
        if (NULL != obj) {
            proto_free(obj);
            obj = NULL;
        }
    }
    return ret;
}

int ble_object_add_service(struct ble_object *obj, struct ble_service **pservice, esp_bt_uuid_t const *uuid, uint16_t start_hdl, uint16_t end_hdl)
{
    int ret = -1;

    struct ble_service *service = NULL;

    if (NULL == (service = proto_malloc(sizeof(struct ble_service)))) {
        ESP_LOGE(TAG, "Not enough memory to allocate service record");
        goto done;
    }

    service->svc = (BleDevice__BleGattService)BLE_DEVICE__BLE_GATT_SERVICE__INIT;

    service->nr_attribs = 0;
    service->attr_ptrs = NULL;
    service->start_hdl = start_hdl;
    service->end_hdl = end_hdl;

    /* Encode the UUID for this service */
    _ble_obj_encode_uuid(uuid, service->uuid_long, &service->uuid);
    service->svc.service_uuid = &service->uuid;

    /* Add to our list o' services */
    obj->nr_services++;

    /* TODO: this needs to be a hell of a lot smarter to avoid heap fragmentation */
    if (NULL == (obj->svc_ptrs = proto_realloc(obj->svc_ptrs, sizeof(BleDevice__BleGattService *) * obj->nr_services))) {
        obj->nr_services--;
        ESP_LOGE(TAG, "Fatal error: could not realloc for requested number of service pointers, aborting");
        goto done;
    }

    obj->svc_ptrs[obj->nr_services - 1] = &service->svc;

    *pservice = service;

    /* Mark that we have interrogated this device */
    obj->dev.has_interrogated = 1;
    obj->dev.interrogated = 1;

    ret = 0;
done:
    if (0 != ret) {
        if (NULL != service) {
            proto_free(service);
            service = NULL;
        }
    }
    return ret;
}

int ble_object_get_service_count(struct ble_object *obj, size_t *pcnt)
{
    int ret = 1;

    if (NULL == obj || NULL == pcnt) {
        goto done;
    }

    *pcnt = obj->nr_services;

    ret = 0;
done:
    return ret;
}

int ble_object_get_service_info(struct ble_object *obj, size_t svc_id, uint16_t *pstart_hdl, uint16_t *pend_hdl)
{
    int ret = -1;

    struct ble_service *svc = NULL;

    if (NULL == obj || NULL == pstart_hdl || NULL == pend_hdl) {
        ESP_LOGE(TAG, "Programmer error: NULL pointer for output values in get_service_info");
        goto done;
    }

    if (svc_id >= obj->nr_services) {
        ESP_LOGE(TAG, "Service %zu does not exist (there are %zu services)", svc_id, obj->nr_services);
        goto done;
    }

    svc = BL_CONTAINER_OF(obj->svc_ptrs[svc_id], struct ble_service, svc);

    *pstart_hdl = svc->start_hdl;
    *pend_hdl = svc->end_hdl;

    ret = 0;
done:
    return ret;
}

int ble_object_get_service(struct ble_object *obj, size_t svc_id, struct ble_service **psvc)
{
    int ret = -1;

    if (NULL == obj || NULL == psvc) {
        ESP_LOGE(TAG, "Programmer error: NULL pointer for output values in get_service");
        goto done;
    }

    if (svc_id >= obj->nr_services) {
        ESP_LOGE(TAG, "Service %zu does not exist (there are %zu services)", svc_id, obj->nr_services);
        goto done;
    }

    *psvc = BL_CONTAINER_OF(obj->svc_ptrs[svc_id], struct ble_service, svc);

    ret = 0;
done:
    return ret;
}

int ble_service_add_attribute(struct ble_service *svc, esp_bt_uuid_t const *uuid, void const *data, size_t data_len)
{
    int ret = -1;

    struct ble_service_attribute *attr = NULL;
    size_t real_data_len = data_len;

    if (data_len > 128) {
        real_data_len = 128;
        ESP_LOGE(TAG, "Attribute data is too large; we won't be encoding it all, alas");
    }

    if (NULL == (attr = proto_malloc(sizeof(struct ble_service_attribute) + real_data_len))) {
        ESP_LOGE(TAG, "Out of memory, could not allocate attribute container");
        goto done;
    }

    attr->attr = (BleDevice__BleGattService__BleGattAttr)BLE_DEVICE__BLE_GATT_SERVICE__BLE_GATT_ATTR__INIT;

    /* Encode the UUID for this attribute */
    _ble_obj_encode_uuid(uuid, attr->uuid_long, &attr->uuid);
    attr->attr.attr_uuid = &attr->uuid;

    if (0 != data_len) {
        /* Feed me data (or a dead cat) */
        memcpy(attr->data, data, real_data_len);
        attr->attr.has_data = 1;
        attr->attr.data.data = attr->data;
        attr->attr.data.len = data_len;
    } else {
        attr->attr.has_data = 0;
        attr->attr.data.len = 0;
    }

    /* Track the attribute for when serialization happens */
    svc->nr_attribs++;

    /* TODO: this needs to be a hell of a lot smarter to avoid heap fragmentation */
    if (NULL == (svc->attr_ptrs = proto_realloc(svc->attr_ptrs, sizeof(BleDevice__BleGattService__BleGattAttr *) * svc->nr_attribs))) {
        svc->nr_attribs--;
        ESP_LOGE(TAG, "Fatal error: could not realloc for requested number of attribute pointers, aborting");
        goto done;
    }

    svc->attr_ptrs[svc->nr_attribs - 1] = &attr->attr;

    ret = 0;
done:
    if (0 != ret) {
        if (NULL != attr) {
            proto_free(attr);
            attr = NULL;
        }
    }
    return ret;
}

void ble_object_delete(struct ble_object **pobj)
{
    struct ble_object *obj = NULL;

    if (NULL == pobj || NULL == *pobj) {
        ESP_LOGW(TAG, "No object was specified, aborting");
        goto done;
    }

    obj = *pobj;

    /* Walk the list of services */
    for (size_t j = 0; j < obj->nr_services; j++) {
        struct ble_service *svc = BL_CONTAINER_OF(obj->svc_ptrs[j], struct ble_service, svc);

        for (size_t i = 0; i < svc->nr_attribs; i++) {
            struct ble_service_attribute *attr = BL_CONTAINER_OF(svc->attr_ptrs[i], struct ble_service_attribute, attr);
            proto_free(attr);
            svc->attr_ptrs[i] = NULL;
        }

        proto_free(svc->attr_ptrs);
        svc->attr_ptrs = NULL;
        proto_free(svc);
        obj->svc_ptrs[j] = NULL;
    }

    proto_free(obj->svc_ptrs);
    obj->svc_ptrs = NULL;
    proto_free(obj);

    *pobj = NULL;

done:
    ;
}

static
void _ble_service_prepare_serialize(struct ble_service *svc)
{
    svc->svc.gatt_attrs = svc->attr_ptrs;
    svc->svc.n_gatt_attrs = svc->nr_attribs;
}

static
void _ble_object_prepare_serialize(struct ble_object *obj)
{
    obj->dev.services = obj->svc_ptrs;
    obj->dev.n_services = obj->nr_services;

    for (size_t i = 0; i < obj->nr_services; i++) {
        struct ble_service *svc = BL_CONTAINER_OF(obj->svc_ptrs[i], struct ble_service, svc);
        _ble_service_prepare_serialize(svc);
    }
}

int ble_object_serialize(struct ble_object *obj, uint8_t **pserialized, size_t *pserialized_len, bool interrogated)
{
    int ret = -1;

    size_t packed = 0;
    uint8_t *serialized = NULL;

    obj->dev.interrogated = interrogated ? 1 : 0;
    obj->dev.has_interrogated = 1;

    _ble_object_prepare_serialize(obj);

    packed = ble_device__get_packed_size(&obj->dev);

    ESP_LOGI(TAG, "Packing object -- length is %zu bytes", packed);

    if (NULL == (serialized = proto_malloc(packed + 2))) {
        ESP_LOGE(TAG, "Failed to allocate memory to pack object into, aborting.");
        goto done;
    }

    /* Pack the length of the protobuf into the first two bytes */
    uint16_t *header = (uint16_t *)serialized;
    *header = packed;

    if (packed != ble_device__pack(&obj->dev, serialized + 2)) {
        ESP_LOGE(TAG, "Packing message failed, length mismatch.");
        goto done;
    }

    *pserialized = serialized;
    *pserialized_len = packed + 2;

    ret = 0;
done:
    if (0 != ret) {
        proto_free(serialized);
        serialized = NULL;
    }
    return ret;
}

