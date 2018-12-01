#pragma once

#include "esp_bt_defs.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

struct ble_object;
struct ble_service;

int ble_object_new(struct ble_object **pobj, unsigned sensor_id, bool connectable, uint8_t const *bda_addr, void const *adv_data, uint32_t adv_len, uint32_t scan_rsp_len, int64_t timestamp);

int ble_object_add_service(struct ble_object *obj, struct ble_service **pservice, esp_bt_uuid_t const *uuid, uint16_t start_hdl, uint16_t end_hdl);
int ble_object_get_service_count(struct ble_object *obj, size_t *pcnt);
int ble_object_get_service_info(struct ble_object *obj, size_t svc_id, uint16_t *pstart_hdl, uint16_t *pend_hdl);
int ble_object_get_service(struct ble_object *obj, size_t svc_id, struct ble_service **psvc);

int ble_service_add_attribute(struct ble_service *svc, esp_bt_uuid_t const *uuid, void const *data, size_t data_len);

int ble_object_serialize(struct ble_object *obj, uint8_t **pserialized, size_t *pserialized_len);

void ble_object_delete(struct ble_object **pobj);

