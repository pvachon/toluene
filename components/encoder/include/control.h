#pragma once

#include <stdint.h>
#include <stdlib.h>

typedef struct mbedtls_x509_crt mbedtls_x509_crt;
typedef struct mbedtls_pk_context mbedtls_pk_context;
enum identity_wifi_auth;

void control_task_init(void);
void control_load_config(void);

/* Signals for the control task */
void control_task_signal_ble_ready(void);
void control_task_signal_ble_gatt_disconnect(void);
void control_task_signal_ble_scan_started(void);
void control_task_signal_ble_scan_complete(void);

void control_task_signal_wifi_up(void);
void control_task_signal_wifi_done(void);
void control_task_signal_wifi_failure(void);
void control_task_signal_wifi_down(void);

void control_task_signal_ntp_done(void);

void control_task_signal_ble_hoover_start(void);
void control_task_signal_ble_hoover_finished(void);

/* Get configuration parameters */

enum identity_wifi_auth control_get_wifi_auth_mode(void);
void control_get_config_wifi(char *essid, char *password, char *username, uint8_t **pca_cert, size_t *pca_cert_len);
void control_get_config_device_info(char const **phostname, uint16_t *pport, uint32_t *pdev_id);
uint32_t control_get_config_sensor_id(void);

void control_get_config_host_ca_crt(mbedtls_x509_crt **pcrt);
void control_get_config_host_client_crt_chain(mbedtls_x509_crt **pcrt_chain);
void control_get_identity_key_pair(mbedtls_pk_context **pkey);

