#pragma once

#include <stdint.h>

void control_task_init(void);
void control_load_config(void);

/* Signals for the control task */
void control_task_signal_ble_ready(void);
void control_task_signal_ble_gatt_disconnect(void);
void control_task_signal_ble_scan_started(void);
void control_task_signal_ble_scan_complete(void);
void control_task_signal_ble_scan_paused(void);

void control_task_signal_wifi_up(void);
void control_task_signal_wifi_done(void);
void control_task_signal_wifi_failure(void);
void control_task_signal_wifi_down(void);

void control_task_signal_ntp_done(void);

/* Get configuration parameters */
void control_get_config_wifi(char *essid, char *password);
void control_get_config_device_info(char const **phostname, uint16_t *pport, uint32_t *pdev_id);

