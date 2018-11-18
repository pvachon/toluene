#pragma once

void control_task_init(void);

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

