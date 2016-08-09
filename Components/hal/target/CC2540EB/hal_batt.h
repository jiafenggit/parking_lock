#ifndef __HAL_BATT_H__
#define __HAL_BATT_H__


#define BATT_UPDATE_VALUE           30000       //∞Î∑÷÷”≤‚“ª¥Œ




typedef uint8 (*hal_batt_cb_t)(uint8 batt);


void hal_process_update_batt_info_event();
void hal_batt_cb_config(hal_batt_cb_t ptr_cb);
void app_batt_start_periodic_update_info();
void hal_batt_init();


#endif