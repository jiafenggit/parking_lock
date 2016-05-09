#if BLE_LOCK_IN_PROJ> 0
#include "hal_board_cfg_ble_lock_in.h"
#elif BLE_CAR_IN_PROJ > 0
#include "hal_board_cfg_ble_car_in.h"
#elif BLE_CONTROL_PROJ> 0
#include "hal_board_cfg_ble_control.h"
#else
#error "no proj defined"
#endif