#include"hal_adc.h"
#include"hal_key.h"
#include"hal_batt.h"
#include"hal_drivers.h"
#include"osal_timers.h"
#include"app_uart_init.h"

hal_batt_cb_t batt_callback;

void app_batt_start_periodic_update_info()
{
  osal_start_reload_timer(Hal_TaskID, HAL_BATT_CHECK_EVENT,BATT_UPDATE_VALUE);
  app_write_string("\r\n开始周期性batt update,周期为(ms):");
  app_write_string(uint16_to_string(BATT_UPDATE_VALUE));
}

void hal_batt_init()
{
  app_write_string("\r\nbatt初始化!");//
}

void hal_process_update_batt_info_event()
{
 uint8 batt; 
 batt=halGetVoltageMonitorInput(BATT_MONITOR_CHN);
 if(batt_callback)
  batt_callback(batt);
 else
  app_write_string("\r\nbatt cb未注册!");
}

void hal_batt_cb_config(hal_batt_cb_t ptr_cb)
{
  if(batt_callback==NULL)
  {
    batt_callback=ptr_cb;
  }

}

