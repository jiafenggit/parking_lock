#include"hal_adc.h"
#include"hal_key.h"
#include"hal_batt.h"
#include"hal_drivers.h"
#include"osal_timers.h"
#include"app_uart_init.h"



#define  BAT_VOLTAGE_SCALE                    4.7 //1:4.7
#define  BAT_FULL_VOLTAGE                     6.2 //6.2v  
#define  BAT_EMPTY_VOLTAGE                    4.4 //4.4v


hal_batt_cb_t batt_callback;

float batt_v=BAT_FULL_VOLTAGE;


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
 uint8 batt_percent;
 batt=halGetVoltageMonitorInput(BATT_MONITOR_CHN);
 
 batt_v=batt*0.01*(BAT_VOLTAGE_SCALE+1); 
 
 batt_percent=(uint8)(100*(batt_v-BAT_EMPTY_VOLTAGE)/(BAT_FULL_VOLTAGE-BAT_EMPTY_VOLTAGE));
 
 app_write_string("v:");
 app_write_string(uint8_to_string((uint8)batt_v));
 
 if(batt_callback)
  batt_callback(batt_percent);
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

