#include"hal_defs.h"
#include"hal_types.h"
#include"osal.h"
#include"osal_timers.h"
#include"hal_scan_car.h"
#include"hal_timer.h"
#include"app_uart_init.h"
#include"hal_drivers.h"
#include"hal_key.h"
#include"hal_adc.h"


static  process_scan_car_event_t process_scan_car_event;

static void hal_stop_to_scan_car();
static void hal_start_to_scan_car();


void hal_scan_car_init()
{
  SCAN_CAR_SET_IO_DIR();

  hal_scan_car_echo_pwr_off();//low turn off echo pwr
  process_scan_car_event=NULL;
  
}

void hal_scan_car_config(process_scan_car_event_t p)//on_board call
{
  if(process_scan_car_event==NULL)
    process_scan_car_event=p; 
}

void hal_process_scan_car_latency_event()//hal_driver call
{
  uint8 car_exsit=CAR_EXSIT;
  uint8 valid=FALSE;
  uint8 distance;
  distance=halGetVoltageMonitorInput(SCAN_CAR_EXSIT_CHN);
if(distance>CAR_EXSIT_MIN_V)// && distance<CAR_EXSIT_MAX_V)
{
  car_exsit=CAR_EXSIT;
  valid=TRUE;
  app_write_string("\r\n扫描到汽车!");
  app_write_string("\r\n距离V:");
  app_write_string(uint8_to_string(distance));
 
}
else
{
  car_exsit=CAR_NOT_EXSIT;
  valid=TRUE;
  app_write_string("\r\n没有扫描到汽车!");
  app_write_string("\r\n距离V:");
  app_write_string(uint8_to_string(distance));
}

  hal_stop_to_scan_car();//stop to scan car
  if(valid==TRUE && process_scan_car_event)
  {
    process_scan_car_event(car_exsit);
  }
  app_write_string("\r\n向系统发送车辆扫描结果!");
}

static void hal_stop_to_scan_car()
{
  hal_scan_car_echo_pwr_off();//close echo pwr
  app_write_string("\r\n关闭车辆扫描!");
}
static void hal_start_to_scan_car()
{
  hal_scan_car_echo_pwr_on();//turn on echo pwr
  osal_start_timerEx(Hal_TaskID,HAL_SCAN_CAR_LATENCY_EVENT,HAL_SCAN_CAR_LATENCY_VALUE);//waiting for echo
  app_write_string("\r\n打开测距电源等待输出...");
}


void app_start_to_scan_car()
{  
  hal_start_to_scan_car();
}
