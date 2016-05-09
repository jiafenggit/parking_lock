#ifndef  __HAL_SCAN_CAR_H__
#define  __HAL_SCAN_CAR_H__

#include"hal_board.h"



#define  SCAN_CAR_EXSIT_ECHO_PORT               P0
#define  SCAN_CAR_EXSIT_ECHO_SBIT               P0_6
#define  SCAN_CAR_EXSIT_ECHO_PWR_PORT           P1
#define  SCAN_CAR_EXSIT_ECHO_PWR_SBIT           P1_7


#define  SCAN_CAR_EXSIT_ECHO_POS                BV(6)
#define  SCAN_CAR_EXSIT_ECHO_PWR_POS            BV(7)


//设置为通用IO和echo输入接口 默认下拉 trigger 输出接口  P0SEL|= SCAN_CAR_EXSIT_ECHO_POS;                    
#define  SCAN_CAR_SET_IO_DIR()                                  \
{                                                               \
  P1SEL&=~SCAN_CAR_EXSIT_ECHO_PWR_POS;                          \
  P1DIR|= SCAN_CAR_EXSIT_ECHO_PWR_POS;                          \
}    

/*
//允许中断触发
#define  SCAN_CAR_ENABLE_INT()         {IEN1|=1<<4;P1IEN|=SCAN_CAR_EXSIT_ECHO_POS;}  
#define  SCAN_CAR_SET_INT_RISE_EDGE()  {PICTL&=~BV(2);}
#define  SCAN_CAR_SET_INT_FALL_EDGE()  {PICTL|=BV(2);}

*/

#define  SCAN_CAR_EXSIT_ECHO_PWR_POLARITY     ACTIVE_LOW

#define  hal_scan_car_echo_pwr_on()       st(SCAN_CAR_EXSIT_ECHO_PWR_SBIT=SCAN_CAR_EXSIT_ECHO_PWR_POLARITY (1);)
#define  hal_scan_car_echo_pwr_off()      st(SCAN_CAR_EXSIT_ECHO_PWR_SBIT=SCAN_CAR_EXSIT_ECHO_PWR_POLARITY (0);)




#define  CAR_EXSIT                    1
#define  CAR_NOT_EXSIT                2
    
   
#define  HAL_SCAN_CAR_LATENCY_VALUE   50//50ms

#define  HAL_SCAN_CAR_INTERVAL_VALUE  5000//5s


#define  CAR_EXSIT_MIN_V              80 //0.8v


typedef uint8 (*process_scan_car_event_t)(uint8 car_exsit);
  
void hal_scan_car_init();
void hal_scan_car_config(process_scan_car_event_t p);//on_board call
void hal_process_scan_car_latency_event();//hal_driver call
void app_start_to_scan_car();

#endif