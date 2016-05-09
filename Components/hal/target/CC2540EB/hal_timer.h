#ifndef __HAL_TIMER_H__
#define __HAL_TIMER_H__

#include"hal_board.h"



#define  TIMER_DIV_1_1      (0<<5)
#define  TIMER_DIV_1_2      (1<<5)
#define  TIMER_DIV_1_4      (2<<5)
#define  TIMER_DIV_1_8      (3<<5)
#define  TIMER_DIV_1_16     (4<<5)
#define  TIMER_DIV_1_32     (5<<5)
#define  TIMER_DIV_1_64     (6<<5)
#define  TIMER_DIV_1_128    (7<<5)


#define  TRIGGER_TIMER_CNT_START()      st( T3CTL|=(1<<4);)
#define  TRIGGER_TIMER_CNT_STOP()       st(T3CTL&=~(1<<4);)

#define  hal_timer3_set_down_mode()     st(T3CTL|=(1<<0);)
#define  hal_timer3_set_div_1_16()      st(T3CTL|=TIMER_DIV_1_32;)
#define  hal_timer3_set_expect_value()  st(T3CC0=10;) //10us trigger  (10+x)x2x38000=1000000us 


void hal_38k_infrared_ray_timer3_init();
void hal_38khz_infrared_ray_start();






#endif