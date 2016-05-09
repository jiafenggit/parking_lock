#include"hal_defs.h"
#include"hal_motor.h"
#include"hal_types.h"
#include"osal.h"
#include "hal_drivers.h"
#include "osal_timers.h"
#include"app_uart_init.h"

static void hal_motor_positive_run();
static void hal_motor_negative_run();
static void hal_motor_stop_run();
static void hal_motor_check_open();//open led power
static void hal_motor_check_close();//close led power
static void hal_start_motor_speed_check_timer();
static void hal_stop_motor_speed_check_timer();
static void hal_start_motor_block_check_timer();
static void hal_stop_motor_block_check_timer();
static void hal_process_motor_verify_latency();//hal driver call
process_motor_event_t  process_motor_event=NULL;

static uint8 cur_motor_state=MOTOR_ON_TOP_STATE;
static uint8 tar_motor_state=MOTOR_ON_TOP_STATE;

static uint8 motor_speed_state=MOTOR_SPEED_ON_INIT;
static uint8 motor_speed_state_original=MOTOR_SPEED_ON_INIT;

void hal_motor_check_init()
{
  MOTOR_CHECK_SET_IO_DIR();  
  hal_motor_stop_run();
}


void hal_motor_config(process_motor_event_t p)//on_board call
{
  if(process_motor_event==NULL)
     process_motor_event=p; 
}

static void hal_start_motor_block_check_timer()
{
 osal_start_timerEx(Hal_TaskID,HAL_MOTOR_CHECK_BLOCK_EVENT,MOTOR_CHECK_BLOCK_VALUE); 
}
static void hal_stop_motor_block_check_timer()
{
 osal_stop_timerEx(Hal_TaskID,HAL_MOTOR_CHECK_BLOCK_EVENT);
}
static void hal_start_motor_speed_check_timer()
{
osal_start_timerEx(Hal_TaskID,HAL_MOTOR_CHECK_SPEED_EVENT,MOTOR_CHECK_SPEED_VALUE);
}
static void hal_stop_motor_speed_check_timer()
{
 osal_stop_timerEx(Hal_TaskID,HAL_MOTOR_CHECK_SPEED_EVENT);
}


void hal_process_motor_check_motor_block_event()
{
  app_write_string("\r\n警告检测到赌转!");
  if(cur_motor_state==MOTOR_ON_POSITIVE_RUN_STATE)
  {
    hal_motor_negative_run();
  }
  else if(cur_motor_state==MOTOR_ON_NEGATIVE_RUN_STATE)
  {
    hal_motor_positive_run();
  }
  else
  {
   app_write_string("\r\n马达转向不对!"); 
  }
}


void hal_process_motor_check_speed_event()
{
  app_write_string("\r\n速度和赌转检测!");
  hal_motor_check_open();//打开光线电源
  hal_start_motor_speed_check_timer();//重新开始速度检测
  
  if(MOTOR_CHECK_PORT & MOTOR_SPEED_ECHO_POS)
  {
   motor_speed_state=MOTOR_SPEED_ON_NO_DRILL;
  }
  else
  {
   motor_speed_state=MOTOR_SPEED_ON_DRILL; 
  }
  
    
  if(motor_speed_state!=motor_speed_state_original)
  {
   app_write_string("\r\n速度有效!重置赌转定时器!"); 
   hal_start_motor_block_check_timer();//重置赌转检测
   
   motor_speed_state_original= motor_speed_state;
  }
  else 
  {
   app_write_string("."); //无效点
  }
  
  if(!(MOTOR_CHECK_PORT & MOTOR_STOP_ECHO_POS))//到达某个端点
  {
   if(motor_speed_state==MOTOR_SPEED_ON_DRILL && cur_motor_state==MOTOR_ON_NEGATIVE_RUN_STATE)//马达向下转动方向正确
   {    
    app_write_string("\r\n马达到达底部限位点!"); 
    app_write_string("\r\n档杆放下!");
   
    cur_motor_state=MOTOR_ON_BOTTOM_STATE;
    hal_motor_stop_run();//关闭马达 
   }
   else if(motor_speed_state==MOTOR_SPEED_ON_NO_DRILL && cur_motor_state==MOTOR_ON_POSITIVE_RUN_STATE)//马达向上转动方向正确
  {
    app_write_string("\r\n马达到达顶部限位点!"); 
    app_write_string("\r\n档杆立起!");
    cur_motor_state=MOTOR_ON_TOP_STATE;
    hal_motor_stop_run();//关闭马达 
   }
   else
   {
    app_write_string("\r\n马达限位点无效!"); 
   }   
  }
 hal_motor_check_close();//关闭光线电源
}


static void hal_motor_positive_run()
{
 if(cur_motor_state!=MOTOR_ON_TOP_STATE)
 {
  cur_motor_state=MOTOR_ON_POSITIVE_RUN_STATE;
  
  motor_negative_inactive();
  motor_positive_active();
  
  hal_start_motor_block_check_timer();
  hal_start_motor_speed_check_timer();
  
  app_write_string("\r\n开启堵转检测!");
  app_write_string("\r\n马达开始正转!");
  
   if(process_motor_event)
  {
    process_motor_event(cur_motor_state);
    app_write_string("\r\n向系统发送马达状态!");
  }
  else
  {
    app_write_string("\r\n马达回调函数没配置!");
  }

 }
 else
 {
   app_write_string("\r\n立杆已是立起状态,马达无需正转!");
 }
}

static void hal_motor_negative_run()
{
  if(cur_motor_state!=MOTOR_ON_BOTTOM_STATE)
  {
  cur_motor_state=MOTOR_ON_NEGATIVE_RUN_STATE;
  
  motor_positive_inactive();
  motor_negative_active();
  
  hal_start_motor_block_check_timer();
  hal_start_motor_speed_check_timer();
  
  app_write_string("\r\n开启堵转检测!");
  app_write_string("\r\n马达开始反转!");
  
  if(process_motor_event)
  {
    process_motor_event(cur_motor_state);
    app_write_string("\r\n向系统发送马达状态!");
  }
  else
  {
    app_write_string("\r\n马达回调函数没配置!");
  }

 }
 else
 {
   app_write_string("\r\n立杆已是放下状态,马达无需反转!");
 }
}


static void hal_motor_stop_run()
{
  hal_stop_motor_block_check_timer();
  hal_stop_motor_speed_check_timer();
  
  motor_positive_inactive();
  motor_negative_inactive();

  
  app_write_string("\r\n马达停止运转!");
  if(process_motor_event)
  {
    process_motor_event(cur_motor_state);
    app_write_string("\r\n向系统发送马达状态!");
  }
  else
  {
    app_write_string("\r\n马达回调函数没配置!");
  } 
}

void app_motor_set_target_state_top()
{
  if(tar_motor_state!=MOTOR_ON_TOP_STATE)
  {
  app_write_string("\r\n设置马达正转!");
  tar_motor_state=MOTOR_ON_TOP_STATE;
  hal_motor_positive_run();
  }
  else
  {
   app_write_string("\r\n马达目标正确无需置顶!");  
  }
   

}
void app_motor_set_target_state_bottom()
{
  if( tar_motor_state!=MOTOR_ON_BOTTOM_STATE)
  {
  app_write_string("\r\n设置马达反转!");
  tar_motor_state=MOTOR_ON_BOTTOM_STATE;
  hal_motor_negative_run();
  }
  else
  {
  app_write_string("\r\n马达目标正确无需置底!");  
  }
  

}

static void hal_motor_check_open()//open led power
{  
 uint8 i=0x80;
 motor_check_trigger_pwr_active(); 
 while(i--);//延时等待光输出稳定
 app_write_string("\r\n打开位置校验电源!");
}
static void hal_motor_check_close()//close led power
{
  motor_check_trigger_pwr_inactive();
 app_write_string("\r\n关闭位置校验电源!");
}

void app_motor_start_periodic_verify_state()
{
  osal_start_reload_timer(Hal_TaskID, HAL_MOTOR_VERIFY_EVENT, MOTOR_VERIFY_VALUE);
  app_write_string("\r\n开始周期性校验,周期为(ms):");
  app_write_string(uint16_to_string(MOTOR_VERIFY_VALUE));
}
void hal_process_motor_verify_event()//hal driver call
{
  if(cur_motor_state==MOTOR_ON_TOP_STATE || cur_motor_state==MOTOR_ON_BOTTOM_STATE)//be sure motor is stop
  {
  hal_process_motor_verify_latency();
  }
  else
  {
     app_write_string("\r\n无法校验!电机正在运转中!");
  }
}
static void hal_process_motor_verify_latency()//hal driver call
{
  uint8 run_state=MOTOR_ON_INIT_STATE;
  hal_motor_check_open();//open led power
  
  if(tar_motor_state==MOTOR_ON_BOTTOM_STATE)
  {
    if(!(!(MOTOR_CHECK_PORT&MOTOR_STOP_ECHO_POS) && !(MOTOR_CHECK_PORT&MOTOR_SPEED_ECHO_POS)))
    {
      run_state=MOTOR_ON_NEGATIVE_RUN_STATE;
    }
  }
  if(tar_motor_state==MOTOR_ON_TOP_STATE )
  {
    if(!(!(MOTOR_CHECK_PORT&MOTOR_STOP_ECHO_POS) && (MOTOR_CHECK_PORT&MOTOR_SPEED_ECHO_POS)))
    {
      run_state=MOTOR_ON_POSITIVE_RUN_STATE;
    }
  }
  
 if( run_state == MOTOR_ON_POSITIVE_RUN_STATE)
 {
   app_write_string("\r\n位置校验-正转!");
   cur_motor_state=MOTOR_ON_INIT_STATE;
   hal_motor_positive_run();

 }
 if(run_state==MOTOR_ON_NEGATIVE_RUN_STATE)
 {
   app_write_string("\r\n位置校验-反转!");
   cur_motor_state=MOTOR_ON_INIT_STATE;
   hal_motor_negative_run();
 }
 if(run_state==MOTOR_ON_INIT_STATE)
 {
   app_write_string("\r\n位置校验为空");

 }
  hal_motor_check_close();//close led power
}