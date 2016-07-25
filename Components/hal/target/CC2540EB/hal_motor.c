#include"hal_defs.h"
#include"hal_adc.h"
#include"hal_types.h"
#include"osal.h"
#include "hal_drivers.h"
#include "osal_timers.h"
#include"app_uart_init.h"
#include"hal_motor.h"


#define  MOTOR_RATED_VOLTAGE              5.5   //5.5v
#define  MOTOR_BLOCK_RES                  3.25 //3.25ohm
#define  MOTOR_BLOCK_CHECK_RES            0.2  //0.2ohm

#define  MOTOR_BLOCK_CHECK_CHN            HAL_ADC_CHANNEL_4
#define  MOTOR_BLOCK_VOLTAGE              (((MOTOR_RATED_VOLTAGE/MOTOR_BLOCK_RES)*MOTOR_BLOCK_CHECK_RES)*100)            

static void hal_motor_positive_run();
static void hal_motor_negative_run();
static void hal_motor_stop_run();
static uint8 hal_get_motor_block_voltage();
static bool hal_motor_is_block();

static void hal_motor_check_open();//open led power
static void hal_motor_check_close();//close led power
static void hal_start_motor_speed_check_timer();
static void hal_stop_motor_speed_check_timer();
static void hal_start_motor_block_check_timer();
static void hal_stop_motor_block_check_timer();
static void hal_process_motor_verify_latency();//hal driver call
static void hal_check_movable_arm_position();
static void hal_send_motor_signal(uint8 signal);
static void hal_process_movable_arm_state();
static void hal_block_process_movable_arm_state();

process_motor_event_t  process_motor_event=NULL;

static uint8 cur_motor_state=MOTOR_STATE_ON_STOP;//马达状态 stop or running
static uint8 cur_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;//当前活动杆状态
static uint8 tar_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;//目标活动杆状态
static uint8 app_tar_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;//app目标位置

void hal_motor_check_init()
{
  MOTOR_CHECK_SET_IO_DIR();  
  
  motor_positive_inactive();
  motor_negative_inactive();
}


void hal_motor_config(process_motor_event_t p)//on_board call
{
  if(process_motor_event==NULL)
     process_motor_event=p; 
}

static uint8 hal_get_motor_block_voltage()
{
  uint8 adc;
  adc=HalAdcRead (MOTOR_BLOCK_CHECK_CHN,HAL_ADC_RESOLUTION_7);
  return adc;
}

static bool hal_motor_is_block()
{
  uint8 v;
  v=hal_get_motor_block_voltage();
  
  app_write_string("\r\nmotor block v:");
  app_write_string(uint8_to_string(v));
  
 
  if(v<0x80 && v>=MOTOR_BLOCK_VOLTAGE)//超过0x80就是负电压不正确
    return TRUE;
  else
    return FALSE;
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

/*活动档杆位置检测*/
static void hal_check_movable_arm_position()
{
    hal_motor_check_open();//打开光线电源
  
   if(!(MOTOR_CHECK_PORT & MOTOR_SPEED_ECHO_POS) && !(MOTOR_CHECK_PORT & MOTOR_STOP_ECHO_POS))//如果同时为低，档杆立起，在90度点。
  {  
    cur_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;  
    app_write_string("\r\n90-90");
   }
 else if(!(MOTOR_CHECK_PORT & MOTOR_SPEED_ECHO_POS) && (MOTOR_CHECK_PORT & MOTOR_STOP_ECHO_POS))//如果speed为低，stop为高，档杆在0-90度范围
  {
    cur_movable_arm_state=MOVABLE_ARM_ON_0_90_STATE;
    app_write_string("\r\n00-90");
   }
 else if((MOTOR_CHECK_PORT & MOTOR_SPEED_ECHO_POS) && !(MOTOR_CHECK_PORT & MOTOR_STOP_ECHO_POS))//如果speed为高，stop为低，档杆在90-180度范围
  {
    cur_movable_arm_state=MOVABLE_ARM_ON_90_180_STATE;
    app_write_string("\r\n90-180");
   }
 else if((MOTOR_CHECK_PORT & MOTOR_SPEED_ECHO_POS) && (MOTOR_CHECK_PORT & MOTOR_STOP_ECHO_POS))//如果speed为高，stop为高，档杆在0度点
  {
    cur_movable_arm_state=MOVABLE_ARM_ON_0_0_STATE;
    app_write_string("\r\n00-00");
   }
   hal_motor_check_close();//关闭光线电源
}

/*堵转处理*/
void hal_process_motor_check_motor_block_event()//检查到堵转就停止运行
{
    app_write_string("\r\n赌转检查...");
  
    if(cur_motor_state!=MOTOR_STATE_ON_STOP)//只要马达在转动就检查堵转
   {
    app_write_string("\r\nnb!");
    if(hal_motor_is_block())//如果检查到堵转 根据活动杆的位置进行相应的处理
    {
     app_write_string("\r\n堵转!"); 
     hal_check_movable_arm_position();
     hal_block_process_movable_arm_state();
    }
    else
    {
     hal_start_motor_block_check_timer();//重新堵转检查定时器
     }
    }
    else if(cur_motor_state==MOTOR_STATE_ON_STOP)
    {
    app_write_string("\r\n堵转检查完成!");  
    }
    else
    {
    hal_start_motor_block_check_timer();//重新堵转检查定时器
    app_write_string("\r\n堵转检查错误!");    
     }
}

/*运行过程处理*/
void hal_process_motor_check_speed_event()
{
  app_write_string("\r\n运行过程检测!");

  hal_check_movable_arm_position();

  if(cur_movable_arm_state!=tar_movable_arm_state)
   {
    hal_start_motor_speed_check_timer();//重新开始运行过程检测定时器   
   }
  else//如果到达指定位置，motor停止运行
  {
    hal_motor_stop_run();//motor 停止运行
    if(cur_movable_arm_state==MOVABLE_ARM_ON_90_90_STATE)
    {  
    hal_send_motor_signal(SIGNAL_MOVABLE_ARM_ON_TOP);//档杆立起
    app_write_string("\r\n档杆立起!");
    }
    if(cur_movable_arm_state==MOVABLE_ARM_ON_0_0_STATE)
    {
    hal_send_motor_signal(SIGNAL_MOVABLE_ARM_ON_BOTTOM);//档杆放下 
    app_write_string("\r\n档杆放下!");
    }
      
  }
  
}

static void hal_send_motor_signal(uint8 signal)
{
  if(process_motor_event)
  {
    process_motor_event(signal);
    app_write_string("\r\n向系统发送马达信号!");
  }
  else
  {
    app_write_string("\r\n马达信号回调函数没配置!");
  } 
}

static void hal_motor_positive_run()
{
  cur_motor_state=MOTOR_STATE_ON_POSITIVE_RUNNING;
  
  motor_negative_inactive();
  motor_positive_active();
  
  hal_start_motor_block_check_timer();
  hal_start_motor_speed_check_timer();
  
  hal_send_motor_signal(SIGNAL_STOP_PERIODIC_VERIFY);//停止校验位置
     
  app_write_string("\r\n开启堵转和过程检测!");
  app_write_string("\r\n马达开始正转!");
  
 }

static void hal_motor_negative_run()
{

  cur_motor_state=MOTOR_STATE_ON_NEGATIVE_RUNNING;
  
  motor_positive_inactive();
  motor_negative_active();
  
  hal_start_motor_block_check_timer();
  hal_start_motor_speed_check_timer();
  
  hal_send_motor_signal(SIGNAL_STOP_PERIODIC_VERIFY);//停止校验位置
  
  app_write_string("\r\n开启堵转和过程检测!");
  app_write_string("\r\n马达开始反转!");
  
}


void hal_motor_stop_run()
{
  cur_motor_state=MOTOR_STATE_ON_STOP;
  
  motor_positive_inactive();
  motor_negative_inactive();

  hal_stop_motor_block_check_timer();
  hal_stop_motor_speed_check_timer();
  
  hal_send_motor_signal(SIGNAL_START_PERIODIC_VERIFY);//开始校验位置
  app_write_string("\r\n马达停止运转!"); 
}

static void hal_process_movable_arm_state()
{
  if(tar_movable_arm_state==MOVABLE_ARM_ON_90_90_STATE)
  {
  switch(cur_movable_arm_state)
  {
  case MOVABLE_ARM_ON_90_90_STATE:
    {
    app_write_string("\r\n当前活动杆位置正确!不动作!" ); 
    }
    break;
  case MOVABLE_ARM_ON_0_0_STATE:

  case MOVABLE_ARM_ON_0_90_STATE:
    {
     hal_motor_positive_run();
    }
    break;
  case MOVABLE_ARM_ON_90_180_STATE:
    {
     hal_motor_negative_run();
    }
    break;
  default:
    {
      app_write_string("\r\n活动杆状态错误!");
    }
    break;
   }
  }
  else if(tar_movable_arm_state==MOVABLE_ARM_ON_0_0_STATE)
  {
     switch(cur_movable_arm_state)
  {
  case MOVABLE_ARM_ON_90_90_STATE:

  case MOVABLE_ARM_ON_0_90_STATE:
    
  case MOVABLE_ARM_ON_90_180_STATE:
    {
      hal_motor_negative_run();
    }
    break;
  case MOVABLE_ARM_ON_0_0_STATE:
    {
    app_write_string("\r\n当前活动杆位置正确!不动作!" );  
    }
    break;
  default:
    {
      app_write_string("\r\n活动杆状态错误!");
    }
    break;
   }
    
  }
 
}

static void hal_block_process_movable_arm_state()
{
  
if(cur_motor_state==MOTOR_STATE_ON_POSITIVE_RUNNING)
{ 
  switch(cur_movable_arm_state)  
  {
    case MOVABLE_ARM_ON_0_0_STATE:
    {
      hal_motor_stop_run();
    }
    break;
    case MOVABLE_ARM_ON_0_90_STATE:
    {
     tar_movable_arm_state=MOVABLE_ARM_ON_0_0_STATE;
     hal_motor_negative_run();
    }
    break;
    case MOVABLE_ARM_ON_90_90_STATE:
    {
     hal_motor_stop_run();
    }
    break;
    case MOVABLE_ARM_ON_90_180_STATE:
    {
     //tar_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;
     //hal_motor_negative_run();
     hal_motor_stop_run();
    }
    break;
    case MOVABLE_ARM_ON_INIT_STATE:
    {
     hal_motor_stop_run(); 
    }
    break;
    default:
    break;
  }
}
else if(cur_motor_state==MOTOR_STATE_ON_NEGATIVE_RUNNING)
{  
  switch(cur_movable_arm_state)  
  {
    case MOVABLE_ARM_ON_0_0_STATE:
    {
      hal_motor_stop_run();
    }
    break;
    case MOVABLE_ARM_ON_0_90_STATE:
    {
     //tar_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;
     //hal_motor_positive_run();
       hal_motor_stop_run();
    }
    break;
    case MOVABLE_ARM_ON_90_90_STATE:
    {
     hal_motor_stop_run();
    }
    break;
    case MOVABLE_ARM_ON_90_180_STATE:
    {
     tar_movable_arm_state=MOVABLE_ARM_ON_INIT_STATE;
     hal_motor_positive_run();
    }
    break;
    case MOVABLE_ARM_ON_INIT_STATE:
    {
     hal_motor_stop_run(); 
    }
    break;
    default:
    break;
  }
 } 
}


void app_movable_arm_set_target_90_90()
{
  if(app_tar_movable_arm_state!=MOVABLE_ARM_ON_90_90_STATE)//如果活动杆目标不是90度
  {
   app_tar_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;
  }
  else
  {
  app_write_string("\r\n目标90活动杆正确!");  
  }
  tar_movable_arm_state=app_tar_movable_arm_state;
  hal_process_movable_arm_state();//处理活动杆状态
  
}

void app_movable_arm_set_target_0_0()
{
  if(app_tar_movable_arm_state!=MOVABLE_ARM_ON_0_0_STATE)//如果活动杆目标不是00度
  {
  app_tar_movable_arm_state=MOVABLE_ARM_ON_0_0_STATE;
  }
  else
  {
  app_write_string("\r\n目标活动杆0正确!");  
  }
  tar_movable_arm_state=app_tar_movable_arm_state;
  hal_process_movable_arm_state();//处理活动杆状态
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
void app_motor_stop_periodic_verify_state()
{
  osal_stop_timerEx(Hal_TaskID, HAL_MOTOR_VERIFY_EVENT);
  app_write_string("\r\n关闭周期性校验!"); 
}
void hal_process_motor_verify_event()//hal driver call
{
  if(cur_motor_state==MOTOR_STATE_ON_STOP)//be sure motor is stop
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
  tar_movable_arm_state=app_tar_movable_arm_state;
  hal_check_movable_arm_position();
  hal_process_movable_arm_state();
  
}