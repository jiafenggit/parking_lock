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
  app_write_string("\r\n�����⵽��ת!");
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
   app_write_string("\r\n���ת�򲻶�!"); 
  }
}


void hal_process_motor_check_speed_event()
{
  app_write_string("\r\n�ٶȺͶ�ת���!");
  hal_motor_check_open();//�򿪹��ߵ�Դ
  hal_start_motor_speed_check_timer();//���¿�ʼ�ٶȼ��
  
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
   app_write_string("\r\n�ٶ���Ч!���ö�ת��ʱ��!"); 
   hal_start_motor_block_check_timer();//���ö�ת���
   
   motor_speed_state_original= motor_speed_state;
  }
  else 
  {
   app_write_string("."); //��Ч��
  }
  
  if(!(MOTOR_CHECK_PORT & MOTOR_STOP_ECHO_POS))//����ĳ���˵�
  {
   if(motor_speed_state==MOTOR_SPEED_ON_DRILL && cur_motor_state==MOTOR_ON_NEGATIVE_RUN_STATE)//�������ת��������ȷ
   {    
    app_write_string("\r\n��ﵽ��ײ���λ��!"); 
    app_write_string("\r\n���˷���!");
   
    cur_motor_state=MOTOR_ON_BOTTOM_STATE;
    hal_motor_stop_run();//�ر���� 
   }
   else if(motor_speed_state==MOTOR_SPEED_ON_NO_DRILL && cur_motor_state==MOTOR_ON_POSITIVE_RUN_STATE)//�������ת��������ȷ
  {
    app_write_string("\r\n��ﵽ�ﶥ����λ��!"); 
    app_write_string("\r\n��������!");
    cur_motor_state=MOTOR_ON_TOP_STATE;
    hal_motor_stop_run();//�ر���� 
   }
   else
   {
    app_write_string("\r\n�����λ����Ч!"); 
   }   
  }
 hal_motor_check_close();//�رչ��ߵ�Դ
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
  
  app_write_string("\r\n������ת���!");
  app_write_string("\r\n��￪ʼ��ת!");
  
   if(process_motor_event)
  {
    process_motor_event(cur_motor_state);
    app_write_string("\r\n��ϵͳ�������״̬!");
  }
  else
  {
    app_write_string("\r\n���ص�����û����!");
  }

 }
 else
 {
   app_write_string("\r\n������������״̬,���������ת!");
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
  
  app_write_string("\r\n������ת���!");
  app_write_string("\r\n��￪ʼ��ת!");
  
  if(process_motor_event)
  {
    process_motor_event(cur_motor_state);
    app_write_string("\r\n��ϵͳ�������״̬!");
  }
  else
  {
    app_write_string("\r\n���ص�����û����!");
  }

 }
 else
 {
   app_write_string("\r\n�������Ƿ���״̬,������跴ת!");
 }
}


static void hal_motor_stop_run()
{
  hal_stop_motor_block_check_timer();
  hal_stop_motor_speed_check_timer();
  
  motor_positive_inactive();
  motor_negative_inactive();

  
  app_write_string("\r\n���ֹͣ��ת!");
  if(process_motor_event)
  {
    process_motor_event(cur_motor_state);
    app_write_string("\r\n��ϵͳ�������״̬!");
  }
  else
  {
    app_write_string("\r\n���ص�����û����!");
  } 
}

void app_motor_set_target_state_top()
{
  if(tar_motor_state!=MOTOR_ON_TOP_STATE)
  {
  app_write_string("\r\n���������ת!");
  tar_motor_state=MOTOR_ON_TOP_STATE;
  hal_motor_positive_run();
  }
  else
  {
   app_write_string("\r\n���Ŀ����ȷ�����ö�!");  
  }
   

}
void app_motor_set_target_state_bottom()
{
  if( tar_motor_state!=MOTOR_ON_BOTTOM_STATE)
  {
  app_write_string("\r\n������ﷴת!");
  tar_motor_state=MOTOR_ON_BOTTOM_STATE;
  hal_motor_negative_run();
  }
  else
  {
  app_write_string("\r\n���Ŀ����ȷ�����õ�!");  
  }
  

}

static void hal_motor_check_open()//open led power
{  
 uint8 i=0x80;
 motor_check_trigger_pwr_active(); 
 while(i--);//��ʱ�ȴ�������ȶ�
 app_write_string("\r\n��λ��У���Դ!");
}
static void hal_motor_check_close()//close led power
{
  motor_check_trigger_pwr_inactive();
 app_write_string("\r\n�ر�λ��У���Դ!");
}

void app_motor_start_periodic_verify_state()
{
  osal_start_reload_timer(Hal_TaskID, HAL_MOTOR_VERIFY_EVENT, MOTOR_VERIFY_VALUE);
  app_write_string("\r\n��ʼ������У��,����Ϊ(ms):");
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
     app_write_string("\r\n�޷�У��!���������ת��!");
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
   app_write_string("\r\nλ��У��-��ת!");
   cur_motor_state=MOTOR_ON_INIT_STATE;
   hal_motor_positive_run();

 }
 if(run_state==MOTOR_ON_NEGATIVE_RUN_STATE)
 {
   app_write_string("\r\nλ��У��-��ת!");
   cur_motor_state=MOTOR_ON_INIT_STATE;
   hal_motor_negative_run();
 }
 if(run_state==MOTOR_ON_INIT_STATE)
 {
   app_write_string("\r\nλ��У��Ϊ��");

 }
  hal_motor_check_close();//close led power
}