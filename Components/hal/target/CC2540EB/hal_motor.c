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
static void hal_check_movable_arm_position();
static void hal_send_motor_state();
static void hal_process_movable_arm_state();

process_motor_event_t  process_motor_event=NULL;

static uint8 cur_motor_state=MOTOR_STATE_ON_RUNNING;//���״̬ stop or running
static uint8 cur_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;//��ǰ���״̬
static uint8 tar_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;//Ŀ����״̬


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

/*�����λ�ü��*/
static void hal_check_movable_arm_position()
{
   if(!(MOTOR_CHECK_PORT & MOTOR_SPEED_ECHO_POS) && !(MOTOR_CHECK_PORT & MOTOR_STOP_ECHO_POS))//���ͬʱΪ�ͣ�����������90�ȵ㡣
  {  
    cur_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;  
   }
 else if(!(MOTOR_CHECK_PORT & MOTOR_SPEED_ECHO_POS) && (MOTOR_CHECK_PORT & MOTOR_STOP_ECHO_POS))//���speedΪ�ͣ�stopΪ�ߣ�������0-90�ȷ�Χ
  {
    cur_movable_arm_state=MOVABLE_ARM_ON_0_90_STATE;
   }
 else if((MOTOR_CHECK_PORT & MOTOR_SPEED_ECHO_POS) && !(MOTOR_CHECK_PORT & MOTOR_STOP_ECHO_POS))//���speedΪ�ߣ�stopΪ�ͣ�������90-180�ȷ�Χ
  {
    cur_movable_arm_state=MOVABLE_ARM_ON_90_180_STATE;
   }
 else if(!(MOTOR_CHECK_PORT & MOTOR_SPEED_ECHO_POS) && (MOTOR_CHECK_PORT & MOTOR_STOP_ECHO_POS))//���speedΪ�ߣ�stopΪ�ߣ�������0�ȵ�
  {
    cur_movable_arm_state=MOVABLE_ARM_ON_0_0_STATE;
   }
 
}

/*��ת����*/
void hal_process_motor_check_motor_block_event()//��鵽��ת��ֹͣ����
{
  app_write_string("\r\n�����⵽��ת!");
  
  hal_motor_stop_run();
}

/*���й��̴���*/
void hal_process_motor_check_speed_event()
{
  app_write_string("\r\n���й��̼��!");
  hal_motor_check_open();//�򿪹��ߵ�Դ
  hal_check_movable_arm_position();
  
  if(cur_movable_arm_state!=tar_movable_arm_state)
   {
    hal_start_motor_speed_check_timer();//���¿�ʼ���й��̼�ⶨʱ�� 
    if(cur_movable_arm_state==MOVABLE_ARM_ON_90_90_STATE)
    {
      hal_start_motor_block_check_timer();//���¿�ʼ��ת��鶨ʱ��
    }
   }
  else//�������ָ��λ�ã�motorֹͣ����
  {
    hal_stop_motor_block_check_timer();//ֹͣ��ת��ⶨʱ��
    hal_motor_stop_run();//motor ֹͣ����
  }
  
 hal_motor_check_close();//�رչ��ߵ�Դ
}

static void hal_send_motor_state()
{
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

static void hal_motor_positive_run()
{
  cur_motor_state=MOTOR_STATE_ON_RUNNING;
  
  motor_negative_inactive();
  motor_positive_active();
  
  hal_start_motor_block_check_timer();
  hal_start_motor_speed_check_timer();
  
  app_write_string("\r\n������ת�͹��̼��!");
  app_write_string("\r\n��￪ʼ��ת!");
  
 }

static void hal_motor_negative_run()
{

  cur_motor_state=MOTOR_STATE_ON_RUNNING;
  
  motor_positive_inactive();
  motor_negative_active();
  
  hal_start_motor_block_check_timer();
  hal_start_motor_speed_check_timer();
  
  app_write_string("\r\n������ת�͹��̼��!");
  app_write_string("\r\n��￪ʼ��ת!");
  
}


static void hal_motor_stop_run()
{
  cur_motor_state=MOTOR_STATE_ON_STOP;
  hal_stop_motor_block_check_timer();
  hal_stop_motor_speed_check_timer();
  
  motor_positive_inactive();
  motor_negative_inactive();

  
  app_write_string("\r\n���ֹͣ��ת!");
  hal_send_motor_state();
  
}

static void hal_process_movable_arm_state()
{
  if(tar_movable_arm_state==MOVABLE_ARM_ON_90_90_STATE)
  {
  switch(cur_movable_arm_state)
  {
  case MOVABLE_ARM_ON_90_90_STATE:
    {
    app_write_string("\r\n��ǰ���λ����ȷ!������!" ); 
    }
    break;
  case MOVABLE_ARM_ON_0_0_STATE:
    {
     hal_motor_positive_run(); 
     hal_send_motor_state();//
    }
    break;
  case MOVABLE_ARM_ON_0_90_STATE:
    {
     hal_motor_positive_run();
    }
    break;
  case MOVABLE_ARM_ON_90_180_STATE:
    {
      hal_motor_negative_run();
      hal_send_motor_state();
    }
    break;
  default:
    {
      app_write_string("\r\n���״̬����!");
    }
    break;
   }
  }
  else if(tar_movable_arm_state==MOVABLE_ARM_ON_0_0_STATE)
  {
     switch(cur_movable_arm_state)
  {
  case MOVABLE_ARM_ON_90_90_STATE:
    {
      hal_motor_negative_run();
    }
    break;
  case MOVABLE_ARM_ON_0_90_STATE:
    {
      hal_motor_negative_run();  
    }
    break;
  case MOVABLE_ARM_ON_90_180_STATE:
    {
      hal_motor_negative_run();
      hal_send_motor_state();
    }
    break;
  case MOVABLE_ARM_ON_0_0_STATE:
    {
    app_write_string("\r\n��ǰ���λ����ȷ!������!" );  
    }
    break;
  default:
    {
      app_write_string("\r\n���״̬����!");
    }
    break;
   }
    
  }
}

void app_movable_arm_set_target_90_90()
{
  if(tar_movable_arm_state!=MOVABLE_ARM_ON_90_90_STATE)//������Ŀ�겻��90��
  {
  tar_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;
  }
  else
  {
  app_write_string("\r\nĿ��90�����ȷ!");  
  }
  hal_process_movable_arm_state();//������״̬
  
}

void app_movable_arm_set_target_0_0()
{
  if(tar_movable_arm_state!=MOVABLE_ARM_ON_0_0_STATE)//������Ŀ�겻��90��
  {
  tar_movable_arm_state=MOVABLE_ARM_ON_0_0_STATE;
  }
  else
  {
  app_write_string("\r\nĿ����0��ȷ!");  
  }
  hal_process_movable_arm_state();//������״̬
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
  if(cur_motor_state==MOTOR_STATE_ON_STOP)//be sure motor is stop
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
  hal_motor_check_open();//open led power 
  
  hal_check_movable_arm_position();
  hal_process_movable_arm_state();
  
  hal_motor_check_close();//close led power
}