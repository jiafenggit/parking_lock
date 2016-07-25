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

static uint8 cur_motor_state=MOTOR_STATE_ON_STOP;//���״̬ stop or running
static uint8 cur_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;//��ǰ���״̬
static uint8 tar_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;//Ŀ����״̬
static uint8 app_tar_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;//appĿ��λ��

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
  
 
  if(v<0x80 && v>=MOTOR_BLOCK_VOLTAGE)//����0x80���Ǹ���ѹ����ȷ
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

/*�����λ�ü��*/
static void hal_check_movable_arm_position()
{
    hal_motor_check_open();//�򿪹��ߵ�Դ
  
   if(!(MOTOR_CHECK_PORT & MOTOR_SPEED_ECHO_POS) && !(MOTOR_CHECK_PORT & MOTOR_STOP_ECHO_POS))//���ͬʱΪ�ͣ�����������90�ȵ㡣
  {  
    cur_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;  
    app_write_string("\r\n90-90");
   }
 else if(!(MOTOR_CHECK_PORT & MOTOR_SPEED_ECHO_POS) && (MOTOR_CHECK_PORT & MOTOR_STOP_ECHO_POS))//���speedΪ�ͣ�stopΪ�ߣ�������0-90�ȷ�Χ
  {
    cur_movable_arm_state=MOVABLE_ARM_ON_0_90_STATE;
    app_write_string("\r\n00-90");
   }
 else if((MOTOR_CHECK_PORT & MOTOR_SPEED_ECHO_POS) && !(MOTOR_CHECK_PORT & MOTOR_STOP_ECHO_POS))//���speedΪ�ߣ�stopΪ�ͣ�������90-180�ȷ�Χ
  {
    cur_movable_arm_state=MOVABLE_ARM_ON_90_180_STATE;
    app_write_string("\r\n90-180");
   }
 else if((MOTOR_CHECK_PORT & MOTOR_SPEED_ECHO_POS) && (MOTOR_CHECK_PORT & MOTOR_STOP_ECHO_POS))//���speedΪ�ߣ�stopΪ�ߣ�������0�ȵ�
  {
    cur_movable_arm_state=MOVABLE_ARM_ON_0_0_STATE;
    app_write_string("\r\n00-00");
   }
   hal_motor_check_close();//�رչ��ߵ�Դ
}

/*��ת����*/
void hal_process_motor_check_motor_block_event()//��鵽��ת��ֹͣ����
{
    app_write_string("\r\n��ת���...");
  
    if(cur_motor_state!=MOTOR_STATE_ON_STOP)//ֻҪ�����ת���ͼ���ת
   {
    app_write_string("\r\nnb!");
    if(hal_motor_is_block())//�����鵽��ת ���ݻ�˵�λ�ý�����Ӧ�Ĵ���
    {
     app_write_string("\r\n��ת!"); 
     hal_check_movable_arm_position();
     hal_block_process_movable_arm_state();
    }
    else
    {
     hal_start_motor_block_check_timer();//���¶�ת��鶨ʱ��
     }
    }
    else if(cur_motor_state==MOTOR_STATE_ON_STOP)
    {
    app_write_string("\r\n��ת������!");  
    }
    else
    {
    hal_start_motor_block_check_timer();//���¶�ת��鶨ʱ��
    app_write_string("\r\n��ת������!");    
     }
}

/*���й��̴���*/
void hal_process_motor_check_speed_event()
{
  app_write_string("\r\n���й��̼��!");

  hal_check_movable_arm_position();

  if(cur_movable_arm_state!=tar_movable_arm_state)
   {
    hal_start_motor_speed_check_timer();//���¿�ʼ���й��̼�ⶨʱ��   
   }
  else//�������ָ��λ�ã�motorֹͣ����
  {
    hal_motor_stop_run();//motor ֹͣ����
    if(cur_movable_arm_state==MOVABLE_ARM_ON_90_90_STATE)
    {  
    hal_send_motor_signal(SIGNAL_MOVABLE_ARM_ON_TOP);//��������
    app_write_string("\r\n��������!");
    }
    if(cur_movable_arm_state==MOVABLE_ARM_ON_0_0_STATE)
    {
    hal_send_motor_signal(SIGNAL_MOVABLE_ARM_ON_BOTTOM);//���˷��� 
    app_write_string("\r\n���˷���!");
    }
      
  }
  
}

static void hal_send_motor_signal(uint8 signal)
{
  if(process_motor_event)
  {
    process_motor_event(signal);
    app_write_string("\r\n��ϵͳ��������ź�!");
  }
  else
  {
    app_write_string("\r\n����źŻص�����û����!");
  } 
}

static void hal_motor_positive_run()
{
  cur_motor_state=MOTOR_STATE_ON_POSITIVE_RUNNING;
  
  motor_negative_inactive();
  motor_positive_active();
  
  hal_start_motor_block_check_timer();
  hal_start_motor_speed_check_timer();
  
  hal_send_motor_signal(SIGNAL_STOP_PERIODIC_VERIFY);//ֹͣУ��λ��
     
  app_write_string("\r\n������ת�͹��̼��!");
  app_write_string("\r\n��￪ʼ��ת!");
  
 }

static void hal_motor_negative_run()
{

  cur_motor_state=MOTOR_STATE_ON_NEGATIVE_RUNNING;
  
  motor_positive_inactive();
  motor_negative_active();
  
  hal_start_motor_block_check_timer();
  hal_start_motor_speed_check_timer();
  
  hal_send_motor_signal(SIGNAL_STOP_PERIODIC_VERIFY);//ֹͣУ��λ��
  
  app_write_string("\r\n������ת�͹��̼��!");
  app_write_string("\r\n��￪ʼ��ת!");
  
}


void hal_motor_stop_run()
{
  cur_motor_state=MOTOR_STATE_ON_STOP;
  
  motor_positive_inactive();
  motor_negative_inactive();

  hal_stop_motor_block_check_timer();
  hal_stop_motor_speed_check_timer();
  
  hal_send_motor_signal(SIGNAL_START_PERIODIC_VERIFY);//��ʼУ��λ��
  app_write_string("\r\n���ֹͣ��ת!"); 
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

  case MOVABLE_ARM_ON_0_90_STATE:
    
  case MOVABLE_ARM_ON_90_180_STATE:
    {
      hal_motor_negative_run();
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
  if(app_tar_movable_arm_state!=MOVABLE_ARM_ON_90_90_STATE)//������Ŀ�겻��90��
  {
   app_tar_movable_arm_state=MOVABLE_ARM_ON_90_90_STATE;
  }
  else
  {
  app_write_string("\r\nĿ��90�����ȷ!");  
  }
  tar_movable_arm_state=app_tar_movable_arm_state;
  hal_process_movable_arm_state();//������״̬
  
}

void app_movable_arm_set_target_0_0()
{
  if(app_tar_movable_arm_state!=MOVABLE_ARM_ON_0_0_STATE)//������Ŀ�겻��00��
  {
  app_tar_movable_arm_state=MOVABLE_ARM_ON_0_0_STATE;
  }
  else
  {
  app_write_string("\r\nĿ����0��ȷ!");  
  }
  tar_movable_arm_state=app_tar_movable_arm_state;
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
void app_motor_stop_periodic_verify_state()
{
  osal_stop_timerEx(Hal_TaskID, HAL_MOTOR_VERIFY_EVENT);
  app_write_string("\r\n�ر�������У��!"); 
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
  tar_movable_arm_state=app_tar_movable_arm_state;
  hal_check_movable_arm_position();
  hal_process_movable_arm_state();
  
}