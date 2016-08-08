#ifndef  __HAL_MOTOR_H__
#define  __HAL_MOTOR_H__


#include"hal_board.h"

#define  MOTOR_CHECK_PORT                  P1

#define  MOTOR_SPEED_ECHO_POS              BV(2)
#define  MOTOR_SPEED_ECHO_SBIT             P1_2
#define  MOTOR_STOP_ECHO_POS               BV(3)
#define  MOTOR_STOP_ECHO_SBIT              P1_3
#define  MOTOR_POSITIVE_RUN_POS            BV(4)
#define  MOTOR_POSITIVE_RUN_SBIT           P1_4
#define  MOTOR_NEGATIVE_RUN_POS            BV(5)
#define  MOTOR_NEGATIVE_RUN_SBIT           P1_5

#define  MOTOR_CHECK_TRIGGER_PWR_PORT      P0       
#define  MOTOR_CHECK_TRIGGER_PWR_POS       BV(5)
#define  MOTOR_CHECK_TRIGGER_PWR_SBIT      P0_5

#define  MOTOR_BLOCK_CHECK_POS             BV(4)


#define  MOTOR_CHECK_SET_IO_DIR()          \
{                                          \
  P1SEL&=~(MOTOR_SPEED_ECHO_POS|MOTOR_STOP_ECHO_POS|MOTOR_POSITIVE_RUN_POS|MOTOR_NEGATIVE_RUN_POS);\
  P1DIR&=~(MOTOR_SPEED_ECHO_POS|MOTOR_STOP_ECHO_POS);\
  P1DIR|=(MOTOR_POSITIVE_RUN_POS|MOTOR_NEGATIVE_RUN_POS);\
  P0SEL&=~(MOTOR_CHECK_TRIGGER_PWR_POS|MOTOR_BLOCK_CHECK_POS);\
  P0DIR|=MOTOR_CHECK_TRIGGER_PWR_POS;\
  P0INP|=MOTOR_BLOCK_CHECK_POS;\
}



#define  MOTOR_SPEED_ON_INIT              0
#define  MOTOR_SPEED_ON_DRILL             1
#define  MOTOR_SPEED_ON_NO_DRILL          2


/*define motor timeout value*/
#define  MOTOR_CHECK_BLOCK_VALUE          200 //0.2s
#define  MOTOR_CHECK_SPEED_VALUE          50 //0.05s
#define  MOTOR_VERIFY_VALUE               2000//2s

/*define motor state */
#define  MOTOR_STATE_ON_POSITIVE_RUNNING         1
#define  MOTOR_STATE_ON_NEGATIVE_RUNNING         2
#define  MOTOR_STATE_ON_STOP                     3

/*define MOVABLE_ARM state */
#define  MOVABLE_ARM_ON_90_90_STATE              1
#define  MOVABLE_ARM_ON_0_0_STATE                2
#define  MOVABLE_ARM_ON_0_90_STATE               3
#define  MOVABLE_ARM_ON_90_180_STATE             4
#define  MOVABLE_ARM_ON_INIT_STATE               5

#define  SIGNAL_START_TO_SCAN_CAR                1
#define  SIGNAL_STOP_TO_SCAN_CAR                 2
#define  SIGNAL_START_PERIODIC_VERIFY            3
#define  SIGNAL_STOP_PERIODIC_VERIFY             4
#define  SIGNAL_MOVABLE_ARM_ON_TOP               5
#define  SIGNAL_MOVABLE_ARM_ON_BOTTOM            6
#define  SIGNAL_MOTOR_IS_BLOCK                   7
#define  SIGNAL_MOTOR_IS_VERIFY                  8


#define  MOTOR_POSITIVE_RUN_POLARITY         ACTIVE_HIGH
#define  MOTOR_NEGATIVE_RUN_POLARITY         ACTIVE_HIGH
#define  MOTOR_CHECK_TRIGGER_POLARITY        ACTIVE_HIGH

#define  motor_positive_active()             st(MOTOR_POSITIVE_RUN_SBIT=MOTOR_POSITIVE_RUN_POLARITY (1);) 
#define  motor_negative_active()             st(MOTOR_NEGATIVE_RUN_SBIT=MOTOR_NEGATIVE_RUN_POLARITY (1);)
#define  motor_check_trigger_pwr_active()    st(MOTOR_CHECK_TRIGGER_PWR_SBIT=MOTOR_CHECK_TRIGGER_POLARITY (1);)

#define  motor_positive_inactive()           st(MOTOR_POSITIVE_RUN_SBIT=MOTOR_POSITIVE_RUN_POLARITY (0);) 
#define  motor_negative_inactive()           st(MOTOR_NEGATIVE_RUN_SBIT=MOTOR_NEGATIVE_RUN_POLARITY (0);)
#define  motor_check_trigger_pwr_inactive()  st(MOTOR_CHECK_TRIGGER_PWR_SBIT=MOTOR_CHECK_TRIGGER_POLARITY (0);)


typedef uint8 (*process_motor_event_t)(uint8 state);


void hal_motor_check_init();
void hal_motor_config(process_motor_event_t p);//on_board call


void hal_process_motor_check_speed_event();
void hal_process_motor_check_motor_block_event();
void hal_process_motor_verify_event();//hal driver call

void app_movable_arm_set_target_90_90();
void app_movable_arm_set_target_0_0();
void app_motor_start_periodic_verify_state();
void app_motor_stop_periodic_verify_state();









#endif