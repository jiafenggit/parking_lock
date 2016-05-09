/**************************************************************************************************
  Filename:       hal_key.h
  Revised:        $Date: 2007-07-06 10:42:24 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Description:    This file contains the interface to the KEY Service.


  Copyright 2005-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED ìAS ISî WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#ifndef HAL_KEY_H
#define HAL_KEY_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 *                                             INCLUDES
 **************************************************************************************************/
#include "hal_board.h"
  
/**************************************************************************************************
 * MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/* Interrupt option - Enable or disable */
#define HAL_KEY_INTERRUPT_DISABLE    0x00
#define HAL_KEY_INTERRUPT_ENABLE     0x01

/* Key state - shift or nornal */
#define HAL_KEY_STATE_NORMAL          0x00
#define HAL_KEY_STATE_SHIFT           0x01

#define HAL_KEY_SW_1 0x01  // Joystick up
#define HAL_KEY_SW_2 0x02  // Joystick right
#define HAL_KEY_SW_5 0x04  // Joystick center
#define HAL_KEY_SW_4 0x08  // Joystick left
#define HAL_KEY_SW_3 0x10  // Joystick down

#define HAL_KEY_SW_6 0x20  // Button S1 if available
#define HAL_KEY_SW_7 0x40  // Button S2 if available

/* Joystick */
#define HAL_KEY_UP     0x01  // Joystick up
#define HAL_KEY_RIGHT  0x02  // Joystick right
#define HAL_KEY_CENTER 0x04  // Joystick center
#define HAL_KEY_LEFT   0x08  // Joystick left
#define HAL_KEY_DOWN   0x10  // Joystick down

/* Buttons */  
#define HAL_PUSH_BUTTON_RIGHT  0x01  // Button right
#define HAL_PUSH_BUTTON_LEFT   0x02  // Button left
#define HAL_PUSH_BUTTON_SELECT 0x04  // Button select
#define HAL_KEY_BUTTON_UP      0x40  // Button up
#define HAL_KEY_BUTTON_DOWN    0x80  // Button down


/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
#define HAL_KEY_RISING_EDGE           0
#define HAL_KEY_FALLING_EDGE          1

#define HAL_KEY_DEBOUNCE_VALUE        25


#define SCAN_CAR_EXSIT_CHN          HAL_ADC_CHANNEL_6
#define BATT_MONITOR_CHN            HAL_ADC_CHANNEL_7

#define  KEY_FUN1                      1
#define  KEY_FUN2                      2


#define   KEYBOARD_PORT                P1  
//#define   K1_PORT_IDEX                 0
#define   K1_PORT_POS                  BV(0)
//#define   K2_PORT_IDEX                 1
#define   K2_PORT_POS                  BV(1)

#define   VOLTAGE_MONITOR_PORT         P0
 //#define   V_MONITOR_PORT_IDEX          7
#define   V_MONITOR_POS                BV(7)

  
#define  KEY_INT_FLAG                  P1IFG
#define  KEY_INT_VECT                  P1INT_VECTOR

//…Ë÷√Œ™Õ®”√IO∫Õ ‰»ÎΩ”ø⁄ 
#define  KEY_SET_IO_DIR()              \
{                                      \
  P1SEL&=~(K1_PORT_POS|K2_PORT_POS);   \
  P1DIR&=~(K1_PORT_POS|K2_PORT_POS);   \
  P0SEL|=V_MONITOR_POS;                \
}  

//‘ –Ì÷–∂œœ¬Ωµ—ÿ¥•∑¢
//‘ –Ì÷–∂œ¥•∑¢
#define  KEY_ENABLE_INT()            st(IEN2|=1<<4;P1IEN|=(K1_PORT_POS|K2_PORT_POS);) 
#define  KEY_SET_INT_RISE_EDGE()     st(PICTL&=~BV(1);)
#define  KEY_SET_INT_FALL_EDGE()     st(PICTL|=BV(1);)

#define clear_keys_port_int_flag()   st(IRCON2&=~(1<<3);)
#define clear_keys_pin_int_flag()    st(KEY_INT_FLAG &=~(K1_PORT_POS|K2_PORT_POS);)  
   

/**************************************************************************************************
 * TYPEDEFS
 **************************************************************************************************/
typedef void (*halKeyCBack_t) (uint8 keys, uint8 state);

/**************************************************************************************************
 *                                             GLOBAL VARIABLES
 **************************************************************************************************/
extern bool Hal_KeyIntEnable;

/**************************************************************************************************
 *                                             FUNCTIONS - API
 **************************************************************************************************/

/*
 * Initialize the Key Service
 */
extern void HalKeyInit( void );

/*
 * Configure the Key Service
 */
extern void HalKeyConfig( bool interruptEnable, const halKeyCBack_t cback);

/*
 * Read the Key status
 */
extern uint8 HalKeyRead( void);

/*
 * Enter sleep mode, store important values
 */
extern void HalKeyEnterSleep ( void );

/*
 * Exit sleep mode, retore values
 */
extern void HalKeyExitSleep ( void );

/*
 * This is for internal used by hal_driver
 */
extern void HalKeyPoll ( void );

/*
 * This is for internal used by hal_sleep
 */
extern bool HalKeyPressed( void );

extern uint8 hal_key_keys(void);                                           

extern uint8 hal_key_int_keys(void);

extern uint8 halGetVoltageMonitorInput(uint8 chn);
/**************************************************************************************************
**************************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
