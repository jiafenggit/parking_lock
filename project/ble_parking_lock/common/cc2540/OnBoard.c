/**************************************************************************************************
  Filename:       OnBoard.c
  Revised:        $Date: 2008-03-18 15:14:17 -0700 (Tue, 18 Mar 2008) $
  Revision:       $Revision: 16604 $

  Description:    This file contains the UI and control for the
                  peripherals on the EVAL development board
  Notes:          This file targets the Chipcon CC2430DB/CC2430EB


  Copyright 2005-2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OnBoard.h"
#include "OSAL.h"
#include "OnBoard.h"

#include "hal_led.h"
#include "hal_key.h"
#if HAL_SCAN_CAR > 0
#include"hal_scan_car.h"
#endif
#if HAL_MOTOR_CHECK > 0
#include"hal_motor.h"
#endif
#if HAL_BATT_CHECK > 0
#include"hal_batt.h"
#endif

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 OnboardKeyIntEnable;


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern uint8 LL_PseudoRand( uint8 *randData, uint8 dataLen );

#if   defined FEATURE_ABL
#include "..\..\util\ABL\app\sbl_app.c"
#elif defined FEATURE_SBL
#include "..\..\util\SBL\app\sbl_app.c"
#elif defined FEATURE_EBL
#include "..\..\util\EBL\app\sbl_app.c"
#elif defined FEATURE_UBL_MSD
#include "..\..\util\UBL\soc_8051\usb_msd\app\ubl_app.c"
#else
void appForceBoot(void);
#endif

/*********************************************************************
 * LOCAL VARIABLES
 */

// Registered keys task ID, initialized to NOT USED.
static uint8 registeredKeysTaskID = NO_TASK_ID;
//register scancar task ID
#if HAL_SCAN_CAR >0
static uint8 registeredScanCarTaskID = NO_TASK_ID;
static uint8 OnBoard_ScanCarCallback(uint8 state);
#endif
#if HAL_MOTOR_CHECK > 0
static uint8 registeredmotorTaskID = NO_TASK_ID;
static uint8 OnBoard_motorCallback(uint8 state);
#endif
#if HAL_BATT_CHECK >0
static uint8 registeredbattTaskID=NO_TASK_ID;
static uint8 OnBoard_battCallback(uint8 batt);
#endif
/*********************************************************************
 * @fn      InitBoard()
 * @brief   Initialize the CC2540DB Board Peripherals
 * @param   level: COLD,WARM,READY
 * @return  None
 */
void InitBoard( uint8 level )
{
  if ( level == OB_COLD )
  {
    // Interrupts off
    osal_int_disable( INTS_ALL );
    // Turn all LEDs off
    HalLedSet( HAL_LED_ALL, HAL_LED_MODE_OFF );
    // Check for Brown-Out reset
//    ChkReset();
  }
  else  // !OB_COLD
  {
    /* Initialize Key stuff */
    OnboardKeyIntEnable = HAL_KEY_INTERRUPT_ENABLE;
    HalKeyConfig( OnboardKeyIntEnable, OnBoard_KeyCallback);
#if HAL_SCAN_CAR > 0
    hal_scan_car_config(OnBoard_ScanCarCallback);
#endif 
#if HAL_MOTOR_CHECK > 0
    hal_motor_config(OnBoard_motorCallback);//on_board call
#endif
#if HAL_BATT_CHECK > 0
    hal_batt_cb_config(OnBoard_battCallback);
#endif
  }
}

/*********************************************************************
 * @fn        Onboard_rand
 *
 * @brief    Random number generator
 *
 * @param   none
 *
 * @return  uint16 - new random number
 *
 *********************************************************************/
uint16 Onboard_rand( void )
{
  uint16 randNum;

  LL_PseudoRand( (uint8 *)&randNum, 2 );

  return ( randNum );
}

/*********************************************************************
 * @fn      _itoa
 *
 * @brief   convert a 16bit number to ASCII
 *
 * @param   num -
 *          buf -
 *          radix -
 *
 * @return  void
 *
 *********************************************************************/
void _itoa(uint16 num, uint8 *buf, uint8 radix)
{
  char c,i;
  uint8 *p, rst[5];

  p = rst;
  for ( i=0; i<5; i++,p++ )
  {
    c = num % radix;  // Isolate a digit
    *p = c + (( c < 10 ) ? '0' : '7');  // Convert to Ascii
    num /= radix;
    if ( !num )
      break;
  }

  for ( c=0 ; c<=i; c++ )
    *buf++ = *p--;  // Reverse character order

  *buf = '\0';
}

/*********************************************************************
 *                        "Keyboard" Support
 *********************************************************************/

/*********************************************************************
 * Keyboard Register function
 *
 * The keyboard handler is setup to send all keyboard changes to
 * one task (if a task is registered).
 *
 * If a task registers, it will get all the keys. You can change this
 * to register for individual keys.
 *********************************************************************/
uint8 RegisterForKeys( uint8 task_id )
{
  // Allow only the first task
  if ( registeredKeysTaskID == NO_TASK_ID )
  {
    registeredKeysTaskID = task_id;
    return ( true );
  }
  else
    return ( false );
}

#if HAL_SCAN_CAR > 0

uint8 register_for_scan_car(uint8 task_id)
{
  // Allow only the first task
  if ( registeredScanCarTaskID == NO_TASK_ID )
  {
    registeredScanCarTaskID = task_id;
    return ( true );
  }
  else
    return ( false );  
}

#endif
#if HAL_MOTOR_CHECK > 0

uint8 register_for_motor(uint8 task_id)
{
  // Allow only the first task
  if ( registeredmotorTaskID == NO_TASK_ID )
  {
    registeredmotorTaskID = task_id;
    return ( true );
  }
  else
    return ( false );  
}

#endif

#if HAL_BATT_CHECK > 0
uint8 register_for_batt(uint8 task_id)
{
  if ( registeredbattTaskID == NO_TASK_ID )
  {
    registeredbattTaskID = task_id;
    return ( true );
  }
  else
    return ( false );  
  
}

#endif

/*********************************************************************
 * @fn      OnBoard_SendKeys
 *
 * @brief   Send "Key Pressed" message to application.
 *
 * @param   keys  - keys that were pressed
 *          state - shifted
 *
 * @return  status
 *********************************************************************/
static uint8 OnBoard_SendKeys( uint8 keys, uint8 state )
{
  keyChange_t *msgPtr;

  if ( registeredKeysTaskID != NO_TASK_ID )
  {
    // Send the address to the task
    msgPtr = (keyChange_t *)osal_msg_allocate( sizeof(keyChange_t) );
    if ( msgPtr )
    {
      msgPtr->hdr.event = KEY_CHANGE;
      msgPtr->state = state;
      msgPtr->keys = keys;

      osal_msg_send( registeredKeysTaskID, (uint8 *)msgPtr );
    
    }
    return ( SUCCESS );
  }
  else
    return ( FAILURE );
}

/*********************************************************************
 * @fn      OnBoard_KeyCallback
 *
 * @brief   Callback service for keys
 *
 * @param   keys  - keys that were pressed
 *          state - shifted
 *
 * @return  void
 *********************************************************************/
static void OnBoard_KeyCallback ( uint8 keys, uint8 state )
{

  OnBoard_SendKeys( keys, state);

}



#if HAL_SCAN_CAR > 0
/*********************************************************************
 * @fn      OnBoard_ScanCarCallback
 *
 * @brief   Callback service for keys
 *
 * @param   none
 *
 * @return  void
 *********************************************************************/
static uint8 OnBoard_ScanCarCallback(uint8 car_exsit)
{
  scan_car_t* msgPtr;
 
  if ( registeredScanCarTaskID != NO_TASK_ID )
  {
    // Send the address to the task
    msgPtr = (scan_car_t *)osal_msg_allocate( sizeof(scan_car_t) );
    if ( msgPtr )
    {
      msgPtr->hdr.event = SCAN_CAR_EVENT;
      msgPtr->state=car_exsit;
      osal_msg_send( registeredScanCarTaskID, (uint8 *)msgPtr );
    }
    return ( SUCCESS );
  }
  else
    return ( FAILURE );

}

#endif

#if HAL_MOTOR_CHECK > 0
/*********************************************************************
 * @fn      OnBoard_motorCallback
 *
 * @brief   Callback service for keys
 *
 * @param   none
 *
 * @return  void
 *********************************************************************/
static uint8 OnBoard_motorCallback(uint8 state)
{
  motor_msg_t* msgPtr;
 
  if ( registeredmotorTaskID != NO_TASK_ID )
  {
    // Send the address to the task
    msgPtr = (motor_msg_t *)osal_msg_allocate( sizeof(motor_msg_t) );
    if ( msgPtr )
    {
      msgPtr->hdr.event = MOTOR_STATE_EVENT;
      msgPtr->state=state;
      osal_msg_send( registeredmotorTaskID, (uint8 *)msgPtr );
    }
    return ( SUCCESS );
  }
  else
    return ( FAILURE );

}

#endif


#if HAL_BATT_CHECK > 0
static uint8 OnBoard_battCallback(uint8 batt)
{
  batt_msg_t* msgPtr;
 
  if ( registeredbattTaskID != NO_TASK_ID )
  {
    // Send the address to the task
    msgPtr = (batt_msg_t *)osal_msg_allocate( sizeof(batt_msg_t) );
    if ( msgPtr )
    {
      msgPtr->hdr.event = BATT_UPDATE_EVENT;
      msgPtr->batt=batt;
      osal_msg_send( registeredbattTaskID, (uint8*)msgPtr );
    }
    return ( SUCCESS );
  }
  else
    return ( FAILURE );
}

#endif 
/*********************************************************************
 * @fn      Onboard_soft_reset
 *
 * @brief   Effect a soft reset.
 *
 * @param   none
 *
 * @return  none
 *
 *********************************************************************/
__near_func void Onboard_soft_reset( void )
{
  HAL_DISABLE_INTERRUPTS();
  asm("LJMP 0x0");
}

#if   defined FEATURE_ABL
#elif defined FEATURE_SBL
#elif defined FEATURE_EBL
#elif defined FEATURE_UBL_MSD
#else
/*********************************************************************
 * @fn      appForceBoot
 *
 * @brief   Common force-boot function for the HCI library to invoke.
 *
 * @param   none
 *
 * @return  void
 *********************************************************************/
void appForceBoot(void)
{
  // Dummy function for HCI library that cannot depend on the SBL build defines.
}
#endif

/*********************************************************************
*********************************************************************/
