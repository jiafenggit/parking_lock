/**************************************************************************************************
  Filename:       simpleBLECentral.h
  Revised:        $Date: 2011-03-03 15:46:41 -0800 (Thu, 03 Mar 2011) $
  Revision:       $Revision: 12 $

  Description:    This file contains the Simple BLE Central sample application
                  definitions and prototypes.

  Copyright 2011 Texas Instruments Incorporated. All rights reserved.

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

#ifndef SIMPLEBLECENTRAL_H
#define SIMPLEBLECENTRAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */


// Simple BLE Central Task Events
#define START_DEVICE_EVT                              0x0001
#define START_DISCOVERY_EVT                           0x0002
#define START_SCAN_EVT                                0x0004
/*
#define START_READ_LOCK_IN_INFO_IN_PEER_EVT           0x0008
#define START_READ_CAR_IN_INFO_IN_PEER_EVT            0x0010
#define START_READ_TIME_STAMP_INFO_IN_PEER_EVT        0x0020
 */
#define  PERIOD_HANDLE_SEND_QUEUE_EVENT               0x0008
#define  PERIOD_HANDLE_RECV_QUEUE_EVENT               0x0010
#define  PERIOD_HANDLE_NET_DEVICE_EVENT               0x0020
#define  PERIOD_HANDLE_REPLY_HEARTBEAT_EVENT          0x0040
#define  PERIOD_HANDLE_UPDATE_TIME_EVENT              0x0080
#define  ATTEMPT_TO_EST_CONN_TIMEOUT_EVT              0x0100

   
/*********************************************************************
 * net param 
 */
#define  MAX_CNT_IN_PER_ID         10   //Ã¿¸öid×î¶à´æ10ÌõÊÚÈ¨ÐÅÏ¢
#define  AUTHORITY_SNV_ID_START    0x80 //ÊÚÈ¨ÐÅÏ¢¿ªÊ¼id
#define  DEFAULT_MAX_MSG_IN_QUEUE  20   //·¢ËÍ¶ÓÁÐºÍ½ÓÊÕ¶ÓÁÐ¶¼ÊÇ×î¶à20¸öÏûÏ¢

   
#define  MSG_HEADER                    '%'
#define  MSG_END                       '#'
#define  ACK_VALUE                     0x55 
#define  HEARTBEAT_VALUE               0xAA  

#define  DEFAULT_NET_TIMEOUT           10000 //10s
  
#define  HANDLE_SEND_QUEUE_PERIOD      2000  //2s
#define  HANDLE_RECV_QUEUE_PERIOD      2000  //2s
#define  HANDLE_POLL_NET_PERIOD        1000  //1s
#define  HANDLE_REPLY_HEARTBEAT_PERIOD 30000 //30s 
#define  HANDLE_UPDATE_TIME_PERIOD     30000//(uint32)24*3600*1000//1Ìì ¸üÐÂÒ»´ÎsysÊ±¼ä
   

#define  MSG_TYPE_REQUEST_TIME         0
#define  MSG_TYPE_REPLY_HEARTBEAT      1
#define  MSG_TYPE_PARKING_INFO         2
#define  MSG_TYPE_REPLY_ACK            3
#define  MSG_TYPE_RECV_TIME            4
#define  MSG_TYPE_RECV_AUTHORITY       5
/*Ä¬ÈÏÊý¾Ý³¤¶È*/ 
#define  DEFAULT_REQUEST_TIME_LEN      6
#define  DEFAULT_PARKING_INFO_LEN      32
#define  DEFAULT_REPLY_ACK_LEN         1
#define  DEFAULT_REPLY_HEARTBEAT_LEN   7
#define  DEFAULT_RECV_TIME_LEN         12
#define  DEFAULT_RECV_AUTHORITY_LEN    7
   
   
typedef struct _msg_node
{
  void *ptr_next;
  void *ptr_msg;
  uint16 timeout;
  uint16 reload_timeout;
  bool  delete_flag;
}msg_node_t;

  
typedef struct _msg_comm
{
  uint8 msg_hdr;
  uint32 msg_id;
  uint8 msg_type;
  uint8 len; 
}msg_comm_t;
   
typedef struct _request_time_msg
{
  uint8 msg_hdr;
  uint32 msg_id;
  uint8 msg_type;
  uint8 len;
  uint8 value[DEFAULT_REQUEST_TIME_LEN];
  uint8 msg_end;
}request_time_msg_t; 
  
typedef struct _upload_parking_info_msg
{
  uint8 msg_hdr;
  uint32 msg_id;
  uint8 msg_type;
  uint8 len;
  uint8 value[DEFAULT_PARKING_INFO_LEN];
  uint8 msg_end;
}upload_parking_info_msg_t; 
/*
typedef struct _upload_batt_info_msg
{
  void *ptr_next;
  uint8 msg_hdr;
  uint32 msg_id;
  uint8 msg_type;
  uint8 len;
  uint8 value[DEFAULT_BATT_INFO_LEN];
  uint8 msg_end;
}upload_batt_info_msg_t; 
*/
typedef struct _replay_ack_msg
{
  uint8 msg_hdr;
  uint32 msg_id;
  uint8 msg_type;
  uint8 len;
  uint8 value[DEFAULT_REPLY_ACK_LEN];
  uint8 msg_end;
}replay_ack_msg_t; 

typedef struct _replay_heartbeat_msg
{
  uint8 msg_hdr;
  uint32 msg_id;
  uint8 msg_type;
  uint8 len;
  uint8 value[DEFAULT_REPLY_HEARTBEAT_LEN];
  uint8 msg_end;
}replay_heartbeat_msg_t; 

typedef struct _recv_time_msg
{
  uint8 msg_hdr;
  uint32 msg_id;
  uint8 msg_type;
  uint8 len;
  uint8 value[DEFAULT_RECV_TIME_LEN];
  uint8 msg_end;
}recv_time_msg_t; 

typedef struct _recv_authority_msg
{
  uint8 msg_hdr;
  uint32 msg_id;
  uint8 msg_type;
  uint8 len;
  uint8 value[DEFAULT_RECV_AUTHORITY_LEN];
  uint8 msg_end;
}recv_authority_msg_t; 


/*********************************************************************
 * MACROS
 */

// LCD macros
#if HAL_LCD == TRUE
#define LCD_WRITE_STRING(str, option)                       HalLcdWriteString( (str), (option))
#define LCD_WRITE_SCREEN(line1, line2)                      HalLcdWriteScreen( (line1), (line2) )
#define LCD_WRITE_STRING_VALUE(title, value, format, line)  HalLcdWriteStringValue( (title), (value), (format), (line) )
#else
#define LCD_WRITE_STRING(str, option)                     
#define LCD_WRITE_SCREEN(line1, line2)                    
#define LCD_WRITE_STRING_VALUE(title, value, format, line)
#endif

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void SimpleBLECentral_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLECENTRAL_H */
