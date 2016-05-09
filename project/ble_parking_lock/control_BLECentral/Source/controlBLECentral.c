/**************************************************************************************************
  Filename:       lock_in_BLECentral.c
  Revised:        $Date: 2011-06-20 11:57:59 -0700 (Mon, 20 Jun 2011) $
  Revision:       $Revision: 28 $

  Description:    This file contains the Simple BLE Central sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

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

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "hal_adc.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "controlBLECentral.h"
#include "osal_snv.h"

#include"osal_clock.h"
#include"error.h"
#include"app_uart_init.h"
#include"hal_w5500.h"
#include"dhcp.h"
#include"socket.h"
#include"net.h"




/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 300

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE               DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         FALSE  

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          TRUE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      400

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY           0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           200 

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1 //wkxboot 1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          FALSE  //TRUE


#define  DEFAULT_START_TO_SCAN_DELAY          100//wkxboot in ms

   
   
#define  DAFAULT_ADDR_MATCH_LEN                       3

#define  PARK_FLAG_ENTER                              1
#define  PARK_FLAG_EXIT                               2

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_LOCK_IN_CHAR,        // Characteristic discovery
  BLE_DISC_STATE_CAR_IN_CHAR,
  BLE_DISC_STATE_TIME_STAMP_CHAR,
};

/*wkxboot*/
enum
{
  BLE_READ_STAE_IDLE,
  BLE_READ_STAE_LOCK_IN,
  BLE_READ_STAE_CAR_IN,
  BLE_READ_STAE_TIME_STAMP,
  BLE_WRITE_STATE_IDLE,
  BLE_WRITE_STATE_LOCK_IN,
  BLE_WRITE_STATE_CAR_IN,
  BLE_WRITE_STATE_TIME_STAMP, 
};
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8 controlBLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "ble control ";

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
//static uint8 simpleBLEScanIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 simpleBLEScanning = FALSE;

// RSSI polling state
static uint8 simpleBLERssi =FALSE;

// Connection handle of current connection 
static uint16 simpleBLEConnHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8 simpleBLEState = BLE_STATE_IDLE;

// Discovery state
static uint8 simpleBLEDiscState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16 simpleBLESvcStartHdl = 0;
static uint16 simpleBLESvcEndHdl = 0;

// Discovered characteristic handle
static uint16 BLE_lock_in_CharHdl = 0;
static uint16 BLE_car_in_CharHdl = 0;
static uint16 BLE_time_stamp_CharHdl = 0;

static msg_node_t *hdr_send_queue,*hdr_recv_queue;
static uint8 send_queue_cnt=0;//÷∏ æ∑¢ÀÕ∂”¡– ˝¡ø
static uint8 recv_queue_cnt=0;//÷∏ æΩ” ’∂”¡– ˝¡ø

static UTCTimeStruct ble_control_time;//sys time struct 
uint32 ble_msg_id=0;

// Value to write
//static uint8 simpleBLECharVal = 0;

// Value read/write toggle
static bool simpleBLEDoWrite = TRUE;

// GATT read/write procedure state
static bool simpleBLEProcedureInProgress = FALSE;

uint8 owner_id[6]={0x11,0x22,0x33,0x44,0x55,0x66};
//discovery target ble addr
uint8 ble_tar_addr[6];
uint8 ble_match_addr[DAFAULT_ADDR_MATCH_LEN]={0xEC,0x24,0xB8};

ble_device_t lock_in_info,car_in_info;
time_stamp_t time_stamp_info;

uint8 cur_read_char_state=BLE_READ_STAE_IDLE;
uint8 cur_write_char_state=BLE_WRITE_STATE_IDLE;

extern uint8 net_socket_status;
extern uint8 net_src_mac[6];
extern uint8 dhcp_src_mac[6];
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void wkxboot_start_discover();
static void wkxboot_stop_discover();
static void wkxboot_connect();
static void wkxboot_disconnect();
static void controlBLECentral_handle_uart_debug_cmd(uint8 dbg_cmd);
static void own_addr_minus_1_to_peer_addr(uint8 *p_own_addr,uint8* p_peer_addr);
static bool ble_addr_is_match(uint8 *addr1,uint8 *addr2);
static bool ble_full_addr_is_match(uint8 *addr1,uint8 *addr2);
static bool ble_find_match_peer_addr_from_list(gapDevRec_t* addr_list,uint8 list_cnt);
static void owner_addr_init(uint8*addr_src);


static void wkxboot_read_lock_in_info_in_peer();
static void wkxboot_read_car_in_info_in_peer();
static void wkxboot_read_time_stamp_info_in_peer();
static void start_to_find_char_handle(uint8 handle_state);
/*******************net ************************************************/
static uint8 request_sys_time(uint32 msg_id);
static uint8 upload_parking_info(uint32 msg_id);
static uint8 reply_sys_ack(uint32 msg_id);
static uint8 reply_sys_heartbeat(uint32 msg_id);
static void ble_control_handle_recv_queue();
static void ble_control_handle_send_queue();
static uint8 add_to_queue(msg_node_t **hdr_queue,msg_comm_t *ptr_msg ,uint16 timeout);
static uint8 delete_node_from_queue(msg_node_t **hdr_queue,msg_node_t *ptr_node);
static msg_node_t *find_node_from_queue_by_msg_id(msg_node_t *hdr_queue,uint32 msg_id);
static msg_node_t *find_node_from_queue_by_msg_type(msg_node_t *hdr_queue,uint8 msg_type);
static uint8 store_authority_msg_in_snv(recv_authority_msg_t *ptr_msg);
static uint8 process_authority_msg_in_snv(ble_device_t *ptr_dev);

//static void process_new_authority(ble_device_t *ptr_dev);
static void ble_process_parking_lock_info();
static void ble_control_set_new_time(UTCTimeStruct *ptr_cur_time,recv_time_msg_t *ptr_msg);
static void app_start_net();


static void controlBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static void controlBLECentralRssiCB( uint16 connHandle, int8  rssi );
static void controlBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void controlBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void controlBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void controlBLECentral_HandleKeys( uint8 shift, uint8 keys );
static void controlBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void controlBLECentralStartDiscovery( void );
//static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
char *bdAddr2Str ( uint8 *pAddr );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  controlBLECentralRssiCB,       // RSSI callback
  controlBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  controlBLECentralPasscodeCB,
  controlBLECentralPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      lock_in_BLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLECentral_Init( uint8 task_id )
{
  controlBLETaskId = task_id;

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );

  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( controlBLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Register for all key events - This app will handle all key events
  RegisterForKeys( controlBLETaskId );
  register_for_uart_debug(controlBLETaskId);
  
  owner_addr_init(net_src_mac);
  
  app_net_init();
  app_start_net();
  
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  // Setup a delayed profile startup
  osal_set_event( controlBLETaskId, START_DEVICE_EVT );

  //hal_motor_start_periodic_verify_state();//start to verify state
}

/*********************************************************************
 * @fn      lock_in_BLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( controlBLETaskId )) != NULL )
    {
      controlBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &simpleBLEBondCB );

    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & START_DISCOVERY_EVT )
  {
    controlBLECentralStartDiscovery();
    
    return ( events ^ START_DISCOVERY_EVT );
  }
  /*wkxboot*/
  if ( events & START_SCAN_EVT )
  {
    wkxboot_start_discover();
    return ( events ^ START_SCAN_EVT );
  }
  
  if(events & PERIOD_HANDLE_SEND_QUEUE_EVENT)
  {
    osal_start_timerEx(controlBLETaskId,PERIOD_HANDLE_SEND_QUEUE_EVENT,HANDLE_SEND_QUEUE_PERIOD);
    ble_control_handle_send_queue();
    return (events ^ PERIOD_HANDLE_SEND_QUEUE_EVENT);
  }
  
  if(events & PERIOD_HANDLE_RECV_QUEUE_EVENT)
  {
    osal_start_timerEx(controlBLETaskId,PERIOD_HANDLE_RECV_QUEUE_EVENT,HANDLE_RECV_QUEUE_PERIOD);
    ble_control_handle_recv_queue();
    return (events ^ PERIOD_HANDLE_RECV_QUEUE_EVENT);
  }
  
    if(events & PERIOD_HANDLE_NET_DEVICE_EVENT)
  {
    osal_start_timerEx(controlBLETaskId,PERIOD_HANDLE_NET_DEVICE_EVENT,HANDLE_POLL_NET_PERIOD);
    app_poll_net_status(HANDLE_POLL_NET_PERIOD);
    return (events ^ PERIOD_HANDLE_NET_DEVICE_EVENT);
  }
  
   if(events & PERIOD_HANDLE_REPLY_HEARTBEAT_EVENT)
  {
    osal_start_timerEx(controlBLETaskId,PERIOD_HANDLE_REPLY_HEARTBEAT_EVENT,HANDLE_REPLY_HEARTBEAT_PERIOD);
    reply_sys_heartbeat(ble_msg_id++);
    return (events ^ PERIOD_HANDLE_REPLY_HEARTBEAT_EVENT);
  }
  if(events & PERIOD_HANDLE_UPDATE_TIME_EVENT)
  {
    osal_start_timerEx(controlBLETaskId,PERIOD_HANDLE_UPDATE_TIME_EVENT,HANDLE_UPDATE_TIME_PERIOD);
    request_sys_time(ble_msg_id++);
    return (events ^ PERIOD_HANDLE_UPDATE_TIME_EVENT);
  }
  // Discard unknown events
  return 0;
}


static void app_start_net()
{
 osal_start_timerEx(controlBLETaskId,PERIOD_HANDLE_SEND_QUEUE_EVENT,HANDLE_SEND_QUEUE_PERIOD);
 osal_start_timerEx(controlBLETaskId,PERIOD_HANDLE_RECV_QUEUE_EVENT,HANDLE_RECV_QUEUE_PERIOD);
 osal_start_timerEx(controlBLETaskId,PERIOD_HANDLE_NET_DEVICE_EVENT,HANDLE_POLL_NET_PERIOD);
 osal_start_timerEx(controlBLETaskId,PERIOD_HANDLE_REPLY_HEARTBEAT_EVENT,HANDLE_REPLY_HEARTBEAT_PERIOD);
 osal_start_timerEx(controlBLETaskId,PERIOD_HANDLE_UPDATE_TIME_EVENT,HANDLE_UPDATE_TIME_PERIOD); 
}

/*********************************************************************
 * @fn      lock_in_BLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void controlBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      controlBLECentral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  case UART_DEBUG_CMD:
      controlBLECentral_handle_uart_debug_cmd( ((uart_debug_cmd_t*)pMsg)->cmd);
      break;
    case GATT_MSG_EVENT:
      controlBLECentralProcessGATTMsg( (gattMsgEvent_t *)pMsg );
      break;
  default:
    break;
  }
}



static void owner_addr_init(uint8*addr_src)
{
  uint8 *ptr_mac=(uint8*)0x7813;
  
  for(uint8 i=0;i<6;i++)
  {
   addr_src[i]=*ptr_mac--;//◊˜Œ™netµƒmac
  }
  osal_memcpy(dhcp_src_mac,addr_src,6);//◊˜Œ™dhcpµƒmac
  osal_memcpy(owner_id,addr_src,6);//◊˜Œ™¿∂—¿ ’ºØ∆˜µƒmac
  app_write_string("\r\nnet macµÿ÷∑≥ı ºªØÕÍ≥…!mac:");
  uint8_array_to_string(addr_src,6);
  app_write_string("\r\nDHCP macµÿ÷∑∏¥÷∆≥ı ºªØÕÍ≥…!dhcp mac:");
  uint8_array_to_string(dhcp_src_mac,6);
  app_write_string("\r\n¿∂—¿ ’ºØ∆˜ macµÿ÷∑∏¥÷∆≥ı ºªØÕÍ≥…!ble mac:");
  uint8_array_to_string(owner_id,6);
}


static void controlBLECentral_handle_uart_debug_cmd(uint8 dbg_cmd)
{
#if UART_DEBUG > 0  
  switch(dbg_cmd)
  {
  case UART_DEBUG_CMD_START_DISCOVER :
     wkxboot_start_discover();
     break;
  case  UART_DEBUG_CMD_STOP_DISCOVER:
    wkxboot_stop_discover();
    break;
  case UART_DEBUG_CMD_CONNECT:
    wkxboot_connect();
    break;
  case UART_DEBUG_CMD_DISCONNECT:
    wkxboot_disconnect();
    break;
  default:
    app_write_string("\r\ninvalid cmd!");
    break;
  }
#else
  (void)dbg_cmd;
#endif 
  
}


/*********************************************************************
 * @fn      lock_in_BLECentral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */

static void wkxboot_read_lock_in_info_in_peer()
{
  app_write_string("\r\nø™ º∂¡»°lock_in–≈œ¢!");
  if ( simpleBLEState == BLE_STATE_CONNECTED &&
              BLE_lock_in_CharHdl != 0 &&
              simpleBLEProcedureInProgress == FALSE )
    {
      uint8 status;
      
      // Do a read or write as long as no other read or write is in progress
      if ( !simpleBLEProcedureInProgress )
      {
        attReadReq_t req;
        
        req.handle = BLE_lock_in_CharHdl;
        status = GATT_ReadCharValue( simpleBLEConnHandle, &req, controlBLETaskId );
        app_write_string("\r\nø™ º∂¡÷µ!status:");
        app_write_string(uint8_to_string(status));
      
      if ( status == SUCCESS )
      {
        cur_read_char_state=BLE_READ_STAE_LOCK_IN;
        simpleBLEProcedureInProgress = TRUE;
        app_write_string("\r\n∂¡÷µΩ¯≥Ã∑µªÿ’˝≥£!");
      }
      else
      {
        cur_read_char_state=BLE_READ_STAE_IDLE;
        app_write_string("\r\n∂¡÷µΩ¯≥Ã∑µªÿ¥ÌŒÛ!◊º±∏∂œø™¡¨Ω”...");
        wkxboot_disconnect();
      }
    }    
  } 
  else
  {
    app_write_string("\r\n¡¨Ω” «∂œø™µƒªÚ’ﬂ¡¨Ω”æ‰±˙Œ™0Œﬁ∑®∂¡»°!");
  }
}

static void wkxboot_read_car_in_info_in_peer()
{
  app_write_string("\r\nø™ º∂¡»°car_in–≈œ¢!");
  if ( simpleBLEState == BLE_STATE_CONNECTED &&
              BLE_car_in_CharHdl != 0 &&
              simpleBLEProcedureInProgress == FALSE )
    {
      uint8 status;
      
      // Do a read or write as long as no other read or write is in progress
      if ( !simpleBLEProcedureInProgress )
      {
        attReadReq_t req;
        
        req.handle = BLE_car_in_CharHdl;
        status = GATT_ReadCharValue( simpleBLEConnHandle, &req, controlBLETaskId );
        app_write_string("\r\nø™ º∂¡÷µ!status:");
        app_write_string(uint8_to_string(status));
      
      if ( status == SUCCESS )
      {
        cur_read_char_state=BLE_READ_STAE_CAR_IN;
        simpleBLEProcedureInProgress = TRUE;
        app_write_string("\r\n∂¡÷µΩ¯≥Ã∑µªÿ’˝≥£!");
      }
      else
      {
        cur_read_char_state=BLE_READ_STAE_IDLE;
        app_write_string("\r\n∂¡÷µΩ¯≥Ã∑µªÿ¥ÌŒÛ!◊º±∏∂œø™¡¨Ω”...");
        wkxboot_disconnect();
      }
    }    
  } 
 else
  {
    app_write_string("\r\n¡¨Ω” «∂œø™µƒªÚ’ﬂ¡¨Ω”æ‰±˙Œ™0Œﬁ∑®∂¡»°!");
  }
}
static void wkxboot_write_car_in_info_in_peer()
{
  app_write_string("\r\nø™ º–¥»Îcar in–≈œ¢!");
  if ( simpleBLEState == BLE_STATE_CONNECTED &&
              BLE_car_in_CharHdl != 0 &&
              simpleBLEProcedureInProgress == FALSE )
    {
      uint8 status;
      
      // Do a read or write as long as no other read or write is in progress
      if ( !simpleBLEProcedureInProgress )
      {
       attWriteReq_t req;       
       req.handle = BLE_car_in_CharHdl;
       req.len = sizeof(ble_device_t);;
       *((ble_device_t*)req.value) = car_in_info;
       req.sig = 0;
       req.cmd = 0;
       status = GATT_WriteCharValue( simpleBLEConnHandle, &req, controlBLETaskId );    
       app_write_string("\r\nø™ º–¥time stamp÷µ!status:");
       app_write_string(uint8_to_string(status));
      
      if ( status == SUCCESS )
      {
        cur_write_char_state=BLE_WRITE_STATE_CAR_IN;
        simpleBLEProcedureInProgress = TRUE;
        app_write_string("\r\ncar in –¥÷µΩ¯≥Ã∑µªÿ’˝≥£!");
      }
      else
      {
        cur_read_char_state=BLE_WRITE_STATE_IDLE;
        app_write_string("\r\ncar in –¥÷µΩ¯≥Ã∑µªÿ¥ÌŒÛ!◊º±∏∂œø™¡¨Ω”...");
        wkxboot_disconnect();
      }
    }
   else
   {
     app_write_string("\r\ncar in simpleBLEProcedureInProgress is true!");
   }
  }
    else
  {
   app_write_string("\r\ncar in¡¨Ω” «∂œø™µƒªÚ’ﬂ¡¨Ω”æ‰±˙Œ™0Œﬁ∑®–¥»Î!");
  }
  
   
}



static void wkxboot_read_time_stamp_info_in_peer()
{
  app_write_string("\r\nø™ º∂¡»°time_stamp–≈œ¢!");
  if ( simpleBLEState == BLE_STATE_CONNECTED &&
              BLE_time_stamp_CharHdl != 0 &&
              simpleBLEProcedureInProgress == FALSE )
    {
      uint8 status;
      
      // Do a read or write as long as no other read or write is in progress
      if ( !simpleBLEProcedureInProgress )
      {
        attReadReq_t req;      
        req.handle = BLE_time_stamp_CharHdl;
        status = GATT_ReadCharValue( simpleBLEConnHandle, &req, controlBLETaskId );
        app_write_string("\r\nø™ º∂¡÷µ!status:");
        app_write_string(uint8_to_string(status));
      
      if ( status == SUCCESS )
      {
        cur_read_char_state=BLE_READ_STAE_TIME_STAMP;
        simpleBLEProcedureInProgress = TRUE;
        app_write_string("\r\n∂¡÷µΩ¯≥Ã∑µªÿ’˝≥£!");
      }
      else
      {
        cur_read_char_state=BLE_READ_STAE_TIME_STAMP;
        cur_read_char_state=BLE_READ_STAE_IDLE;
        app_write_string("\r\n∂¡÷µΩ¯≥Ã∑µªÿ¥ÌŒÛ!◊º±∏∂œø™¡¨Ω”...");
        wkxboot_disconnect();
      }
    }    
  }
  else
  {
    app_write_string("\r\n¡¨Ω” «∂œø™µƒªÚ’ﬂ¡¨Ω”æ‰±˙Œ™0Œﬁ∑®∂¡»°!");
  }
}

static void wkxboot_write_time_stamp_info_in_peer()
{
  app_write_string("\r\nø™ º–¥»Îtime_stamp–≈œ¢!");
  if ( simpleBLEState == BLE_STATE_CONNECTED &&
              BLE_time_stamp_CharHdl != 0 &&
              simpleBLEProcedureInProgress == FALSE )
    {
      uint8 status;
      
      // Do a read or write as long as no other read or write is in progress
      if ( !simpleBLEProcedureInProgress )
      {
       attWriteReq_t req;       
       req.handle = BLE_time_stamp_CharHdl;
       req.len = sizeof(time_stamp_t);;
       *((time_stamp_t*)req.value) = time_stamp_info;
       req.sig = 0;
       req.cmd = 0;
       status = GATT_WriteCharValue( simpleBLEConnHandle, &req, controlBLETaskId );    
       app_write_string("\r\nø™ º–¥time stamp÷µ!status:");
       app_write_string(uint8_to_string(status));
      
      if ( status == SUCCESS )
      {
        cur_write_char_state=BLE_WRITE_STATE_TIME_STAMP;
        simpleBLEProcedureInProgress = TRUE;
        app_write_string("\r\ntime stamp –¥÷µΩ¯≥Ã∑µªÿ’˝≥£!");
      }
      else
      {
        cur_write_char_state=BLE_WRITE_STATE_IDLE;
        app_write_string("\r\ntime stamp –¥÷µΩ¯≥Ã∑µªÿ¥ÌŒÛ!◊º±∏∂œø™¡¨Ω”...");
        wkxboot_disconnect();
      }
    }
   else
   {
     app_write_string("\r\ntime stamp simpleBLEProcedureInProgress is true!");
   }
  }
    else
  {
   app_write_string("\r\ntime stamp ¡¨Ω” «∂œø™µƒªÚ’ﬂ¡¨Ω”æ‰±˙Œ™0Œﬁ∑®–¥»Î!");
  }
  
   
}


static void controlBLECentral_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter

 if ( keys & KEY_FUN1 )
  {
    app_write_string("\r\nœµÕ≥ ’µΩKEY_FUN1");
    if ( simpleBLEState == BLE_STATE_CONNECTED &&
              BLE_lock_in_CharHdl != 0 &&
              simpleBLEProcedureInProgress == FALSE )
    {
      uint8 status;
      
      // Do a read or write as long as no other read or write is in progress
      if ( simpleBLEDoWrite )
      {
        // Do a write
        attWriteReq_t req;
        
        req.handle = BLE_lock_in_CharHdl;
        req.len = sizeof(ble_device_t);;
        *((ble_device_t*)req.value) = lock_in_info;
        req.sig = 0;
        req.cmd = 0;
        status = GATT_WriteCharValue( simpleBLEConnHandle, &req, controlBLETaskId );    
        app_write_string("\r\nø™ º–¥÷µ!status:");
        app_write_string(uint8_to_string(status));
      }
      else
      {
        // Do a read
        attReadReq_t req;
        
        req.handle = BLE_lock_in_CharHdl;
        status = GATT_ReadCharValue( simpleBLEConnHandle, &req, controlBLETaskId );
        app_write_string("\r\nø™ º∂¡÷µ!status:");
        app_write_string(uint8_to_string(status));
      }
      
      if ( status == SUCCESS )
      {
        simpleBLEProcedureInProgress = TRUE;
        simpleBLEDoWrite = !simpleBLEDoWrite;
      }
    }    
  } 
  if ( keys & KEY_FUN2 )
  {
    app_write_string("\r\nœµÕ≥ ’µΩKEY_FUN2");
  }
}

/*********************************************************************
 * @fn      lock_in_BLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void controlBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  if ( simpleBLEState != BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    app_write_string("\r\n∂¡–¥¥ÌŒÛ!√ª”–¡¨Ω”!");
    return;
  }
  
  app_write_string("\r\nΩ¯»Î∂¡–¥ªÿ”¶...method=");
  app_write_string(uint8_to_string(pMsg->method));
 
  if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP )
    {
      uint8 status = pMsg->msg.errorRsp.errCode;
      app_write_string("\r\nATT_READ_RSP Read Error err_code:");
      app_write_string(uint8_to_string(status));
      simpleBLEProcedureInProgress = FALSE; 
      wkxboot_disconnect();
    }
    else 
    {
      // After a successful read, display the read value
      if(cur_read_char_state==BLE_READ_STAE_LOCK_IN)
      {
      lock_in_info = *((ble_device_t*)pMsg->msg.readRsp.value);
      app_write_string("\r\n∂¡µΩlock inµƒ÷µ «:\r\n");
      uint8_array_to_string((uint8*)&lock_in_info,sizeof(ble_device_t));
      simpleBLEProcedureInProgress = FALSE;
      
      app_write_string("\r\n∂¡÷µΩ¯≥Ãstatus:");
      app_write_string(uint8_to_string(pMsg->hdr.status)); 
      wkxboot_read_car_in_info_in_peer();
      }
      else if(cur_read_char_state==BLE_READ_STAE_CAR_IN)
      {
      car_in_info = *((ble_device_t*)pMsg->msg.readRsp.value);
      app_write_string("\r\n∂¡µΩcar inµƒ÷µ «:\r\n");
      uint8_array_to_string((uint8*)&car_in_info,sizeof(ble_device_t));
      simpleBLEProcedureInProgress = FALSE;
      
      app_write_string("\r\ncar in ∂¡÷µΩ¯≥Ãstatus:");
      app_write_string(uint8_to_string(pMsg->hdr.status)); 
      wkxboot_read_time_stamp_info_in_peer();
      }
      else if(cur_read_char_state==BLE_READ_STAE_TIME_STAMP)
      {
      time_stamp_info = *((time_stamp_t*)pMsg->msg.readRsp.value);
      app_write_string("\r\n∂¡µΩtime stampµƒ÷µ «:\r\n");
      uint8_array_to_string((uint8*)&time_stamp_info,sizeof(time_stamp_t));
      simpleBLEProcedureInProgress = FALSE;
      cur_read_char_state=BLE_READ_STAE_IDLE;//
      app_write_string("\r\ntime stamp ∂¡÷µΩ¯≥Ãstatus:");
      app_write_string(uint8_to_string(pMsg->hdr.status)); 
      
      ble_process_parking_lock_info();
      
      }   
    }  
    
  }
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    
    if ( pMsg->method == ATT_ERROR_RSP)//== ATT_ERROR_RSP )
    {
      uint8 status = pMsg->msg.errorRsp.errCode;
      app_write_string("\r\nATT_WRITE_RSP write Error err_code:");
      app_write_string(uint8_to_string(status)); 
      simpleBLEProcedureInProgress = FALSE; 
      
      wkxboot_disconnect();
       
    }
    else
    {   
      if(cur_write_char_state==BLE_WRITE_STATE_TIME_STAMP)
      {
         simpleBLEProcedureInProgress = FALSE;
      // After a succesful write, display the value that was written and increment value
         app_write_string("\r\ntime stamp–¥µƒ÷µ «:\r\n");
         uint8_array_to_string((uint8*)&time_stamp_info,sizeof(time_stamp_t));           
         wkxboot_write_car_in_info_in_peer(); //–¥»Î∆˚≥µ»®œﬁ
      }
      else if(cur_write_char_state==BLE_WRITE_STATE_CAR_IN)
      {
         simpleBLEProcedureInProgress = FALSE;
         cur_write_char_state=BLE_WRITE_STATE_IDLE;
 
         app_write_string("\r\ncar in–¥µƒ÷µ «:\r\n");
         uint8_array_to_string((uint8*)&car_in_info,sizeof(ble_device_t));   
         
         wkxboot_disconnect();//÷˜∂Ø∂œø™¡¨Ω”
      }
    }   
      
  }
  else if ( simpleBLEDiscState != BLE_DISC_STATE_IDLE )
  {
    simpleBLEGATTDiscoveryEvent( pMsg );
  }
}



static void own_addr_minus_1_to_peer_addr(uint8 *p_own_addr,uint8* p_peer_addr)
{

 for(uint8 i=0;i<6;i++)
 {
   p_peer_addr[i]=p_own_addr[i];
 }
 for(uint8 i=0;i<6;i++)
 {
 if(p_peer_addr[i]>0)
 {
    p_peer_addr[i]-=1;
    for(uint8 j=0;j<i;j++)
    {
      p_peer_addr[j]=0xff;
    }
    break;
 }
}
app_write_string("\r\n≥µ‘ÿƒø±Í¿∂—¿µÿ÷∑:");
app_write_string(bdAddr2Str(p_peer_addr));
}

static bool ble_addr_is_match(uint8 *addr1,uint8 *addr2)
{
  for(uint8 i=B_ADDR_LEN-1;i>=DAFAULT_ADDR_MATCH_LEN;i--) 
  {
    if(addr1[i]!=*addr2++)
    {
      app_write_string("\r\n¿∂—¿µÿ÷∑≤ª∆•≈‰!");
      return FALSE;  
    }
  }
  app_write_string("\r\n¿∂—¿µÿ÷∑∆•≈‰!");
  return TRUE; 
}

static bool ble_full_addr_is_match(uint8 *addr1,uint8 *addr2)
{
 for(uint8 i=0;i<6;i++)
 {
  if(addr1[i]!=addr2[i])
  {
    app_write_string("\r\n¿∂—¿ÕÍ’˚µƒµÿ÷∑≤ª∆•≈‰!");
    return FALSE;   
  }
 }
 app_write_string("\r\n¿∂—¿ÕÍ’˚µƒµÿ÷∑∆•≈‰!");
 return TRUE; 
}
static bool ble_find_match_peer_addr_from_list(gapDevRec_t* addr_list,uint8 list_cnt)
{
  static uint8 cur_cnt=0;
  gapDevRec_t* ptr_addr;
  ptr_addr=addr_list;
  for(uint8 i=cur_cnt;i<list_cnt;i++)
  {
    if(ble_addr_is_match(ptr_addr->addr,ble_match_addr))
    {
      cur_cnt=i+1;
      osal_memcpy(ble_tar_addr,ptr_addr->addr,B_ADDR_LEN);
      app_write_string("\r\n’“µΩƒø±Íµÿ÷∑≤¢∏¥÷∆!¡–±Ì÷µ:");
      app_write_string(uint8_to_string(cur_cnt));
      return TRUE;
    }
    ptr_addr++;
  }
    
    cur_cnt=0;
    app_write_string("\r\n√ª”–’“µΩƒø±Íµÿ÷∑!¡–±Ì÷µπÈ0");
  return FALSE;
}


static void wkxboot_start_discover()
{
  if(!simpleBLEScanning)
  { 
       simpleBLEScanning = TRUE;
       simpleBLEScanRes = 0;
        
        GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                       DEFAULT_DISCOVERY_WHITE_LIST );  
     app_write_string("\r\nø™ º…®√Ë.....");
  }
 else
 {
  app_write_string("\r\n“—æ≠‘⁄…®√Ë-----");
 }
}

static void wkxboot_stop_discover()
{
   if(simpleBLEScanning)
  { 
       simpleBLEScanning = FALSE;
       //simpleBLEScanRes = 0;
        
       GAPCentralRole_CancelDiscovery(); 
       app_write_string("\r\n…®√Ëπÿ±’");
  }
 else
 {
  app_write_string("\r\n…®√Ë“—æ≠πÿ±’£°");
 }
}

static void wkxboot_connect()
{
    bStatus_t rt_status;
    uint8 addrType;
    uint8 *peerAddr;
    // Connect or disconnect
    if ( simpleBLEState == BLE_STATE_IDLE )
    {
        peerAddr=ble_tar_addr;
        addrType=ADDRTYPE_PUBLIC;
        simpleBLEState = BLE_STATE_CONNECTING;
        rt_status=GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                                DEFAULT_LINK_WHITE_LIST,
                                                addrType, peerAddr );
       if(SUCCESS==rt_status)
       {
         app_write_string("\r\n’˝‘⁄¡¨Ω”...");
         app_write_string(bdAddr2Str( peerAddr ));
       }
       else
       {
         simpleBLEState =BLE_STATE_IDLE;
         LL_CreateConnCancel();
         app_write_string("\r\n÷’÷π¡¨Ω”!¡¨Ω”π˝≥Ã≥ˆ¥Ì!status:");
         app_write_string(uint8_to_string(rt_status));
         app_write_string("\r\n1√Î÷”∫Û÷ÿ–¬ø™ º…®√Ë!");
         osal_start_timerEx(controlBLETaskId,START_SCAN_EVT,DEFAULT_START_TO_SCAN_DELAY);//1s
       }
    }      
    else
    {
       app_write_string("\r\n’˝‘⁄…®√ËªÚ’ﬂ¡¨Ω”◊¥Ã¨£°Œﬁ∑®¡¨Ω”£°");
    }
}

static void wkxboot_disconnect()
{
 if ( simpleBLEState == BLE_STATE_CONNECTING ||
      simpleBLEState == BLE_STATE_CONNECTED )
    {
      // disconnect
      simpleBLEState = BLE_STATE_DISCONNECTING;

      GAPCentralRole_TerminateLink( simpleBLEConnHandle );
      
      app_write_string( "\r\n’˝‘⁄πÿ±’¡¨Ω”..." ); 
    }
 else
 {
   if(simpleBLEState == BLE_STATE_IDLE)
   app_write_string( "\r\n¡¨Ω”“—æ≠ «∂œø™◊¥Ã¨£°" ); 
   if(simpleBLEState == BLE_STATE_DISCONNECTING)
   app_write_string( "\r\n“—æ≠‘⁄πÿ±’¡¨Ω”÷–£°" ); 
 }
  
}


/*********************************************************************
 * @fn      lock_in_BLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void controlBLECentralRssiCB( uint16 connHandle, int8 rssi )
{
  //  app_write_string( "RSSI -dB:", (uint8) (-rssi), 10, HAL_LCD_LINE_1 );
  app_write_string("RSSI -dB:");
  app_write_string(uint8_to_string((uint8)(-rssi)));
}


/*********************************************************************
 * @fn      lock_in_BLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void controlBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
      /***wkxboot***/
        app_write_string("\r\nble4.0…Ë±∏control≥ı ºªØÕÍ≥…!");
        app_write_string("\r\n±æª˙…Ë±∏µÿ÷∑:");
        app_write_string(bdAddr2Str(pEvent->initDone.devAddr));
        own_addr_minus_1_to_peer_addr(pEvent->initDone.devAddr,ble_tar_addr);
       // owner_info_init(pEvent->initDone.devAddr);
        app_write_string("\r\n1√Î÷”∫Ûø™ ºµ⁄“ª¥Œ…®√Ë!");
        osal_start_timerEx(controlBLETaskId,START_SCAN_EVT,DEFAULT_START_TO_SCAN_DELAY);//1s
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        app_write_string("\r\n∑¢œ÷…Ë±∏:");
        app_write_string(bdAddr2Str( pEvent->deviceInfo.addr ));
        
        // if filtering device discovery results based on service UUID
        
        if(ble_addr_is_match(pEvent->deviceInfo.addr,ble_tar_addr))
        {
          //simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
         
          //app_write_string("\r\nÃÌº”µΩ…Ë±∏¡–±Ì!");
          
          wkxboot_stop_discover();//πÿ±’…®√Ë
          wkxboot_connect();//÷±Ω”¡¨Ω”
          
        }
        else
        {
          app_write_string("\r\nÃ¯π˝..ºÃ–¯…®√Ë...");
        }
        /*
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )
        {
          if ( simpleBLEFindSvcUuid( SIMPLEPROFILE_SERV_UUID,
                                     pEvent->deviceInfo.pEvtData,
                                     pEvent->deviceInfo.dataLen ) )
          {
            simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
          }
        }
        */
        /****wkxboot***/
       // simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        simpleBLEScanning = FALSE;
        
       /****wkxboot***/
        /*
        simpleBLEScanRes = pEvent->discCmpl.numDevs;

        if(simpleBLEScanRes==0)
        {
        app_write_string("\r\n…®√Ë≥¨ ± ¬º˛!");  
        app_write_string("\r\n0.1√Î÷”∫Û÷ÿ–¬ø™ º…®√Ë!");
        osal_start_timerEx(controlBLETaskId,START_SCAN_EVT,DEFAULT_START_TO_SCAN_DELAY);//1s
        }
        else if (simpleBLEScanRes <= DEFAULT_MAX_SCAN_RES)
        {
          app_write_string("\r\n…®√ËµΩ…Ë±∏ ˝¡ø:");
          app_write_string(uint8_to_string(simpleBLEScanRes));
          osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                     (sizeof( gapDevRec_t ) * simpleBLEScanRes) );
         if(ble_find_match_peer_addr_from_list(simpleBLEDevList,simpleBLEScanRes))
         {
          wkxboot_connect();
         }
         else
         {
          app_write_string("\r\n…®√ËµΩµƒ¿∂—¿≤ª∆•≈‰!");  
          app_write_string("\r\n0.1√Î÷”∫Û÷ÿ–¬ø™ º…®√Ë!");
          osal_start_timerEx(controlBLETaskId,START_SCAN_EVT,DEFAULT_START_TO_SCAN_DELAY);//1s
         }
        }
      }
        */
      app_write_string("\r\nÕÍ≥…“ª∏ˆ…®√Ë÷‹∆⁄ ¬º˛!");  
      osal_start_timerEx(controlBLETaskId,START_SCAN_EVT,DEFAULT_START_TO_SCAN_DELAY);//0.1s
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if ( pEvent->gap.hdr.status == SUCCESS )
        {          
          simpleBLEState = BLE_STATE_CONNECTED;
          simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;
          simpleBLEProcedureInProgress = TRUE;    

          // If service discovery not performed initiate service discovery
          if ( BLE_lock_in_CharHdl == 0 ||BLE_car_in_CharHdl==0||BLE_time_stamp_CharHdl==0)
          {
            osal_start_timerEx( controlBLETaskId, START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
            
          }                     
          app_write_string("\r\n¡¨Ω”≥…π¶!¡¨Ω”µƒ¿∂—¿µÿ÷∑:");
          app_write_string(bdAddr2Str(pEvent->linkCmpl.devAddr));
          app_write_string("\r\n¡¨Ω”µƒconnhandle:");
          app_write_string(uint16_to_string(simpleBLEConnHandle));
          if(simpleBLERssi)
          {
            app_write_string("\r\n µ ±œ‘ ærssi÷µ.");
            GAPCentralRole_StartRssi(simpleBLEConnHandle,DEFAULT_RSSI_PERIOD);
          }
          app_write_string("\r\nø™ º∑¢œ÷services.....");
     
        }
        else
        {
          simpleBLEState = BLE_STATE_IDLE;
          simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
          simpleBLERssi = FALSE;
          simpleBLEDiscState = BLE_DISC_STATE_IDLE;
 
          app_write_string("\r\n¡¨Ω” ß∞‹,status:");
          app_write_string(uint8_to_string(pEvent->gap.hdr.status));
          app_write_string("\r\n1√Î÷”∫Û÷ÿ–¬ø™ º…®√Ë!");
        
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        simpleBLEState = BLE_STATE_IDLE;
        simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
        simpleBLERssi = FALSE;
        simpleBLEDiscState = BLE_DISC_STATE_IDLE;
        BLE_lock_in_CharHdl =0;
        BLE_car_in_CharHdl=0;
        BLE_time_stamp_CharHdl=0;
        cur_read_char_state=BLE_READ_STAE_IDLE;
        cur_write_char_state=BLE_WRITE_STATE_IDLE;
        simpleBLEProcedureInProgress = FALSE;
        
        app_write_string("\r\n¡¨Ω”∂œø™!‘≠“Ú:");
        app_write_string(uint8_to_string(pEvent->linkTerminate.reason));          
        
        if(ble_find_match_peer_addr_from_list(simpleBLEDevList,simpleBLEScanRes))
        {
          wkxboot_connect();
        }
        else
        {
        app_write_string("\r\n1√Î÷”∫Û÷ÿ–¬ø™ º…®√Ë!");
        osal_start_timerEx(controlBLETaskId,START_SCAN_EVT,DEFAULT_START_TO_SCAN_DELAY);//1s
        }
          
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        app_write_string("\r\nlink param update!");
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void controlBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    app_write_string( "Pairing started");
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      app_write_string( "Pairing success");
    }
    else
    {
      app_write_string( "Pairing fail");
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      app_write_string( "Bonding success");
    }
  }
}

/*********************************************************************
 * @fn      lock_in_BLECentralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void controlBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
#if (HAL_LCD == TRUE)

  uint32  passcode;
  uint8   str[7];

  // Create random passcode
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    app_write_string( "Passcode:" );
    app_write_string( (char *) _ltoa(passcode, str, 10));
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
#endif
}

/*********************************************************************
 * @fn      lock_in_BLECentralStartDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void controlBLECentralStartDiscovery( void )
{
  uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(PARAM_BLE_CAR_IN_PROFILE_SERV_UUID),
                                   HI_UINT16(PARAM_BLE_CAR_IN_PROFILE_SERV_UUID) };
  
  // Initialize cached handles
  simpleBLESvcStartHdl = simpleBLESvcEndHdl = 0;
  BLE_lock_in_CharHdl =BLE_car_in_CharHdl=BLE_time_stamp_CharHdl=0;
  simpleBLEDiscState = BLE_DISC_STATE_SVC;
  
  // Discovery simple BLE service
  GATT_DiscPrimaryServiceByUUID( simpleBLEConnHandle,
                                 uuid,
                                 ATT_BT_UUID_SIZE,
                                 controlBLETaskId );
}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{ 
  if ( simpleBLEDiscState == BLE_DISC_STATE_SVC )
  {
    // Service found, store handles
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
         pMsg->msg.findByTypeValueRsp.numInfo > 0 )
    {
      app_write_string("\r\nservice numInfo:");
      app_write_string(uint8_to_string(pMsg->msg.findByTypeValueRsp.numInfo));
      
      simpleBLESvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
      simpleBLESvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      app_write_string("\r\n∑¢œ÷µƒserviceø™ ºhandle:");
      app_write_string(uint16_to_string(simpleBLESvcStartHdl));
      app_write_string("\r\n∑¢œ÷µƒserviceΩ· ¯handle:");
      app_write_string(uint16_to_string(simpleBLESvcEndHdl));
      
    }   
    // If procedure complete
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
         pMsg->hdr.status == bleProcedureComplete )      
    {
        app_write_string("\r\nserver procedure complete:");
      if ( simpleBLESvcStartHdl != 0 )
      {
        // Discover characteristic
        start_to_find_char_handle(BLE_DISC_STATE_LOCK_IN_CHAR);
      }
      /****wkxboot***/
      else
      {
        simpleBLEDiscState =BLE_DISC_STATE_IDLE;
        simpleBLEProcedureInProgress = FALSE; 
        app_write_string("\r\nservice start handleŒ™0!◊º±∏∂œø™¡¨Ω”...");
        wkxboot_disconnect();
      }
    }
    if ( pMsg->method == ATT_ERROR_RSP )
    {
        simpleBLEDiscState =BLE_DISC_STATE_IDLE;
        simpleBLEProcedureInProgress = FALSE; 
        app_write_string("\r\nservice handle∑¢œ÷≥ˆ¥Ì!◊º±∏∂œø™¡¨Ω”...");
        wkxboot_disconnect();
    }
  }
  else if ( simpleBLEDiscState == BLE_DISC_STATE_LOCK_IN_CHAR )
  {
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP)
    {  
      if( pMsg->msg.readByTypeRsp.numPairs > 0 )
      {
      BLE_lock_in_CharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],
                                          pMsg->msg.readByTypeRsp.dataList[1] );
      
      app_write_string("\r\nlcok in numPairs:");
      app_write_string(uint8_to_string( pMsg->msg.readByTypeRsp.numPairs));
      app_write_string("\r\nªÒ»°µƒlock_in_char handle:");
      app_write_string(uint16_to_string(BLE_lock_in_CharHdl));
             
     }
    else
    {
      app_write_string("\r\nlock in numPairs:");
      app_write_string(uint8_to_string( pMsg->msg.readByTypeRsp.numPairs));

      app_write_string("\r\nlock_in_char handle∑¢œ÷ÕÍ≥…!");
      start_to_find_char_handle(BLE_DISC_STATE_CAR_IN_CHAR);         
    }
   }
   if( pMsg->method == ATT_ERROR_RSP)
   {
     app_write_string("\r\nlock in handle error rsp!");
     simpleBLEDiscState = BLE_DISC_STATE_IDLE;
     simpleBLEProcedureInProgress = FALSE; 
     app_write_string("\r\nlock_in_char handle∑¢œ÷≥ˆ¥Ì!◊º±∏∂œø™¡¨Ω”...");
     wkxboot_disconnect();
   }
  }
  else if ( simpleBLEDiscState == BLE_DISC_STATE_CAR_IN_CHAR )
  {
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP)
    {
      if(  pMsg->msg.readByTypeRsp.numPairs > 0 )
     {
      BLE_car_in_CharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],
                                          pMsg->msg.readByTypeRsp.dataList[1] );     
      app_write_string("\r\ncar in numPairs:");
      app_write_string(uint8_to_string( pMsg->msg.readByTypeRsp.numPairs));
      
      app_write_string("\r\nªÒ»°µƒcar_in_char handle:");
      app_write_string(uint16_to_string(BLE_car_in_CharHdl));                    
     }
    else
    {
      app_write_string("\r\ncar in numPairs:");
      app_write_string(uint8_to_string( pMsg->msg.readByTypeRsp.numPairs));

      app_write_string("\r\ncar_in_char handle∑¢œ÷ÕÍ≥…!");
      start_to_find_char_handle(BLE_DISC_STATE_TIME_STAMP_CHAR);          
    }
   }
   if(pMsg->method == ATT_ERROR_RSP)
   {
      app_write_string("\r\ncar in handle error rsp!");
      simpleBLEDiscState = BLE_DISC_STATE_IDLE;
      simpleBLEProcedureInProgress = FALSE; 
      app_write_string("\r\ncar_in_char handle∑¢œ÷≥ˆ¥Ì!◊º±∏∂œø™¡¨Ω”...");
      wkxboot_disconnect();
   }
 
  } 
  else if ( simpleBLEDiscState == BLE_DISC_STATE_TIME_STAMP_CHAR )
  {
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP)
    { 
     if( pMsg->msg.readByTypeRsp.numPairs > 0 )
     {
      BLE_time_stamp_CharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],
                                          pMsg->msg.readByTypeRsp.dataList[1] );
      app_write_string("\r\ntime stamp numPairs:");
      app_write_string(uint8_to_string( pMsg->msg.readByTypeRsp.numPairs));
      
      app_write_string("\r\nªÒ»°µƒtime_stamp_char handle:");
      app_write_string(uint16_to_string(BLE_time_stamp_CharHdl));
    }
    else
    {
      simpleBLEDiscState = BLE_DISC_STATE_IDLE;
      simpleBLEProcedureInProgress = FALSE; 

            /***wkxboot***/  
     app_write_string("\r\nÀ˘”–service∫Õhandles∑¢œ÷ÕÍ±œ!");
     wkxboot_read_lock_in_info_in_peer();
   
     }
    }
    if(pMsg->method == ATT_ERROR_RSP)
    {
      app_write_string("\r\ntime stamp handle error rsp!");
      simpleBLEDiscState = BLE_DISC_STATE_IDLE;
      simpleBLEProcedureInProgress = FALSE; 
      wkxboot_disconnect();
      app_write_string("\r\ntime stamp char handle∑¢œ÷≥ˆ¥Ì!◊º±∏∂œø™¡¨Ω”...");
    }
 
  }    
}



static void start_to_find_char_handle(uint8 handle_state)
{
  attReadByTypeReq_t req;
  if ( simpleBLESvcStartHdl != 0 )
      {
        // Discover characteristic
        simpleBLEDiscState =handle_state;// BLE_DISC_STATE_LOCK_IN_CHAR;
        
        req.startHandle = simpleBLESvcStartHdl;
        req.endHandle = simpleBLESvcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        
        if(handle_state==BLE_DISC_STATE_LOCK_IN_CHAR)
        {
        req.type.uuid[0] = LO_UINT16(PARAM_BLE_LOCK_IN_CHAR_UUID);
        req.type.uuid[1] = HI_UINT16(PARAM_BLE_LOCK_IN_CHAR_UUID);
        app_write_string("\r\nø™ º π”√lock_in_UUIDªÒ»°char handle");
        }
        if(handle_state==BLE_DISC_STATE_CAR_IN_CHAR)
        {
        req.type.uuid[0] = LO_UINT16(PARAM_BLE_CAR_IN_CHAR_UUID);
        req.type.uuid[1] = HI_UINT16(PARAM_BLE_CAR_IN_CHAR_UUID);
        app_write_string("\r\nø™ º π”√car_in_UUIDªÒ»°char handle");
        }
        if(handle_state==BLE_DISC_STATE_TIME_STAMP_CHAR)
        {
        req.type.uuid[0] = LO_UINT16(PARAM_BLE_TIME_STAMP_CHAR_UUID);
        req.type.uuid[1] = HI_UINT16(PARAM_BLE_TIME_STAMP_CHAR_UUID);
        app_write_string("\r\nø™ º π”√time_stamp_UUIDªÒ»°char handle");
        }
        GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, controlBLETaskId );
      }
}

/************************************ net **************************************/
static void dbg_display_sys_time(UTCTimeStruct *ptr_cur_time)
{
//osal_ConvertUTCTime(ptr_cur_time,osal_getClock());//

app_write_string(uint16_to_string(ble_control_time.year));
app_write_string(uint8_to_string(ble_control_time.month));
app_write_string(uint8_to_string(ble_control_time.day));
app_write_string(uint8_to_string(ble_control_time.hour));
app_write_string(uint8_to_string(ble_control_time.minutes));
app_write_string(uint8_to_string(ble_control_time.seconds)); 
}

static void ble_control_set_new_time(UTCTimeStruct *ptr_cur_time,recv_time_msg_t *ptr_msg)
{
if(ptr_msg->msg_type==MSG_TYPE_RECV_TIME && ptr_cur_time)
{
ptr_cur_time->year=2000+ ptr_msg->value[6];
ptr_cur_time->month=ptr_msg->value[7];
ptr_cur_time->day=ptr_msg->value[8];
ptr_cur_time->hour=ptr_msg->value[9];
ptr_cur_time->minutes=ptr_msg->value[10];
ptr_cur_time->seconds=ptr_msg->value[11];

osal_setClock(osal_ConvertUTCSecs(ptr_cur_time));//…Ë÷√–¬ ±º‰since 2000.1.1 seconds
app_write_string("\r\n…Ë÷√¡À–¬ ±º‰:");
dbg_display_sys_time(ptr_cur_time);
}
}

static UTCTimeStruct *ble_control_get_cur_time()
{
  osal_ConvertUTCTime(&ble_control_time,osal_getClock()); 
  
  return &ble_control_time;
}

static void ble_process_parking_lock_info()
{
  UTCTimeStruct *ptr_cur_time;
  ptr_cur_time=ble_control_get_cur_time();
  app_write_string("\r\nµ±«∞µƒ ±º‰:");
  dbg_display_sys_time(ptr_cur_time);
  
  if(time_stamp_info.flag==PARK_FLAG_ENTER)//»Î≥°±Í ∂
  {
   time_stamp_info.enter_time[0]=(uint8)ptr_cur_time->year;
   time_stamp_info.enter_time[1]=(uint8)ptr_cur_time->month;
   time_stamp_info.enter_time[2]=(uint8)ptr_cur_time->day; 
   time_stamp_info.enter_time[3]=(uint8)ptr_cur_time->hour;
   time_stamp_info.enter_time[4]=(uint8)ptr_cur_time->minutes;
   time_stamp_info.enter_time[5]=(uint8)ptr_cur_time->seconds;
   
   osal_mem_cpy(time_stamp_info.exit_time,time_stamp_info.enter_time,6);//“ª÷¬ 
   app_write_string("\r\n÷ª”–»Î≥°±Í ∂!");
  }
 else if(time_stamp_info.flag==PARK_FLAG_EXIT)
  {
   time_stamp_info.exit_time[0]=(uint8)ptr_cur_time->year;
   time_stamp_info.exit_time[1]=(uint8)ptr_cur_time->month;
   time_stamp_info.exit_time[2]=(uint8)ptr_cur_time->day; 
   time_stamp_info.exit_time[3]=(uint8)ptr_cur_time->hour;
   time_stamp_info.exit_time[4]=(uint8)ptr_cur_time->minutes;
   time_stamp_info.exit_time[5]=(uint8)ptr_cur_time->seconds; 
   app_write_string("\r\n”–≥ˆ≥°±Í ∂!");
  if(upload_parking_info(ble_msg_id++)==SUCCESS)//…œ¥´Õ£≥µ–≈œ¢
  {
   time_stamp_info.flag=PARK_FLAG_ENTER;
   osal_mem_cpy(time_stamp_info.enter_time,time_stamp_info.exit_time,6);
   app_write_string("\r\n≥µ¡æ–≈œ¢“—…œ¥´!");
   }
   else
   {
    app_write_string("\r\n≥µ¡æ–≈œ¢“—…œ¥´ ß∞‹!");  
   }
  }
  else
  {
    app_write_string("\r\n≥µ¡æ≥ˆ≥°±Í ∂¥ÌŒÛ!");
  }
   //process_new_authority(&car_in_info);//¥¶¿Ì»®œﬁ 
   app_write_string("\r\n◊º±∏¥¶¿Ì ⁄»®–≈œ¢!");
   process_authority_msg_in_snv(&car_in_info);
   wkxboot_write_time_stamp_info_in_peer();//œÚ≥µ‘ÿ¿∂—¿–¥»Î»Î≥° ±º‰  
}


/*
static void process_new_authority(ble_device_t *ptr_dev)
{
   msg_node_t *ptr_temp_node; 
   ptr_temp_node=find_node_from_queue_by_msg_type(hdr_recv_queue,MSG_TYPE_RECV_AUTHORITY);
   
   while(ptr_temp_node)
   {
    if( ble_full_addr_is_match(((recv_authority_msg_t*)ptr_temp_node->ptr_msg)->value,ptr_dev->ble_id))
    {
      ptr_dev->ble_active=((recv_authority_msg_t*)ptr_temp_node->ptr_msg)->value[6];//»®œﬁ…Ë÷√
      if(ptr_wait_for_delete_node)//Õ¨“ª∏ˆ≥µ≤ªÕ¨»®œﬁ£¨«“µ⁄“ª∏ˆ»®œﬁ√ª”–±ª¥¶¿Ì£¨–Ë“™…æ≥˝µ⁄“ª∏ˆ»®œﬁ
        ptr_wait_for_delete_node->delete_flag=TRUE;
      
        ptr_wait_for_delete_node=ptr_temp_node;//wait for delete 
    }
    ptr_temp_node=find_node_from_queue_by_msg_type(ptr_temp_node->ptr_next,MSG_TYPE_RECV_AUTHORITY);//∂‡∏ˆ≤ªÕ¨»®œﬁ“ª¥ŒΩ‚æˆ
  }
}
*/


static void ble_control_handle_send_queue()
{
  msg_node_t *ptr_next_node;
  msg_node_t *ptr_search_node;
  
  app_write_string("\r\n÷‹∆⁄¬÷—Ø∑¢ÀÕ∂”¡–!");
  if(net_socket_status!= SOCKET_ESTABLISHED)
  {
   app_write_string("\r\nÕ¯¬ÁŒ¥”Î∑˛ŒÒ∆˜Ω®¡¢¡¨Ω”,÷‹∆⁄¬÷—Ø∑¢ÀÕ∂”¡–over!");
   return ;
  }
  ptr_search_node=hdr_send_queue;
  while(ptr_search_node)
  {   
   app_write_string("\r\nµ±«∞∑¢ÀÕ∂”¡–¥Û–°:");
   app_write_string(uint8_to_string(send_queue_cnt));//
   
   ptr_next_node=ptr_search_node->ptr_next;
   if(!ptr_search_node->delete_flag)
   {
   app_write_string("\r\n∑¢ÀÕ∂”¡–’“µΩ“ªÃı¥˝¥¶¿Ìœ˚œ¢!");
   if(ptr_search_node->timeout>0)
   {   
    if(ptr_search_node->timeout>HANDLE_SEND_QUEUE_PERIOD)
    {
    ptr_search_node->timeout-=HANDLE_SEND_QUEUE_PERIOD;
    app_write_string("\r\nŒ¥µΩ∑¢ÀÕ ±º‰≤ª∑¢ÀÕ!");
    }
    else
    {
    ptr_search_node->timeout=0;
    app_write_string("\r\n“—µΩ∑¢ÀÕ ±º‰ø™ º∑¢ÀÕ!");
    }
   }
   if(ptr_search_node->timeout==0)
   {
   ptr_search_node->timeout=ptr_search_node->reload_timeout;//µ»¥˝ªÿ”¶≥¨ ±
   switch(((msg_comm_t*)ptr_search_node->ptr_msg)->msg_type)
   {
    case MSG_TYPE_REPLY_ACK:
    { 
     socket_send(NET_SOCKET,(uint8*)ptr_search_node->ptr_msg,sizeof(replay_ack_msg_t));
     ptr_search_node->delete_flag=TRUE;//–Ë“™…æ≥˝
    }
    break;
    case MSG_TYPE_REPLY_HEARTBEAT:
     {
     socket_send(NET_SOCKET,(uint8*)ptr_search_node->ptr_msg,sizeof(replay_heartbeat_msg_t));  
     }
     break;
    case MSG_TYPE_REQUEST_TIME:
     {
     socket_send(NET_SOCKET,(uint8*)ptr_search_node->ptr_msg,sizeof(request_time_msg_t));  
     }
     break;
    case MSG_TYPE_PARKING_INFO:
     {
     socket_send(NET_SOCKET,(uint8*)ptr_search_node->ptr_msg,sizeof(upload_parking_info_msg_t));  
     }
     break;
   default:
     {
     app_write_string("\r\n¥˝∑¢ÀÕœ˚œ¢¿‡–Õ¥ÌŒÛ!");
     ptr_search_node->delete_flag=TRUE;//–Ë“™…æ≥˝
     }
     break;     
     }
    }
   }
   if(ptr_search_node->delete_flag)//’‚Ãıœ˚œ¢–Ë“™…æ≥˝
   {
     if(delete_node_from_queue(&hdr_send_queue,ptr_search_node)==SUCCESS)
      send_queue_cnt--;
   }
   ptr_search_node=ptr_next_node;   
  }
  app_write_string("\r\n÷‹∆⁄¬÷—Ø∑¢ÀÕ∂”¡–Ω· ¯!");
}

static void ble_control_handle_recv_queue()
{
  msg_node_t *ptr_search_node;
  msg_node_t *ptr_next_node;
  msg_node_t *ptr_temp_node;
  ptr_search_node=hdr_recv_queue;

  app_write_string("\r\n÷‹∆⁄¬÷—ØΩ” ’∂”¡–!");
  while(ptr_search_node)
  {  
   app_write_string("\r\nµ±«∞Ω” ’∂”¡–¥Û–°:");
   app_write_string(uint8_to_string(recv_queue_cnt));//
  
   ptr_next_node=ptr_search_node->ptr_next;
   app_write_string("\r\nΩ” ’∂”¡–’“µΩ“ªÃı¥˝¥¶¿Ìœ˚œ¢!");
   switch(((msg_comm_t*)ptr_search_node->ptr_msg)->msg_type)
   {
    case MSG_TYPE_RECV_TIME:
    { 
     app_write_string("\r\n ’µΩtime!");
     ptr_temp_node=find_node_from_queue_by_msg_id(hdr_send_queue,((msg_comm_t*)ptr_search_node->ptr_msg)->msg_id);
     if(ptr_temp_node)
     {
       ble_control_set_new_time(&ble_control_time,(recv_time_msg_t*)ptr_search_node->ptr_msg);
       ptr_temp_node->delete_flag=TRUE;//‘⁄∑¢ÀÕ∂”¡–÷–±Í ∂–Ë“™…æ≥˝
       app_write_string("\r\n‘⁄∑¢ÀÕ∂”¡–÷–’“µΩrecv time∂‘”¶µƒrequest msg!");
     }
     else
     {
      app_write_string("\r\n¥ÌŒÛ!√ª”–‘⁄∑¢ÀÕ∂”¡–÷–’“µΩrecv time ∂‘”¶µƒrequest msg!"); 
     }
     //reply_sys_ack(((msg_comm_t*)ptr_search_node->ptr_msg)->msg_id);//‘› ±≤ª–Ë“™ªÿ”¶ack
     ptr_search_node->delete_flag=TRUE;//‘⁄Ω” ’∂”¡–±Í ∂–Ë“™…æ≥˝
    }
    break;
    case MSG_TYPE_RECV_AUTHORITY:
     { 
       app_write_string("\r\n ’µΩauthority!");
       if(store_authority_msg_in_snv((recv_authority_msg_t*)ptr_search_node->ptr_msg))//¥¶¿Ì≥…π¶æÕªÿ”¶ack
       {
       reply_sys_ack(((msg_comm_t*)ptr_search_node->ptr_msg)->msg_id);
       app_write_string("\r\nauthority“—æ≠±ª¥¶¿Ì,∏√ÃıΩ” ’œ˚œ¢Ω´…æ≥˝!");
       }
       ptr_search_node->delete_flag=TRUE;
     // ptr_temp_node=find_node_from_queue_by_msg_id(hdr_recv_queue,((msg_comm_t*)ptr_search_node->ptr_msg)->msg_id);  
     //process_new_authority();
       
     }
     break;
    case MSG_TYPE_REPLY_ACK:
     {
     app_write_string("\r\n ’µΩack!");
     ptr_temp_node=find_node_from_queue_by_msg_id(hdr_send_queue,((msg_comm_t*)ptr_search_node->ptr_msg)->msg_id);
     if(ptr_temp_node)
     {
       ptr_temp_node->delete_flag=TRUE;//‘⁄∑¢ÀÕ∂”¡–÷–±Í ∂–Ë“™…æ≥˝
       app_write_string("\r\n‘⁄∑¢ÀÕ∂”¡–÷–’“µΩack∂‘”¶µƒmsg!");
     }
     else
     {
      app_write_string("\r\n¥ÌŒÛ!√ª”–‘⁄∑¢ÀÕ∂”¡–÷–’“µΩack∂‘”¶µƒmsg!"); 
     }
     ptr_search_node->delete_flag=TRUE;
     }
     break;
   default:
     {
     app_write_string("\r\n¥˝¥¶¿ÌµƒΩ” ’œ˚œ¢¿‡–Õ¥ÌŒÛ!");
     ptr_search_node->delete_flag=TRUE;
     }
     break;     
    }
   if(ptr_search_node->delete_flag)
   {
    if(delete_node_from_queue(&hdr_recv_queue,ptr_search_node)==SUCCESS);
     recv_queue_cnt--;
   }
   ptr_search_node=ptr_next_node;
  }
  app_write_string("\r\n÷‹∆⁄¬÷—ØΩ” ’∂”¡–Ω· ¯!");
} 
  

static uint8 request_sys_time(uint32 msg_id)
{
  request_time_msg_t *ptr_msg;
  msg_node_t *ptr_temp_node;
  uint8 status;
  
  ptr_temp_node=find_node_from_queue_by_msg_type(hdr_send_queue,MSG_TYPE_REQUEST_TIME);
  if(ptr_temp_node)
  {
    app_write_string("\r\n‘⁄send_queue÷–’“µΩŒ¥ªÿ”¶µƒrequest time,–Ë“™µ»¥˝ªÿ”¶!");  
    return SUCCESS;
  }
  else
  {
  app_write_string("\r\nŒ¥‘⁄send_queue÷–’“µΩŒ¥ªÿ”¶µƒrequest time,–Ë“™‘Ÿ¥Œ∑¢ÀÕ!");
  ptr_msg=osal_mem_alloc(sizeof(request_time_msg_t));
  if(ptr_msg)
  {
    ptr_msg->msg_hdr=MSG_HEADER;
    ptr_msg->msg_id=msg_id;
    ptr_msg->msg_type=MSG_TYPE_REQUEST_TIME;
    ptr_msg->len=DEFAULT_REQUEST_TIME_LEN;
    osal_memcpy(ptr_msg->value,owner_id,DEFAULT_REQUEST_TIME_LEN);
    ptr_msg->msg_end=MSG_END;
    
    app_write_string("\r\nrequest time ◊º±∏º”»Înet msg∑¢ÀÕ∂”¡–!");
    status=add_to_queue(&hdr_send_queue,(msg_comm_t*)ptr_msg,DEFAULT_NET_TIMEOUT);
    return status;
   }
  }
  return ADDR_NULL_ERROR;
}

static uint8 upload_parking_info(uint32 msg_id)
{
  upload_parking_info_msg_t *ptr_msg;
  uint8 status;
  ptr_msg=(upload_parking_info_msg_t*)osal_mem_alloc(sizeof(upload_parking_info_msg_t));
  if(ptr_msg)
  {
    ptr_msg->msg_hdr=MSG_HEADER;
    ptr_msg->msg_id=msg_id;
    ptr_msg->msg_type=MSG_TYPE_PARKING_INFO;
    ptr_msg->len=DEFAULT_PARKING_INFO_LEN;
    osal_memcpy(ptr_msg->value,owner_id,6);
    osal_memcpy(ptr_msg->value+6,car_in_info.ble_id,6);//car in id
    ptr_msg->value[12]=car_in_info.ble_battery;//car in batt
    osal_memcpy(ptr_msg->value+13,lock_in_info.ble_id,6);//lock in id
    ptr_msg->value[19]=lock_in_info.ble_battery;//lock in batt
    osal_memcpy(ptr_msg->value+20,time_stamp_info.enter_time,6);//enter time stamp
    osal_memcpy(ptr_msg->value+26,time_stamp_info.exit_time,6);//exit time stamp
    ptr_msg->msg_end=MSG_END; 
  
  app_write_string("\r\nparking info◊º±∏º”»Înet msg∑¢ÀÕ∂”¡–!");
  status=add_to_queue(&hdr_send_queue,(msg_comm_t*)ptr_msg,DEFAULT_NET_TIMEOUT);
  if(status==SUCCESS)
  {
   app_write_string("\r\nparking infotº”»Înet msg∑¢ÀÕ∂”¡–≥…π¶!"); 
   return SUCCESS;
  }
  else
  {
   app_write_string("\r\nparking infotº”»Înet msg∑¢ÀÕ∂”¡– ß∞‹!"); 
   return status;
  }
 
  }
   app_write_string("\r\nparking info …Í«Îø’º‰≥ˆ¥Ì!"); 
   return ADDR_NULL_ERROR;
  
}

static uint8 reply_sys_ack(uint32 msg_id)
{
  replay_ack_msg_t *ptr_msg;
  uint8 status;
  ptr_msg=osal_mem_alloc(sizeof(replay_ack_msg_t));
  if(ptr_msg)
  {
    ptr_msg->msg_hdr=MSG_HEADER;
    ptr_msg->msg_id=msg_id;
    ptr_msg->msg_type=MSG_TYPE_REPLY_ACK;
    ptr_msg->len=DEFAULT_REPLY_ACK_LEN;
    ptr_msg->value[0]=ACK_VALUE;//ack
    ptr_msg->msg_end=MSG_END;
  }
  app_write_string("\r\nreplay ack◊º±∏º”»Înet msg∑¢ÀÕ∂”¡–!");
  status=add_to_queue(&hdr_send_queue,(msg_comm_t*)ptr_msg,0);
  if(status==SUCCESS)
  {
  app_write_string("\r\nreplay ackº”»Înet msg∑¢ÀÕ∂”¡–≥…π¶!"); 
  }
  return status;  
}

static uint8 reply_sys_heartbeat(uint32 msg_id)
{
  replay_heartbeat_msg_t *ptr_msg;
  msg_node_t *ptr_temp_node;
  uint8 status;
  
  ptr_temp_node=find_node_from_queue_by_msg_type(hdr_send_queue,MSG_TYPE_REPLY_HEARTBEAT);
  if(ptr_temp_node)
  {
    app_write_string("\r\n‘⁄send_queue÷–’“µΩŒ¥ªÿ”¶µƒheartbeat,–Ë“™µ»¥˝ªÿ”¶!");
    
    return SUCCESS;
  }
  else
  {
  app_write_string("\r\nŒ¥‘⁄send_queue÷–’“µΩŒ¥ªÿ”¶µƒheartbeat,–Ë“™‘Ÿ¥Œ∑¢ÀÕ!");
  ptr_msg=(replay_heartbeat_msg_t*)osal_mem_alloc(sizeof(replay_heartbeat_msg_t));
  if(ptr_msg)
  {
    ptr_msg->msg_hdr=MSG_HEADER;
    ptr_msg->msg_id=msg_id;
    ptr_msg->msg_type=MSG_TYPE_REPLY_HEARTBEAT;
    ptr_msg->len=DEFAULT_REPLY_HEARTBEAT_LEN;
    osal_memcpy(ptr_msg->value,owner_id,6);
    ptr_msg->value[6]=HEARTBEAT_VALUE;
    ptr_msg->msg_end=MSG_END;
  
   app_write_string("\r\nreplay heartbeat◊º±∏º”»Înet msg∑¢ÀÕ∂”¡–!");
   status=add_to_queue(&hdr_send_queue,(msg_comm_t*)ptr_msg,DEFAULT_NET_TIMEOUT);
   return status;
  }
 }
 return ADDR_NULL_ERROR;
}
void ble_control_net_protocol_parse(uint8 *ptr_recv,uint16 recv_len)
{
  uint8 *ptr_buff;
  uint8 *ptr_msg_end;
  uint8 *ptr_buff_end;
  uint8  msg_len;
  msg_comm_t *ptr_msg;
  
  ptr_buff=ptr_recv;
  ptr_buff_end=ptr_recv+recv_len;
  while(1)
  {
  ptr_msg=(msg_comm_t*)ptr_buff;
  msg_len=ptr_msg->len;
  ptr_msg_end=ptr_buff+sizeof(msg_comm_t)+msg_len;
  if(ptr_msg->msg_hdr==MSG_HEADER && ptr_msg->msg_type<6 && ptr_msg_end<=ptr_buff_end && *ptr_msg_end==MSG_END)//Ω‚ŒˆµΩœ˚œ¢
  {
    app_write_string("\r\nΩ‚Œˆ–≠“È∏Ò Ω≥…π¶£°");
   switch(ptr_msg->msg_type)
   {
   case MSG_TYPE_RECV_TIME:
    { 
     
     if(msg_len==DEFAULT_RECV_TIME_LEN && ble_full_addr_is_match(owner_id,((recv_time_msg_t*)ptr_msg)->value))
     {
     recv_time_msg_t *ptr_time_msg;  
     app_write_string("\r\nbuff ’µΩtime!");
     ptr_time_msg=osal_mem_alloc(sizeof(recv_time_msg_t));
     if(ptr_time_msg)
     {
      osal_memcpy(ptr_time_msg,ptr_msg,sizeof(recv_time_msg_t));    
      add_to_queue(&hdr_recv_queue,(msg_comm_t*)ptr_time_msg,0);
     }
     }
     else
     {
     app_write_string("\r\nbuff ’µΩtimeµƒ≥§∂»ªÚ’ﬂ∆•≈‰µÿ÷∑¥ÌŒÛ,∫ˆ¬‘!");  
     }
    }
    break;
    case MSG_TYPE_RECV_AUTHORITY:
     { 
     if(msg_len==DEFAULT_RECV_AUTHORITY_LEN)
     {
     recv_authority_msg_t *ptr_auth_msg;  
     app_write_string("\r\nbuff ’µΩauthority!");
     ptr_auth_msg=osal_mem_alloc(sizeof(recv_authority_msg_t));
     if(ptr_auth_msg)
     {
      osal_memcpy(ptr_auth_msg,ptr_msg,sizeof(recv_authority_msg_t));    
      add_to_queue(&hdr_recv_queue,(msg_comm_t*)ptr_auth_msg,0);
     }
     }
     else
     {
      app_write_string("\r\nbuff ’µΩauthority≥§∂»¥ÌŒÛ,∫ˆ¬‘!");
     }
    }
     break;
    case MSG_TYPE_REPLY_ACK:
     {
     if(msg_len==DEFAULT_REPLY_ACK_LEN)
     {
     replay_ack_msg_t *ptr_ack_msg;  
     app_write_string("\r\nbuff ’µΩnet ack!");
     ptr_ack_msg=osal_mem_alloc(sizeof(replay_ack_msg_t));
     if(ptr_ack_msg)
     {
      osal_memcpy(ptr_ack_msg,ptr_msg,sizeof(replay_ack_msg_t));    
      add_to_queue(&hdr_recv_queue,(msg_comm_t*)ptr_ack_msg,0);   
     }  
     }
     else
     {
     app_write_string("\r\nbuff ’µΩnet ack≥§∂»¥ÌŒÛ,∫ˆ¬‘!");
     }
    }
    break;
   }  
   ptr_buff=ptr_msg_end+1;
  } 
  else
  {
    ptr_buff++;
    if(ptr_buff>ptr_buff_end)
    {
    app_write_string("\r\n–≠“ÈΩ‚ŒˆΩ· ¯!");
    return ;
    }
  }
 }//while
} 
static uint8 add_to_queue(msg_node_t **hdr_queue,msg_comm_t *ptr_msg ,uint16 timeout)
{
  msg_node_t *ptr_search;
  uint8 *ptr_queue_cnt;
  
   if(hdr_queue==&hdr_send_queue)
   {
     ptr_queue_cnt=&send_queue_cnt;
   }
  if(hdr_queue==&hdr_recv_queue)
  {
     ptr_queue_cnt=&recv_queue_cnt;
  }

  if(ptr_msg && hdr_queue && *ptr_queue_cnt<DEFAULT_MAX_MSG_IN_QUEUE)
  {
   msg_node_t *ptr_node=(msg_node_t*)osal_mem_alloc(sizeof(msg_node_t));
   if(!ptr_node)
   {
   osal_mem_free(ptr_msg);
   return  ADDR_NULL_ERROR;
   }
   ptr_node->ptr_msg=ptr_msg;
   ptr_node->ptr_next=NULL;
   ptr_node->reload_timeout=timeout;
   ptr_node->timeout=0;//¡¢º¥∑¢ÀÕ
   ptr_node->delete_flag=FALSE;
   
  if(*hdr_queue==NULL)
  {
   *hdr_queue=ptr_node; 
  }
  else
  {  
    ptr_search=*hdr_queue;
    while(ptr_search->ptr_next)
    {
      ptr_search=ptr_search->ptr_next;
    }    
    ptr_search->ptr_next=ptr_node;
  }
  (*ptr_queue_cnt)++;//÷∏ æ∂”¡– ˝¡ø
  return SUCCESS;
 }
 else
 {
   if(*ptr_queue_cnt>=DEFAULT_MAX_MSG_IN_QUEUE)
   {
     app_write_string("\r\n add to queue failed!queue is full");
   }
   else
   {
     app_write_string("\r\n add to queue failed!addr is null");
   }
  osal_mem_free(ptr_msg);// Õ∑≈œ»«∞…Í«Îµƒœ˚œ¢◊ ‘¥
 }
 return ADDR_NULL_ERROR;
}

static uint8 store_authority_msg_in_snv(recv_authority_msg_t *ptr_msg)
{
 uint8 snv_id;
 uint8 cnt;
 uint8 status;
 uint8 *ptr_buff;
 snv_id=ptr_msg->value[5]; 
 
 app_write_string("\r\n¥Ê¥¢snv msg,‘§±∏id:");
 app_write_string(uint8_to_string(snv_id));
 
 if(snv_id<AUTHORITY_SNV_ID_START)//<80
 {
  app_write_string("\r\n¥Ê¥¢snv msg,id<80");
  snv_id+=AUTHORITY_SNV_ID_START;
 }
 app_write_string("\r\n¥Ê¥¢snv msg, µº id:");
 app_write_string(uint8_to_string(snv_id));
 
 status=osal_snv_read(snv_id,1,&cnt);
 if(status!=SUCCESS || (status==SUCCESS && cnt==0))//√ª”–¥Ê¥¢ ˝æ› ªÚ’ﬂ¥Ê¥¢Œ™0
 {
  app_write_string("\r\n¥Ê¥¢snv msg,√ª”–’“µΩªÚ’ﬂ¥Ê‘⁄µƒ–≈œ¢Œ™0:");

  ptr_buff=osal_mem_alloc(1+sizeof(recv_authority_msg_t)); 
  if(ptr_buff)
  {
  osal_memcpy(ptr_buff+1,ptr_msg,sizeof(recv_authority_msg_t));
  cnt=1;
  ptr_buff[0]=cnt;
  app_write_string("\r\n¥Ê¥¢snv msg,¥Ê¥¢ ⁄»®–≈œ¢Œ™:");
  uint8_array_to_string(ptr_buff,1+sizeof(recv_authority_msg_t));
 
  status= osal_snv_write(snv_id,1+sizeof(recv_authority_msg_t),ptr_buff);
  osal_mem_free(ptr_buff);
  return status;
  }
  app_write_string("\r\n¥Ê¥¢snv msg,∑÷≈‰ø’º‰≥ˆ¥Ì!");
  return ADDR_NULL_ERROR;
 }
 else
 {
   if(cnt<MAX_CNT_IN_PER_ID)
   {
    ptr_buff=osal_mem_alloc(1+sizeof(recv_authority_msg_t)*(cnt+1)); //∂‡…Í«Î“ª∏ˆ
    if(ptr_buff)
    {
    status=osal_snv_read(snv_id,1+sizeof(recv_authority_msg_t)*cnt,ptr_buff);
    if(status==SUCCESS)
    {
    app_write_string("\r\n¥Ê¥¢snv msg,’“µΩ“—æ≠¥Ê‘⁄–≈œ¢Œ™:");
    uint8_array_to_string(ptr_buff,1+sizeof(recv_authority_msg_t)*cnt);
    
    recv_authority_msg_t *ptr_search;
    ptr_search=(recv_authority_msg_t*)(ptr_buff+1);
    for(uint8 i=0;i<cnt;i++)
    {
     if(ble_full_addr_is_match(ptr_search->value,ptr_msg->value))
     {
       app_write_string("\r\n¥Ê¥¢snv msg,’“µΩ∆•≈‰µÿ÷∑");
       if(ptr_search->value[6]!=ptr_msg->value[6])//œ‡Õ¨»®œﬁ≤ª◊˜¥¶¿Ì
       {
        app_write_string("\r\n¥Ê¥¢snv msg,»®œﬁ≤ªÕ¨,“—æ≠¥¶¿Ì!");
        ptr_search->value[6]=ptr_msg->value[6];
        status=osal_snv_write(snv_id,1+sizeof(recv_authority_msg_t)*cnt,ptr_buff);//–¥ªÿsnv  
        osal_mem_free(ptr_buff);//
        return status;
       }
       app_write_string("\r\n¥Ê¥¢snv msg,»®œﬁœ‡Õ¨,≤ª¥¶¿Ì!");
       osal_mem_free(ptr_buff);      
       return SUCCESS;
     }
     ptr_search++;//¥¶¿Ìœ¬“ª∏ˆ
    }
    
    *ptr_search=*ptr_msg;//»Áπ˚√ª”–∆•≈‰µÿ÷∑æÕ÷±Ω”∏≥÷µ
     cnt++;
     ptr_buff[0]=cnt;
     app_write_string("\r\n¥Ê¥¢snv msg,√ª”–’“µΩ∆•≈‰µÿ÷∑,º”»Î¥Ê¥¢!");
     app_write_string("\r\n¥Ê¥¢snv msg,µ±«∞¥Ê¥¢µƒ ⁄»®–≈œ¢Œ™:");
     uint8_array_to_string(ptr_buff,1+sizeof(recv_authority_msg_t)*cnt);
     
     status=osal_snv_write(snv_id,1+sizeof(recv_authority_msg_t)*cnt,ptr_buff);//–¥ªÿsnv cnt“—æ≠∏ƒ±‰    
     osal_mem_free(ptr_buff);
     return status;
   }
    osal_mem_free(ptr_buff);
    app_write_string("\r\n¥Ê¥¢snv msg,∂¡¥Ê‘⁄µƒ ⁄»®–≈œ¢≥ˆ¥Ì!");
    return ADDR_NULL_ERROR;
   }
    app_write_string("\r\n¥Ê¥¢snv msg,∂¡¥Ê‘⁄µƒ ⁄»®–≈œ¢≥ˆ¥Ì!");
    return ADDR_NULL_ERROR;
  }
  app_write_string("\r\n¥Ê¥¢snv msg,¥Ê¥¢ ⁄»®–≈œ¢≥¨œﬁ!");
 }
 
 return ADDR_NULL_ERROR;
}

static uint8 process_authority_msg_in_snv(ble_device_t *ptr_dev)
{
  uint8 status;
  uint8 *ptr_buff;
  uint8 cnt;
  uint8 snv_id;
  
 snv_id=ptr_dev->ble_id[5]; 
 
 app_write_string("\r\n¥¶¿Ìsnv msg ‘§±∏id:");
 app_write_string(uint8_to_string(snv_id));
 
 if(snv_id<AUTHORITY_SNV_ID_START)//>80
 {
   app_write_string("\r\n¥¶¿Ìsnv msg id<80");
   snv_id+=AUTHORITY_SNV_ID_START;
 }  
 app_write_string("\r\n¥¶¿Ìsnv msg  µº id:");
 app_write_string(uint8_to_string(snv_id));
 
 status= osal_snv_read(snv_id,1,&cnt);
 if(status!=SUCCESS || (status==SUCCESS && cnt==0))
 {
   app_write_string("\r\n¥¶¿Ìsnv msg √ª”– ⁄»®µƒ–≈œ¢:");
   return SUCCESS;//√ª”– ⁄»®–≈œ¢
 }
 else
 {
  if(cnt<=MAX_CNT_IN_PER_ID)
  {
  ptr_buff=osal_mem_alloc(1+sizeof(recv_authority_msg_t)*cnt);
  if(ptr_buff)
  {
  status=osal_snv_read(snv_id,1+sizeof(recv_authority_msg_t)*cnt,ptr_buff);
  if(status==SUCCESS)
  {
   app_write_string("\r\n¥¶¿Ìsnv msg”–±£¥Ê ⁄»®µƒ–≈œ¢:");
   uint8_array_to_string(ptr_buff,1+sizeof(recv_authority_msg_t)*cnt);
   
   recv_authority_msg_t *ptr_search;
   ptr_search=(recv_authority_msg_t*)(ptr_buff+1);
   for(uint8 i=0;i<cnt;i++)
   {
     if(ble_full_addr_is_match(ptr_search->value,ptr_dev->ble_id))//’“µΩ ⁄»®–≈œ¢
     {
      app_write_string("\r\n¥¶¿Ìsnv msg’“µΩ∆•≈‰µÿ÷∑!");
      
      ptr_dev->ble_active=ptr_search->value[6];//¥¶¿Ì»®œﬁ     
      for(uint8 j=i+1;j<cnt;j++)
      {
      *ptr_search=*(ptr_search+1);
       ptr_search++;
      }
      cnt--;
      ptr_buff[0]=cnt;
      app_write_string("\r\n¥¶¿Ìsnv msg∫Ûµ±«∞±£¥Êµƒ–≈œ¢:");
      uint8_array_to_string(ptr_buff,sizeof(recv_authority_msg_t)*cnt+1);
      status= osal_snv_write(snv_id,1+sizeof(recv_authority_msg_t)*cnt,ptr_buff);
      
      osal_mem_free(ptr_buff);
      return status;
     }
    ptr_search++;
   }
  osal_mem_free(ptr_buff);
  app_write_string("\r\n¥¶¿Ìsnv msg√ª”–’“µΩ∆•≈‰µÿ÷∑!");
  return SUCCESS;  
  }
  osal_mem_free(ptr_buff);
  app_write_string("\r\n¥¶¿Ìsnv msg∂¡±£¥Êµƒ ⁄»®–≈œ¢≥ˆ¥Ì!");
  return ADDR_NULL_ERROR;
  }
  app_write_string("\r\n¥¶¿Ìsnv msg…Í«Îø’º‰≥ˆ¥Ì!"); 
  return ADDR_NULL_ERROR;
 }
 else//≤Œ ˝¥ÌŒÛæÕπÈ0
 {
  app_write_string("\r\n¥¶¿Ìsnv msg ⁄»®–≈œ¢ ˝¡ø≥¨œﬁπÈ0!");
  cnt=0;
  osal_snv_write(snv_id,1,&cnt); 
  } 
 }
 return PARAM_INVALID_ERROR;
}

static msg_node_t *find_node_from_queue_by_msg_id(msg_node_t *hdr_queue,uint32 msg_id)
{
 msg_node_t *ptr_search_node=NULL;
 ptr_search_node=hdr_queue;
 while(ptr_search_node)
 {
   if(((msg_comm_t*)ptr_search_node->ptr_msg)->msg_id==msg_id)
   return ptr_search_node;
   else
   ptr_search_node=ptr_search_node->ptr_next;  
 }
 return NULL;
}

static msg_node_t *find_node_from_queue_by_msg_type(msg_node_t *hdr_queue,uint8 msg_type)
{
 msg_node_t *ptr_search_node;
 ptr_search_node=hdr_queue;
 while(ptr_search_node)
 {
   if(((msg_comm_t*)ptr_search_node->ptr_msg)->msg_type==msg_type)
   return ptr_search_node;
   else
   ptr_search_node=ptr_search_node->ptr_next;  
 }
 return NULL;
}
/*
static void delete_queue(msg_node_t **hdr_queue)
{
  msg_node_t *ptr_search_node;
  msg_node_t *ptr_next_node;
  ptr_search_node=*hdr_queue;
  while(ptr_search_node)
  {
   ptr_next_node=ptr_search_node->ptr_next;
   
   osal_mem_free(ptr_search_node->ptr_msg);
   osal_mem_free(ptr_search_node);
   ptr_search_node=ptr_next_node;
  }
  *hdr_queue=NULL;
}
*/
static uint8 delete_node_from_queue(msg_node_t **hdr_queue,msg_node_t *ptr_node)
{
  
  msg_node_t *ptr_search_node;
  msg_node_t *ptr_prev_node;
  msg_node_t *ptr_next_node;

  if(*hdr_queue && ptr_node)
  {
    ptr_prev_node=NULL;
    ptr_search_node=*hdr_queue;
    
  while(ptr_search_node)
  {
     ptr_next_node=ptr_search_node->ptr_next;
    if(ptr_search_node==ptr_node)
    {
      osal_mem_free(ptr_search_node->ptr_msg);
      osal_mem_free(ptr_search_node);
      if(ptr_prev_node==NULL)//µ±«∞ «Õ∑–Ë“™…æ≥˝
      {    
        *hdr_queue=ptr_next_node;
      }
      else
      {
        ptr_prev_node->ptr_next=ptr_next_node;
      }
     app_write_string("\r\n…æ≥˝œ˚œ¢≥…π¶!");
     return SUCCESS;
    }
    else
    {
     ptr_prev_node=ptr_search_node; 
     ptr_search_node=ptr_next_node;
    }
   } 
  }
  app_write_string("\r\n…æ≥˝œ˚œ¢ ß∞‹!");
  return NOT_FOUND_ERROR;
}


/*********************************************************************
 * @fn      simpleBLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
/*
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 )
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}
*/

/*********************************************************************
 * @fn      lock_in_BLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
/*
static bool lock_in_BLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 )
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}
*/
/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  
  // If result count not at max
  if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < simpleBLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
    
    // Increment scan result count
    simpleBLEScanRes++;
  }
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}

/*********************************************************************
*********************************************************************/
