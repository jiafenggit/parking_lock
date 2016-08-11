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
#include "lock_in_BLECentral.h"

#include"app_uart_init.h"
#include"hal_motor.h"
#include"hal_batt.h"
#include"hal_watchdog.h"



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
#define DEFAULT_SCAN_DURATION                 500

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
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      800

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY           0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           300 //¶Ï¿ªÁ¬½Ó³¬Ê±Ê±¼äÉèÖÃ

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                 GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           500 //wkxboot 1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          FALSE  //TRUE


#define  ATTEMPT_TO_EST_CONN_TIMEOUT_VALUE    2000
   
#define  DEFAULT_START_TO_SCAN_DELAY          500//wkxboot in ms


#define  CAR_SIG_NO_EXSIT_TRIGGER_VALUE      (5000/(DEFAULT_SCAN_DURATION+DEFAULT_START_TO_SCAN_DELAY))//5SÄÚÃ»ÓÐ¼ì²âµ½ÐÅºÅ¾ÍÁ¢Æðµµ¸Ë


#define  PARK_FLAG_ENTER                      1
#define  PARK_FLAG_EXIT                       2
   
#define  LED1_PERIOD_FLASH_VALUE              2000//2sÉÁË¸Ò»´Î
#define  FEED_WATCH_DOG_VALUE                 900 //900msÎ¹Ò»´Î
   

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
static uint8 lock_in_BLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "lock in Central";

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
//static uint8 simpleBLEScanIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 simpleBLEScanning = FALSE;

// RSSI polling state
static uint8 simpleBLERssi =TRUE;

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

// Value read/write toggle
static bool simpleBLEDoWrite = TRUE;

// GATT read/write procedure state
static bool simpleBLEProcedureInProgress = FALSE;

//discovery target ble addr
uint8 ble_tar_addr[6];
ble_device_t owner_info,echo_info;
uint8 car_sig_exsit_cnt=0;
bool  movable_arm_on_top=TRUE;


ble_device_t lock_in_info,car_in_info;
time_stamp_t time_stamp_info;

uint8 cur_read_char_state=BLE_READ_STAE_IDLE;
uint8 cur_write_char_state=BLE_WRITE_STATE_IDLE;



/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void wkxboot_start_discover();
static void wkxboot_stop_discover();
static void wkxboot_connect();
static void wkxboot_disconnect();
static void lock_in_BLECentral_handle_scan_car_event(scan_car_t* pMsg);
static void lock_in_BLECentral_handle_uart_debug_cmd(uint8 dbg_cmd);
static void own_addr_minus_1_to_peer_addr(uint8 *p_own_addr,uint8* p_peer_addr);
static bool ble_addr_is_match(uint8 *addr1,uint8 *addr2);
static bool ble_find_match_peer_addr_from_list(gapDevRec_t* addr_list,uint8 list_cnt);

static void lock_in_BLECentral_handle_motor_state_event(motor_msg_t* pMsg);
static void lock_in_BLECentral_Handlebatt(uint8 batt);

static void owner_info_init(uint8* addr_src);
static void wkxboot_read_car_in_info_in_peer();
static void wkxboot_read_lock_in_info_in_peer();
static void wkxboot_read_time_stamp_info_in_peer();

static void wkxboot_write_time_stamp_info_in_peer();
static void wkxboot_write_lock_in_info_in_peer();

static void wkxboot_process_park_authority();

static void lock_in_BLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static void lock_in_BLECentralRssiCB( uint16 connHandle, int8  rssi );
static uint8 lock_in_BLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void lock_in_BLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void lock_in_BLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void lock_in_BLECentral_HandleKeys( uint8 shift, uint8 keys );
static void lock_in_BLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void lock_in_BLECentralStartDiscovery( void );
//static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
char *bdAddr2Str ( uint8 *pAddr );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  lock_in_BLECentralRssiCB,       // RSSI callback
  lock_in_BLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  lock_in_BLECentralPasscodeCB,
  lock_in_BLECentralPairStateCB
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
void lock_in_BLECentral_Init( uint8 task_id )
{
  lock_in_BLETaskId = task_id;

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );


 /*
  GAP_SetParamValue(TGAP_CONN_EST_INT_MIN,DEFAULT_UPDATE_MIN_CONN_INTERVAL);
  GAP_SetParamValue(TGAP_CONN_EST_INT_MAX,DEFAULT_UPDATE_MAX_CONN_INTERVAL);
  GAP_SetParamValue(TGAP_CONN_EST_LATENCY,DEFAULT_UPDATE_SLAVE_LATENCY);
  */
  
  GAP_SetParamValue( TGAP_CONN_EST_SUPERV_TIMEOUT,DEFAULT_UPDATE_CONN_TIMEOUT);
  uint16 min_con,max_con,to,lat;

  min_con= GAP_GetParamValue(TGAP_CONN_EST_INT_MIN);
  max_con=GAP_GetParamValue(TGAP_CONN_EST_INT_MAX);
  lat=GAP_GetParamValue(TGAP_CONN_EST_LATENCY);
  to=GAP_GetParamValue( TGAP_CONN_EST_SUPERV_TIMEOUT);
  
 app_write_string("\r\nµ±Ç°Á´½Ó²ÎÊý:");
 app_write_string( uint16_to_string(min_con));
 app_write_string( uint16_to_string(max_con));
 app_write_string( uint16_to_string(lat));
 app_write_string( uint16_to_string(to));

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
  GATT_RegisterForInd( lock_in_BLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Register for all key events - This app will handle all key events
  RegisterForKeys( lock_in_BLETaskId );

  register_for_uart_debug(lock_in_BLETaskId);
  register_for_motor(lock_in_BLETaskId);
  
  register_for_batt(lock_in_BLETaskId);
  app_batt_start_periodic_update_info();
  app_motor_start_periodic_verify_state();//ÖÜÆÚÐ£Ñé³µÎ»Ëø
  
  hal_watchdog_init(WATCHDOG_MODE,CLOCK_PERIOD_1000MS);
  osal_start_timerEx(lock_in_BLETaskId,FEED_WATCH_DOG_EVT,FEED_WATCH_DOG_VALUE);
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  HCI_EXT_ExtendRfRangeCmd();
  // Setup a delayed profile startup
  osal_set_event( lock_in_BLETaskId, START_DEVICE_EVT );

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
uint16 lock_in_BLECentral_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( lock_in_BLETaskId )) != NULL )
    {
      lock_in_BLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      
    
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
    lock_in_BLECentralStartDiscovery( );
    
    return ( events ^ START_DISCOVERY_EVT );
  }
  /*wkxboot*/
  if ( events & START_SCAN_EVT )
  {
    wkxboot_start_discover();

    return ( events ^ START_SCAN_EVT );
  }

  if( events & ATTEMPT_TO_EST_CONN_TIMEOUT_EVT )
  {
    if(simpleBLEState == BLE_STATE_CONNECTING)//Èç¹ûÒÀÈ»ÔÚµÈ´ýÁ¬½Ó Ö¤Ã÷Á¬½Ó¹ý³Ì³¬Ê±
    {
    app_write_string("\r\nÊÔÍ¼½¨Á¢Á¬½Ó¹ý³Ì³¬Ê±!¹Ø±ÕestÇëÇó!");
    GAPCentralRole_TerminateLink(GAP_CONNHANDLE_INIT);
    }
    
    return ( events ^ ATTEMPT_TO_EST_CONN_TIMEOUT_EVT );
  }

   if ( events & LED1_PERIOD_FLASH_EVT )
  {
    osal_start_timerEx(lock_in_BLETaskId,LED1_PERIOD_FLASH_EVT,LED1_PERIOD_FLASH_VALUE);
    HalLedSet(HAL_LED_1,HAL_LED_MODE_BLINK);
    
 #if (OSALMEM_METRICS)
    {
    uint16 i;
    i=osal_heap_mem_used();
    app_write_string("\r\nmem used:");
    app_write_string(uint16_to_string(i));   
    i= osal_heap_block_cnt();
    app_write_string("\r\nmem block:");
    app_write_string(uint16_to_string(i));
    i= osal_heap_block_free();
    app_write_string("\r\nmem free:");
    app_write_string(uint16_to_string(i));
    }
#endif
       
    return ( events ^ LED1_PERIOD_FLASH_EVT );
  }
  
   if ( events & FEED_WATCH_DOG_EVT )
  {
    osal_start_timerEx(lock_in_BLETaskId,FEED_WATCH_DOG_EVT,FEED_WATCH_DOG_VALUE);
    
    hal_feed_watchdog();   
    return ( events ^ FEED_WATCH_DOG_EVT );
  }
  
  // Discard unknown events
  return 0;
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
static void lock_in_BLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      lock_in_BLECentral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  case MOTOR_SIGNAL_EVENT:    
      lock_in_BLECentral_handle_motor_state_event((motor_msg_t*)pMsg);
      break;
  case UART_DEBUG_CMD:
      lock_in_BLECentral_handle_uart_debug_cmd( ((uart_debug_cmd_t*)pMsg)->cmd);
      break;
  case GATT_MSG_EVENT:
      lock_in_BLECentralProcessGATTMsg( (gattMsgEvent_t *)pMsg );
      break;
  case BATT_UPDATE_EVENT:
      lock_in_BLECentral_Handlebatt(((batt_msg_t *)pMsg)->batt);
      break;
  default:
    break;
  }
}


static void lock_in_BLECentral_Handlebatt(uint8 batt)
{
  owner_info.ble_battery=batt;
  app_write_string("\r\n¸üÐÂowner batt Íê³É,µ±Ç°owner batt(%):");
  app_write_string(uint8_to_string(owner_info.ble_battery));
}


static void owner_info_init(uint8*addr_src)
{
  osal_memcpy(owner_info.ble_id,addr_src,B_ADDR_LEN);//¸´ÖÆµØÖ·
 
  owner_info.ble_active=CAR_HAS_PARKING_AUTHORITY;
  owner_info.ble_battery=100;//default 100%
  app_write_string("\r\n±¾µØ½á¹¹ÐÅÏ¢³õÊ¼»¯Íê³É!");
}


static void lock_in_BLECentral_handle_uart_debug_cmd(uint8 dbg_cmd)
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
  case  UART_DEBUG_CMD_START_ADC:
    app_write_string("\r\nV is:");
    app_write_string(uint8_to_string(halGetVoltageMonitorInput(BATT_MONITOR_CHN)));
    break;
  case  UART_DEBUG_CMD_START_SCAN_CAR :
    app_write_string("\r\n¿ªÊ¼É¨Ãè³µÁ¾...");
    break;
  case UART_DEBUG_CMD_START_POSITIVE_RUN:
     app_movable_arm_set_target_90_90();
     break;
  case UART_DEBUG_CMD_START_NEGATIVE_RUN:
     app_movable_arm_set_target_0_0();
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
        app_write_string("\r\n¿ªÊ¼Ê¹ÓÃlock_in_UUID»ñÈ¡char handle");
        }
        if(handle_state==BLE_DISC_STATE_CAR_IN_CHAR)
        {
        req.type.uuid[0] = LO_UINT16(PARAM_BLE_CAR_IN_CHAR_UUID);
        req.type.uuid[1] = HI_UINT16(PARAM_BLE_CAR_IN_CHAR_UUID);
        app_write_string("\r\n¿ªÊ¼Ê¹ÓÃcar_in_UUID»ñÈ¡char handle");
        }
        if(handle_state==BLE_DISC_STATE_TIME_STAMP_CHAR)
        {
        req.type.uuid[0] = LO_UINT16(PARAM_BLE_TIME_STAMP_CHAR_UUID);
        req.type.uuid[1] = HI_UINT16(PARAM_BLE_TIME_STAMP_CHAR_UUID);
        app_write_string("\r\n¿ªÊ¼Ê¹ÓÃtime_stamp_UUID»ñÈ¡char handle");
        }
        GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, lock_in_BLETaskId );
      }
}


static void wkxboot_read_car_in_info_in_peer()
{
    app_write_string("\r\n¿ªÊ¼¶ÁÈ¡car_inÐÅÏ¢!");
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
        status = GATT_ReadCharValue( simpleBLEConnHandle, &req, lock_in_BLETaskId );
        app_write_string("\r\n¿ªÊ¼¶ÁÖµ!status:");
        app_write_string(uint8_to_string(status));
      
      if ( status == SUCCESS )
      {
        cur_read_char_state=BLE_READ_STAE_CAR_IN;
        simpleBLEProcedureInProgress = TRUE;
        app_write_string("\r\n¶ÁÖµ½ø³Ì·µ»ØÕý³£!");
      }
      else
      {
        cur_read_char_state=BLE_READ_STAE_IDLE;
        app_write_string("\r\n¶ÁÖµ½ø³Ì·µ»Ø´íÎó!×¼±¸¶Ï¿ªÁ¬½Ó...");
        wkxboot_disconnect();
      }
    }    
  } 
 else
  {
    app_write_string("\r\nÁ¬½ÓÊÇ¶Ï¿ªµÄ»òÕßÁ¬½Ó¾ä±úÎª0ÎÞ·¨¶ÁÈ¡!");
  }
}

static void wkxboot_read_lock_in_info_in_peer()
{
  app_write_string("\r\n¿ªÊ¼¶ÁÈ¡lock_inÐÅÏ¢!");
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
        status = GATT_ReadCharValue( simpleBLEConnHandle, &req, lock_in_BLETaskId );
        app_write_string("\r\n¿ªÊ¼¶ÁÖµ!status:");
        app_write_string(uint8_to_string(status));
      
      if ( status == SUCCESS )
      {
        cur_read_char_state=BLE_READ_STAE_LOCK_IN;
        simpleBLEProcedureInProgress = TRUE;
        app_write_string("\r\n¶ÁÖµ½ø³Ì·µ»ØÕý³£!");
      }
      else
      {
        cur_read_char_state=BLE_READ_STAE_IDLE;
        app_write_string("\r\n¶ÁÖµ½ø³Ì·µ»Ø´íÎó!×¼±¸¶Ï¿ªÁ¬½Ó...");
        wkxboot_disconnect();
      }
    }    
  } 
  else
  {
    app_write_string("\r\nÁ¬½ÓÊÇ¶Ï¿ªµÄ»òÕßÁ¬½Ó¾ä±úÎª0ÎÞ·¨¶ÁÈ¡!");
  }
}

static void wkxboot_read_time_stamp_info_in_peer()
{
  app_write_string("\r\n¿ªÊ¼¶ÁÈ¡time_stampÐÅÏ¢!");
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
        status = GATT_ReadCharValue( simpleBLEConnHandle, &req, lock_in_BLETaskId );
        app_write_string("\r\n¿ªÊ¼¶ÁÖµ!status:");
        app_write_string(uint8_to_string(status));
      
      if ( status == SUCCESS )
      {
        cur_read_char_state=BLE_READ_STAE_TIME_STAMP;
        simpleBLEProcedureInProgress = TRUE;
        app_write_string("\r\n¶ÁÖµ½ø³Ì·µ»ØÕý³£!");
      }
      else
      {
        cur_read_char_state=BLE_READ_STAE_IDLE;
        app_write_string("\r\n¶ÁÖµ½ø³Ì·µ»Ø´íÎó!×¼±¸¶Ï¿ªÁ¬½Ó...");
        wkxboot_disconnect();
      }
    }    
  }
  else
  {
    app_write_string("\r\nÁ¬½ÓÊÇ¶Ï¿ªµÄ»òÕßÁ¬½Ó¾ä±úÎª0ÎÞ·¨¶ÁÈ¡!");
  }
}

static void wkxboot_write_lock_in_info_in_peer()
{
  app_write_string("\r\n¿ªÊ¼Ïò³µÔØÀ¶ÑÀÐ´Èë³µÎ»ËøÐÅÏ¢!");
  if ( simpleBLEState == BLE_STATE_CONNECTED &&
              BLE_lock_in_CharHdl != 0 &&
              simpleBLEProcedureInProgress == FALSE )
    {
      uint8 status;
      
      // Do a read or write as long as no other read or write is in progress
      if ( !simpleBLEProcedureInProgress )
      {
        // Do a write
        attWriteReq_t req;
        
       req.pValue = GATT_bm_alloc( simpleBLEConnHandle, ATT_WRITE_REQ, sizeof(ble_device_t), NULL );
       if(req.pValue!= NULL)
        {
        req.handle = BLE_lock_in_CharHdl;
        req.len = sizeof(ble_device_t);
        *((ble_device_t*)req.pValue) = owner_info;
        req.sig = 0;
        req.cmd = 0;
        status = GATT_WriteCharValue( simpleBLEConnHandle, &req, lock_in_BLETaskId );    
        app_write_string("\r\n¿ªÊ¼Ð´Öµ!status:");
        app_write_string(uint8_to_string(status));
      
        if ( status == SUCCESS )
        {
        simpleBLEProcedureInProgress = TRUE;
        cur_write_char_state=BLE_WRITE_STATE_LOCK_IN;
        app_write_string("\r\nÐ´Öµ½ø³Ì·µ»ØÕý³£!");
        }
        else
        {
        GATT_bm_free( (gattMsg_t *)&req, ATT_WRITE_REQ );
        cur_write_char_state=BLE_WRITE_STATE_IDLE;
        app_write_string("\r\nlock inÐ´Öµ½ø³Ì·µ»Ø´íÎó!×¼±¸¶Ï¿ªÁ¬½Ó...");
        wkxboot_disconnect();
        }
        }
    }    
  } 
}

static void wkxboot_write_time_stamp_info_in_peer()
{
  app_write_string("\r\n¿ªÊ¼Ð´Èëtime_stampÐÅÏ¢!");

  
  if ( simpleBLEState == BLE_STATE_CONNECTED &&
              BLE_time_stamp_CharHdl != 0 &&
              simpleBLEProcedureInProgress == FALSE )
    {
      uint8 status;
      
      // Do a read or write as long as no other read or write is in progress
      if ( !simpleBLEProcedureInProgress )
      {
       attWriteReq_t req;    
       
       req.pValue = GATT_bm_alloc( simpleBLEConnHandle, ATT_WRITE_REQ, sizeof(time_stamp_t), NULL );
      if(req.pValue!= NULL)
      {
       req.handle = BLE_time_stamp_CharHdl;
       req.len = sizeof(time_stamp_t);;
       *((time_stamp_t*)req.pValue) = time_stamp_info;
       req.sig = 0;
       req.cmd = 0;
       status = GATT_WriteCharValue( simpleBLEConnHandle, &req, lock_in_BLETaskId );    
       app_write_string("\r\n¿ªÊ¼Ð´time stampÖµ!status:");
       app_write_string(uint8_to_string(status));
      
      if ( status == SUCCESS )
      {
        cur_write_char_state=BLE_WRITE_STATE_TIME_STAMP;
        simpleBLEProcedureInProgress = TRUE;
        app_write_string("\r\ntime stamp Ð´Öµ½ø³Ì·µ»ØÕý³£!");
      }
      else
      {
        GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ );
        cur_write_char_state=BLE_WRITE_STATE_IDLE;
        app_write_string("\r\ntime stamp Ð´Öµ½ø³Ì·µ»Ø´íÎó!×¼±¸¶Ï¿ªÁ¬½Ó...");
        wkxboot_disconnect();
      }
      }
    }
   else
   {
     app_write_string("\r\ntime stamp simpleBLEProcedureInProgress is true!");
   }
  }
  else
  {
   app_write_string("\r\ntime stamp Á¬½ÓÊÇ¶Ï¿ªµÄ»òÕßÁ¬½Ó¾ä±úÎª0ÎÞ·¨Ð´Èë!");
  }
  
   
}

static void wkxboot_process_park_authority()
{
  if(car_in_info.ble_active==CAR_HAS_PARKING_AUTHORITY)
  {   
   app_write_string("\r\nÄ¿±êÀ¶ÑÀÓÐÊÚÈ¨,¿ÉÒÔÍ£³µ!");
   app_movable_arm_set_target_0_0();//·ÅÏÂµµ¸Ë  
  }
  else
  {
   app_write_string("\r\nÄ¿±êÀ¶ÑÀÃ»ÓÐÊÚÈ¨,½ûÖ¹Í£³µ!"); 
   wkxboot_disconnect();//¶Ï¿ªÁ¬½Ó
  }
  
}

static void wkxboot_process_echo_info()
{
   lock_in_info.ble_active=CAR_HAS_PARKING_AUTHORITY;
   lock_in_info.ble_battery=owner_info.ble_battery;
   time_stamp_info.flag=PARK_FLAG_EXIT;//ÉèÖÃ³ö³¡±êÖ¾
   wkxboot_write_time_stamp_info_in_peer(); 
  
}

static void lock_in_BLECentral_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter

 if ( keys & KEY_FUN1 )
  {
 app_write_string("\r\nÏµÍ³ÊÕµ½KEY_FUN2");
  } 
 if ( keys & KEY_FUN2 )
  {
    app_write_string("\r\nÏµÍ³ÊÕµ½KEY_FUN2");
  }
}

/*********************************************************************
 * @fn      lock_in_BLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void lock_in_BLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  if ( simpleBLEState != BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    app_write_string("\r\n¶ÁÐ´´íÎó!Ã»ÓÐÁ¬½Ó!");
    GATT_bm_free( &pMsg->msg, pMsg->method );
    return;
  }
  
  app_write_string("\r\n½øÈë¶ÁÐ´»ØÓ¦...method=");
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
      /*
      if(cur_read_char_state==BLE_READ_STAE_LOCK_IN)
      {
      lock_in_info = *((ble_device_t*)pMsg->msg.readRsp.value);
      app_write_string("\r\n¶Áµ½lock inµÄÖµÊÇ:\r\n");
      uint8_array_to_string((uint8*)&lock_in_info,sizeof(ble_device_t));
      simpleBLEProcedureInProgress = FALSE;
      
      app_write_string("\r\n¶ÁÖµ½ø³Ìstatus:");
      app_write_string(uint8_to_string(pMsg->hdr.status)); 
      wkxboot_read_car_in_info_in_peer();
      }
      else if(cur_read_char_state==BLE_READ_STAE_CAR_IN)
      {
      car_in_info = *((ble_device_t*)pMsg->msg.readRsp.value);
      app_write_string("\r\n¶Áµ½car inµÄÖµÊÇ:\r\n");
      uint8_array_to_string((uint8*)&car_in_info,sizeof(ble_device_t));
      simpleBLEProcedureInProgress = FALSE;
      
      app_write_string("\r\ncar in ¶ÁÖµ½ø³Ìstatus:");
      app_write_string(uint8_to_string(pMsg->hdr.status)); 
      wkxboot_read_time_stamp_info_in_peer();
      }
      else if(cur_read_char_state==BLE_READ_STAE_TIME_STAMP)
      {
      time_stamp_info = *((time_stamp_t*)pMsg->msg.readRsp.value);
      app_write_string("\r\n¶Áµ½time stampµÄÖµÊÇ:\r\n");
      uint8_array_to_string((uint8*)&time_stamp_info,sizeof(time_stamp_t));
      simpleBLEProcedureInProgress = FALSE;
      cur_read_char_state=BLE_READ_STAE_IDLE;//
      app_write_string("\r\ntime stamp ¶ÁÖµ½ø³Ìstatus:");
      app_write_string(uint8_to_string(pMsg->hdr.status)); 
      
      wkxboot_process_echo_info();//´¦Àí»ØÐ´µÄÐÅÏ¢
      
      }   
     */
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
         app_write_string("\r\ntime stampÐ´µÄÖµÊÇ:\r\n");
         uint8_array_to_string((uint8*)&time_stamp_info,sizeof(time_stamp_t));           
         wkxboot_write_lock_in_info_in_peer(); //Ð´ÈëÆû³µÈ¨ÏÞ
        
      }
      else if(cur_write_char_state==BLE_WRITE_STATE_CAR_IN)
      {
        /*
         simpleBLEProcedureInProgress = FALSE;
         cur_write_char_state=BLE_WRITE_STATE_IDLE;
 
         app_write_string("\r\ncar inÐ´µÄÖµÊÇ:\r\n");
         uint8_array_to_string((uint8*)&car_in_info,sizeof(ble_device_t));   
         
         wkxboot_disconnect();//Ö÷¶¯¶Ï¿ªÁ¬½Ó
        */
      }
      else if(cur_write_char_state==BLE_WRITE_STATE_LOCK_IN)
      {
         simpleBLEProcedureInProgress = FALSE;
         cur_write_char_state=BLE_WRITE_STATE_IDLE;
 
         app_write_string("\r\nlock inÐ´µÄÖµÊÇ:\r\n");
         uint8_array_to_string((uint8*)&lock_in_info,sizeof(ble_device_t));   
         app_write_string("\r\n»ØÐ´Êý¾ÝÍê³É!");
    /*     
    if ( simpleBLEState == BLE_STATE_CONNECTED )//¸üÐÂ²ÎÊý
    {
    uint8 i= GAPCentralRole_UpdateLink( simpleBLEConnHandle,
                                 DEFAULT_UPDATE_MIN_CONN_INTERVAL,
                                 DEFAULT_UPDATE_MAX_CONN_INTERVAL,
                                 DEFAULT_UPDATE_SLAVE_LATENCY,
                                 DEFAULT_UPDATE_CONN_TIMEOUT );
    app_write_string("\r\n·µ»Ø½á¹û:");
    app_write_string(uint8_to_string(i));
    }
    */
      }
    }   
      
  }
  else if ( simpleBLEDiscState != BLE_DISC_STATE_IDLE )
  {
    simpleBLEGATTDiscoveryEvent( pMsg );
  }
  
  GATT_bm_free( &pMsg->msg, pMsg->method );
}


/*******wkxboot*********/
static void lock_in_BLECentral_handle_motor_state_event(motor_msg_t* pMsg)
{
  app_write_string("\r\nÏµÍ³ÊÕµ½Âí´ïÐÅºÅ!");
  
   if(pMsg->state==SIGNAL_START_PERIODIC_VERIFY)
  {
    app_motor_start_periodic_verify_state();//¿ªÆôÐ£ÑéÎ»ÖÃ
    car_sig_exsit_cnt=0;

  }
    
  if(pMsg->state==SIGNAL_STOP_PERIODIC_VERIFY)
  {  
   app_motor_stop_periodic_verify_state();//¹Ø±ÕÐ£ÑéÎ»ÖÃ 
  }
  if(pMsg->state==SIGNAL_MOVABLE_ARM_ON_TOP)
  {  
   movable_arm_on_top=TRUE;
    app_write_string("\r\non top");
  }
  if(pMsg->state==SIGNAL_MOVABLE_ARM_ON_BOTTOM)
  {  
   movable_arm_on_top=FALSE;
   app_write_string("\r\non bot");
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
app_write_string("\r\n³µÔØÄ¿±êÀ¶ÑÀµØÖ·:");
app_write_string(bdAddr2Str(p_peer_addr));
}

static bool ble_addr_is_match(uint8 *addr1,uint8 *addr2)
{
  for(uint8 i=0;i<6;i++) 
  {
    if(*addr1++!=*addr2++)
    {
      app_write_string("\r\nÀ¶ÑÀµØÖ·²»Æ¥Åä!");
      return FALSE;  
    }
  }
  app_write_string("\r\nÀ¶ÑÀµØÖ·Æ¥Åä!");
  return TRUE; 
}

static bool ble_find_match_peer_addr_from_list(gapDevRec_t* addr_list,uint8 list_cnt)
{
  gapDevRec_t* ptr_addr;
  ptr_addr=addr_list;
  for(uint8 i=0;i<list_cnt;i++)
  {
    if(ble_addr_is_match(ptr_addr->addr,ble_tar_addr))
    {
      osal_memcpy(ble_tar_addr,ptr_addr->addr,B_ADDR_LEN);
      app_write_string("\r\nÕÒµ½Ä¿±êµØÖ·²¢¸´ÖÆ!ÁÐ±íÖµ:");
      app_write_string(uint8_to_string(i));
      return TRUE;
    }
    ptr_addr++;
  }    
    app_write_string("\r\nÃ»ÓÐÕÒµ½Ä¿±êµØÖ·!");
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
     app_write_string("\r\n¿ªÊ¼É¨Ãè.....");
  }
 else
 {
  app_write_string("\r\nÒÑ¾­ÔÚÉ¨Ãè-----");
 }
}

static void wkxboot_stop_discover()
{
   if(simpleBLEScanning)
  { 
       simpleBLEScanning = FALSE;
       //simpleBLEScanRes = 0;
        
       GAPCentralRole_CancelDiscovery(); 
       app_write_string("\r\nÉ¨Ãè¹Ø±Õ");
  }
 else
 {
  app_write_string("\r\nÉ¨ÃèÒÑ¾­¹Ø±Õ£¡");
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
         app_write_string("\r\nÕýÔÚÁ¬½Ó...");
         app_write_string(bdAddr2Str( peerAddr ));
         
         osal_start_timerEx(lock_in_BLETaskId,ATTEMPT_TO_EST_CONN_TIMEOUT_EVT,ATTEMPT_TO_EST_CONN_TIMEOUT_VALUE);//½¨Á¢Á¬½ÓµÄ¹ý³Ì³¬Ê±
         app_write_string("\r\n¿ªÊ¼¼àÊÓÁ¬½Ó³¬Ê±...");
         
       }
       else
       {
         simpleBLEState =BLE_STATE_IDLE;
        // LL_CreateConnCancel();
         app_write_string("\r\nÖÕÖ¹Á¬½Ó!Á¬½Ó¹ý³Ì³ö´í!status:");
         app_write_string(uint8_to_string(rt_status));
         app_write_string("\r\n0.5ÃëÖÓºóÖØÐÂ¿ªÊ¼É¨Ãè!");
         osal_start_timerEx(lock_in_BLETaskId,START_SCAN_EVT,DEFAULT_START_TO_SCAN_DELAY);//
       }
      
    }      
    else
    {
       app_write_string("\r\nÕýÔÚÉ¨Ãè»òÕßÁ¬½Ó×´Ì¬£¡ÎÞ·¨Á¬½Ó£¡");
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
      
      app_write_string( "\r\nÕýÔÚ¹Ø±ÕÁ¬½Ó..." ); 
    }
 else
 {
   if(simpleBLEState == BLE_STATE_IDLE)
   app_write_string( "\r\nÁ¬½ÓÒÑ¾­ÊÇ¶Ï¿ª×´Ì¬£¡" ); 
   if(simpleBLEState == BLE_STATE_DISCONNECTING)
   app_write_string( "\r\nÒÑ¾­ÔÚ¹Ø±ÕÁ¬½ÓÖÐ£¡" ); 
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
static void lock_in_BLECentralRssiCB( uint16 connHandle, int8 rssi )
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
static uint8 lock_in_BLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
      /***wkxboot***/
        app_write_string("\r\nble4.0Éè±¸³õÊ¼»¯Íê³É!");
        app_write_string("\r\n±¾»úÉè±¸µØÖ·:");
        app_write_string(bdAddr2Str(pEvent->initDone.devAddr));
        own_addr_minus_1_to_peer_addr(pEvent->initDone.devAddr,ble_tar_addr);
        owner_info_init(pEvent->initDone.devAddr);
        app_write_string("\r\n1ÃëÖÓºó¿ªÊ¼µÚÒ»´ÎÉ¨Ãè!");
        osal_start_timerEx(lock_in_BLETaskId,START_SCAN_EVT,DEFAULT_START_TO_SCAN_DELAY);//1s
        
        osal_start_timerEx(lock_in_BLETaskId,LED1_PERIOD_FLASH_EVT,LED1_PERIOD_FLASH_VALUE);//led1¿ªÊ¼ÉÁË¸
        app_movable_arm_set_target_90_90();//init stateµµ¸ËÊúÆð
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        app_write_string("\r\n·¢ÏÖÉè±¸:");
        app_write_string(bdAddr2Str( pEvent->deviceInfo.addr ));
        
        // if filtering device discovery results based on service UUID
        
        app_write_string("\r\n·¢ÏÖµÄÊý¾ÝÖµ:");
        uint8_array_to_string(pEvent->deviceInfo.pEvtData,pEvent->deviceInfo.dataLen);
        
        if(ble_addr_is_match(pEvent->deviceInfo.addr,ble_tar_addr) && pEvent->deviceInfo.pEvtData[9]== CAR_HAS_PARKING_AUTHORITY)
        {
          app_movable_arm_set_target_0_0();//½µÏÂµµ¸Ë
        
          car_sig_exsit_cnt=0;//´æÔÚ´ÎÊý=0
          //simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );        
          app_write_string("\r\nÄ¿±êÕýÈ·!");
          wkxboot_stop_discover();//¹Ø±ÕÉ¨Ãè
          wkxboot_connect();//Ö±½ÓÁ¬½Ó
        }
        else
        {
         app_write_string("\r\nÄ¿±ê´íÎóÌø¹ý..¼ÌÐøÉ¨Ãè..."); 
        }
 
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
       simpleBLEScanning = FALSE;
        
       app_write_string("\r\nÍê³ÉÒ»¸öÉ¨ÃèÖÜÆÚ!");
       if(simpleBLEState == BLE_STATE_IDLE)//Èç¹ûÊÇ¿ÕÏÐ×´Ì¬¾Í¿ªÆôÉ¨Ãè
       {
        osal_start_timerEx(lock_in_BLETaskId,START_SCAN_EVT,DEFAULT_START_TO_SCAN_DELAY);
        app_write_string("\r\ncar_sig:");
        app_write_string(uint8_to_string(car_sig_exsit_cnt));
        
        if(!movable_arm_on_top && ++car_sig_exsit_cnt> CAR_SIG_NO_EXSIT_TRIGGER_VALUE)
         {
         car_sig_exsit_cnt=0;
         app_movable_arm_set_target_90_90();
         }
        
        app_write_string("\r\n¿ÕÏÐ×´Ì¬½«¼ÌÐøÉ¨Ãè!");
       }
       else
       {
       app_write_string("\r\n·Ç¿ÕÏÐ×´Ì¬²»ÔÙÉ¨Ãè!");
        }
       break;
      }
    case GAP_LINK_ESTABLISHED_EVENT:
      {
        osal_stop_timerEx(lock_in_BLETaskId,ATTEMPT_TO_EST_CONN_TIMEOUT_EVT);//¹Ø±ÕÁ¬½Ó¹ý³Ì³¬Ê±¶¨Ê±Æ÷
          
        if ( pEvent->gap.hdr.status == SUCCESS )
        {    
          simpleBLEState = BLE_STATE_CONNECTED;
          simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;
          simpleBLEProcedureInProgress = TRUE;    

          // If service discovery not performed initiate service discovery
          /*//²»ÔÙ½øÐÐÊý¾ÝÍ¨ÐÅ¡£Ö±½ÓÁ¬½Ó³É¹¦ºó¾Í²»ÎÊ¡£
          if ( BLE_lock_in_CharHdl == 0 )
          {
            osal_start_timerEx( lock_in_BLETaskId, START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
            app_write_string("\r\n¿ªÊ¼·¢ÏÖservices.....");
          }      
          */
          app_write_string("\r\nÁ¬½Ó³É¹¦!Á¬½ÓµÄÀ¶ÑÀµØÖ·:");
          app_write_string(bdAddr2Str(pEvent->linkCmpl.devAddr));
          app_write_string("\r\nÁ¬½ÓµÄconnhandle:");
          app_write_string(uint16_to_string(simpleBLEConnHandle));
          
          if(simpleBLERssi)
          {
            app_write_string("\r\nÏÔÊ¾rssiÖµ.");
            GAPCentralRole_StartRssi(simpleBLEConnHandle,DEFAULT_RSSI_PERIOD);
          }

          
        }
        else
        {
          simpleBLEState = BLE_STATE_IDLE;
          simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
          //simpleBLERssi = FALSE;
          simpleBLEDiscState = BLE_DISC_STATE_IDLE;
 
          app_write_string("\r\nÁ¬½ÓÊ§°Ü,status:");
          app_write_string(uint8_to_string(pEvent->gap.hdr.status));
          app_write_string("\r\nÉÔºóÖØÐÂ¿ªÊ¼É¨Ãè!");
          
          osal_start_timerEx(lock_in_BLETaskId,START_SCAN_EVT,DEFAULT_START_TO_SCAN_DELAY);
           
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        osal_stop_timerEx(lock_in_BLETaskId,ATTEMPT_TO_EST_CONN_TIMEOUT_EVT);//¹Ø±ÕÁ¬½Ó¹ý³Ì³¬Ê±¶¨Ê±Æ÷
          
        simpleBLEState = BLE_STATE_IDLE;
        simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
        //simpleBLERssi = FALSE;
        simpleBLEDiscState = BLE_DISC_STATE_IDLE;
        BLE_lock_in_CharHdl = 0;
        simpleBLEProcedureInProgress = FALSE;
        
        app_write_string("\r\nÁ¬½Ó¶Ï¿ª!Ô­Òò:");
        app_write_string(uint8_to_string(pEvent->linkTerminate.reason));          

        
        app_write_string("\r\nÒ»¶ÎÊ±¼äºóÖØÐÂ¿ªÊ¼É¨Ãè!");
        osal_start_timerEx(lock_in_BLETaskId,START_SCAN_EVT,DEFAULT_START_TO_SCAN_DELAY);
          
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
  
  return (TRUE);
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void lock_in_BLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
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
static void lock_in_BLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
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
static void lock_in_BLECentralStartDiscovery( void )
{
  uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(PARAM_BLE_CAR_IN_PROFILE_SERV_UUID),
                                   HI_UINT16(PARAM_BLE_CAR_IN_PROFILE_SERV_UUID) };
  
  // Initialize cached handles
  simpleBLESvcStartHdl = simpleBLESvcEndHdl = BLE_lock_in_CharHdl = 0;

  simpleBLEDiscState = BLE_DISC_STATE_SVC;
  
  // Discovery simple BLE service
  GATT_DiscPrimaryServiceByUUID( simpleBLEConnHandle,
                                 uuid,
                                 ATT_BT_UUID_SIZE,
                                 lock_in_BLETaskId );
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
      
      //simpleBLESvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
      //simpleBLESvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
      
      simpleBLESvcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      simpleBLESvcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      
      app_write_string("\r\n·¢ÏÖµÄservice¿ªÊ¼handle:");
      app_write_string(uint16_to_string(simpleBLESvcStartHdl));
      app_write_string("\r\n·¢ÏÖµÄservice½áÊøhandle:");
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
        start_to_find_char_handle(BLE_DISC_STATE_CAR_IN_CHAR);//·¢ÏÖ³µÔØÀ¶ÑÀÐÅÏ¢¾ä±ú
      }
      /****wkxboot***/
      else
      {
        simpleBLEDiscState =BLE_DISC_STATE_IDLE;
        simpleBLEProcedureInProgress = FALSE; 
        app_write_string("\r\nservice start handleÎª0!×¼±¸¶Ï¿ªÁ¬½Ó...");
        wkxboot_disconnect();
      }
    }
    if ( pMsg->method == ATT_ERROR_RSP )
    {
        simpleBLEDiscState =BLE_DISC_STATE_IDLE;
        simpleBLEProcedureInProgress = FALSE; 
        app_write_string("\r\nservice handle·¢ÏÖ³ö´í!×¼±¸¶Ï¿ªÁ¬½Ó...");
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
      BLE_lock_in_CharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.pDataList[0],
                                          pMsg->msg.readByTypeRsp.pDataList[1] );
      
      lock_in_info = *((ble_device_t*)&pMsg->msg.readByTypeRsp.pDataList[2]);//±£´ælock_in_infoÐÅÏ¢
      
      app_write_string("\r\nlcok in numPairs:");
      app_write_string(uint8_to_string( pMsg->msg.readByTypeRsp.numPairs));
      
      app_write_string("\r\n»ñÈ¡µÄlock_in_char handle:");
      app_write_string(uint16_to_string(BLE_lock_in_CharHdl));
      
      app_write_string("\r\nÊÕµ½lock inµÄÖµÊÇ:\r\n");
      uint8_array_to_string((uint8*)&lock_in_info,sizeof(ble_device_t));
             
     }
    else
    {
      app_write_string("\r\nlock in numPairs:");
      app_write_string(uint8_to_string( pMsg->msg.readByTypeRsp.numPairs));

      app_write_string("\r\nlock_in_char handle·¢ÏÖÍê³É!");
      
                  /***wkxboot***/  
     app_write_string("\r\nËùÓÐservic handles char·¢ÏÖÍê±Ï!×¼±¸»ØÐ´Êý¾Ý!");
     wkxboot_process_echo_info();//´¦Àí»ØÐ´Êý¾Ý
     //wkxboot_read_lock_in_info_in_peer();
        
    }
   }
   if( pMsg->method == ATT_ERROR_RSP)
   {
     app_write_string("\r\nlock in handle error rsp!");
     simpleBLEDiscState = BLE_DISC_STATE_IDLE;
     simpleBLEProcedureInProgress = FALSE; 
     app_write_string("\r\nlock_in_char handle·¢ÏÖ³ö´í!×¼±¸¶Ï¿ªÁ¬½Ó...");
     wkxboot_disconnect();
   }
  }
  else if ( simpleBLEDiscState == BLE_DISC_STATE_CAR_IN_CHAR )//·¢ÏÖÁË³µÔØÀ¶ÑÀÐÅÏ¢¾ä±úÐÅÏ¢£¬´ËÐÅÏ¢°üÀ¨¾ä±úºÍ´æ´¢µÄÊý¾Ý
  {
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP)
    {
      if(  pMsg->msg.readByTypeRsp.numPairs > 0 )
     {
      BLE_car_in_CharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.pDataList[0],
                                          pMsg->msg.readByTypeRsp.pDataList[1] );    
      
      car_in_info = *((ble_device_t*)&pMsg->msg.readByTypeRsp.pDataList[2]);//±£´æcar_in_infoÐÅÏ¢
      
    //  wkxboot_process_park_authority();//´¦ÀíÊÚÈ¨ÐÅÏ¢

      app_write_string("\r\ncar in numPairs:");
      app_write_string(uint8_to_string( pMsg->msg.readByTypeRsp.numPairs));
      
      app_write_string("\r\n»ñÈ¡µÄcar_in_char handle:");
      app_write_string(uint16_to_string(BLE_car_in_CharHdl));  
      
      app_write_string("\r\nÊÕµ½car inµÄÖµÊÇ:\r\n");
      uint8_array_to_string((uint8*)&car_in_info,sizeof(ble_device_t));
      
     }
    else
    {
      app_write_string("\r\ncar in numPairs:");
      app_write_string(uint8_to_string( pMsg->msg.readByTypeRsp.numPairs));

      app_write_string("\r\ncar_in_char ·¢ÏÖÍê³É!");
      start_to_find_char_handle(BLE_DISC_STATE_TIME_STAMP_CHAR);          
    }
   }
   if(pMsg->method == ATT_ERROR_RSP)
   {
      app_write_string("\r\ncar in handle error rsp!");
      simpleBLEDiscState = BLE_DISC_STATE_IDLE;
      simpleBLEProcedureInProgress = FALSE; 
      app_write_string("\r\ncar_in_char handle·¢ÏÖ³ö´í!×¼±¸¶Ï¿ªÁ¬½Ó...");
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
      BLE_time_stamp_CharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.pDataList[0],
                                          pMsg->msg.readByTypeRsp.pDataList[1] );
      
      time_stamp_info = *((time_stamp_t*)&pMsg->msg.readByTypeRsp.pDataList[2]);
      
      app_write_string("\r\ntime stamp numPairs:");
      app_write_string(uint8_to_string( pMsg->msg.readByTypeRsp.numPairs));
      
      app_write_string("\r\n»ñÈ¡µÄtime_stamp_char handle:");
      app_write_string(uint16_to_string(BLE_time_stamp_CharHdl));
      
      app_write_string("\r\nÊÕµ½time stampµÄÖµÊÇ:\r\n");
      uint8_array_to_string((uint8*)&time_stamp_info,sizeof(time_stamp_t));
    }
    else
    {
      simpleBLEDiscState = BLE_DISC_STATE_IDLE;
      simpleBLEProcedureInProgress = FALSE; 

      app_write_string("\r\ntime stamp char ·¢ÏÖÍê³É!");
      start_to_find_char_handle(BLE_DISC_STATE_LOCK_IN_CHAR); 
   
     }
    }
    if(pMsg->method == ATT_ERROR_RSP)
    {
      app_write_string("\r\ntime stamp handle error rsp!");
      simpleBLEDiscState = BLE_DISC_STATE_IDLE;
      simpleBLEProcedureInProgress = FALSE; 
      wkxboot_disconnect();
      app_write_string("\r\ntime stamp char handle·¢ÏÖ³ö´í!×¼±¸¶Ï¿ªÁ¬½Ó...");
    }
 
  }    
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
