/**************************************************************************************************
  Filename:       car_in_BLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#include "peripheral.h"

#include "gapbondmgr.h"

#include "car_in_BLEPeripheral.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

#include"app_uart_init.h"


#if HAL_BATT_CHECK >0
#include"hal_batt.h"
#endif


#include"bcomdef.h"
#include"osal_snv.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   5000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL           800

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL//GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     400

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     400

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          250 //1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         2

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF




// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15


#define  CAR_AUHTORITY_FLAG_ID                0x80 //�洢����Ȩ����Ϣ
#define  CUR_CONNET_TARGET_LOCK_IN            0x11 //��ǰ���ӵ���lockin
#define  CUR_CONNET_TARGET_CONTROL            0x22 //��ǰ���ӵ���control

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
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x69,   // 'i'
  0x6d,   // 'm'
  0x70,   // 'p'
  0x6c,   // 'l'
  0x65,   // 'e'
  0x42,   // 'B'
  0x4c,   // 'L'
  0x45,   // 'E'
  0x50,   // 'P'
  0x65,   // 'e'
  0x72,   // 'r'
  0x69,   // 'i'
  0x70,   // 'p'
  0x68,   // 'h'
  0x65,   // 'e'
  0x72,   // 'r'
  0x61,   // 'a'
  0x6c,   // 'l'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( PARAM_BLE_CAR_IN_PROFILE_SERV_UUID ),
  HI_UINT16( PARAM_BLE_CAR_IN_PROFILE_SERV_UUID ),
  0x02,
  GAP_ADTYPE_SERVICE_DATA,
  0x88
};



// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "DGJ car in device";

uint8 lock_in_addr[B_ADDR_LEN];
uint8 control_addr[B_ADDR_LEN]={0x88,0x88,0x88,0xC6,0x0E,0x00};

uint8 addr_update_flag=FALSE;//�Ƿ���Ҫ�ӳ��㲥ʱ��
uint8 cur_conn_target=CUR_CONNET_TARGET_CONTROL;//��ǰ�㲥�������Ӷ���

uint8 adv_type=GAP_ADTYPE_ADV_IND;//GAP_ADTYPE_ADV_LDC_DIRECT_IND;
uint8 adv_direct_type=ADDRTYPE_PUBLIC;
uint8 adv_channel=GAP_ADVCHAN_ALL;
uint8 adv_filter=GAP_FILTER_POLICY_ALL;



/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void own_addr_add_1_to_peer_addr(uint8 *p_own_addr,uint8* p_peer_addr);
static void adv_inderct_addr_update();
static void start_info_init();
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );

static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
static void simpleBLEPeripheral_Handlebatt(uint8 batt);


static char *bdAddr2Str ( uint8 *pAddr );


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;

  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    /*
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif
*/
    

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime =5000;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
 
    /*****wkxboot****/
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN,0);//����general�㲥��ʱ�¼� 0 =��Զ���¼�waiting
    
    GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE,sizeof(uint8),&adv_type);
    GAPRole_SetParameter(GAPROLE_ADV_DIRECT_TYPE,sizeof(uint8),&adv_direct_type);
    GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP,sizeof(uint8),&adv_channel);
    GAPRole_SetParameter(GAPROLE_ADV_FILTER_POLICY,sizeof(uint8),&adv_filter);
    //GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR,B_ADDR_LEN,lock_in_addr);
    
    // Set the GAP Role Parameters

    
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
    
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }
  //set tx power
  HCI_EXT_SetTxPowerCmd(LL_EXT_TX_POWER_4_DBM);//4dbm
  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = FALSE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
    /*
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
    */
    
  }

 RegisterForKeys( simpleBLEPeripheral_TaskID );
#if HAL_BATT_CHECK > 0
 register_for_batt(simpleBLEPeripheral_TaskID);
 app_batt_start_periodic_update_info();
#endif 
 
 
#if defined( CC2540_MINIDK )

  SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile

  // Register for all key events - This app will handle all key events

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low

#endif // #if defined( CC2540_MINIDK )

#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  
#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

}

static void start_info_init()
{
  ble_device_t car_in;//����������Ϣ
  uint8 ownAddress[B_ADDR_LEN];
  ble_device_t lock_in;//��λ����Ϣ
  time_stamp_t park_time;//������ʱ�� 
  
  
 if(osal_snv_read(CAR_AUHTORITY_FLAG_ID,1,&car_in.ble_active)==SUCCESS)
  {
   app_write_string("\r\n��snv����Ȩ����Ϣ,�Ѹ���!");
  }
 else
 {
   car_in.ble_active=CAR_HAS_PARKING_AUTHORITY;
   app_write_string("\r\nû����snv����Ȩ����Ϣ,����Ĭ��ֵ!"); 
 }   
 
  advertData[9]=car_in.ble_active;
  GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );//���¹㲥����
  
  GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
  osal_memcpy(car_in.ble_id,ownAddress,B_ADDR_LEN);//�����Լ��ĵ�ַ��
  car_in.ble_battery=100;//100%
//
  lock_in.ble_active=CAR_HAS_PARKING_AUTHORITY;
  lock_in.ble_battery=100;
  osal_memcpy(lock_in.ble_id,lock_in_addr,B_ADDR_LEN);//id
// 
  osal_memset(&park_time,0,sizeof(time_stamp_t));//ʱ��Ĭ��0
  park_time.flag=TIME_FLAG_ENTER;
  
  SimpleProfile_SetParameter( PARAM_BLE_CAR_IN_CHAR, sizeof ( ble_device_t ), &car_in );
  SimpleProfile_SetParameter( PARAM_BLE_LOCK_IN_CHAR, sizeof ( ble_device_t ), &lock_in );
  SimpleProfile_SetParameter( PARAM_BLE_TIME_STAMP_CHAR, sizeof ( time_stamp_t ), &park_time );
}


static void adv_inderct_addr_update()
{
  if(addr_update_flag)//����Ѿ������ӹ������
  {
  addr_update_flag=FALSE;
  GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN,12000);//����general�㲥��ʱ�¼� 0 =��Զ���¼�waiting 12s
  }
  else
  {
  GAP_SetParamValue(TGAP_GEN_DISC_ADV_MIN,1500);//����general�㲥��ʱ�¼� 0 =��Զ���¼�waiting 1.5s 
  }
  
  if(cur_conn_target==CUR_CONNET_TARGET_CONTROL)
  {
  cur_conn_target=CUR_CONNET_TARGET_LOCK_IN;
  //GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR,B_ADDR_LEN,lock_in_addr);
  GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
  app_write_string("\r\n�л���lock in��ַ:");
  app_write_string(bdAddr2Str(lock_in_addr));
  }
 else if(cur_conn_target==CUR_CONNET_TARGET_LOCK_IN)
  {
    cur_conn_target=CUR_CONNET_TARGET_CONTROL; 
   // GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR,B_ADDR_LEN,control_addr);
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    app_write_string("\r\n�л���control��ַ:");
    app_write_string(bdAddr2Str(control_addr));
  }
}
/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
    performPeriodicTask();

    return (events ^ SBP_PERIODIC_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
   case BATT_UPDATE_EVENT:
      simpleBLEPeripheral_Handlebatt(((batt_msg_t *)pMsg)->batt);
      break;
  default:
    // do nothing
    break;
  }
}

static void simpleBLEPeripheral_Handlebatt(uint8 batt)
{
  ble_device_t car_in; 
  SimpleProfile_GetParameter(PARAM_BLE_CAR_IN_CHAR,&car_in );
  car_in.ble_battery=batt;
  SimpleProfile_SetParameter(PARAM_BLE_CAR_IN_CHAR,sizeof(ble_device_t),&car_in );
  app_write_string("\r\n����car in batt ���,��ǰcar in:");
  app_write_string(uint8_to_string(car_in.ble_battery));
}
/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
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
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  (void) shift;
  (void) keys;
  /*
  uint8 SK_Keys = 0;

  VOID shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_1 )
  {
    SK_Keys |= SK_KEY_LEFT;
  }

  if ( keys & HAL_KEY_SW_2 )
  {

    SK_Keys |= SK_KEY_RIGHT;

    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    // Note:  If PLUS_BROADCASTER is define this condition is ignored and
    //        Device may advertise during connections as well. 
#ifndef PLUS_BROADCASTER  
    if( gapProfileState != GAPROLE_CONNECTED )
    {
#endif // PLUS_BROADCASTER
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
#ifndef PLUS_BROADCASTER
    }
#endif // PLUS_BROADCASTER
  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
  */
}


/*����Ŀ��lock in������ַ*/
static void own_addr_add_1_to_peer_addr(uint8 *p_own_addr,uint8* p_peer_addr)
{

 for(uint8 i=0;i<6;i++)
 {
   p_peer_addr[i]=p_own_addr[i];
 }
 for(uint8 i=0;i<6;i++)
 {
 if(p_peer_addr[i]<0xff)
 {
    p_peer_addr[i]+=1;
    for(uint8 j=0;j<i;j++)
    {
      p_peer_addr[j]=0x00;
    }
    break;
 }
}
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
  
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
        
        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        own_addr_add_1_to_peer_addr(ownAddress,lock_in_addr);
        start_info_init();
        app_write_string("\r\n�����ʼ�����!��ַΪ:");
        app_write_string( bdAddr2Str( ownAddress ));
        app_write_string("\r\nĿ��lock in��ַΪ:");
        app_write_string( bdAddr2Str( lock_in_addr));
        app_write_string("\r\nĿ��control��ַΪ:");
        app_write_string( bdAddr2Str( control_addr));
      
        //GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR,B_ADDR_LEN,control_addr);
        //GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
        //GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        /*
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        */
        app_write_string("\r\n��ʼ����㲥...");
        
      }
      break;

    case GAPROLE_CONNECTED:
      {   
        app_write_string("\r\n�豸������!");
        addr_update_flag=TRUE;
        uint8 adv_enabled_status = 0;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &adv_enabled_status); // Turn off Advertising
        /*
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          
#ifdef PLUS_BROADCASTER
        // Only turn advertising on for this state when we first connect
        // otherwise, when we go from connected_advertising back to this state
        // we will be turning advertising back on.
        */
        /*
        if ( first_conn_flag == 0 ) 
        {
          uint8 adv_enabled_status = 1;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &adv_enabled_status); // Turn on Advertising
          first_conn_flag = 1;
        }
#endif // PLUS_BROADCASTER
        */
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      {
        app_write_string("\r\n�豸���Ӻ�Ĺ㲥...");
        /*
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        */
      }
      break;      
    case GAPROLE_WAITING:
      {
        //uint8 adv_enabled_status = 0;
        //GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &adv_enabled_status); // Turn off Advertising
        app_write_string("\r\n�㲥��ͣ,��ַ�л���ȴ�500ms���¿�ʼ�㲥!");
        adv_inderct_addr_update();
        //HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        //app_write_string("\r\n�رչ㲥,��ʼ�л���ַ:");
   
        //GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
        //app_write_string("\r\n�л������㲥");
        //adv_inderct_addr_update();
        //adv_enabled_status = 1;
        //GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &adv_enabled_status); // Turn on Advertising
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        app_write_string("\r\n����ͨ�ų�ʱ!");
        app_write_string("\r\n׼�����¹㲥!");
        /*
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
          
#ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        first_conn_flag = 0;
#endif //#ifdef (PLUS_BROADCASTER)
        */
      }
      break;

    case GAPROLE_ERROR:
      {
        /*
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        */
        app_write_string("\r\n�㲥����!");
      }
      break;

    default:
      {
        /*
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        */
         app_write_string("\r\n��������!");
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{
 
 // uint8 valueToCopy;
 // uint8 stat;

  // Call to retrieve the value of the third characteristic in the profile
  //stat = SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &valueToCopy);

 // if( stat == SUCCESS )
 // {
    /*
     * Call to set that value of the fourth characteristic in the profile. Note
     * that if notifications of the fourth characteristic have been enabled by
     * a GATT client device, then a notification will be sent every time this
     * function is called.
     */
 //   SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof(uint8), &valueToCopy);
 // }

}
/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  ble_device_t new_value;
  time_stamp_t new_time;
  switch( paramID )
  {
    case PARAM_BLE_CAR_IN_CHAR:
      SimpleProfile_GetParameter( PARAM_BLE_CAR_IN_CHAR, &new_value );
     /*
      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
    */
      advertData[9]=new_value.ble_active;
      GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );//���¹㲥����
      osal_snv_write(CAR_AUHTORITY_FLAG_ID,1,&new_value.ble_active);//�����µ���Ȩ��Ϣ
      
      app_write_string("\r\nble_car_in����д!Ŀǰֵ:\r\n");
      uint8_array_to_string((uint8*)&new_value,sizeof(ble_device_t));
      
     
      
      break;

    case PARAM_BLE_LOCK_IN_CHAR:
      SimpleProfile_GetParameter( PARAM_BLE_LOCK_IN_CHAR, &new_value );
/*
      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
*/
      app_write_string("\r\nble_lock_in����д!Ŀǰֵ:\r\n");
      uint8_array_to_string((uint8*)&new_value,sizeof(ble_device_t));
      break;
    case PARAM_BLE_TIME_STAMP_CHAR:
      SimpleProfile_GetParameter( PARAM_BLE_TIME_STAMP_CHAR, &new_time );
/*
      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
*/
      app_write_string("\r\ntime_stamp����д!Ŀǰֵ:\r\n");
      uint8_array_to_string((uint8*)&new_time,sizeof(time_stamp_t));
    default:
      // should not reach here!
      break;
  }
}

//#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
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
//#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

/*********************************************************************
*********************************************************************/
