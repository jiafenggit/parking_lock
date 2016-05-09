#ifndef __DHCP_H__ 
#define __DHCP_H__

#include"hal_types.h"
#include"hal_defs.h"


#define DHCP_DEBUG            1

#define DHCP_SOCKET           SOCKET0

#define  DHCP_STATE_IDLE        1
#define  DHCP_STATE_DISCOVER    2
#define  DHCP_STATE_REQUEST     3
#define  DHCP_STATE_COMPLETE    4


#define  DISCOVER_MSG_ID        0x11223344
#define  OFFER_MSG_ID           0x11223345
#define  REQUEST_MSG_ID         0x11223346
#define  DECLINE_MSG_ID         0x11223347
#define  RELEASE_MSG_ID         0x11223348

#define  DHCP_BOOT_REQUEST      1
#define  DHCP_BOOT_REPLY        2

#define  DHCP_SEND_TRY          2 //s
#define  DHCP_REPLY_TIMEOUT     2500 //2.5s
typedef enum options_type
{
DHCP_MSG_TYPE =          53,
DHCP_SERVER_ID=          54,
DHCP_IP_LEASE_TIME=      51,
DHCP_RENEWAL_TIME=       58,
DHCP_REBINDING_TIME=     59,
DHCP_SUBMASK_ID  =        1,
DHCP_ROUTER_IP   =        3,
DHCP_CLIENT_ID   =       61,
DHCP_DNS_ID      =        6,
DHCP_REQUEST_PARAM=       55,
DHCP_REQUEST_IP   =       50,
DHCP_HOST_NAME    =       12,
DHCP_END_OPTION   =      255

}options_type_t;

typedef enum opt_msg_type
{
DHCP_MSG_TYPE_DISCOVER=    1,
DHCP_MSG_TYPE_OFFER   =    2,
DHCP_MSG_TYPE_REQUEST =    3,
DHCP_MSG_TYPE_DECLINE =    4,
DHCP_MSG_TYPE_ACK     =    5,
DHCP_MSG_TYPE_NACK    =    6,
DHCP_MSG_TYPE_RELEASE =    7,
DHCP_MSG_TYPE_INVALID =    0xff
}opt_msg_type_t;

typedef struct dhcp_msg
{
  uint8  op;
  uint8  htype;
  uint8  hlen;
  uint8  hops;
  uint32 msg_id;
  uint16 secs;
  uint16 flags;
  uint8  ciaddr[4];
  uint8  yiaddr[4];
  uint8  siaddr[4];
  uint8  giaddr[4];
  uint8  chaddr[16];
  uint8  s_name[64];//64
  uint8  file[128];//128
  uint8  options[50];//312
}dhcp_msg_t;


void dhcp_callback_register();
void dhcp_poll_status_procedure(uint16 elapse_time);
















#endif