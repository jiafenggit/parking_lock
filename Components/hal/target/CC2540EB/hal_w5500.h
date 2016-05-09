#ifndef __HAL_W5500_H__
#define __HAL_W5500_H__

#define  SOCKET_TXBUFF_MAX_SIZE       2048
#define  SOCKET_RXBUFF_MAX_SIZE       2048
#define  SOCKET_REGISTER_MAX_OFFSET   0x002F
#define  GENERAL_REGISTER_MAX_OFFSET  0x002E




#define  VBV(v,x)                   ((v)<<(x))

#define  PROTOCOL_MACRAW            4
#define  PROTOCOL_UDP               2
#define  PROTOCOL_TCP               1
#define  PROTOCOL_CLOSE             0

#define  OPMD_HARDWARE              0
#define  OPMD_SOFTWARE              1

#define  OPMDC_10MBT_HALF           0
#define  OPMDC_10MBT_FULL           1
#define  OPMDC_100MBT_HALF          2
#define  OPMDC_100MBT_FULL          3
#define  OPMDC_100MBT_HALF_AUTO     4
#define  OPMDC_NO_USE               5
#define  OPMDC_POWER_DOWN           6
#define  OPMDC_FULL_AUTO            7

#define  SPD_100MBT_BASED             1
#define  SPD_10MBT_BASED              0

#define  DPX_FULL                     1
#define  DPX_HALF                     0



#define  BSB_GENERAL_REG             0 //5 bit [7:3]

#define  BSB_SOCKET0_REG             1
#define  BSB_SOCKET0_TXBUFF          2
#define  BSB_SOCKET0_RXBUFF          3

#define  BSB_SOCKET1_REG             5
#define  BSB_SOCKET1_TXBUFF          6
#define  BSB_SOCKET1_RXBUFF          7


#define  BSB_SOCKET2_REG             9
#define  BSB_SOCKET2_TXBUFF          10
#define  BSB_SOCKET2_RXBUFF          11

#define  BSB_SOCKET3_REG             13
#define  BSB_SOCKET3_TXBUFF          14
#define  BSB_SOCKET3_RXBUFF          15

#define  BSB_SOCKET4_REG             17
#define  BSB_SOCKET4_TXBUFF          18
#define  BSB_SOCKET4_RXBUFF          19

#define  BSB_SOCKET5_REG             21
#define  BSB_SOCKET5_TXBUFF          22
#define  BSB_SOCKET5_RXBUFF          23

#define  BSB_SOCKET6_REG             25
#define  BSB_SOCKET6_TXBUFF          26
#define  BSB_SOCKET6_RXBUFF          27

#define  BSB_SOCKET7_REG             29
#define  BSB_SOCKET7_TXBUFF          30
#define  BSB_SOCKET7_RXBUFF          31


/***定义RWB的读或者写*****/           
#define  RWB_WRITE                   1  //1 bit bit2
#define  RWB_READ                    0
/***定义om的操作字长******/
#define  OPT_MODE_VDM                0  //2 bits [0:1]
#define  OPT_MODE_FDM1               1
#define  OPT_MODE_FDM2               2
#define  OPT_MODE_FDM4               3

/***定义socket的命令序列******/
#define SOCKET_CMD_OPEN              0x01
#define SOCKET_CMD_LISTEN            0x02
#define SOCKET_CMD_CONNECT           0x04
#define SOCKET_CMD_DISCON            0x08
#define SOCKET_CMD_CLOSE             0x10
#define SOCKET_CMD_SEND              0x20
#define SOCKET_CMD_SEND_MAC          0x21
#define SOCKET_CMD_SEND_KEEP         0x22
#define SOCKET_CMD_RECV              0x40

/***定义socket 的标号******/
#define SOCKET0                      0
#define SOCKET1                      1
#define SOCKET2                      2
#define SOCKET3                      3
#define SOCKET4                      4
#define SOCKET5                      5
#define SOCKET6                      6
#define SOCKET7                      7
#define ALL_SOCKETS                  0xff

/***定义socket的状态值******/
#define SOCKET_CLOSED                0x00
#define SOCKET_INIT                  0x13
#define SOCKET_LISTEN                0x14
#define SOCKET_ESTABLISHED           0x17
#define SOCKET_CLOSE_WAIT            0x1C
#define SOCKET_UDP                   0x22


/******定义socket中断的类型位移******/
#define  SEND_OK                    BV(4)
#define  TIME_OUT                   BV(3)
#define  RECV                       BV(2)
#define  DIS_CONN                   BV(1)
#define  CONNECTED                  BV(0)


/******定义寄存器偏移地址*********/

#define GENERAL_REGISTER_MODE_OFFSET          0x0000
#define GENERAL_REGISTER_GATEWAY_OFFSET       0x0001
#define GENERAL_REGISTER_SUBMASK_OFFSET       0x0005
#define GENERAL_REGISTER_SRCMAC_OFFSET        0x0009
#define GENERAL_REGISTER_SRCIP_OFFSET         0x000F
#define GENERAL_REGISTER_INTLEVEL_OFFSET      0x0013
#define GENERAL_REGISTER_INT_OFFSET           0x0015
#define GENERAL_REGISTER_INTMASK_OFFSET       0x0016
#define GENERAL_REGISTER_SOCKETINT_OFFSET     0x0017
#define GENERAL_REGISTER_SOCKETINTMASK_OFFSET 0x0018
#define GENERAL_REGISTER_RETRYTIME_OFFSET     0x0019
#define GENERAL_REGISTER_RETRYCNT_OFFSET      0x001B
#define GENERAL_REGISTER_UNREACHIP_OFFSET     0x0028
#define GENERAL_REGISTER_UNREACHPORT_OFFSET   0x002C
#define GENERAL_REGISTER_PHYCFG_OFFSET        0x002E

//socket register
#define SOCKET_REGISTER_MODE_OFFSET           0x0000
#define SOCKET_REGISTER_CMD_OFFSET            0x0001
#define SOCKET_REGISTER_INT_OFFSET            0x0002
#define SOCKET_REGISTER_STATUS_OFFSET         0x0003
#define SOCKET_REGISTER_SRCPORTNUM_OFFSET     0x0004
#define SOCKET_REGISTER_DESTMAC_OFFSET        0x0006
#define SOCKET_REGISTER_DESTIP_OFFSET         0x000C
#define SOCKET_REGISTER_DESTPORT_OFFSET       0x0010
#define SOCKET_REGISTER_MTU_OFFSET            0x0012
#define SOCKET_REGISTER_TOS_OFFSET            0x0015
#define SOCKET_REGISTER_TTL_OFFSET            0x0016
#define SOCKET_REGISTER_RXBUFSIZE_OFFSET      0x001E
#define SOCKET_REGISTER_TXBUFSIZE_OFFSET      0x001F
#define SOCKET_REGISTER_TXFREESIZE_OFFSET     0x0020
#define SOCKET_REGISTER_TXR_OFFSET            0x0022
#define SOCKET_REGISTER_TXW_OFFSET            0x0024
#define SOCKET_REGISTER_RXUSERDSIZE_OFFSET    0x0026
#define SOCKET_REGISTER_RXR_OFFSET            0x0028
#define SOCKET_REGISTER_RXW_OFFSET            0x002A
#define SOCKET_REGISTER_INTMASK_OFFSET        0x002C
#define SOCKET_REGISTER_FRAG_OFFSET           0x002D
#define SOCKET_REGISTER_KEEPLIVE_OFFSET       0x002F

/***********定义通用寄存器内功能元素位置******************/
#define GRMODE_RST_POS             BV(7)
#define GRMODE_WOL_POS             BV(5)
#define GRMODE_PB_POS              BV(4)
#define GRMODE_PPPOE_POS           BV(3)
#define GRMODE_FARP_POS            BV(1)

#define GRINT_CONFLICT_POS         BV(7)
#define GRINT_UNREACH_POS          BV(6)
#define GRINT_PPPOE_POS            BV(5)
#define GRINT_MP_POS               BV(4)

#define GRINTMASK_IPCONFLICT_POS   BV(7)
#define GRINTMASK_UNREACH_POS      BV(6)
#define GRINTMASK_PPPOE_POS        BV(5)
#define GRINTMASK_MP_POS           BV(4)

#define GRSOCKETINT_S7_POS         BV(7)
#define GRSOCKETINT_S6_POS         BV(6)
#define GRSOCKETINT_S5_POS         BV(5)
#define GRSOCKETINT_S4_POS         BV(4)
#define GRSOCKETINT_S3_POS         BV(3)
#define GRSOCKETINT_S2_POS         BV(2)
#define GRSOCKETINT_S1_POS         BV(1)
#define GRSOCKETINT_S0_POS         BV(0)

#define GRSOCKETINTMASK_S7_POS     BV(7)
#define GRSOCKETINTMASK_S6_POS     BV(6)
#define GRSOCKETINTMASK_S5_POS     BV(5)
#define GRSOCKETINTMASK_S4_POS     BV(4)
#define GRSOCKETINTMASK_S3_POS     BV(3)
#define GRSOCKETINTMASK_S2_POS     BV(2)
#define GRSOCKETINTMASK_S1_POS     BV(1)
#define GRSOCKETINTMASK_S0_POS     BV(0)

#define GRPHYCFG_RST_POS           BV(7)
#define GRPHYCFG_OPMDHARD_POS              VBV(OPMD_HARDWARE,6)
#define GRPHYCFG_OPMDSOFT_POS              VBV(OPMD_SOFTWARE,6)
#define GRPHYCFG_OPMDC10MBTHALF_POS        VBV(OPMDC_10MBT_HALF,3)
#define GRPHYCFG_OPMDC10MBTFULL_POS        VBV(OPMDC_10MBT_FULL,3)
#define GRPHYCFG_OPMDC100MBTHALF_POS       VBV(OPMDC_100MBT_HALF,3)
#define GRPHYCFG_OPMDC100MBTFULL_POS       VBV(OPMDC_100MBT_FULL,3)
#define GRPHYCFG_OPMDC100MBTHALFAUTO_POS   VBV(OPMDC_100MBT_HALF,3)
#define GRPHYCFG_OPMDCNOUSE_POS            VBV(OPMDC_NO_USE,3)
#define GRPHYCFG_OPMDCPOWERDOWN_POS        VBV(OPMDC_POWER_DOWN,3)
#define GRPHYCFG_OPMDCFULLAUTO_POS         VBV(OPMDC_FULL_AUTO,3)
#define GRPHYCFG_DPX_POS           BV(2)
#define GRPHYCFG_SPD_POS           BV(1)
#define GRPHYCFG_LNK_POS           BV(0)

/***********定义SOCKET寄存器内功能元素位置******************/
#define SRMODE_UDPMULTI_POS        BV(7)
#define SRMODE_UDPBCASTB_POS       BV(6)
#define SRMODE_NDACK_POS           BV(5)
#define SRMODE_UDPUCASTB_POS       BV(4)
#define SRMODE_PROTOCOLCLOSE_POS   VBV(PROTOCOL_CLOSE,0)
#define SRMODE_PROTOCOLUDP_POS     VBV(PROTOCOL_UDP,0)
#define SRMODE_PROTOCOLTCP_POS     VBV(PROTOCOL_TCP,0)
#define SRMODE_PROTOCOLMACRAW_POS  VBV(PROTOCOL_MACRAW,0)

#define SRINT_SENDOK_POS           BV(4)
#define SRINT_TIMEOUT_POS          BV(3)
#define SRINT_RECV_POS             BV(2)
#define SRINT_DISCONN_POS          BV(1)
#define SRINT_CONN_POS             BV(0)

#define SRINTMASK_SENDOK_POS       BV(4)
#define SRINTMASK_TIMEOUT_POS      BV(3)
#define SRINTMASK_RECV_POS         BV(2)
#define SRINTNASK_DISCONN_POS      BV(1)
#define SRINTMASK_CONN_POS         BV(0)

#define MAX_BSB_NUM               ((MAX_SOCKET_NUM*4)+3)


#define PARAM_SRC_IP               1
#define PARAM_SRC_PORT             2
#define PARAM_SRC_MAC              3
#define PARAM_SUB_MASK             4
#define PARAM_GATE_WAY             5
#define PARAM_RETRY_TIME           6
#define PARAM_RETRY_CNT            7
#define PARAM_INT_MASK             8
#define PARAM_DST_IP               9
#define PARAM_DST_PORT             10
#define PARAM_DST_MAC              11
#define PARAM_SOCKET_PROTOCOL      12
#define PARAM_SOCKET_INT           13
#define PARAM_SOCKET_CALLBACK      14
#define PARAM_SOCKET_MTU           15








#define DEFAULT_UDP_RECV_HDR_SIZE      8 //ip+port+check

#define NETDEV_NOT_LINKED              1
#define NETDEV_LINKED                  2


#define DEFAULT_SOCKET_NUM             2
#define MAX_SOCKET_NUM                 8





typedef void (*socket_recv_callback_t)(uint16 recv_len);
typedef void (*socket_int_event_callback_t)(uint8 new_event);
typedef void (*socket_status_change_callback_t)(uint8 new_status);
typedef void (*sys_int_event_callback_t)(uint8 new_event);
typedef void (*sys_status_change_callback_t)(uint8 new_status);


typedef struct hal_socket_recv_callback
{
  socket_recv_callback_t pf_socket_recv_cb;
  uint8* ptr_buffer;
  uint16 buffer_size;  
}hal_socket_recv_callback_t;


typedef struct _hal_socket_callback
{
  hal_socket_recv_callback_t      recv_callback;
  socket_int_event_callback_t     pf_int_callback;//socket int
  socket_status_change_callback_t pf_status_change_callback;//socket status
  sys_int_event_callback_t     pf_sys_int_callback;
  sys_status_change_callback_t pf_sys_status_change_callback;
  uint8 socket_status;
}hal_socket_callback_t;



uint8 hal_net_device_set_param(uint8 socket,uint8 param,uint8 *ptr_value);
uint8 hal_net_device_send(uint8 socket,uint8 *ptr_buff,uint16 len);

uint8 hal_net_device_send_cmd(uint8 socket,uint8 s_cmd);
uint8 hal_net_device_get_link_status();

void hal_net_device_poll_device_int();
void hal_net_device_poll_device_status();
void hal_net_device_register_socket_callback(uint8 socket,hal_socket_callback_t *cb);
uint8 hal_net_device_send_to(uint8 socket,uint8* ptr_dst_ip,uint8* ptr_dst_port, uint8* ptr_buff,uint16 len);
 void hal_net_device_software_reset();
#endif