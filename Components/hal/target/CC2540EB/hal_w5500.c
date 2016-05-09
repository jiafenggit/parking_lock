#include"iocc2541.h"

#include"hal_defs.h"
#include"hal_types.h"
#include"hal_w5500.h"
#include"hal_io_spi.h"
#include"error.h"
#include"dhcp.h"
#include"app_uart_init.h"

/******** local func*************/
static uint8 w5500_trans_data(uint8 bsb,uint16 offset,uint8 *ptr_data,uint16 data_len);
static uint8 w5500_recv_data(uint8 bsb,uint16 offset,uint8 *ptr_buff,uint16 data_len);
static uint8 w5500_set_socket_register_value(uint8 socket,uint16 offset,uint8 *ptr_value,uint16 value_len);
static uint8 w5500_get_socket_register_value(uint8 socket,uint16 offset,uint8 *ptr_value,uint16 value_len);
static uint8 w5500_set_general_register_value(uint16 offset,uint8 *ptr_value,uint16 value_len);
static uint8 w5500_get_general_register_value(uint16 offset,uint8 *ptr_value,uint16 value_len);

static uint8 hal_net_device_set_txbuffer_value(uint8 socket,uint16 offset,uint8 *ptr_value,uint16 value_len);
static uint8 hal_net_device_get_rxbuffer_value(uint8 socket,uint16 offset,uint8 *ptr_value,uint16 value_len);

static uint8 hal_net_device_get_tx_free_size(uint8 socket,uint16 *ptr_free_size);
static uint8 hal_net_device_get_tx_wptr(uint8 socket,uint16 *ptr_wptr);

static uint8 hal_net_device_get_rx_recv_size(uint8 socket ,uint16 *ptr_recv_size);
static uint8 hal_net_device_get_rx_rptr(uint8 socket,uint16 *ptr_rptr);
static uint8 hal_net_device_set_rx_rptr(uint8 socket,uint16 ptr_rptr);

static uint8 hal_net_device_handle_socket_recv_event(uint8 socket);
static void hal_net_device_handle_socket_status(uint8 socket,uint8 status);
static void hal_net_device_handle_link_status_change(uint8 new_status);
static void hal_net_device_set_link_status(uint8 new_status);

static uint8 hal_net_device_get_socket_status(uint8 socket);
static void hal_net_device_set_socket_status(uint8 socket,uint8 new_status);


static void hal_net_device_poll_sys_int();
static void hal_net_device_poll_socket_int();


extern volatile uint8 uart_trans_state;
extern volatile uint8 uart_recv_state;

uint8 hal_net_device_link_status=NETDEV_NOT_LINKED;
hal_socket_callback_t socket_cb_array[DEFAULT_SOCKET_NUM];


#define w5500_select()   SPI_CS_SELECT()
#define w5500_deselect() SPI_CS_DESELECT()



static uint8 w5500_trans_data(uint8 bsb,uint16 offset,uint8 *ptr_data,uint16 data_len)
{
 uint8 trans_hdr[3];
 if(ptr_data)
 {
  w5500_select();
 //ww5500 addr
 trans_hdr[0]=offset>>8;
 trans_hdr[1]=offset;
 //w5500 bsb
 trans_hdr[2]=bsb<<3|RWB_WRITE<<2|OPT_MODE_VDM;
 
 hal_spi_trans(trans_hdr,3);
 hal_spi_trans(ptr_data,data_len);
 w5500_deselect();
 return SUCCESS;
 }
 return ADDR_NULL_ERROR;
}


static uint8 w5500_recv_data(uint8 bsb,uint16 offset,uint8 *ptr_buff,uint16 data_len)
{
 uint8 trans_hdr[3];
 if(ptr_buff)
 {
 w5500_select();
  //ww5500 addr
 trans_hdr[0]=offset>>8;
 trans_hdr[1]=offset;
 //w5500 bsb
 trans_hdr[2]=bsb<<3|RWB_READ<<2|OPT_MODE_VDM;

 hal_spi_trans(trans_hdr,3);
 hal_spi_recv(ptr_buff,data_len);
 w5500_deselect();
 
 return SUCCESS;
 }
 return ADDR_NULL_ERROR;
}





static uint8 hal_net_device_set_txbuffer_value(uint8 socket,uint16 offset,uint8 *ptr_value,uint16 value_len)
{
  uint8 status;
  if(socket>MAX_BSB_NUM)//max bsb num=max_socket_num*4+3
 {
  return SOCKET_INVALID_ERROR;
  }
 if(ptr_value==NULL)
 {
   return ADDR_NULL_ERROR;
 }
 status=w5500_trans_data(BSB_SOCKET0_TXBUFF+4*socket,offset,ptr_value,value_len);
 
 return status;
  
}
static uint8 hal_net_device_get_rxbuffer_value(uint8 socket,uint16 offset,uint8 *ptr_value,uint16 value_len)
{
  uint8 status;
 if(socket>MAX_BSB_NUM)//max bsb num=max_socket_num*4+3
 {
  return SOCKET_INVALID_ERROR;
  }
 if(ptr_value==NULL)
 {
   return ADDR_NULL_ERROR;
 }
 status=w5500_recv_data(BSB_SOCKET0_RXBUFF+4*socket,offset,ptr_value,value_len);
 return status; 
}


static uint8 w5500_set_socket_register_value(uint8 socket,uint16 offset,uint8 *ptr_value,uint16 value_len)
{
  uint8 status;
 if(socket>MAX_BSB_NUM)//max bsb num=max_socket_num*4+3
 {
  return SOCKET_INVALID_ERROR;
  }
 if(offset>SOCKET_REGISTER_MAX_OFFSET)
 {
   return OFFSET_INVALID_ERROR;
 }
 if(ptr_value==NULL)
 {
   return ADDR_NULL_ERROR;
 }
status=w5500_trans_data(BSB_SOCKET0_REG+4*socket,offset,ptr_value,value_len);
 
 return status;
}


static uint8 w5500_get_socket_register_value(uint8 socket,uint16 offset,uint8 *ptr_value,uint16 value_len)
{
  uint8 status;
  if(socket>MAX_BSB_NUM)//max bsb num=max_socket_num*4+3
 {
  return SOCKET_INVALID_ERROR;
  }
  if(offset>SOCKET_REGISTER_MAX_OFFSET)
 {
   return OFFSET_INVALID_ERROR;
 }
 if(ptr_value==NULL)
 {
   return ADDR_NULL_ERROR;
 }
 status=w5500_recv_data(BSB_SOCKET0_REG+4*socket,offset,ptr_value,value_len);
 return status;
}

static uint8 w5500_set_general_register_value(uint16 offset,uint8 *ptr_value,uint16 value_len)
{
  uint8 status;
 if(offset>GENERAL_REGISTER_MAX_OFFSET)
 {
   return OFFSET_INVALID_ERROR;
 }
 if(ptr_value==NULL)
 {
   return ADDR_NULL_ERROR;
 }
 
 status=w5500_trans_data(BSB_GENERAL_REG,offset,ptr_value,value_len);
 return status;
}


static uint8 w5500_get_general_register_value(uint16 offset,uint8 *ptr_value,uint16 value_len)
{
  uint8 status;
  if(offset>GENERAL_REGISTER_MAX_OFFSET)
 {
   return OFFSET_INVALID_ERROR;
 }
 if(ptr_value==NULL)
 {
   return ADDR_NULL_ERROR;
 }
  status=w5500_recv_data(BSB_GENERAL_REG,offset,ptr_value,value_len);
  return status;
}


 void hal_net_device_software_reset()
{
 
   uint8 value;
   uint8 i=100;//delay 
   
   hal_spi_init();
   uint8 reset=GRMODE_RST_POS;
   w5500_set_general_register_value(GENERAL_REGISTER_PHYCFG_OFFSET,&reset,1); 
   while(i--);

   hal_spi_init();
  
  value=GRMODE_PB_POS|GRMODE_FARP_POS;//ping and arp force
  w5500_set_general_register_value(GENERAL_REGISTER_MODE_OFFSET,&value,1);
  
  value=GRINTMASK_IPCONFLICT_POS|GRINTMASK_UNREACH_POS;//int confict unreach
  w5500_set_general_register_value(GENERAL_REGISTER_INTMASK_OFFSET,&value,1);
  
  value=GRPHYCFG_OPMDCFULLAUTO_POS|GRPHYCFG_OPMDSOFT_POS|GRPHYCFG_RST_POS;//auto full dpx
  w5500_set_general_register_value(GENERAL_REGISTER_PHYCFG_OFFSET,&value,1);   
}


uint8 hal_net_device_set_param(uint8 socket,uint8 param,uint8 *ptr_value)
 { 
  uint8 value;
  uint8 status=PARAM_INVALID_ERROR;
 switch(param)
{
  /***** general param***********/
 case PARAM_SRC_IP:
 {
  VOID socket;  
  status=w5500_set_general_register_value(GENERAL_REGISTER_SRCIP_OFFSET,ptr_value,4);
 }
  break;
  case PARAM_SRC_PORT:
 { 
  status=w5500_set_socket_register_value(socket,SOCKET_REGISTER_SRCPORTNUM_OFFSET,ptr_value,2);
 }
  break;
  case PARAM_SRC_MAC:
 {
   VOID socket;  
   status=w5500_set_general_register_value(GENERAL_REGISTER_SRCMAC_OFFSET,ptr_value,6);
 }
  break;
  case PARAM_SUB_MASK:
 {
   VOID socket;  
   status=w5500_set_general_register_value(GENERAL_REGISTER_SUBMASK_OFFSET,ptr_value,4);
 }
  break;
  case PARAM_GATE_WAY:
 {
   VOID socket;  
  status=w5500_set_general_register_value(GENERAL_REGISTER_GATEWAY_OFFSET,ptr_value,4);
 }
  break;
 case PARAM_RETRY_TIME:
 {
   VOID socket;  
   status=w5500_set_general_register_value(GENERAL_REGISTER_RETRYTIME_OFFSET,ptr_value,2); 
 }
  break;
 case PARAM_RETRY_CNT:
 {
   VOID socket;  
   status=w5500_set_general_register_value(GENERAL_REGISTER_RETRYCNT_OFFSET,ptr_value,1);
 }
  break;  
 case PARAM_INT_MASK:
   {
   status=w5500_get_general_register_value(GENERAL_REGISTER_SOCKETINTMASK_OFFSET,&value,1);
   value|=BV(socket);
   status=w5500_set_general_register_value(GENERAL_REGISTER_SOCKETINTMASK_OFFSET,&value,1);  
   }
  break;
  /***** socket param***********/
  case PARAM_DST_IP:
 {
   status=w5500_set_socket_register_value(socket,SOCKET_REGISTER_DESTIP_OFFSET,ptr_value,4);
   
   uint8 ip[4];
   w5500_get_socket_register_value(socket,SOCKET_REGISTER_DESTIP_OFFSET,ip,4);
   app_write_string("dst ip:");
   for(uint8 i=0;i<4;i++)
   app_write_string(uint8_to_string(ip[i]));

 }
  break;
  case PARAM_DST_PORT:
 {
   status=w5500_set_socket_register_value(socket,SOCKET_REGISTER_DESTPORT_OFFSET,ptr_value,2);
 }
  break;
  case PARAM_DST_MAC:
 {
   status=w5500_set_socket_register_value(socket,SOCKET_REGISTER_DESTMAC_OFFSET,ptr_value,6);
 }
  break;
  case PARAM_SOCKET_PROTOCOL:
 {
   status=w5500_set_socket_register_value(socket,SOCKET_REGISTER_MODE_OFFSET,ptr_value,1);
 }
  break;
 
 case PARAM_SOCKET_INT:
 {
   status=w5500_set_socket_register_value(socket,SOCKET_REGISTER_INTMASK_OFFSET,ptr_value,1);
 }
  break;
 case PARAM_SOCKET_CALLBACK:
 {
 socket_cb_array[socket]=*((hal_socket_callback_t*)ptr_value);
 status=SUCCESS;
 }
  break;
 case PARAM_SOCKET_MTU:
 {
  status=w5500_set_socket_register_value(socket,SOCKET_REGISTER_MTU_OFFSET,ptr_value,2);
 }
  break;
  default:
  break;
 }
 
 return status;
}


uint8 hal_net_device_send_cmd(uint8 socket,uint8 s_cmd)
{
 bool valid=FALSE;
 uint8 status;
 uint8 cmd=s_cmd;
 switch(cmd)
 {
 case SOCKET_CMD_OPEN:
   valid=TRUE;
   break;
 case SOCKET_CMD_LISTEN:
   valid=TRUE;
   break;
  case SOCKET_CMD_CONNECT:
    valid=TRUE;
   break;
 case SOCKET_CMD_DISCON:
   valid=TRUE;
   break;
 case SOCKET_CMD_CLOSE:
   valid=TRUE;
   break;
 case SOCKET_CMD_SEND:
   valid=TRUE;
   break;
 case SOCKET_CMD_SEND_MAC:
   valid=TRUE;
   break;
 case SOCKET_CMD_SEND_KEEP:
   valid=TRUE;
   break;
 case SOCKET_CMD_RECV:
   valid=TRUE;
   break;
 default:
   break;  
 }
  if(valid)
  status=w5500_set_socket_register_value(socket,SOCKET_REGISTER_CMD_OFFSET,&cmd,1); 
  else
  status=CMD_INVALID_ERROR;
 return status;
}

static uint8 hal_net_device_get_tx_free_size(uint8 socket,uint16 *ptr_free_size)
{
  uint16 free_size=0;
  uint8 buffer[2];
  uint8 status; 
  if(ptr_free_size)
  {
  status =w5500_get_socket_register_value(socket,SOCKET_REGISTER_TXFREESIZE_OFFSET,buffer,2);
  free_size=buffer[0];
  free_size<<=8;
  free_size+=buffer[1];
  
  *ptr_free_size=free_size;
  return status; 
  }
  return ADDR_NULL_ERROR;
}

static uint8 hal_net_device_get_tx_wptr(uint8 socket,uint16 *ptr_wptr)//读取发送缓存的写指针 store in w5500_recv.buffer[0][1]
{
  uint16 ptr=0;
  uint8 buffer[2];
  uint8 status;
  if(ptr_wptr)
  {
  status=w5500_get_socket_register_value(socket,SOCKET_REGISTER_TXW_OFFSET,buffer,2);
  ptr=buffer[0];
  ptr<<=8;
  ptr+=buffer[1];
  
  *ptr_wptr=ptr;
  return status;
  }
  return ADDR_NULL_ERROR;
}
static uint8 hal_net_device_set_tx_wptr(uint8 socket,uint16 ptr)//写入发送缓存的写指针
{
  uint8 buffer[2];
  uint8 status;
  
  buffer[0]=ptr>>8;
  buffer[1]=ptr;
  status=w5500_set_socket_register_value(socket,SOCKET_REGISTER_TXW_OFFSET,buffer,2);
  
  return status;
}


static uint8 hal_net_device_get_rx_recv_size(uint8 socket ,uint16 *ptr_recv_size)//读取接收缓存的缓存长度
{
  uint16 recv_size=0;
  uint8 buffer[2];
  uint8 status;
  if(ptr_recv_size)
  {
  status=w5500_get_socket_register_value(socket,SOCKET_REGISTER_RXUSERDSIZE_OFFSET,buffer,2);
  recv_size=buffer[0];
  recv_size<<=8;
  recv_size+=buffer[1];
  *ptr_recv_size=recv_size;
  
  return status;
  }
  
  return ADDR_NULL_ERROR;
}

static uint8 hal_net_device_get_rx_rptr(uint8 socket,uint16 *ptr_rptr)//读取接收缓存的读指针
{ 
  uint16 ptr=0;
  uint8 buffer[2];
  uint8 status;
  if(ptr_rptr)
  {
  status=w5500_get_socket_register_value(socket,SOCKET_REGISTER_RXR_OFFSET,buffer,2);
  ptr=buffer[0];
  ptr<<=8;
  ptr+=buffer[1];
  *ptr_rptr=ptr;
  
  return status;
  }
  return ADDR_NULL_ERROR;
}


static uint8 hal_net_device_set_rx_rptr(uint8 socket,uint16 ptr)//写入接收缓存的读指针
{
  uint8 buffer[2];
  uint8 status;
  buffer[0]=ptr>>8;
  buffer[1]=ptr;
  status=w5500_set_socket_register_value(socket,SOCKET_REGISTER_RXR_OFFSET,buffer,2);
  
  return status;
}

uint8 hal_net_device_send_to(uint8 socket,uint8* ptr_dst_ip,uint8* ptr_dst_port, uint8* ptr_buff,uint16 len)
{
  uint8 value;
  uint8 status;
  uint32 *addr;

  status=w5500_get_socket_register_value(socket,SOCKET_REGISTER_MODE_OFFSET,&value,1);
  if(status!=SUCCESS)
    return status;
  if((value&0x0f)!=PROTOCOL_UDP)
  return NOT_MATCHED_ERROR;
  
  status=w5500_get_socket_register_value(socket,SOCKET_REGISTER_STATUS_OFFSET,&value,1);
  if(status!=SUCCESS)
    return status;
  if(value!=SOCKET_UDP)
  return NOT_INIT_ERROR;
  addr=(uint32*)ptr_dst_ip;
  if(*addr==0)
  return PARAM_INVALID_ERROR;
  status=hal_net_device_set_param(socket,PARAM_DST_IP,ptr_dst_ip);
  {
   uint8 ip[4];
   w5500_get_socket_register_value(socket,SOCKET_REGISTER_DESTIP_OFFSET,ip,4);
   app_write_string("send dst ip:");
   for(uint8 i=0;i<4;i++)
   app_write_string(uint8_to_string(ip[i]));
  }
  if(status!=SUCCESS)
  return status;
  status=hal_net_device_set_param(socket,PARAM_DST_PORT,ptr_dst_port);
  {
    uint8 port[2];
   w5500_get_socket_register_value(socket,SOCKET_REGISTER_DESTPORT_OFFSET,port,2);
   app_write_string("send dst port:");
   for(uint8 i=0;i<2;i++)
   app_write_string(uint8_to_string(port[i]));
  }
  if(status!=SUCCESS)
  return status;
  status=hal_net_device_send(socket,ptr_buff,len);
  
 return status; 
}

uint8 hal_net_device_send(uint8 socket,uint8 *ptr_buff,uint16 len)//tcp send data
{
  uint16 free_size;
  uint16 cur_tx_wptr=0;
  uint8 status;
  status=hal_net_device_get_tx_wptr(socket,&cur_tx_wptr);
  if(status!=SUCCESS)
  {
    return status;
  }
   status=hal_net_device_get_tx_free_size(socket,&free_size);
  if(status!=SUCCESS)
  {
    return status;
  }
  if(free_size<len)
  {
    return NO_MEM_ERROR;
  }
  status=hal_net_device_set_txbuffer_value(socket,cur_tx_wptr,ptr_buff,len);
  cur_tx_wptr+=len;//overflow auto 16bit 0xffff
  if(status!=SUCCESS)
  {
    return status;
  }
  status=hal_net_device_set_tx_wptr(socket,cur_tx_wptr);
   if(status!=SUCCESS)
  {
    return status;
  }
  status=hal_net_device_send_cmd(socket,SOCKET_CMD_SEND);

  return status;
  
}



static void hal_net_device_handle_socket_int_event(uint8 socket,uint8 int_events)
{
  if(int_events&RECV)
  {
    app_write_string("\r\nhal socket recv data!");  
    hal_net_device_handle_socket_recv_event(socket);  
  }
  if(int_events&SEND_OK)
  {
    app_write_string("\r\nhal socket send ok!");  
  }
  if(int_events&TIME_OUT)
  {
     app_write_string("\r\nhal socket send timeout!");  
  }
  if(int_events&DIS_CONN)
  {
    app_write_string("\r\nhal socket disconnect!");  
  }
  if(int_events&CONNECTED)
  {
     app_write_string("\r\nhal socket connected!");  
  }
  
  if(socket_cb_array[socket].pf_int_callback)
  socket_cb_array[socket].pf_int_callback(int_events);
}

static uint8 hal_net_device_handle_socket_recv_event(uint8 socket)//处理接收缓存中的数据
{
  uint16 cur_rx_rptr=0;
  uint16 recv_size=0;
  uint16 recv_temp_size;
  uint16 buffer_size;
  uint8* ptr_buffer;
  uint8 status;
  if(socket>DEFAULT_SOCKET_NUM)
  return SOCKET_INVALID_ERROR;
  
  ptr_buffer=socket_cb_array[socket].recv_callback.ptr_buffer;
  if(ptr_buffer==NULL)
  return ADDR_NULL_ERROR;
  
  buffer_size=socket_cb_array[socket].recv_callback.buffer_size;
  status=hal_net_device_get_rx_recv_size(socket,&recv_size);
  
  if(status!=SUCCESS)
  {
    return status;
  }
  status=hal_net_device_get_rx_rptr(socket,&cur_rx_rptr);
  if(status!=SUCCESS)
  {
    return status;
  }
  
  while(recv_size)
  {
   if(recv_size>buffer_size)//
   { 
     recv_temp_size=buffer_size;
     recv_size-=buffer_size;
   }
   else
   {
     recv_temp_size=recv_size;
     recv_size=0;
   }
   status=hal_net_device_get_rxbuffer_value(socket,cur_rx_rptr,ptr_buffer,recv_temp_size);
   if(status!=SUCCESS)
   {
     return status;
   }
  //call recv callback
  if(socket_cb_array[socket].recv_callback.pf_socket_recv_cb)
  {
  socket_cb_array[socket].recv_callback.pf_socket_recv_cb(recv_temp_size);
  }
  cur_rx_rptr+=recv_temp_size;//overflow auto 16bit 0xffff rx的读指针更新
  }
  
  status=hal_net_device_set_rx_rptr(socket,cur_rx_rptr);//写回接收缓存的读指针 与接收缓存写指针一致
  if(status!=SUCCESS)
  {
    return status;
  }
  status=hal_net_device_send_cmd(socket,SOCKET_CMD_RECV);//发送接收命令
  if(status!=SUCCESS)
  {
    return status;
  } 
  return SUCCESS;
}


static void  hal_net_device_poll_socket_int()//处理中断例程
{
  uint8 int_events;
  uint8 status; 
  
 status=w5500_get_general_register_value(GENERAL_REGISTER_SOCKETINT_OFFSET,&int_events,1);
 if(status==SUCCESS && int_events!=0)
 {   
 w5500_set_general_register_value(GENERAL_REGISTER_SOCKETINT_OFFSET,&int_events,1);//立即清除所有中断标志  
 for(uint8 socket=SOCKET0;socket<DEFAULT_SOCKET_NUM;socket++)
 { 
   status=w5500_get_socket_register_value(socket,SOCKET_REGISTER_INT_OFFSET,&int_events,1);//读取中断值
 if(status==SUCCESS && int_events!=0)
  {
   w5500_set_socket_register_value(socket,SOCKET_REGISTER_INT_OFFSET,&int_events,1); //立即清除所有中断标志
   hal_net_device_handle_socket_int_event(socket,int_events);
   app_write_string("\r\nsocket int!socket num:");
   app_write_string(uint8_to_string(socket));
   }
  } //end for..
 }
} 


static void hal_net_device_handle_socket_status(uint8 socket,uint8 new_status)
{
  uint8 valid=FALSE;
  switch(new_status)
  {
  case SOCKET_CLOSED:
    {
    valid=TRUE;
    app_write_string("\r\nsocket is closed!");
    }
    break;
  case SOCKET_CLOSE_WAIT:
    {
    valid=TRUE;
    app_write_string("\r\nsocket is close_wait!");
    }
    break;  
  case SOCKET_INIT:   
   {
    valid=TRUE;
    app_write_string("\r\nsocket tcp is inited!");
   }   
    break;
  case SOCKET_LISTEN:
   {
   valid=TRUE;
   app_write_string("\r\nsocket tcp server is listen!");
   }   
    break;
  case SOCKET_ESTABLISHED:
   {
    valid=TRUE;
    app_write_string("\r\nsocket tcp is established!");
   }   
    break;
  case SOCKET_UDP:
   {
    valid=TRUE;
    app_write_string("\r\nsocket udp is init!");
     uint8 ip[4];
   w5500_get_socket_register_value(socket,SOCKET_REGISTER_DESTIP_OFFSET,ip,4);
   app_write_string("udp init dst ip:");
   for(uint8 i=0;i<4;i++)
     app_write_string(uint8_to_string(ip[i]));
   }   
    break;
  default:
    break;   
  }
  if(valid)
  {
   hal_net_device_set_socket_status(socket,new_status);
   socket_cb_array[socket].pf_status_change_callback(new_status);
  }
}

 void hal_net_device_poll_socket_status()
{
  uint8 status;
  uint8 new_status;
  uint8 old_status;
 for(uint8 socket=SOCKET0;socket<DEFAULT_SOCKET_NUM;socket++)
 {
 status=w5500_get_socket_register_value(socket,SOCKET_REGISTER_STATUS_OFFSET,&new_status,1);//
 old_status=hal_net_device_get_socket_status(socket);
 if(status==SUCCESS && old_status!=new_status)
 {
  hal_net_device_handle_socket_status(socket,new_status);
 }
 }
}





static void hal_net_device_set_socket_status(uint8 socket,uint8 new_status)
{
  socket_cb_array[socket].socket_status=new_status;
}

static uint8 hal_net_device_get_socket_status(uint8 socket)
{
  return socket_cb_array[socket].socket_status;
}



static void hal_net_device_handle_sys_int_event(uint8 events)
{

if(events&GRINT_CONFLICT_POS)
{
  app_write_string("\r\nhal ip comflict!");
}
if(events&GRINT_UNREACH_POS)
{
  app_write_string("\r\nhal ip unreach!");
}
if(events&GRINT_PPPOE_POS)
{
  app_write_string("\r\nhal ppoe disconnect!");
}
if(events&GRINT_MP_POS)
{
  app_write_string("\r\nhal mp!");
}
for(uint8 socket=0;socket<DEFAULT_SOCKET_NUM;socket++)
{
if(socket_cb_array[socket].pf_sys_int_callback)
socket_cb_array[socket].pf_sys_int_callback(events);
}

}

static void hal_net_device_poll_sys_int()
{
  uint8 int_events;
  uint8 status;
  status=w5500_get_general_register_value(GENERAL_REGISTER_INT_OFFSET,&int_events,1);
 if(status==SUCCESS && int_events!=0)
 {
   w5500_set_general_register_value(GENERAL_REGISTER_INT_OFFSET,&int_events,1);  
   hal_net_device_handle_sys_int_event(int_events);
  }
 
}
void hal_net_device_poll_device_int()
{
  hal_net_device_poll_sys_int();
  hal_net_device_poll_socket_int();
}


uint8 hal_net_device_get_link_status()
{
  return hal_net_device_link_status;
}

static void hal_net_device_set_link_status(uint8 new_status)
{
  hal_net_device_link_status=new_status;
}


static void hal_net_device_handle_link_status_change(uint8 new_status)
{
   hal_net_device_set_link_status(new_status); 
   if(new_status==NETDEV_LINKED)
   { 
    app_write_string("\r\n网络设备已连接!");
   }
  else
  { 
    app_write_string("\r\n网络设备未连接!");
   }  
 app_write_string("\r\n开始执行link status回调函数!"); 
 for(uint8 i=0;i<DEFAULT_SOCKET_NUM;i++)
 {
   socket_cb_array[i].pf_sys_status_change_callback(new_status);
 }
   
}



static void hal_net_device_poll_link_status()
{
  uint8 status;
  uint8 new_status;
  uint8 old_status;
  
  old_status=hal_net_device_get_link_status();
  w5500_get_general_register_value(GENERAL_REGISTER_PHYCFG_OFFSET,&status,1);
  if(status&GRPHYCFG_LNK_POS)
  {  
    new_status=NETDEV_LINKED;
  }
  else
  {
   new_status=NETDEV_NOT_LINKED; 
  }
  if(old_status!=new_status)
  {
    hal_net_device_handle_link_status_change(new_status);
  }
   
}



void hal_net_device_poll_device_status()
{
  hal_net_device_poll_link_status();
  hal_net_device_poll_socket_status(); 
}

void hal_net_device_register_socket_callback(uint8 socket,hal_socket_callback_t *cb)
{
  if(cb)
  {
    socket_cb_array[socket]=*cb;
    socket_cb_array[socket].socket_status=SOCKET_CLOSED;
    app_write_string("\r\n注册socket回调函数!");
  }
  else
  {
    app_write_string("\r\n错误!socket回调函数为空!");
  }
}
