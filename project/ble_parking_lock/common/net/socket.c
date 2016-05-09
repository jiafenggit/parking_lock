#include"hal_types.h"
#include"hal_defs.h"
#include"hal_w5500.h"
#include"socket.h"
#include"error.h"
#include"comdef.h"


uint8 socket_rxtx_buff[SOCKET_DEFAULT_SIZE];

socket_recv_callback_t            *ptr_socket_recv_cb;
socket_int_event_callback_t       *ptr_socket_int_cb;
socket_status_change_callback_t   *ptr_socket_stat_change_cb;




void  osal_mem_set(uint8*paddr,uint8 value,uint16 len)
{
  if(paddr)
  {
    for(uint16 i=0;i<len;i++)
    {
      *paddr++=value;
    }
  }

}
void osal_mem_cpy(uint8* dst,uint8*src,uint16 len)
{
  if(dst&&src)
  {
   while(len--)
   {
    *dst++=*src++;
   }
  } 
}

uint16 osal_mem_cmp(uint8 *dst1,uint8 *dst2,uint16 len)
{
  if(len==0 || dst1==NULL || dst2==NULL)
  return 0xffff;
  
  while(len--)
  {
    if(*dst1++ != *dst2++)
      return len;
  }
  return 0;
}

uint8 socket_init(socket_t *ptr_socket)
{
  uint8 socket;
  uint8 status;
  uint8 re_status=SUCCESS;
  if(ptr_socket==NULL)
  return ADDR_NULL_ERROR;
 
  socket=ptr_socket->socket;
  status=hal_net_device_set_param(socket,PARAM_SRC_IP,ptr_socket->ptr_src_ip);
  re_status+=status;
  status=hal_net_device_set_param(socket,PARAM_SRC_PORT,ptr_socket->ptr_src_port);
  re_status+=status;
  status=hal_net_device_set_param(socket,PARAM_SRC_MAC,ptr_socket->ptr_src_mac);
  re_status+=status;
  status=hal_net_device_set_param(socket,PARAM_SUB_MASK,ptr_socket->ptr_sub_mask);
  re_status+=status;
  status=hal_net_device_set_param(socket,PARAM_GATE_WAY,ptr_socket->ptr_gate_way);
  re_status+=status;
  status=hal_net_device_set_param(socket,PARAM_RETRY_TIME,ptr_socket->ptr_retry_time);
  re_status+=status;
  status=hal_net_device_set_param(socket,PARAM_RETRY_CNT,ptr_socket->ptr_retry_cnt);
  re_status+=status;
  status=hal_net_device_set_param(socket,PARAM_SOCKET_MTU,ptr_socket->ptr_mtu);
  re_status+=status;
  status=hal_net_device_set_param(socket,PARAM_SOCKET_PROTOCOL,ptr_socket->ptr_protocol);
  re_status+=status;
  status=hal_net_device_set_param(socket,PARAM_DST_IP,ptr_socket->ptr_dst_ip);
  //re_status+=status;
  //status=hal_net_device_set_param(socket,PARAM_DST_MAC,ptr_socket->ptr_dst_mac);
  re_status+=status;
  status=hal_net_device_set_param(socket,PARAM_DST_PORT,ptr_socket->ptr_dst_port);
  re_status+=status;
    
  if(re_status!=SUCCESS)
  return INIT_FAILD_ERROR;

 return SUCCESS;
}

uint8 socket_open(uint8 socket)
{
  return hal_net_device_send_cmd(socket,SOCKET_CMD_OPEN); 
}

uint8 socket_close(uint8 socket)
{
  return hal_net_device_send_cmd(socket,SOCKET_CMD_CLOSE); 
}

uint8 socket_send_to(uint8 socket,uint8* ptr_dst_ip,uint8* ptr_dst_port,uint8 *ptr_buff,uint16 len)
{
  return hal_net_device_send_to(socket,ptr_dst_ip,ptr_dst_port,ptr_buff,len);
}

uint8 socket_send(uint8 socket,uint8* ptr_buff,uint16 buff_len)
{
 return hal_net_device_send(socket,ptr_buff,buff_len);
  
}
uint8 socket_connect(uint8 socket)
{
  uint8 status;
  status=hal_net_device_send_cmd(socket,SOCKET_CMD_CONNECT);
  return status;
}
uint8 socket_disconnect(uint8 socket)
{
  uint8 status;
  status=hal_net_device_send_cmd(socket,SOCKET_CMD_DISCON);
  return status;
}
uint8 socket_set_param(uint8 socket,uint8 param,uint8 *ptr_value)
{
  return hal_net_device_set_param(socket,param,ptr_value);
}
void register_socket_callback(uint8 socket,hal_socket_callback_t *cb)
{
 hal_net_device_register_socket_callback( socket, cb);
}
