#ifndef  __SOCKET_H__
#define  __SOCKET_H__

#define  SOCKET_DEFAULT_SIZE  300




typedef struct _socket
{
  uint8  socket;
  uint8* ptr_src_ip;
  uint8* ptr_src_port;
  uint8* ptr_src_mac;
  uint8* ptr_sub_mask;
  uint8* ptr_gate_way; 
  uint8* ptr_retry_time;
  uint8* ptr_retry_cnt;
  uint8* ptr_mtu;
  uint8* ptr_dst_ip;
  uint8* ptr_dst_port;
  uint8* ptr_dst_mac;
  uint8* ptr_protocol;
  
}socket_t;



typedef hal_socket_callback_t socket_callback_t;




void osal_mem_set(uint8*paddr,uint8 value,uint16 len);
void osal_mem_cpy(uint8* dst,uint8*src,uint16 len);
uint16 osal_mem_cmp(uint8 *dst1,uint8 *dst2,uint16 len);



uint8 socket_init(socket_t *ptr_socket);
uint8 socket_open(uint8 socket);
uint8 socket_close(uint8 socket);
uint8 socket_connect(uint8 socket);
uint8 socket_disconnect(uint8 socket);
uint8 socket_send(uint8 socket,uint8* ptr_buff,uint16 buff_len);
uint8 socket_send_to(uint8 socket,uint8* ptr_dst_ip,uint8* ptr_dst_port,uint8 *ptr_buff,uint16 len);
uint8 socket_set_param(uint8 socket,uint8 param,uint8 *ptr_value);

void register_socket_callback(uint8 socket,hal_socket_callback_t *cb);










#endif