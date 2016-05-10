#include"hal_defs.h"
#include"hal_types.h"
#include"iocc2541.h"
#include"hal_board_cfg_ble_control.h"
#include"hal_w5500.h"
#include"dhcp.h"
#include"socket.h"
#include"error.h"
#include"app_uart_init.h"
#include"net.h"


extern uint8 socket_rxtx_buff[SOCKET_DEFAULT_SIZE];
extern uint8 a_src_ip[4];
extern uint8 a_sub_mask[4];
extern uint8 a_gate_way[4];
extern uint32 dhcp_new_ip_flag;


/*****************默认值***********************************/
uint8 net_src_ip[4]={192,168,1,158};
uint8 net_src_mac[6]={0x00,0x0e,0xc6,0x44,0x55,0x66};
uint8 net_src_port[2]={0x04,0x5e};//1118;{0x22,0xb8};//8888
uint8 net_sub_mask[4]={255,255,255,0};
uint8 net_gate_way[4]={192,168,1,1};
uint8 net_dst_ip[4]={114,55,94,244};
uint8 net_dst_mac[6]={0xff,0xff,0xff,0xff,0xff,0xff};
uint8 net_dst_port[2]={0x22,0x60};//8800
uint8 net_protocol=SRMODE_PROTOCOLUDP_POS;//SRMODE_PROTOCOLTCP_POS
uint8 net_retry_time[2]={0x20,0x00};//800ms
uint8 net_retry_cnt[1]={4};//4次
uint8 net_mtu[2]={0x05,0xB4};//1460

uint32 net_ip_flag=0;
uint8 net_ip_comflict=FALSE;
uint8 net_socket_status=SOCKET_CLOSED;
uint8 net_link_status=NETDEV_NOT_LINKED;
uint8 *ptr_net_buff=socket_rxtx_buff;

extern void ble_control_net_protocol_parse(uint8 *ptr_recv,uint16 recv_len);

static uint8 net_socket_init();
static void net_callback_register();
static uint8 net_socket_connect();
static uint8 net_socket_open();
static uint8 net_socket_close();

//call back func
static void net_socket_recv_callback(uint16 ptr_recv_len)
{
  uint8 *ptr;
  uint16 next=0;
  ptr=ptr_net_buff;
  app_write_string("\r\n执行net socket接收回调函数!");
#if NET_DEBUG > 0    
  for(uint16 i=0;i<ptr_recv_len;i++)
  {
    app_write_string(uint8_to_string(*ptr++));
    while(next++<1000);
    next=0;   
  }
#endif
  
  app_write_string("\r\nnet完成数据接收!"); 
  app_write_string("\r\nnet进入协议解析程序!"); 
  ptr=ptr_net_buff;
  
  ble_control_net_protocol_parse(ptr,ptr_recv_len);
  
  
}

static void net_socket_int_event_callback(uint8 new_event)
{
  app_write_string("\r\n执行net socket中断事件回调函数!"); 
  if(new_event&SEND_OK)
  {
    app_write_string("\r\n执行net socket send ok!"); 
  }
  if(new_event&DIS_CONN)
  {
    app_write_string("\r\nnet tcb disconnect!");
   if(net_socket_status!=SOCKET_CLOSED)
   {
    net_socket_close();//关闭net app 重新开始
    }
  }
  if(new_event&CONNECTED)
  {
     app_write_string("\r\nnet tcb connected!");
  }
  if(new_event&TIME_OUT)
  {  
    app_write_string("\r\nnet 建立连接或者数据发送超时!");
  /*
   if(net_socket_status!=SOCKET_CLOSED)
   {
    net_socket_close();//关闭net app 重新开始
    }
    */   
  } 
}

static void net_socket_status_change_callback(uint8 new_status)
{
  app_write_string("\r\n执行net socket状态改变回调函数!");
  net_socket_status=new_status;
  if(new_status==SOCKET_CLOSED)
  {
  app_write_string("\r\nnet socket closed!");
  }
  if(new_status==SOCKET_CLOSE_WAIT)
  {
  app_write_string("\r\nnet socket close_wait!");
  }
  
  if(new_status==SOCKET_UDP)   
  {
   app_write_string("\r\nnet socket udp!");
   osal_mem_cpy(ptr_net_buff,"hello,wkxboot! net udp init!",32);
   while(socket_send_to(NET_SOCKET,net_dst_ip,net_dst_port,ptr_net_buff,32)!=SUCCESS);
   app_write_string("\r\n发送hello,wkxboot! net udp init!");
  }
  if(new_status==SOCKET_INIT)   
  {
   app_write_string("\r\nnet socket init!");
  }
  if(new_status==SOCKET_ESTABLISHED)   
  {
   app_write_string("\r\nnet socket  tcb establish!");
   osal_mem_cpy(ptr_net_buff,"hello,wkxboot! net tcb establish!",38);
   while(socket_send(NET_SOCKET,ptr_net_buff,38)!=SUCCESS);
   app_write_string("\r\n发送hello,wkxboot! net tcb establish!");
  }

  
}
static void net_sys_status_change_callback(uint8 new_status)
{
  app_write_string("\r\n执行net sys状态改变回调函数!"); 
 
  net_link_status=new_status;//保存当前link状态
 
 if(net_link_status==NETDEV_LINKED)
 {
  app_write_string("\r\nnet device link!");  
 }
 if(net_link_status==NETDEV_NOT_LINKED) 
 {
  app_write_string("\r\nnet device not link!");
 }

}
static void net_sys_int_event_callback(uint8 new_event)
{
 app_write_string("\r\n执行net sys状态改变回调函数!"); 
 if(new_event&GRINT_CONFLICT_POS)
 {
 net_ip_comflict=TRUE;
 app_write_string("\r\n执行net ip comflict!");  

 }
 if(new_event&GRINT_UNREACH_POS)
 {
 app_write_string("\r\n执行net ip unreach!");  
 
 }
 
}


static void net_callback_register()
{ 
  socket_callback_t socket_callback;
  osal_mem_set((uint8*)&socket_callback,0,sizeof(socket_callback_t));
 
  socket_callback.pf_int_callback=net_socket_int_event_callback;
  socket_callback.pf_status_change_callback=net_socket_status_change_callback;
  socket_callback.recv_callback.pf_socket_recv_cb=net_socket_recv_callback;
  socket_callback.recv_callback.ptr_buffer=ptr_net_buff;
  socket_callback.recv_callback.buffer_size=SOCKET_DEFAULT_SIZE;
  socket_callback.pf_sys_int_callback=net_sys_int_event_callback;
  socket_callback.pf_sys_status_change_callback=net_sys_status_change_callback;
  socket_callback.socket_status=net_socket_status;
    
  register_socket_callback(NET_SOCKET,&socket_callback);
}

static uint8 net_socket_init()
{
  socket_t socket_param;
  osal_mem_set((uint8*)&socket_param,0,sizeof(socket_t));
      
  socket_param.socket=NET_SOCKET;
  socket_param.ptr_src_ip=net_src_ip;
  socket_param.ptr_sub_mask=net_sub_mask;
  socket_param.ptr_gate_way=net_gate_way;
  socket_param.ptr_src_port=net_src_port;
  socket_param.ptr_src_mac=net_src_mac;
  socket_param.ptr_retry_time=net_retry_time;
  socket_param.ptr_retry_cnt=net_retry_cnt;
  socket_param.ptr_dst_ip=net_dst_ip;
  socket_param.ptr_dst_port=net_dst_port;
  socket_param.ptr_dst_mac=net_dst_mac;
  socket_param.ptr_mtu=net_mtu;
  socket_param.ptr_protocol=&net_protocol;
 
  app_write_string("net socket init start!");
  
  return socket_init(&socket_param);
}


static void net_socket_param_refresh()
{
  osal_mem_cpy(net_src_ip,a_src_ip,4);
  osal_mem_cpy(net_sub_mask,a_sub_mask,4);
  osal_mem_cpy(net_gate_way,a_gate_way,4);
  
}

static uint8 net_socket_connect()
{
  return socket_connect(NET_SOCKET);
}
static uint8 net_socket_open()
{
  return socket_open(NET_SOCKET);
}
static uint8 net_socket_close()
{
  return socket_close(NET_SOCKET);
}


void net_poll_status_procedure()
{
  if(net_link_status==NETDEV_LINKED)
  {
  if(dhcp_new_ip_flag==0)//dhcp 无效
  {
    net_ip_flag=0;//ip无效
    if(net_socket_status!=SOCKET_CLOSED)
    {
     net_socket_close();//关闭net app 重新开始
    }
  }
  else
  { 
   if(dhcp_new_ip_flag!=net_ip_flag)//ip需要更新
   {   
    net_socket_param_refresh();//更新ip等资源
    net_ip_flag=dhcp_new_ip_flag;//net ip    
   if(net_socket_status!=SOCKET_CLOSED)
   {
    net_socket_close();//关闭net app 重新开始
    }
   }
  }
  /***********************/
  if(net_socket_status==SOCKET_UDP)//-------------------
  {
    //todo

  }
  if(net_socket_status==SOCKET_INIT)//tcp init
  {
   net_socket_connect(); 
  }

  if(net_socket_status==SOCKET_CLOSED || net_socket_status==SOCKET_CLOSE_WAIT)//如果netsocket 是关闭的或者半关闭重新初始化连接
  {
   if(dhcp_new_ip_flag==net_ip_flag && net_ip_flag!=0  )//如果ip有效
   {
    net_socket_init();     
    net_socket_open();//打开当前net socket 
   }
  } 
  if(net_ip_comflict)
  {
     net_ip_comflict=FALSE;//使IP冲突无效
     net_ip_flag=0;//使ip无效  
     hal_net_device_software_reset();//如果ip冲突 就重启设备
  }
 }
 else//断开link 就关闭net socket
 {   
   if(net_socket_status!=SOCKET_CLOSED)//-----------------
   {
   net_ip_comflict=FALSE;//使IP冲突无效
   net_ip_flag=0;//使ip无效  
   net_socket_close();//关闭当前net socket  
   }
 }
}

void app_net_init()
{
  app_write_string("\r\nhello wkxboot!! w5500 operate is started!");
  hal_net_device_software_reset();
  
  dhcp_callback_register();//dhcp callback
  net_callback_register(); //net callback 

}
void app_poll_net_status(uint16 elapse_time)//定时轮询网络状态
{
  hal_net_device_poll_device_status();//轮询所有status
  hal_net_device_poll_device_int();//轮询所有int
  
  net_poll_status_procedure();
  dhcp_poll_status_procedure(elapse_time);
 
}