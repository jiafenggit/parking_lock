#include"hal_types.h"
#include"hal_defs.h"
#include"comdef.h"
#include"dhcp.h"
#include"hal_w5500.h"
#include"socket.h"
#include"app_uart_init.h"
#include"error.h"


static void dhcp_socket_recv_callback(uint16 ptr_recv_len);
static void dhcp_socket_int_event_callback(uint8 new_event);
static void dhcp_socket_status_change_callback(uint8 new_status);
static void dhcp_sys_int_event_callback(uint8 new_event);
static void dhcp_sys_status_change_callback(uint8 new_status);

static uint8 dhcp_send_msg(uint8 msg_tyep);
static uint8 dhcp_client_init();

static void dhcp_msg_param_init(dhcp_msg_t *ptr_msg,uint8 msg_type);

static uint8* dhcp_find_options(uint8 *addr_opt,options_type_t option,uint8* value_len );
static bool dhcp_find_opt_msg_type(uint8 *addr_opt,opt_msg_type_t msg_type);

static void dhcp_send_msg_procedure();
static void dhcp_process_reply_msg();
static void dhcp_reset_reply_timer();
static void dhcp_reset_procedure();
static uint8 dhcp_socket_open();
static uint8 dhcp_socket_close();
static void dhcp_process_send_timeout();
static void dhcp_check_reply_timeout(uint16 elapse_time);//in ms
static void dhcp_check_renewal_timeout(uint16 elapse_time);


uint32 dhcp_new_ip_flag=0;
uint8 dhcp_state=DHCP_STATE_IDLE;
uint8 dhcp_socket_status=SOCKET_CLOSED;
uint8 dhcp_link_status=NETDEV_NOT_LINKED;
uint16 dhcp_reply_timeout;
uint32 ip_renewal_timeout;

/**********Ĭ�ϵ�DHCP IP submask gateway�Ȳ���************/

uint8  magic_cookie[4]={0x63,0x82,0x53,0x63};
uint32 discover_msg_id=0x11223344;
uint32 request_msg_id=0x11223345;

uint8 dhcp_src_ip[4]={0};
uint8 dhcp_src_mac[6]={0x00,0x0e,0xc6,0x44,0x55,0x66};
uint8 dhcp_src_port[2]={0,68};//68
uint8 dhcp_sub_mask[4]={0};
uint8 dhcp_gate_way[4]={0};
uint8 dhcp_dst_ip[4]={255,255,255,255};
uint8 dhcp_dst_mac[6]={0xff,0xff,0xff,0xff,0xff,0xff};
uint8 dhcp_dst_port[2]={0,67};//67
uint8 dhcp_protocol=SRMODE_PROTOCOLUDP_POS;
uint8 dhcp_retry_time[2]={0x20,0x00};//800ms ��ʱ
uint8 dhcp_retry_cnt[1]={4};//4��
uint8 dhcp_mtu[2]={0x05,0xB4};//1460

/**********�����IP submask gateway��DHCP serverIP************/

uint8 a_src_ip[4];
uint8 a_sub_mask[4];
uint8 a_gate_way[4];
uint8 a_lease_time[4];
uint8 a_renewal_time[4];
uint8 a_rebind_time[4];
uint8 s_dhcp_ip[4];

extern uint8 socket_rxtx_buff[SOCKET_DEFAULT_SIZE];
uint8 *ptr_dhcp_buff=socket_rxtx_buff;

uint8 dhcp_client_init()
{
  socket_t socket_param;
  osal_mem_set((uint8*)&socket_param,0,sizeof(socket_t));
  
  socket_param.socket=DHCP_SOCKET;
  socket_param.ptr_src_ip=dhcp_src_ip;
  socket_param.ptr_src_mac=dhcp_src_mac;
  socket_param.ptr_src_port=dhcp_src_port;
  socket_param.ptr_sub_mask=dhcp_sub_mask;
  
  socket_param.ptr_dst_ip=dhcp_dst_ip;
  socket_param.ptr_dst_port=dhcp_dst_port;
  socket_param.ptr_dst_mac=dhcp_dst_mac;
  
  socket_param.ptr_gate_way=dhcp_gate_way;
  socket_param.ptr_retry_time=dhcp_retry_time;
  socket_param.ptr_retry_cnt=dhcp_retry_cnt;
  socket_param.ptr_mtu=dhcp_mtu;
  socket_param.ptr_protocol=&dhcp_protocol;
 
 
 return socket_init(&socket_param);
  
}


void dhcp_callback_register()
{
  socket_callback_t socket_callback;  
  osal_mem_set((uint8*)&socket_callback,0,sizeof(socket_callback_t));
  
 // socket_callback.socket_status=STATUS_INVALID;
  socket_callback.pf_int_callback=dhcp_socket_int_event_callback;
  socket_callback.pf_status_change_callback=dhcp_socket_status_change_callback;
  socket_callback.recv_callback.pf_socket_recv_cb=dhcp_socket_recv_callback;
  socket_callback.pf_sys_int_callback=dhcp_sys_int_event_callback;
  socket_callback.pf_sys_status_change_callback=dhcp_sys_status_change_callback;
  socket_callback.recv_callback.ptr_buffer=ptr_dhcp_buff;
  socket_callback.recv_callback.buffer_size=SOCKET_DEFAULT_SIZE;
  
  register_socket_callback(DHCP_SOCKET,&socket_callback);
}


static void dhcp_msg_param_init(dhcp_msg_t *ptr_msg,uint8 msg_type)
{
 uint8 i=0;

 osal_mem_set((uint8*)ptr_msg,0,sizeof(dhcp_msg_t)); 
 ptr_msg->op=DHCP_BOOT_REQUEST;
 ptr_msg->htype=1;
 ptr_msg->hlen=6;
 osal_mem_cpy(ptr_msg->chaddr,dhcp_src_mac,6);

 ptr_msg->secs=0;
 ptr_msg->flags=0x0080;//0x80000 

 ptr_msg->options[i++]=magic_cookie[0];
 ptr_msg->options[i++]=magic_cookie[1];
 ptr_msg->options[i++]=magic_cookie[2];
 ptr_msg->options[i++]=magic_cookie[3];
 ptr_msg->options[i++]=DHCP_MSG_TYPE;
 ptr_msg->options[i++]=1;
 ptr_msg->options[i++]=msg_type;
 
 ptr_msg->options[i++]=DHCP_CLIENT_ID;
 ptr_msg->options[i++]=7;
 ptr_msg->options[i++]=1;
 ptr_msg->options[i++]=dhcp_src_mac[0];
 ptr_msg->options[i++]=dhcp_src_mac[1];
 ptr_msg->options[i++]=dhcp_src_mac[2];
 ptr_msg->options[i++]=dhcp_src_mac[3];
 ptr_msg->options[i++]=dhcp_src_mac[4];
 ptr_msg->options[i++]=dhcp_src_mac[5];
 
 
 ptr_msg->options[i++]=DHCP_HOST_NAME;
 ptr_msg->options[i++]=6;
 ptr_msg->options[i++]='w';
 ptr_msg->options[i++]='u';
 ptr_msg->options[i++]='l';
 ptr_msg->options[i++]='o';
 ptr_msg->options[i++]='n';
 ptr_msg->options[i++]='g';
 
 switch(msg_type)
 {
 case DHCP_MSG_TYPE_DISCOVER:
 {
 ptr_msg->msg_id=discover_msg_id;
 
 ptr_msg->options[i++]=DHCP_REQUEST_PARAM;
 ptr_msg->options[i++]=3;
 ptr_msg->options[i++]=DHCP_REQUEST_IP;
 ptr_msg->options[i++]=DHCP_SUBMASK_ID;
 ptr_msg->options[i++]=DHCP_ROUTER_IP;
 ptr_msg->options[i++]=DHCP_END_OPTION;
  }
 break;
 case DHCP_MSG_TYPE_REQUEST:
 {
 ptr_msg->msg_id=request_msg_id; 
 
 ptr_msg->options[i++]=DHCP_REQUEST_IP;
 ptr_msg->options[i++]=4;
 ptr_msg->options[i++]=a_src_ip[0];
 ptr_msg->options[i++]=a_src_ip[1];
 ptr_msg->options[i++]=a_src_ip[2];
 ptr_msg->options[i++]=a_src_ip[3];
 
 ptr_msg->options[i++]=DHCP_SERVER_ID;
 ptr_msg->options[i++]=4;
 ptr_msg->options[i++]=s_dhcp_ip[0];
 ptr_msg->options[i++]=s_dhcp_ip[1];
 ptr_msg->options[i++]=s_dhcp_ip[2];
 ptr_msg->options[i++]=s_dhcp_ip[3];
 
  ptr_msg->options[i++]=DHCP_SUBMASK_ID;
 ptr_msg->options[i++]=4;
 ptr_msg->options[i++]=a_sub_mask[0];
 ptr_msg->options[i++]=a_sub_mask[1];
 ptr_msg->options[i++]=a_sub_mask[2];
 ptr_msg->options[i++]=a_sub_mask[3];
  ptr_msg->options[i++]=DHCP_ROUTER_IP;
 ptr_msg->options[i++]=4;
 ptr_msg->options[i++]=a_gate_way[0];
 ptr_msg->options[i++]=a_gate_way[1];
 ptr_msg->options[i++]=a_gate_way[2];
 ptr_msg->options[i++]=a_gate_way[3];
 ptr_msg->options[i++]=DHCP_END_OPTION;
  }
 break;
 default:
 break;
 }
}


uint8 dhcp_send_msg(uint8 msg_tyep)
{
 dhcp_msg_t *ptr_msg;
 ptr_msg=(dhcp_msg_t*)ptr_dhcp_buff;
 dhcp_msg_param_init(ptr_msg,msg_tyep);
 
 return socket_send(DHCP_SOCKET,(uint8*)ptr_msg,sizeof(dhcp_msg_t));
}

uint8 dhcp_process_ack()
{
  uint8 *addr_opt;
  dhcp_msg_t *ptr_msg;
  uint8 value_len;
  uint8 *ptr_addr;
  
  ptr_msg=(dhcp_msg_t*)(ptr_dhcp_buff+DEFAULT_UDP_RECV_HDR_SIZE);
  addr_opt=ptr_msg->options+4;

  if(ptr_msg->op!=DHCP_BOOT_REPLY)
  {
  app_write_string("\r\n����dhcp reply!");
  return NOT_MATCHED_ERROR;
  }
  if(osal_mem_cmp(ptr_msg->chaddr,dhcp_src_mac,6)!=0)
  {
  app_write_string("\r\n���Ǳ���mac!");
  return NOT_MATCHED_ERROR;
  }
  if(ptr_msg->msg_id!=request_msg_id)
  {
  app_write_string("\r\n����request msg id!");
  return NOT_MATCHED_ERROR; 
  }
  
  if(dhcp_find_opt_msg_type(addr_opt,DHCP_MSG_TYPE_ACK))
  {
  ptr_addr=NULL;
  ptr_addr=dhcp_find_options(addr_opt,DHCP_RENEWAL_TIME,&value_len);//renewal time
  if(ptr_addr && value_len==4)
  {
    osal_mem_cpy(a_renewal_time,ptr_addr,value_len);
    ip_renewal_timeout=a_renewal_time[0]*((uint32)1<<24)+a_renewal_time[1]*((uint32)1<<16)+a_renewal_time[2]*((uint16)1<<8)+a_renewal_time[3];
    ip_renewal_timeout*=1000;//ת����ms
    app_write_string("\r\n��ack ��opt�з��� ip renewal time������:");
    
   }
  else
  {
    app_write_string("\r\nû����ack��opt�з��ֵķ��� ip renewal time!");    
    return NOT_FOUND_ERROR;
   }
    
    
   app_write_string("\r\n����ack! dhcp ���!");
   app_write_string("\r\n����IP:");
   uint8_array_to_string(a_src_ip,4);
   app_write_string("\r\n����MAC:");
   uint8_array_to_string(dhcp_src_mac,6);
   app_write_string("\r\n��������:");
   uint8_array_to_string(a_sub_mask,4);
   app_write_string("\r\n����·��:");
   uint8_array_to_string(a_gate_way,4);
   app_write_string("\r\nDHCP server IP:");
   uint8_array_to_string(s_dhcp_ip,4);
   app_write_string("\r\n IP renewal time:");
   uint8_array_to_string(a_renewal_time,4);
   return SUCCESS;
  }
 else if(dhcp_find_opt_msg_type(addr_opt,DHCP_MSG_TYPE_NACK))
 {
    app_write_string("\r\n����nack,���뱻�ܾ���������������!");  
    
    return NOT_FOUND_ERROR;
  } 
  else
  {
    app_write_string("\r\nû�з���ack or nack,������������!");  
  }
  return NOT_FOUND_ERROR;
}

uint8 dhcp_process_offer()
{
  uint8 *addr_opt;
  dhcp_msg_t *ptr_msg;
  uint8 value_len;
  uint8 *ptr_addr=NULL;
  
  ptr_msg=(dhcp_msg_t*)(ptr_dhcp_buff+DEFAULT_UDP_RECV_HDR_SIZE);
  addr_opt=ptr_msg->options+4;//magic cookie size 4
 
  if(ptr_msg->op!=DHCP_BOOT_REPLY)
  {
  app_write_string("\r\n����dhcp reply!");
  return NOT_MATCHED_ERROR;
  }
  if(osal_mem_cmp(ptr_msg->chaddr,dhcp_src_mac,6)!=0)
  {
  app_write_string("\r\n���Ǳ���mac!");
  return NOT_MATCHED_ERROR;
  }
  if(ptr_msg->msg_id!=discover_msg_id)
  {
  app_write_string("\r\n����discover msg id!");
  return NOT_MATCHED_ERROR; 
  }
  
  if(dhcp_find_opt_msg_type(addr_opt,DHCP_MSG_TYPE_OFFER))
  {
  app_write_string("\r\n����offer!"); 
  ptr_addr=dhcp_find_options(addr_opt,DHCP_REQUEST_IP,&value_len);//request ip
  if(ptr_addr && value_len==4)
  {   
    app_write_string("\r\n���ַ����IP������!");
    osal_mem_cpy(a_src_ip,ptr_addr,4);
  }
  else
  {
    app_write_string("\r\nû����opt�з��ֵ�IP");
    osal_mem_cpy(a_src_ip,ptr_msg->yiaddr,4);
    app_write_string("\r\n��ciaddr����ip���!");
  }
  ptr_addr=NULL;
  ptr_addr=dhcp_find_options(addr_opt,DHCP_SERVER_ID,&value_len);//server ip
  if(ptr_addr && value_len==4)
  {
    osal_mem_cpy(s_dhcp_ip,ptr_addr,value_len);
    app_write_string("\r\n����server ip������!");
   }
  else
  {
    app_write_string("\r\nû����opt�з��ֵ�server IP");
    osal_mem_cpy(s_dhcp_ip,ptr_msg->siaddr,4);
    app_write_string("\r\n��siaddr����ip���!");
   }
  
  ptr_addr=NULL;
  ptr_addr=dhcp_find_options(addr_opt,DHCP_SUBMASK_ID,&value_len);//submask 
  if(ptr_addr && value_len==4)
  {
    osal_mem_cpy(a_sub_mask,ptr_addr,value_len);
    app_write_string("\r\n���ַ����submask������!");
   }
  else
  {
    app_write_string("\r\nû�з��ַ����submask!!");
    return NOT_FOUND_ERROR;
   }
  
  ptr_addr=NULL;
  ptr_addr=dhcp_find_options(addr_opt,DHCP_ROUTER_IP,&value_len);//router ip
  if(ptr_addr && value_len==4)
  {
    osal_mem_cpy(a_gate_way,ptr_addr,value_len);
    app_write_string("\r\n���ַ����router������!");
   }
  else
  {
    app_write_string("\r\nrouter value len error!");
    return NOT_FOUND_ERROR;
   }
  
  ptr_addr=NULL;
  ptr_addr=dhcp_find_options(addr_opt,DHCP_IP_LEASE_TIME,&value_len);//lease time
  if(ptr_addr && value_len==4)
  {
    osal_mem_cpy(a_lease_time,ptr_addr,value_len);
    app_write_string("\r\n��opt�з��� ip lease time������!");
   }
  else
  {
    app_write_string("\r\nû����opt�з��ֵķ��� ip lease time!");    
    return NOT_FOUND_ERROR;
   }
  
  ptr_addr=NULL;
  ptr_addr=dhcp_find_options(addr_opt,DHCP_RENEWAL_TIME,&value_len);//renewal time
  if(ptr_addr && value_len==4)
  {
    osal_mem_cpy(a_renewal_time,ptr_addr,value_len);
    ip_renewal_timeout=a_renewal_time[0]*((uint32)1<<24)+a_renewal_time[1]*((uint32)1<<16)+a_renewal_time[2]*((uint16)1<<8)+a_renewal_time[3];
    ip_renewal_timeout*=1000;//ת����ms
    app_write_string("\r\n��opt�з��� ip renewal time������:");
    
   }
  else
  {
    app_write_string("\r\nû����opt�з��ֵķ��� ip renewal time!");    
    return NOT_FOUND_ERROR;
   }
  
  ptr_addr=NULL;
  ptr_addr=dhcp_find_options(addr_opt,DHCP_REBINDING_TIME,&value_len);//rebind time 
  if(ptr_addr && value_len==4)
  {
    osal_mem_cpy(a_rebind_time,ptr_addr,value_len);
    app_write_string("\r\n��opt�з��� ip rebind time������!");
   }
  else
  {
    app_write_string("\r\nû����opt�з��ֵķ��� ip rebind time!");    
    return NOT_FOUND_ERROR;
   }
  
  }
  else
  {
    app_write_string("\r\nû�з���offer!"); 
    return NOT_FOUND_ERROR;
  }
 

  return SUCCESS;
}


static uint8* dhcp_find_options(uint8* opt_addr,options_type_t option,uint8* value_len )
{
  uint8 *ptr_addr;
  ptr_addr=opt_addr;
    
 while(ptr_addr < ptr_dhcp_buff+sizeof(dhcp_msg_t))
  {
   if(option == *ptr_addr)
   {
     *value_len=*(ptr_addr+1);
     return ptr_addr+2;      //return value addr 
   }
   else
  {
   ptr_addr+=*(ptr_addr+1)+2;
   }
  }
 return NULL;
}


static bool dhcp_find_opt_msg_type(uint8 *addr_opt,opt_msg_type_t msg_type)
{
  uint8* ptr_addr;
  uint8 value_len;
  uint8 value;
  
  ptr_addr=dhcp_find_options(addr_opt,DHCP_MSG_TYPE,&value_len);
  if(ptr_addr && value_len==1)
  {
   value=*ptr_addr;
   if(value==msg_type)
   {
     app_write_string("\r\n�ҵ�msg_type value!");
     return TRUE;
   }
   else
   {
    app_write_string("\r\n msg_type value ����!");
   }
   
  }
  else
  {
    app_write_string("\r\n��ַΪ��,����ֵ���Ȳ�ƥ��!");
  }
  
 return FALSE;
  
}



/*********************************************************************/
/*DHCP����                                                           */
/*����link ״̬������                                                 */
/*********************************************************************/

static void dhcp_send_msg_procedure()
{
  if(dhcp_socket_status==SOCKET_UDP && dhcp_link_status==NETDEV_LINKED)//���Ϊ�� �͵ȴ���ʱ
  {
  dhcp_reset_reply_timer();
    
  switch(dhcp_state)
  {
  case DHCP_STATE_IDLE:
  {
   app_write_string("\r\ndhcp �ǿ���״̬����Ҫ����!");
  } 
  break;
  
  case DHCP_STATE_DISCOVER:
  {
   dhcp_send_msg(DHCP_MSG_TYPE_DISCOVER);
   app_write_string("\r\n����dhcp discover!");
  }
  break;
  
  case DHCP_STATE_REQUEST:
  {
   dhcp_send_msg(DHCP_MSG_TYPE_REQUEST);
   app_write_string("\r\n����dhcp request!");
  }
  break;
  
   case DHCP_STATE_COMPLETE:
  {
   app_write_string("\r\ndhcp ���!");
  }
  break;
  default:
  {
   app_write_string("\r\ndhcp state error!");
  }
  break;
  }
 }
}


static void dhcp_process_reply_msg()
{
  
  if(dhcp_state==DHCP_STATE_IDLE)
  {
   app_write_string("\r\nreply msg ����,dhcp��û�п�ʼ!");
  } 
  else if(dhcp_state==DHCP_STATE_DISCOVER)
  {
    if(dhcp_process_offer()==SUCCESS)
    {
     dhcp_state=DHCP_STATE_REQUEST;            
    }
    
  }
  else if(dhcp_state==DHCP_STATE_REQUEST)
  {
    if(dhcp_process_ack()==SUCCESS)
    {
     dhcp_state=DHCP_STATE_COMPLETE;
     //to do 
     dhcp_new_ip_flag++;//����ip��־
     
    }
    else
    {
     dhcp_socket_close();//����dhcp  
    }

  } 
  
  
  if(dhcp_state==DHCP_STATE_COMPLETE)
  {   
    app_write_string("\r\ndhcp ����reply msg dhcp �Ѿ������!");   
  }
  else
  {
   dhcp_send_msg_procedure(); 
  }
  
}

static void dhcp_reset_reply_timer()
{
  dhcp_reply_timeout=DHCP_REPLY_TIMEOUT;
}


static void dhcp_reset_procedure()
{  
  dhcp_new_ip_flag=0;//dhcpʧ�ܾ�ʹip��Ч
  dhcp_state=DHCP_STATE_IDLE;//����Ϊdiscover״̬
  dhcp_reset_reply_timer();//����reply ��ʱ��ʱ��
  dhcp_client_init();  
  
  app_write_string("\r\ndhcp reset over!"); 
  
}

static uint8 dhcp_socket_open()
{
  app_write_string("\r\n����dhcp socket!");
  return socket_open(DHCP_SOCKET);
}
static uint8 dhcp_socket_close()
{
  app_write_string("\r\n�ر�dhcp socket!");
  return socket_close(DHCP_SOCKET);
}


static void dhcp_process_send_timeout()
{
   app_write_string("\r\n���ͳ�ʱ����,׼������dhcp!");
   dhcp_socket_close();//�ر�dhcp ���¿�ʼ
}

static void dhcp_check_reply_timeout(uint16 elapse_time)//in ms
{
  if(dhcp_socket_status==SOCKET_UDP && dhcp_state!=DHCP_STATE_COMPLETE)//���socket udp ���� ͬʱû�����dhcp
  {
    if(dhcp_reply_timeout>0)//dhcp���ڽ�����
    {
    if(dhcp_reply_timeout>elapse_time)  
    {
     dhcp_reply_timeout-=elapse_time;
     }
    else
    {
     dhcp_reply_timeout=0;
    }
   if(dhcp_reply_timeout==0)
   {
     app_write_string("\r\ndhcp reply ��ʱ!");
     dhcp_socket_close();//�ر�dhcp ���¿�ʼ
    }
   }
  }
}


static void dhcp_check_renewal_timeout(uint16 elapse_time)
{
  if(dhcp_state==DHCP_STATE_COMPLETE)
  {
  if(ip_renewal_timeout>elapse_time)
  {
    ip_renewal_timeout-=elapse_time;
  }
  else
  {
    ip_renewal_timeout=0;
  }
  
  if(ip_renewal_timeout==0)
  {
    dhcp_state=DHCP_STATE_REQUEST;
    dhcp_send_msg_procedure();//ֱ������ԭ����IP
    
  }
 }
}


//call back func
static void dhcp_socket_recv_callback(uint16 ptr_recv_len)
{
  uint8 *ptr;
  uint16 next=0;
  ptr=ptr_dhcp_buff;
  app_write_string("\r\nִ��dhcp socket���ջص�����!");
 
#if DHCP_DEBUG > 0  
  for(uint16 i=0;i<ptr_recv_len;i++)
  {
    app_write_string(uint8_to_string(*ptr++));
    while(next++<1000);
    next=0;   
  }
#else
  (void) ptr;
#endif
  
  app_write_string("\r\ndhcp������ݽ���,׼������dhcp reply!"); 
  dhcp_process_reply_msg();
  
}



static void dhcp_socket_int_event_callback(uint8 new_event)
{
  app_write_string("\r\nִ��dhcp socket�ж��¼��ص�����!"); 
  if(new_event&SEND_OK)
  {
   app_write_string("\r\nDHCP msg ���ͳɹ�!");
  }
  if(new_event&TIME_OUT)
  {
    app_write_string("\r\nDHCP msg ���ͳ�ʱ!");
    dhcp_process_send_timeout();// 
  }
}


static void dhcp_socket_status_change_callback(uint8 new_status)
{
  app_write_string("\r\nִ��dhcp socket״̬�ı�ص�����!");
  dhcp_socket_status=new_status;//���浱ǰsocket״̬
  
  if(dhcp_socket_status==SOCKET_UDP)
  {
   app_write_string("\r\ndhcp socket udp!"); 
 
  }
  if(dhcp_socket_status==SOCKET_CLOSED)
  {
   app_write_string("\r\ndhcp socket closed!"); 
  }
}


static void dhcp_sys_status_change_callback(uint8 new_status)
{
 app_write_string("\r\nִ��dhcp sys״̬�ı�ص�����!"); 
 
 dhcp_link_status=new_status;//���浱ǰlink״̬
 
 if(new_status==NETDEV_LINKED)
 {
  app_write_string("\r\ndhcp device link!");  
 }
 if(new_status==NETDEV_NOT_LINKED) 
 {
  app_write_string("\r\ndhcp device not link!");
 }
}


static void dhcp_sys_int_event_callback(uint8 new_event)
{
 app_write_string("\r\nִ��dhcp sys�жϻص�����!"); 
 if(new_event&GRINT_CONFLICT_POS)
 app_write_string("\r\ndhcp ip comflict!");  
 if(new_event&GRINT_UNREACH_POS)
 app_write_string("\r\ndhcp ip unreach!");  
 
}


void dhcp_poll_status_procedure(uint16 elapse_time)
{
  dhcp_check_reply_timeout(elapse_time);//in ms
  dhcp_check_renewal_timeout(elapse_time);
  
  
  if(dhcp_link_status==NETDEV_LINKED)
  {
   if(dhcp_state!=DHCP_STATE_COMPLETE)//�����û�����
   {    
     if(dhcp_state==DHCP_STATE_IDLE)//��û�п�ʼ
     {
     if(dhcp_socket_status==SOCKET_UDP)//-------------------
     {
      dhcp_state=DHCP_STATE_DISCOVER;//��ʼ����dhcp
      dhcp_send_msg_procedure();
      }
     }
   }
  if(dhcp_socket_status==SOCKET_CLOSED)//���dhcp socket �ǹرյ�
   {
     dhcp_reset_procedure();
     dhcp_client_init();     
     dhcp_socket_open();//�򿪵�ǰdhcp socket 
   }     
 }
 else//�Ͽ�link �͹ر�dhcp socket
 {   
   if(dhcp_socket_status!=SOCKET_CLOSED)//-----------------
   {
   dhcp_new_ip_flag=0;
   dhcp_socket_close();//�رյ�ǰdhcp socket  
   }
 }
  
}


