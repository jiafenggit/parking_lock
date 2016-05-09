#include"hal_types.h"
#include"hal_defs.h"
#include"OSAL.h"
#include"npi.h"
#include"hal_uart.h"
#include"app_uart_init.h"
#include "hal_key.h"
#include"OnBoard.h"
#include "comdef.h"

//register uart_debug recv cmd
static uint8 registered_uart_dbg_taskid=NO_TASK_ID;


uint8 register_for_uart_debug(uint8 task_id)
{
#if UART_DEBUG > 0 
  if(registered_uart_dbg_taskid == NO_TASK_ID)
  {
    registered_uart_dbg_taskid=task_id;
    return (true);
  }   
    else
     return (false);
#else
  (void)task_id;
#endif   
}

/****usart uart_recv_callback****/
void uart_recv_callback(uint8 port,uint8 event)
{
  uint8 rx_buffer[128];
  uint8 valid_len=0;
  (void)event;
  
  if((valid_len=NPI_RxBufLen())>0)
  {
  NPI_ReadTransport(rx_buffer, valid_len);
#if UART_DEBUG > 0
  if(valid_len==1 )
  {
    uart_debug_cmd_t *msgPtr;
   
    msgPtr =(uart_debug_cmd_t*)osal_msg_allocate( sizeof(uart_debug_cmd_t));
    if ( msgPtr )
    {
      msgPtr->hdr.event = UART_DEBUG_CMD;
      msgPtr->cmd = rx_buffer[0];

      if(registered_uart_dbg_taskid!=NO_TASK_ID)
      {
      osal_msg_send( registered_uart_dbg_taskid, (uint8 *)msgPtr );     
      app_write_string("\r\n收到命令已向任务发送.");
      app_write_string(uint8_to_string(rx_buffer[0]));//cmd value
      }
      else
      {
       app_write_string("\r\n任务ID未注册."); 
      }
    }
    else
    {
      app_write_string("\r\n内存不足命令未向任务发送.");
    }
  }
  else
  {
    app_write_string("\r\n非法命令."); 
  }
  
#endif 
 }
}


/****usart output****/

void app_write_string(char *p)
{
#if  UART_DEBUG > 0  
NPI_WriteTransport((uint8*)p,osal_strlen(p));
#else
(void) p;
#endif
}


/******uint16 --->0x string***/
char *uint16_to_string(uint16 src)
{
#if UART_DEBUG >0
  uint8       i;
  char     dst[2];
  char* pAddr=dst;
  dst[0]=LO_UINT16(src);
  dst[1]=HI_UINT16(src);
  
  char        hex[] = "0123456789ABCDEF";
  static char str[7+2];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += 2;
  
  for ( i = 2; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  //*pStr = 0;
  str[6]='\r';
  str[7]='\n';
  str[8]=0;
  return str;
#else
  (void)src;
  return (char*) 0;
#endif
}

/******uint8 --->0x string***/
char *uint8_to_string(uint8 src)
{
#if UART_DEBUG > 0
  uint8       i;
  char     dst[1];
  char*pAddr=dst;
  dst[0]=src;
  
  char        hex[] = "0123456789ABCDEF";
  static char str[5+2];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += 1;
  
  for ( i = 1; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  //*pStr = 0;
  str[4]='\r';
  str[5]='\n';
  str[6]=0;
  return str;
  
#else
  (void)src;
  return (char*)0;
#endif 
}

void uint8_array_to_string(uint8 *pbuff,uint8 len)
{
#if UART_DEBUG >0
  while(len>0)
  {
  app_write_string(uint8_to_string(*pbuff++));
  len--;
  }
#else
  (void)pbuff;
  (void)len;
#endif
}