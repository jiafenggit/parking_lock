#include"hal_types.h"
#include"hal_defs.h"
#include"OSAL.h"
#include"npi.h"
#include"hal_uart.h"
#include"app_uart_init.h"


/****usart uart_recv_callback****/
void uart_recv_callback(uint8 port,uint8 event)
{
  uint8 rx_buffer[128];
  uint8 valid_len=0;
  (void)event;
  if((valid_len=NPI_RxBufLen())>0)
  NPI_ReadTransport(rx_buffer, valid_len);
#if UART_DEBUG > 0
  NPI_WriteTransport(rx_buffer,valid_len);
#endif
}


/****usart output****/

void app_write_string(char *p)
{
#if UART_DEBUG > 0
NPI_WriteTransport((uint8*)p,osal_strlen(p));
#else
(void) p;
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