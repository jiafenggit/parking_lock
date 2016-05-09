#include "osal.h"

typedef struct uart_debug_cmd
{
  osal_event_hdr_t hdr;
  uint8  cmd;
}uart_debug_cmd_t;


#define  UART_DEBUG_CMD_START_DISCOVER       '1'
#define  UART_DEBUG_CMD_STOP_DISCOVER        '2'
#define  UART_DEBUG_CMD_CONNECT              '3'
#define  UART_DEBUG_CMD_DISCONNECT           '4'
#define  UART_DEBUG_CMD_START_ADC            '5'
#define  UART_DEBUG_CMD_START_SCAN_CAR       '6'
#define  UART_DEBUG_CMD_START_POSITIVE_RUN   '7'
#define  UART_DEBUG_CMD_START_NEGATIVE_RUN   '8'



uint8 register_for_uart_debug(uint8 task_id);
void uart_recv_callback(uint8 port,uint8 event);
void app_write_string(char *p);
char *uint16_to_string(uint16 src);
char *uint8_to_string(uint8 src);
void uint8_array_to_string(uint8 *pbuff,uint8 len);