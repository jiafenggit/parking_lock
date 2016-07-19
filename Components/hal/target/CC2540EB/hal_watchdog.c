#include "hal_defs.h"
#include "hal_mcu.h"
#include "hal_types.h"
#include "hal_watchdog.h"
#include "app_uart_init.h"




void hal_watchdog_init(uint8 mode,uint8 interval)
{
 if(mode>WATCHDOG_TIMER)
  return ;
 
 if(interval>CLOCK_PERIOD_2MS)
  return ;
 
 WDCTL|=(mode<<2)|(interval<<0);
 app_write_string("\r\nwatchdog init");
}

void hal_feed_watchdog()
{
  WDCTL|=(CLEAR_WATCHDOG_CMD_1<<4);
  WDCTL|=(CLEAR_WATCHDOG_CMD_2<<4);
  
  app_write_string("\r\nfeed dog!");
}









