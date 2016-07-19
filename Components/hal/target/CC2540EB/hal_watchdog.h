#ifndef  HAL_WATCHDOG_H
#define  HAL_WATCHDOG_H





#define  WATCHDOG_IDLE              0x00
#define  WATCHDOG_RESERVED          0x01
#define  WATCHDOG_MODE              0x02
#define  WATCHDOG_TIMER             0x03

#define  CLOCK_PERIOD_1000MS        0x00
#define  CLOCK_PERIOD_250MS         0x01
#define  CLOCK_PERIOD_15MS          0x02
#define  CLOCK_PERIOD_2MS           0x03

#define  CLEAR_WATCHDOG_CMD_1       0x0A
#define  CLEAR_WATCHDOG_CMD_2       0x05



void hal_watchdog_init(uint8 mode,uint8 interval);
void hal_feed_watchdog();





#endif