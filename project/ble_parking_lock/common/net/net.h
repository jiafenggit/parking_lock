#ifndef __NET_H__
#define __NET_H__

#define  NET_DEBUG             1

#define  NET_SOCKET            SOCKET1
#define  NET_STATUS_OK         1
#define  NET_STATE_ERR         2


void app_net_init();
void app_poll_net_status(uint16 elapse_time);//¶¨Ê±ÂÖÑ¯ÍøÂç×´Ì¬



#endif