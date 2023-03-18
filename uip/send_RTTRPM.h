#ifndef __DHCPC_H__
#define __DHCPC_H__
#include "stm32f1xx_hal.h"
#include "timer.h"
#include "pt.h"

#define u16_t uint16_t
#define u8_t uint8_t

struct RTTRPM_state {
  struct pt pt;
  struct uip_udp_conn *conn;
  struct timer timer;
  u16_t ticks;
  const void *mac_addr;
  int mac_len;
  
  u8_t serverid[4];

  u16_t lease_time[2];
  u16_t ipaddr[2];
  u16_t netmask[2];
  u16_t dnsaddr[2];
  u16_t default_router[2];
};

void RTTRPM_init(const void *mac_addr, int mac_len);

void RTTRPM_appcall(void);

typedef struct RTTRPM_state uip_udp_appstate_t;
#define UIP_UDP_APPCALL RTTRPM_appcall


#endif /* __DHCPC_H__ */
