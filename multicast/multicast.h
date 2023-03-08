#include "stm32f1xx_hal.h"
#include "pt.h"

#define u16_t uint16_t
#define u8_t uint8_t

struct multicast_state {
  struct pt pt;
  char state;
  struct uip_udp_conn *conn;
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

void multicast_init(const void *mac_addr, int mac_len);

void multicast_appcall(void);

typedef struct multicast_state uip_udp_appstate_t;
#define UIP_UDP_APPCALL multicast_appcall