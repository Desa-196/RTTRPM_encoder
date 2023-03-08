#include <stdio.h>
#include <string.h>

#include "uip.h"
#include "multicast.h"
#include "pt.h"
#include "enc28j60.h"
#include "cmsis_os.h"


#define STATE_INITIAL         0
#define STATE_SENDING         1
#define STATE_OFFER_RECEIVED  2
#define STATE_CONFIG_RECEIVED 3



static struct multicast_state s;

extern xQueueHandle q;

struct rttrpm_msg {
	//Заголовок пакета
	uint16_t	integer_signature;
	uint16_t	float_signature;
	uint16_t 	header_version;
	uint32_t 	packet_id;
	uint8_t 	packet_format;
	uint16_t 	packet_size;
	uint32_t 	packet_context;
	uint8_t 	h_count_module;
	
	//Packet Module without Timestamp
	
	uint8_t 	htype;
	uint16_t 	hmodul_size;
	uint8_t 	name_size;
	uint8_t 	name[11];
	uint8_t 	count_modul;
	
	//Centroid position
	
	uint8_t		type2;
	uint16_t	size;
	uint16_t	latency;
	double	  x_position;
	double	  y_position;
	double	  z_position;

};

static void
send_discover(void)
{  
  uip_send(uip_appdata, 105);
}

/*---------------------------------------------------------------------------*/
static
PT_THREAD(multicast_dhcp(void))
{
  PT_BEGIN(&s.pt);
  
	while(1)
	{
		char msg[20];
		WAIT_DATA_TO_SEND(&s->pt, uxQueueMessagesWaiting(q) > 0);
		int64_t xQmotorPosition;
		xQueueReceive( q, &( xQmotorPosition ), portMAX_DELAY ); 
		sprintf(msg, "%lld \r\n", xQmotorPosition);
		PSOCK_SEND_STR(&s->pt, msg);
	}
	

  PT_END(&s.pt);
}
/*---------------------------------------------------------------------------*/

void
multicast_init(const void *mac_addr, int mac_len)
{
  uip_ipaddr_t addr;
  
  s.mac_addr = mac_addr;
  s.mac_len  = mac_len;

  s.state = STATE_INITIAL;
  uip_ipaddr(addr, 238,210,10,3);
  s.conn = uip_udp_new(&addr, HTONS(24002));
  if(s.conn != NULL)
	{
  }
  PT_INIT(&s.pt);
}
/*---------------------------------------------------------------------------*/
void
dhcpc_appcall(void)
{
  multicast_dhcp();
}
/*---------------------------------------------------------------------------*/
