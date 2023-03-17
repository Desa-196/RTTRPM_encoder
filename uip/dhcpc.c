/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack
 *
 * @(#)$Id: dhcpc.c,v 1.2 2006/06/11 21:46:37 adam Exp $
 */

#include <stdio.h>
#include <string.h>

#include "uip.h"
#include "dhcpc.h"
#include "timer.h"
#include "pt.h"
#include "enc28j60.h"
#include "cmsis_os.h"
#include "main.h"


#include "math.h"


#define STATE_INITIAL         0
#define STATE_SENDING         1
#define STATE_OFFER_RECEIVED  2
#define STATE_CONFIG_RECEIVED 3

extern uint32_t latency[3];
extern xQueueHandle q;
extern xQueueHandle xQueueCommand;

//Множитель
extern int multiplier;
//Смещение
extern int offset;


//считаем номера пакетов и добавляем их в заголовок, необходимо для корректной работы предсказания будующего положения.
uint16_t index_rttrpm_packet = 0;
int count = 0;

#define SIZE_NAME 15

//Структура заголовка пакета RTTRPM
struct rttrpm
{
	//Заголовок rttrp пакета
	uint16_t	sigInt;									//Устанавливает порядок бит для типа int 
	uint16_t	sigFloat; 							//Устанавливает порядок бит для типа float
	uint16_t	version;								//Устанавливает версию RTTRPM
	uint32_t	id;											//Устанавливает номер пакета
	uint8_t		format;									//Формат пакета
	uint16_t	size_rttrpm_packet;			//Размер RTTRPM пакета заголовок+все включенные модули
	uint32_t	context;								//Контекст для передачи лубой пользовательской информации
	uint8_t		numOfTrackableModules;  //Количество вложенных в RTTRPM пакет модулей
	
	//TrackableWithoutTimestamp
	uint8_t type;
	uint16_t size_trackable_modul;
	uint8_t nameSize;
	char name[SIZE_NAME];
	uint8_t numOfModule;	
}

__attribute__((packed)) 
rttrpm_header = 

{
	.sigInt 				= 0x4154,
	.sigFloat				= 0x4334,
	.version 				= 0x0200,
	.id							= 0x00000000,
	.format 				= 1,
	.size_rttrpm_packet 	= 0,
	.context 				= 0xAF000000,
	.numOfTrackableModules 	= 1,
	.type 					= 1,
	.numOfModule 			= 1,
	.nameSize 				= SIZE_NAME
};


//Структура модуля позиции для передачи координат объекта
struct centroidPosition
{
	uint8_t type;
	uint16_t size;
	uint16_t latency;
	double x;
	double y;
	double z;
}__attribute__((packed)) 



position_modul = 

{
	.type = 2,
	.size = 0x1d00, //29
	.latency = 0, //Задержка между измерениями 10 млс
	.x = 0,
	.y = 0,
	.z = 0
};

//Структура модуля ориентации, передает кватернионы 
struct orientationEuler
{
	uint8_t type;
	uint16_t size;
  uint16_t latency;
	uint16_t order;
	double x;
  double y;
  double z;

}__attribute__((packed))

orientation_modul_euler = 

{
	.type = 4,
	.size = 0x1f00, //37
	.latency = 0x0100,
	.order = 0x0213,
	.x = 0,
	.y = 0,
	.z = 0,
};

//Структура модуля ориентации, передает кватернионы 
struct orientationQuaternion
{
	uint8_t type;
	uint16_t size;
  uint16_t latency;
	double x;
  double y;
  double z;
  double w;
}__attribute__((packed))

orientation_modul = 

{
	.type = 3,
	.size = 0x2500, //37
	.latency = 0x0100,
	.x = 0,
	.y = 0,
	.z = 0,
	.w = 0,
};

static struct dhcpc_state s;

struct dhcp_msg {
  u8_t op, htype, hlen, hops;
  u8_t xid[4];
  u16_t secs, flags;
  u8_t ciaddr[4];
  u8_t yiaddr[4];
  u8_t siaddr[4];
  u8_t giaddr[4];
  u8_t chaddr[16];
#ifndef UIP_CONF_DHCP_LIGHT
  u8_t sname[64];
  u8_t file[128];
#endif
  u8_t options[312];
};

#define BOOTP_BROADCAST 0x8000

#define DHCP_REQUEST        1
#define DHCP_REPLY          2
#define DHCP_HTYPE_ETHERNET 1
#define DHCP_HLEN_ETHERNET  6
#define DHCP_MSG_LEN      236

#define DHCPC_SERVER_PORT  67
#define DHCPC_CLIENT_PORT  68

#define DHCPDISCOVER  1
#define DHCPOFFER     2
#define DHCPREQUEST   3
#define DHCPDECLINE   4
#define DHCPACK       5
#define DHCPNAK       6
#define DHCPRELEASE   7

#define DHCP_OPTION_SUBNET_MASK   1
#define DHCP_OPTION_ROUTER        3
#define DHCP_OPTION_DNS_SERVER    6
#define DHCP_OPTION_REQ_IPADDR   50
#define DHCP_OPTION_LEASE_TIME   51
#define DHCP_OPTION_MSG_TYPE     53
#define DHCP_OPTION_SERVER_ID    54
#define DHCP_OPTION_REQ_LIST     55
#define DHCP_OPTION_HOST_NAME    12
#define DHCP_OPTION_END         255

static const u8_t xid[4] = {0xad, 0xde, 0x12, 0x23};
static const u8_t magic_cookie[4] = {99, 130, 83, 99};
static const u8_t client_identifier[] = "EncoderSender";

static u8_t *
add_msg_host_name(u8_t *optptr)
{
  *optptr++ = DHCP_OPTION_HOST_NAME;

  *optptr++ = sizeof(client_identifier);
	for(int8_t i = 0; i<sizeof(client_identifier); i++)
	{
		*optptr++ = client_identifier[i];
	} 
  return optptr;
}
/*---------------------------------------------------------------------------*/
static u8_t *
add_msg_type(u8_t *optptr, u8_t type)
{
  *optptr++ = DHCP_OPTION_MSG_TYPE;
  *optptr++ = 1;
  *optptr++ = type;
  return optptr;
}
/*---------------------------------------------------------------------------*/
static u8_t *
add_server_id(u8_t *optptr)
{
  *optptr++ = DHCP_OPTION_SERVER_ID;
  *optptr++ = 4;
  memcpy(optptr, s.serverid, 4);
  return optptr + 4;
}
/*---------------------------------------------------------------------------*/
static u8_t *
add_req_ipaddr(u8_t *optptr)
{
  *optptr++ = DHCP_OPTION_REQ_IPADDR;
  *optptr++ = 4;
  memcpy(optptr, s.ipaddr, 4);
  return optptr + 4;
}
/*---------------------------------------------------------------------------*/
static u8_t *
add_req_options(u8_t *optptr)
{
  *optptr++ = DHCP_OPTION_REQ_LIST;
  *optptr++ = 3;
  *optptr++ = DHCP_OPTION_SUBNET_MASK;
  *optptr++ = DHCP_OPTION_ROUTER;
  *optptr++ = DHCP_OPTION_DNS_SERVER;
  return optptr;
}
/*---------------------------------------------------------------------------*/
static u8_t *
add_end(u8_t *optptr)
{
  *optptr++ = DHCP_OPTION_END;
  return optptr;
}
/*---------------------------------------------------------------------------*/
static void
create_msg(register struct dhcp_msg *m)
{
  m->op = DHCP_REQUEST;
  m->htype = DHCP_HTYPE_ETHERNET;
  m->hlen = s.mac_len;
  m->hops = 0;
  memcpy(m->xid, xid, sizeof(m->xid));
  m->secs = 0;
  m->flags = HTONS(BOOTP_BROADCAST); /*  Broadcast bit. */
  /*  uip_ipaddr_copy(m->ciaddr, uip_hostaddr);*/
  memcpy(m->ciaddr, uip_hostaddr, sizeof(m->ciaddr));
  memset(m->yiaddr, 0, sizeof(m->yiaddr));
  memset(m->siaddr, 0, sizeof(m->siaddr));
  memset(m->giaddr, 0, sizeof(m->giaddr));
  memcpy(m->chaddr, s.mac_addr, s.mac_len);
  memset(&m->chaddr[s.mac_len], 0, sizeof(m->chaddr) - s.mac_len);
#ifndef UIP_CONF_DHCP_LIGHT
  memset(m->sname, 0, sizeof(m->sname));
  memset(m->file, 0, sizeof(m->file));
#endif

  memcpy(m->options, magic_cookie, sizeof(magic_cookie));
}
/*---------------------------------------------------------------------------*/
static void
send_discover(void)
{
  u8_t *end;
  struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;

  create_msg(m);

  end = add_msg_type(&m->options[4], DHCPDISCOVER);
	end = add_msg_host_name(&m->options[7]);
  end = add_req_options(end);
  end = add_end(end);

  uip_send(uip_appdata, end - (u8_t *)uip_appdata);
}
/*---------------------------------------------------------------------------*/
static void
send_request(void)
{
  u8_t *end;
  struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;

  create_msg(m);
  
  end = add_msg_type(&m->options[4], DHCPREQUEST);
	end = add_msg_host_name(&m->options[7]);
  end = add_server_id(end);
  end = add_req_ipaddr(end);
  end = add_end(end);
  
  uip_send(uip_appdata, end - (u8_t *)uip_appdata);
}
/*---------------------------------------------------------------------------*/
static u8_t
parse_options(u8_t *optptr, int len)
{
  u8_t *end = optptr + len;
  u8_t type = 0;

  while(optptr < end) {
    switch(*optptr) {
    case DHCP_OPTION_SUBNET_MASK:
      memcpy(s.netmask, optptr + 2, 4);
      break;
    case DHCP_OPTION_ROUTER:
      memcpy(s.default_router, optptr + 2, 4);
      break;
    case DHCP_OPTION_DNS_SERVER:
      memcpy(s.dnsaddr, optptr + 2, 4);
      break;
    case DHCP_OPTION_MSG_TYPE:
      type = *(optptr + 2);
      break;
    case DHCP_OPTION_SERVER_ID:
      memcpy(s.serverid, optptr + 2, 4);
      break;
    case DHCP_OPTION_LEASE_TIME:
      memcpy(s.lease_time, optptr + 2, 4);
      break;
    case DHCP_OPTION_END:
      return type;
    }

    optptr += optptr[1] + 2;
  }
  return type;
}
/*---------------------------------------------------------------------------*/
static u8_t
parse_msg(void)
{
  struct dhcp_msg *m = (struct dhcp_msg *)uip_appdata;
  
  if(m->op == DHCP_REPLY &&
     memcmp(m->xid, xid, sizeof(xid)) == 0 &&
     memcmp(m->chaddr, s.mac_addr, s.mac_len) == 0) {
    memcpy(s.ipaddr, m->yiaddr, 4);
    return parse_options(&m->options[4], uip_datalen());
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(handle_dhcp(void))
{
  PT_BEGIN(&s.pt);
  /* try_again:*/
  s.state = STATE_SENDING;
  s.ticks = CLOCK_SECOND;

  do {
    send_discover();
    timer_set(&s.timer, s.ticks);
    PT_WAIT_UNTIL(&s.pt, uip_newdata() || timer_expired(&s.timer));
		
    if(uip_newdata() && parse_msg() == DHCPOFFER) {
			uip_flags &= ~UIP_NEWDATA;
      s.state = STATE_OFFER_RECEIVED;
      break;
    }

    if(s.ticks < CLOCK_SECOND * 60) {
      s.ticks *= 2;
    }
  } while(s.state != STATE_OFFER_RECEIVED);

  s.ticks = CLOCK_SECOND;

  do {

    send_request();
    timer_set(&s.timer, s.ticks);
    PT_WAIT_UNTIL(&s.pt, uip_newdata() || timer_expired(&s.timer));
		if(uip_newdata() && parse_msg() == DHCPACK) {
			uip_flags &= ~UIP_NEWDATA;
      s.state = STATE_CONFIG_RECEIVED;
      break;
    }

    if(s.ticks <= CLOCK_SECOND * 10) {
      s.ticks += CLOCK_SECOND;
    } else {
      PT_RESTART(&s.pt);
    }
  } while(s.state != STATE_CONFIG_RECEIVED);
  
#if 0
  printf("Got IP address %d.%d.%d.%d\n",
	 uip_ipaddr1(s.ipaddr), uip_ipaddr2(s.ipaddr),
	 uip_ipaddr3(s.ipaddr), uip_ipaddr4(s.ipaddr));
  printf("Got netmask %d.%d.%d.%d\n",
	 uip_ipaddr1(s.netmask), uip_ipaddr2(s.netmask),
	 uip_ipaddr3(s.netmask), uip_ipaddr4(s.netmask));
  printf("Got DNS server %d.%d.%d.%d\n",
	 uip_ipaddr1(s.dnsaddr), uip_ipaddr2(s.dnsaddr),
	 uip_ipaddr3(s.dnsaddr), uip_ipaddr4(s.dnsaddr));
  printf("Got default router %d.%d.%d.%d\n",
	 uip_ipaddr1(s.default_router), uip_ipaddr2(s.default_router),
	 uip_ipaddr3(s.default_router), uip_ipaddr4(s.default_router));
  printf("Lease expires in %ld seconds\n",
	 ntohs(s.lease_time[0])*65536ul + ntohs(s.lease_time[1]));
#endif

  dhcpc_configured(&s);
  
  /*  timer_stop(&s.timer);*/

  /*
   * PT_END restarts the thread so we do this instead. Eventually we
   * should reacquire expired leases here.
   */
  while(1) {
    PT_YIELD(&s.pt);
  }

  PT_END(&s.pt);
}
/*---------------------------------------------------------------------------*/
void  dhcpc_configured(const struct dhcpc_state *s)
{
	uip_sethostaddr(s->ipaddr);
  uip_setnetmask(s->netmask);
  uip_setdraddr(s->default_router);
}

void
dhcpc_init(const void *mac_addr, int mac_len)
{
  uip_ipaddr_t addr;
  
  s.mac_addr = mac_addr;
  s.mac_len  = mac_len;

  s.state = STATE_INITIAL;
  uip_ipaddr(addr, 255,255,255,255);
  s.conn = uip_udp_new(&addr, HTONS(DHCPC_SERVER_PORT));
	
	uip_ipaddr_t addr_multicast;
  uip_ipaddr(addr_multicast, 238,210,10,3);
	//uip_ipaddr(addr_multicast, 172,20,70,2);
  uip_udp_new(&addr_multicast, HTONS(24002));
	
  if(s.conn != NULL) {
    uip_udp_bind(s.conn, HTONS(DHCPC_CLIENT_PORT));
  }
  //PT_INIT(&s.pt);
}
/*---------------------------------------------------------------------------*/
void
dhcpc_appcall(void)
{
	if(count == 1000)count = 0;
	else count++;
	
	if(uip_udp_conn->rport == HTONS(24002))
	{
		//if(count == 500)LD_TG;
		//Если в очереди появилось сообщение о новой позиции, отправляем 
		if(uxQueueMessagesWaiting(q) > 0)
		{
			
			count = 0;
			
			double xQmotorPosition[3];
			
			xQueueReceive( q, &( xQmotorPosition ), portMAX_DELAY ); 
			
			//position_modul.x = (xQmotorPosition[2] + offset)*multiplier;
			position_modul.x = xQmotorPosition[2]/multiplier;
			position_modul.y = xQmotorPosition[0];
			position_modul.z = xQmotorPosition[1];
			
			//Добавляем задержку между измерениями, она расчитывается в main.c
			position_modul.latency = __REV16(latency[2]);
//			
//			orientation_modul.w = cos(	(((xQmotorPosition)*3.14)/180)	/2.0		);
//			orientation_modul.x = 0;
//			orientation_modul.y = sin(	(((xQmotorPosition)*3.14)/180)	/2.0	);
//			orientation_modul.z = 0;
			
			//Добавляем в заголовок номер пакета. __REV16 - меняет порядок бит на обратный
			rttrpm_header.id = __REV16(index_rttrpm_packet);
			//Добавляем в заголовок размер RTTRPM-пакета, 
			rttrpm_header.size_rttrpm_packet 		= __REV16(33 + sizeof(position_modul));
			rttrpm_header.size_trackable_modul 	= __REV16(15 + sizeof(position_modul));
			
			
			
			//Имя источника данных, указывается в Tracking Input->Trackable ID программы WATCHOUT
			char name[] = "encoder2";
			memcpy(rttrpm_header.name, name, sizeof(name));

//			orientation_modul_euler.x = 0;
//			orientation_modul_euler.y = ((xQmotorPosition)*3.14)/180;
//			orientation_modul_euler.z = 0;

			//Объевляем массив msg в который засунем RTTRPM пакет для отправки
			uint8_t msg[sizeof(rttrpm_header) + sizeof(position_modul)];
			
			//Копируем в msg заголовок, начиная с 0-го элемента
			memcpy(&msg, &rttrpm_header, sizeof(rttrpm_header));
			//Копируеи в msg сразу после заголовка, модуль позиционирования (position_modul)
			memcpy(&msg[sizeof(rttrpm_header)], &position_modul, sizeof(position_modul));
			
			uint16_t	stat_link = enc28j60PhyReadScan()&0x04;
			if(stat_link != 0) LD_RED_OFF;
			else LD_RED_ON;
			
			//Отправляем пакет
			uip_send(msg, sizeof(msg));
			
			//Вычисляем номер пакета, если выходим за границы int16, обнуляем значение
			if(index_rttrpm_packet == 65535) index_rttrpm_packet = 0;
			else index_rttrpm_packet++;

			
			taskYIELD();
		}
	}
	else
	{
		//handle_dhcp();
	}
}
/*---------------------------------------------------------------------------*/
void
dhcpc_request(void)
{
  u16_t ipaddr[2];
  
  if(s.state == STATE_INITIAL) {
    uip_ipaddr(ipaddr, 0,0,0,0);
    uip_sethostaddr(ipaddr);
    /*    handle_dhcp(PROCESS_EVENT_NONE, NULL);*/
  }
}
/*---------------------------------------------------------------------------*/
