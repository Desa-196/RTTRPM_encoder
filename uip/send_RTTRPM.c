#include <stdio.h>
#include <string.h>

#include "uip.h"
#include "send_RTTRPM.h"
#include "timer.h"
#include "pt.h"
#include "enc28j60.h"
#include "cmsis_os.h"
#include "main.h"


#include "math.h"

extern uint32_t latency[3];
extern xQueueHandle q;
extern xQueueHandle xQueueCommand;

//Множитель
extern int multiplier;
//Смещение
extern int offset;


//считаем номера пакетов и добавляем их в заголовок, необходимо для корректной работы предсказания будующего положения.
uint16_t index_rttrpm_packet = 0;

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

static struct RTTRPM_state s;



void
RTTRPM_init(const void *mac_addr, int mac_len)
{
  uip_ipaddr_t addr;
  
  s.mac_addr = mac_addr;
  s.mac_len  = mac_len;

  uip_ipaddr(addr, 255,255,255,255);
  
	
	uip_ipaddr_t addr_multicast;
  uip_ipaddr(addr_multicast, 238,210,10,3);
	//uip_ipaddr(addr_multicast, 172,20,70,2);
  s.conn = uip_udp_new(&addr_multicast, HTONS(24002));
	if(s.conn != NULL) {
    uip_udp_bind(s.conn, HTONS(1025));
  }
  //PT_INIT(&s.pt);
}
/*---------------------------------------------------------------------------*/
void
RTTRPM_appcall(void)
{
	
	if(uip_udp_conn->rport == HTONS(24002))
	{
		//if(count == 500)LD_TG;
		//Если в очереди появилось сообщение о новой позиции, отправляем 
		if(uxQueueMessagesWaiting(q) > 0)
		{
			
			double xQmotorPosition[3];
			
			xQueueReceive( q, &( xQmotorPosition ), portMAX_DELAY ); 
			
			//position_modul.x = (xQmotorPosition[2] + offset)*multiplier;
			position_modul.x = (xQmotorPosition[2]+offset)/multiplier;
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
			rttrpm_header.size_rttrpm_packet 		= __REV16(38 + sizeof(position_modul));
			rttrpm_header.size_trackable_modul 	= __REV16(5 + sizeof(rttrpm_header.name) + sizeof(position_modul));
			
			
			
			//Имя источника данных, указывается в Tracking Input->Trackable ID программы WATCHOUT
			
			char name[] = "pytest2";
			
			memcpy(rttrpm_header.name, name, sizeof(name)-1);

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

