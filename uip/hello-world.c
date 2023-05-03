/**
 * \addtogroup helloworld
 * @{
 */

/**
 * \file
 *         An example of how to write uIP applications
 *         with protosockets.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

/*
 * This is a short example of how to write uIP applications using
 * protosockets.
 */

/*
 * We define the application state (struct hello_world_state) in the
 * hello-world.h file, so we need to include it here. We also include
 * uip.h (since this cannot be included in hello-world.h) and
 * <string.h>, since we use the memcpy() function in the code.
 */
#include "hello-world.h"
#include <stdlib.h>
#include "uip.h"
#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "enc28j60.h"
#include "main.h"
/*
 * Declaration of the protosocket function that handles the connection
 * (defined at the end of the code).
 */
 
 //Множитель
extern int multiplier;
//Смещение
extern int offset;
extern uint8_t reg;

static int handle_connection(struct hello_world_state *s);
struct hello_world_state *s;
uint8_t state_connection2 = 0;
uint8_t isConnected = 0;

extern xQueueHandle q;
extern xQueueHandle xQueueCommand;
/*---------------------------------------------------------------------------*/
/*
 * The initialization function. We must explicitly call this function
 * from the system initialization code, some time after uip_init() is
 * called.
 */
void
hello_world_init(void)
{
  /* We start to listen for connections on TCP port 1000. */
  uip_listen(HTONS(9101));
	uip_listen(HTONS(9102));
}
/*---------------------------------------------------------------------------*/
/*
 * In hello-world.h we have defined the UIP_APPCALL macro to
 * hello_world_appcall so that this funcion is uIP's application
 * function. This function is called whenever an uIP event occurs
 * (e.g. when a new connection is established, new data arrives, sent
 * data is acknowledged, data needs to be retransmitted, etc.).
 */
void
hello_world_appcall(void)
{
  /*
   * The uip_conn structure has a field called "appstate" that holds
   * the application state of the connection. We make a pointer to
   * this to access it easier.
   */
  s = &(uip_conn->appstate);

  /*
   * If a new connection was just established, we should initialize
   * the protosocket in our applications' state structure.
   */
	//Если кто-то приконнектился
  if(uip_connected()) {
    //Если порт к которому приконнектились 9101
		if(uip_conn->lport == HTONS(9101)){
			//Если уже есть подключение к этому порту, то отклоняем новое подключение.
			if(isConnected == 1)
			{
				uip_abort();
				return;
			}
			//Если до этого подключений не было, то запускаем новый протопоток обработки соединения
			else{
				PSOCK_INIT(&s->p, s->inputbuffer, sizeof(s->inputbuffer));
				isConnected = 1;
			}
		}
		//Если порт к которому приконнектились 9102, то запускаем новый протопоток обработки комманд
		if(uip_conn->lport == HTONS(9102)){
			
			PSOCK_INIT(&s->p, s->inputbuffer, sizeof(s->inputbuffer));

		}
			//xQueueSend( xQueueCommand , ( void * ) SEND_MOTOR_POSITION, portMAX_DELAY  );
		
  }


  /*
   * Finally, we run the protosocket function that actually handles
   * the communication. We pass it a pointer to the application state
   * of the current connection.
   */
	if(uip_conn->lport == HTONS(9101)){
		if (uip_closed() || uip_aborted() || uip_timedout()) 
		{
			isConnected = 0;
			uip_abort();
		}
		handle_connection(s);
	}
	else if(uip_conn->lport == HTONS(9102))
	{
		if (uip_closed() || uip_aborted() || uip_timedout()) 
		{
			uip_abort();
		}
		handle_connection_command(s);
	}
	//handle_connection(s);
}


/*
 * This is the protosocket function that handles the communication. A
 * protosocket function must always return an int, but must never
 * explicitly return - all return statements are hidden in the PSOCK
 * macros.
 */


static int
handle_connection_command(struct hello_world_state *s)
{
	PSOCK_BEGIN(&s->p);

  PSOCK_SEND_STR(&s->p, "Hello. this is encoder to RTTRPM modul.\n\rPlease enter command. \n\rEnter exit to disconnect.\n\r");
	sprintf(s->value, "%i", reg);
	PSOCK_SEND_STR(&s->p, s->value);
	memset(s->inputbuffer, 0, 40);
  PSOCK_READTO(&s->p, '\n');
	
	
	char *p = strtok(s->inputbuffer,"=");

	if(p != NULL)		
	{
		memset(s->name, 0, 40);
		memset(s->value, 0, 40);
		
		memcpy(s->name, p, strlen(p));
		
		p = strtok(NULL, "=");
		
		if(p != NULL)
		{
			memcpy(s->value, p, strlen(p));
			s->value[strlen(p)-1] = 0;
			
			//Записываем ссылку на первое вхождение символа ','
			char *ref = strchr(s->value, ',');
			//Если есть такой символ, меняем его на точку
			if(ref) ref[0] = '.';
		}
		
	}

	//Если в cтроке name есть подстрока exit разрываем соединение. Реализовал отдельным if, так как PSOCK_END должен быть в последнем else, иначе ругается при компиляции
	if( strstr(s->name, "exit") == NULL )
	{
		PSOCK_SEND_STR(&s->p, "\n");
		//Если в cтроке name есть подстрока multi то считаем что хотим установить множитель
		if( strstr(s->name, "getmulti") )
		{		
			sprintf(s->value, "%i", multiplier);
			PSOCK_SEND_STR(&s->p, "Multiply = ");
			PSOCK_SEND_STR(&s->p, s->value);
			PSOCK_SEND_STR(&s->p, "\n\r");
		}		
		else if ( strstr(s->name, "getoffset") )
		{
			sprintf(s->value, "%i", offset);
			PSOCK_SEND_STR(&s->p, "Offset = ");
			PSOCK_SEND_STR(&s->p, s->value);
			PSOCK_SEND_STR(&s->p, "\n\r");
		}
		else if( strstr(s->name, "multi") )
		{		
			char * ptrEnd;
			multiplier = atoi (s->value);
			memset(s->value, 0, 40);
			sprintf(s->value, "%i", multiplier);
			PSOCK_SEND_STR(&s->p, "Set multiply = ");
			PSOCK_SEND_STR(&s->p, s->value);
			PSOCK_SEND_STR(&s->p, "\n\r");
		}		
		else if ( strstr(s->name, "offset") )
		{
			char * ptrEnd;
			offset = atoi (s->value);
			memset(s->value, 0, 40);
			sprintf(s->value, "%i", offset);
			PSOCK_SEND_STR(&s->p, "Set offset = ");
			PSOCK_SEND_STR(&s->p, s->value);
			PSOCK_SEND_STR(&s->p, "\n\r");
		}
		else if ( strstr(s->name, "getproperty") )
		{
			char * ptrEnd;
			offset = atoi (s->value);
			memset(s->value, 0, 40);
			sprintf(s->value, "%i", offset);
			PSOCK_SEND_STR(&s->p, "Set offset = ");
			PSOCK_SEND_STR(&s->p, s->value);
			PSOCK_SEND_STR(&s->p, "\n\r");
		}

		else
		{
				PSOCK_SEND_STR(&s->p, "Command not found\n\r");
		}
		PSOCK_SEND_STR(&s->p, "\n");
	}
	else
	{
		PSOCK_SEND_STR(&s->p, "Exit\n\r");
		PSOCK_CLOSE(&s->p);
		PSOCK_END(&s->p);
	}
	

	
	/*
  PSOCK_BEGIN(&s->p);
	while(1)
	{	
		PSOCK_READTO(&s->p, '\n');
		strncpy(s->name, s->inputbuffer, sizeof(s->name));
		xQueueSend( xQueueCommand , ( void * ) &s->name, portMAX_DELAY  );
	}
	PSOCK_CLOSE(&s->p);
	PSOCK_END(&s->p);
	*/

}

static int
handle_connection(struct hello_world_state *s)
{
  PSOCK_BEGIN(&s->p);

	while(1)
	{
		char msg[20];
		WAIT_DATA_TO_SEND(&s->p, uxQueueMessagesWaiting(q) > 0);
		double xQmotorPosition;
		xQueueReceive( q, &( xQmotorPosition ), portMAX_DELAY ); 
		sprintf(msg, "%lf \r\n", xQmotorPosition);
		PSOCK_SEND_STR(&s->p, msg);
	}
  PSOCK_CLOSE(&s->p);
  PSOCK_END(&s->p);

}
/*---------------------------------------------------------------------------*/
