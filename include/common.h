#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <string.h>

#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "list.h"
#include "queue.h"

#define malloc pvPortMalloc
#define free   vPortFree

// this macros can be used to determine message's type
// msg - must be pointer obtained from queue
#define get_msg_type( msg ) ( *((uint32_t*)msg) )

// this structure defines acknowledge message protocol
typedef struct ack_mesg_t
{
	uint32_t type; 			// must be always 0
	uint32_t service_id;	// id of service which has sent acknowledge
	uint32_t mesg_id;		// id of message that can be used by service which is acknowledged
							// this id equal to id that service specified when sent message
							// function of this parameters is to identify message from messages that
							// service has sent (service could send several messages to one service)
} ack_mesg_t;

extern void hardware_fail( void );

#endif // COMMON_H
