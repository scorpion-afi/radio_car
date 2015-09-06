#ifndef LED_SERVICE_PROTOCOL_H
#define LED_SERVICE_PROTOCOL_H

#include "common.h"

// This file defines led_control service communication protocol.

// Acknowledge - is feature which allows to inform service, that has sent message,
// that message was handled. If you want to be acknowledged, turn on it, simple
// set ack_on field to 1 and specify id of service to be acknowledged.
// If you allocated memory for message or something tied with it you can use
// this feature to free memory in your thread.
// Note: responsibility to free is only your, only you know when memory can be freed,
//       this feature only allow you to know that the service, which you has sent message,
//		 no longer needs in this memory.

// type:
//   1 - set up led blinking:
//       duration - duration of all impulses, in ms
//       period	  - period of one impulse, in ms

// this structure defines message structure to be sent to this service
typedef struct led_service_mesg_t
{
	uint32_t type;		// type of message, indicates how to interpret parameters
	uint32_t ack_on;	// whether acknowledge will be sent or not (0 - off, 1 - on)
	uint32_t service_id_to_ack;	// id of service which acknowledge will be sent
	uint32_t mesg_id;	// id associated with this message, used for acknowledge
	uint32_t duration;
	uint32_t period;

} led_service_mesg_t;

#endif // LED_SERVICE_PROTOCOL_H
