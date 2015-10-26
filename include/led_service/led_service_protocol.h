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
//     short_data[0]:   duration - duration of all impulses, in ms
//     short_data[1]:   period   - period of one impulse, in ms


#endif // LED_SERVICE_PROTOCOL_H
