#ifndef LED_SERVICE_H
#define LED_SERVICE_H

#include "common.h"

// this function is called before FreeRTOS scheduler starts, in main function
// return 0 if failed
int led_service_create( void );


// ==========================================================================================
// ==========================================================================================


// call next functions only after FreeRTOS has been launched !!!

// send request to led service to flash a led
//   duration - duration of all impulses, in ms
//   period   - period of one impulse, in ms
void led_flash( unsigned int duration, unsigned int period, TickType_t ticks_to_wait );

// send request to led service to flash a led (to call from irq handler)
//   duration - duration of all impulses, in ms
//   period   - period of one impulse, in ms
// NOTE: this function makes forced context switching
void led_flash_irq( unsigned int duration, unsigned int period );

#endif // LED_SERVICE_H
