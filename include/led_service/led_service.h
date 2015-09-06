#ifndef LED_SERVICE_H
#define LED_SERVICE_H

// this function is called before FreeRTOS scheduler starts, in main function
// return 0 if failed
int led_service_create( void );

#endif // LED_SERVICE_H
