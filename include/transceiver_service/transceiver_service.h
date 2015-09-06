#ifndef TRANSCEIVER_SERVICE_H
#define TRANSCEIVER_SERVICE_H

// this function is called before FreeRTOS scheduler starts, in main function
// return 0 if failed
int transceiver_service_create( void );

#endif // TRANSCEIVER_SERVICE_H
