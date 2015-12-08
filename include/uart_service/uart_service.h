#ifndef UART_SERVICE_H
#define UART_SERVICE_H

#include "common.h"

/**
 * @brief this function is called before FreeRTOS scheduler starts, in main function
 *
 * @return 0 if failed
 */
int uart_service_create( void );


// ==========================================================================================
// ==========================================================================================


// call next functions only after FreeRTOS has been launched !!!

/**
 * @brief send string to PC
 *
 * param[in] str - string to send over uart interface, must be truncated by '/0' symbol
 *
 * Note: string will be copied in previously allocated memory in heap
 *       inside function.
 */
void uart_send_str( const char* const str );

/**
 * @brief send string to PC (from irq handler)
 *
 * param[in] str - string to send over uart interface, must be truncated by '/0' symbol
 *
 * Note: string will be copied in previously allocated memory in heap
 *       inside function.
 *
 * NOTE: this function makes forced context switching
 */
void uart_send_str_irq( const char* const str );

#endif // UART_SERVICE_H
