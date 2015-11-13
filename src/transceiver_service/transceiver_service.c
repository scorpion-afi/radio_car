// transceiver control service

#include "transceiver_service.h"
#include "led_service.h"

#include "service_control.h"

// queue that stores requests to this service
static QueueHandle_t cur_serv_queue;

// id of this service
static uint32_t cur_serv_id;

// transceiver service thread-handler
//==============================================================================
static void transceiver_thread( void* params )
{
  portTickType last_wake_time;

  last_wake_time = xTaskGetTickCount();

  while( 1 )
  {
    led_flash( 600, 200, portMAX_DELAY );

    // thread will be awaken each three second (3000 ms) and send message to led_service to blink led
    // xTimeIncrement (second parameter) - is interval in slices
    // portTICK_RATE_MS - slice time in ms
    vTaskDelayUntil( &last_wake_time, 3000 / portTICK_RATE_MS );
  }
}

// this function is called before FreeRTOS scheduler starts, in main function
// return 0 if failed
//==============================================================================
int transceiver_service_create( void )
{
  thread_t thread =
  {
      .thread_name = transceiver_thread,
      .name = services_names[1],
      .stack_depth = 128,
      .params = NULL,
      .priority = 2,
      .hndl = NULL	// we aren't interesting in this handle
      };

  queue_t queue =
  {
      .elm_size = 1,
      .length = 1,
      .queue_id = &cur_serv_queue	// we will use this queue's id for reading events from queue
      };

  cur_serv_id = service_create( &thread, &queue );

  return cur_serv_id;
}
