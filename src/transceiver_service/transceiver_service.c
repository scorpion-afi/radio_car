// transceiver control service

#include "transceiver_service.h"
#include "led_service_protocol.h"

#include "service_control.h"

// queue that stores requests to this service
static QueueHandle_t cur_serv_queue;

// id of this service
static uint32_t cur_serv_id;

// transceiver service thread-handler
//==============================================================================
static void transceiver_thread( void* params )
{
  common_msg_t led_serv_mesg = { 0, };
  uint32_t led_serv_id;

  common_msg_t reply = { 0, };

  portTickType last_wake_time;
  BaseType_t res;

  led_serv_id = get_service_id( "led_service" );
  if( !led_serv_id )
    hardware_fail();

  // prepare message to send to led_service
  led_serv_mesg.type = 1;
  led_serv_mesg.short_data[0] = 1000;
  led_serv_mesg.short_data[1] = 250;

  last_wake_time = xTaskGetTickCount();

  while( 1 )
  {
    // we explicitly turn on reply ability - we specify service which to send reply as last parameter
    send_mesg( led_serv_id, &led_serv_mesg, portMAX_DELAY, cur_serv_id );
    wait_reply( cur_serv_id, &reply, portMAX_DELAY );

    // thread will be awaken each five second (5000 ms) and send message to led_service to blink led
    // xTimeIncrement (second parameter) - is interval in slices
    // portTICK_RATE_MS - slice time in ms
    vTaskDelayUntil( &last_wake_time, 5000 / portTICK_RATE_MS );
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
      .length = 5,
      .queue_id = &cur_serv_queue	// we will use this queue's id for reading events from queue
      };

  cur_serv_id = service_create( &thread, &queue );

  return cur_serv_id;
}
