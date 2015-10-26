// transceiver control service

#include "transceiver_service.h"
#include "led_service_protocol.h"

#include "service_control.h"

// queue that stores requests to this service
static QueueHandle_t cur_service_queue;

// id of this service
static uint32_t cur_service_id;

// transceiver service thread-handler
//==============================================================================
static void transceiver_thread( void* params )
{
  BaseType_t res;
  uint32_t led_service_id;
  portTickType last_wake_time;
  common_msg_t led_service_mesg = { 0, };
  common_msg_t current_service_msg = { 0, };
  common_msg_t reply = { 0, };

  led_service_id = get_service_id( "led_service" );
  if( !led_service_id )
    hardware_fail();

  // prepare message to send to led_service
  led_service_mesg.type = 1;
  led_service_mesg.short_data[0] = 1000;
  led_service_mesg.short_data[1] = 250;

  last_wake_time = xTaskGetTickCount();

  while( 1 )
  {
    send_mesg( led_service_id, &led_service_mesg, portMAX_DELAY, cur_service_id );
    wait_reply( cur_service_id, &reply, portMAX_DELAY );

    /*
    res = xQueueReceive( cur_service_queue, ( void * )&current_service_msg, portMAX_DELAY );
    if( res != pdTRUE )
      hardware_fail();

    switch( current_service_msg.type )
    {
      // acknowledge handling
      case 0 :
        ;	// currently there are nothing to do
          // we can make next cast (ack_mesg_t*)temp and we will be able to use
          // next fields: service_id and mesg_id
      break;

      default :
        hardware_fail();	// what message we have received ?
      break;
    }
     */

    // thread will be awaken each five second (5000 ms) and send message to led_service to blink led
    // xTimeIncrement (second parameter) - is interval in slices
    // portTICK_RATE_MS - slice time in ms
    vTaskDelayUntil( &last_wake_time, 2000 / portTICK_RATE_MS );
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
      .queue_id = &cur_service_queue	// we will use this queue's id for reading events from queue
      };

  cur_service_id = service_create( &thread, &queue );

  return cur_service_id;
}
