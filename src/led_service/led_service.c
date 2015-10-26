// led control service

#include "led_service.h"
#include "led_service_private.h"
#include "led_service_protocol.h"

#include "service_control.h"

// queue that stores requests to this service
static QueueHandle_t service_queue;

// id of this service
static uint32_t service_id;

// led service thread-handler
//==============================================================================
static void led_thread( void* params )
{
  common_msg_t current_service_msg;
  portTickType cur_tick_num, start_tick_num;
  BaseType_t res;
  int ret;

  led_init();

  while( 1 )
  {
    res = xQueueReceive( service_queue, ( void* )&current_service_msg, portMAX_DELAY );
    if( res != pdTRUE )
      hardware_fail();

    switch( current_service_msg.type )
    {
      case 0 :
        ; // acknowledge handling
      break;

      case 1 :
      {
        start_tick_num = xTaskGetTickCount();
        cur_tick_num = start_tick_num;

        // of course it's not accurate time measurement, but...
        while( cur_tick_num < ( start_tick_num + current_service_msg.short_data[0] / portTICK_RATE_MS ) )
        {
          led_blink( current_service_msg.short_data[1] );
          cur_tick_num = xTaskGetTickCount();
        }
      }
      break;

      default :
        hardware_fail();	// what message we have received ?
      break;
    }

    if( must_send_reply( &current_service_msg ) )
    {
      ret = send_reply( service_id, &current_service_msg, portMAX_DELAY );
      if( !ret )
        hardware_fail();	// maybe it's very strictly ?
    }
  }
}

// this function is called before FreeRTOS scheduler starts, in main function
// return 0 if failed
//==============================================================================
int led_service_create( void )
{
  thread_t thread =
  {
      .thread_name = led_thread,
      .name = services_names[0],
      .stack_depth = 128,
      .params = NULL,
      .priority = 1,
      .hndl = NULL	// we aren't interesting in this handle
      };

  queue_t queue =
  {
      .length = 5,
      .queue_id = &service_queue // we will use this queue's id for reading events from queue
      };

  service_id = service_create( &thread, &queue );

  return service_id;
}
