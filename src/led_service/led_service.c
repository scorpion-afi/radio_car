// led control service

#include "led_service.h"
#include "led_service_private.h"
#include "led_service_protocol.h"

#include "service_control.h"

// queue that stores requests to this service
static QueueHandle_t cur_serv_queue;

// id of this (current) service
static uint32_t cur_serv_id;

// led service thread-handler
//==============================================================================
static void led_thread( void* params )
{
  portTickType cur_tick_num, start_tick_num;
  common_msg_t cur_serv_msg = { 0, };
  BaseType_t res;
  int ret;

  led_init();

  while( 1 )
  {
    res = xQueueReceive( cur_serv_queue, ( void* )&cur_serv_msg, portMAX_DELAY );
    if( res != pdTRUE )
      hardware_fail();

    switch( cur_serv_msg.type )
    {
      case 0 :
        ; // acknowledge handling
      break;

      case 1 :
      {
        start_tick_num = xTaskGetTickCount();
        cur_tick_num = start_tick_num;

        // of course it's not accurate time measurement, but...
        while( cur_tick_num < ( start_tick_num + cur_serv_msg.short_data[0] / portTICK_RATE_MS ) )
        {
          led_blink( cur_serv_msg.short_data[1] );
          cur_tick_num = xTaskGetTickCount();
        }
      }
      break;

      default :
        hardware_fail();	// what message we have received ?
      break;
    }

    if( must_send_reply( &cur_serv_msg ) )
    {
      ret = send_reply( cur_serv_id, &cur_serv_msg, portMAX_DELAY );
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
      .queue_id = &cur_serv_queue // we will use this queue's id for reading events from queue
      };

  cur_serv_id = service_create( &thread, &queue );

  return cur_serv_id;
}
