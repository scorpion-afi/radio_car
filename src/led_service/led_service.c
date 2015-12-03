// led control service

#include "led_service.h"
#include "led_service_private.h"

#include "service_control.h"

// id of this (current) service
static uint32_t cur_serv_id;

// queue that stores requests to this service
static QueueHandle_t cur_serv_queue;

typedef struct led_request_t
{
  unsigned int duration;
  unsigned int period;
} led_request_t;

// send request to led service to flash a led
//   duration - duration of all impulses, in ms
//   period   - period of one impulse, in ms
//   ticks_to_wait - amount of ticks, you will be sleep during which, 0 - means function returns control immediately
//==============================================================================
void led_flash( unsigned int duration, unsigned int period, TickType_t ticks_to_wait )
{
  led_request_t led_request = { 0, };

  if( !cur_serv_id )
    return;

  led_request.duration = duration;
  led_request.period = period;
  send_mesg( cur_serv_id, &led_request, ticks_to_wait );
}

// send request to led service to flash a led (to call from irq handler)
//   duration - duration of all impulses, in ms
//   period   - period of one impulse, in ms
// NOTE: this function makes forced context switching
//==============================================================================
void led_flash_irq( unsigned int duration, unsigned int period )
{
  portBASE_TYPE force_context_switch = pdFALSE; // must be explicitly reset
  led_request_t led_request = { 0, };

  if( !cur_serv_id )
    return;

  led_request.duration = duration;
  led_request.period = period;

  xQueueSendFromISR( cur_serv_queue, &led_request, &force_context_switch );

  // after irq processing will be finished switch to woken high priority task immediately
  portEND_SWITCHING_ISR( force_context_switch );
}


//========================================================================================================
//========================================================================================================


// led service thread-handler
//==============================================================================
static void led_thread( void* params )
{
  portTickType cur_tick_num, start_tick_num;
  led_request_t cur_serv_msg = { 0, };
  BaseType_t res;

  led_init();

  while( 1 )
  {
    res = xQueueReceive( cur_serv_queue, ( void* )&cur_serv_msg, portMAX_DELAY );
    if( res != pdTRUE )
      hardware_fail();

    start_tick_num = xTaskGetTickCount();
    cur_tick_num = start_tick_num;

    // of course it's not accurate time measurement, but...
    while( cur_tick_num < ( start_tick_num + cur_serv_msg.duration / portTICK_RATE_MS ) )
    {
      led_blink( cur_serv_msg.period );
      cur_tick_num = xTaskGetTickCount();
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
      .elm_size = sizeof(led_request_t),
      .length = 5,
      .queue_id = &cur_serv_queue // we will use this queue's id for reading events from queue
      };

  cur_serv_id = service_create( &thread, &queue );

  return cur_serv_id;
}
