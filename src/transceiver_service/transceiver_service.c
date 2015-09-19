// transceiver control service

#include "transceiver_service.h"
#include "transceiver_private.h"
#include "led_service_protocol.h"

#include "service_control.h"

// queue that stores requests to this service
static QueueHandle_t service_queue;

// id of this service
static uint32_t service_id;

// transceiver service thread-handler
//==============================================================================
static void transceiver_thread( void* params )
{
  int ret;
  BaseType_t res;
  uint32_t led_service_id;
  portTickType last_wake_time;
  led_service_mesg_t led_service_mesg;

  void* temp; // currently we haven't protocol for this service

  // initialize n_rf24l01 library and transceiver
  ret = init_n_rf24l01();
  if( ret )
    hardware_fail();

  led_service_id = get_service_id( "led_service" );
  if( !led_service_id )
    hardware_fail();

  // prepare message to send to led_service
  led_service_mesg.type = 1;
  led_service_mesg.ack_on = 1;
  led_service_mesg.service_id_to_ack = service_id;
  led_service_mesg.duration = 1000;
  led_service_mesg.period = 250;

  last_wake_time = xTaskGetTickCount();

  while( 1 )
  {
    send_mesg( led_service_id, &led_service_mesg, portMAX_DELAY );
    led_service_mesg.mesg_id++;

    res = xQueueReceive( service_queue, ( void * )&temp, portMAX_DELAY );
    if( res != pdTRUE || !temp )
      hardware_fail();

    switch( get_msg_type( temp ) )
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
      .queue_id = &service_queue	// we will use this queue's id for reading events from queue
      };

  service_id = service_create( &thread, &queue );

  return service_id;
}
