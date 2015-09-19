#include "service_control.h"
#include "linux_kernel_list.h"

// this structure describes service
typedef struct service_element_t
{
  struct list_head list_item;	// for insert in list
  uint32_t service_id;		// id of service
  const char* service_name;	// name of service
  QueueHandle_t queue_id;		// id of queue, which associated with this service
} service_element_t;

const char* services_names[] =
{
    "led_service",
    "transceiver_service" };

// amount of bytes pointer consists of (we are on a 32 bit architecture)
typedef enum queue_item_size_t
{
  SIZE_OF_PTR = 4
} queue_item_size_t;

// list of all services (list of struct service_element_t)
static struct list_head services_list;

// internal function
// allocate memory for new service, fill service_name, queue_id and service_id fields
// and add service to list of services
// return service's id, 0 if failed
//============================================================================================================
static uint32_t _add_service_to_list( const char* service_name, QueueHandle_t queue_id )
{
  service_element_t* service = NULL;
  static int first = 1;
  static uint32_t service_id = 1; // first service has id equal 1, because of 0 identifies fail

  if( !service_name )
    return 0;

  // we must initialize list before use it
  if( first )
  {
    first = 0;
    INIT_LIST_HEAD( &services_list );
  }

  service = malloc( sizeof(service_element_t) );
  if( !service )
    return 0;

  list_add_tail( &service->list_item, &services_list );

  service->service_id = service_id;
  service->service_name = service_name;
  service->queue_id = queue_id;

  return service_id++;
}

// create service: thread with associated queue
// thread_info - pointers to structure described thread creation info (look to service_control.h)
// queue_info - pointers to structure described queue creation info (look to service_control.h)
// return service's id, 0 if failed

// Note: each service's queue stores pointers to data, not data !!!
//============================================================================================================
uint32_t service_create( thread_t* thread_info, queue_t* queue_info )
{
  uint32_t res;
  BaseType_t ret;
  QueueHandle_t queue_id;

  if( !thread_info || !queue_info )
    return 0;

  ret = xTaskCreate( thread_info->thread_name, thread_info->name, thread_info->stack_depth, thread_info->params,
      thread_info->priority, thread_info->hndl );
  if( ret != pdPASS )
    return 0;

  queue_id = xQueueCreate( queue_info->length, SIZE_OF_PTR );
  if( !queue_id )
    goto fail_1;

  if( queue_info->queue_id )
    *queue_info->queue_id = queue_id;

  res = _add_service_to_list( thread_info->name, queue_id );
  if( !res )
    goto fail_2;

  return res;

  fail_2: vQueueDelete( queue_id );
  fail_1: if( thread_info->hndl )
    vTaskDelete( *thread_info->hndl );
  return 0;
}

// give service's id from service's name
// service_name - name of service (look to services_names array to know existed services names)
// return service's id, 0 if failed
//============================================================================================================
uint32_t get_service_id( const char* service_name )
{
  service_element_t* service = NULL;

  if( !service_name )
    return 0;

  // list over list of services for look up service
  list_for_each_entry( service, &services_list, list_item )
  {
    if( !strcmp( service_name, service->service_name ) )
      return service->service_id;
  }

  return 0;
}

// send a message to queue associated with service identified by @service_id
// service_id - id of service message must be sent which
// mesg - pointer to data, VALUE of pointer will be copied to queue (queued to queue)
// ticks_to_wait - amount of ticks, task will be sleep during which, 0 - means function return
// 				   control immediately
// return 0 if failed

// Note: you must keep safe data you have passed pointer to, due to queue stores just a pointer
//		 and when service-sink will handle this message it must have correct data.
//============================================================================================================
int send_mesg( uint32_t service_id, const void* mesg, TickType_t ticks_to_wait )
{
  BaseType_t res;
  service_element_t* service = NULL;

  if( !service_id || !mesg )
    return 0;

  // list over list of services for look up service
  list_for_each_entry( service, &services_list, list_item )
  {
    if( service->service_id == service_id )
    {
      res = xQueueSendToBack( service->queue_id, &mesg, ticks_to_wait );
      if( res != pdTRUE )
        return 0;

      return 1;
    }
  }

  return 0;
}

// send acknowledge message
// service_from_id - id of service which sends acknowledge
// service_to_id   - id of service which will receive acknowledge
// mesg_id         - id of message, look to ack_mesg_t declaration
// return 0 if failed
//============================================================================================================
int send_ack_mesg( uint32_t service_from_id, uint32_t service_to_id, uint32_t mesg_id )
{
  int ret;
  static ack_mesg_t ack_mesg;

  if( !service_from_id || !service_to_id )
    return 0;

  ack_mesg.type = 0;
  ack_mesg.service_id = service_from_id;
  ack_mesg.mesg_id = mesg_id;

  ret = send_mesg( service_to_id, &ack_mesg, portMAX_DELAY );
  if( !ret )
    return 0;

  return 1;
}
