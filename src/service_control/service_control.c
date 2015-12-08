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
    "transceiver_service",
    "uart_service"
};

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
// queue_info - pointers to structure described queue creation info (look to service_control.h),
//              specify NULL if you don't want to have queue within service
// return service's id, 0 if failed
//============================================================================================================
uint32_t service_create( thread_t* thread_info, queue_t* queue_info )
{
  uint32_t res;
  BaseType_t ret;
  QueueHandle_t queue_id = 0;
  TaskHandle_t hndl;

  if( !thread_info )
    return 0;

  ret = xTaskCreate( thread_info->thread_name, thread_info->name, thread_info->stack_depth, thread_info->params,
      thread_info->priority, &hndl );
  if( ret != pdPASS )
    return 0;

  if( thread_info->hndl )
    *thread_info->hndl = hndl;

  if( queue_info && queue_info->length && queue_info->elm_size )
  {
    queue_id = xQueueCreate( queue_info->length, queue_info->elm_size );
    if( !queue_id )
      goto fail_1;

    if( queue_info->queue_id )
      *queue_info->queue_id = queue_id;
  }

  res = _add_service_to_list( thread_info->name, queue_id );
  if( res )
    return res;

  if( !queue_id )
    goto fail_1;

  vQueueDelete( queue_id );

  fail_1: vTaskDelete( hndl );
  return 0;
}

// send a message to queue associated with service identified by @serv_id_to
// serv_id_to - id of service message must be sent which
// data - pointer to data to be copied (enqueue) into queue
// ticks_to_wait - amount of ticks, service will be sleep during which, 0 - means function returns control immediately
// return 0 if failed
// NOTE: function will copy amount of bytes specified during service creation, from memory @data points to
//============================================================================================================
int send_mesg( uint32_t serv_id_to, const void* data, TickType_t ticks_to_wait )
{
  BaseType_t res;
  service_element_t* service = NULL;

  if( !data || !serv_id_to )
    return 0;

  // list over list of services for look up service
  list_for_each_entry( service, &services_list, list_item )
  {
    if( service->service_id == serv_id_to )
    {
      if( !service->queue_id )
        return 0;

      res = xQueueSendToBack( service->queue_id, data, ticks_to_wait );
      if( res != pdTRUE )
        return 0;

      return 1;
    }
  }

  return 0;
}
