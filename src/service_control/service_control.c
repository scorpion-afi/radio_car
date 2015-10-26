#include "service_control.h"
#include "linux_kernel_list.h"

// 0x0f - mask for service id to reply (up to 16 services)
// 0xf0 - mask for service id from which reply has been sent (up to 16 services)
// 0xffffff00 - mask for message counter

// message counter - id of message that can be used by service which is acknowledged.
// this id equal to id that service specified when sent message.
// function of this parameters is to identify message from messages that
// service has sent (service could send several messages to one service).

// these macros control @control field in common_msg_t structure
#define set_id_to_reply( control, id ) do { control &= ~0x0f; id &= 0x0f; control |= id; } while(0)
#define get_id_to_reply( control ) ( control & 0x0f )

#define set_id_from_reply( control, id ) do { control &= ~0xf0; id &= 0xf; control |= id << 4; } while(0)
#define get_id_from_reply( control ) ( ( control & 0xf0 ) >> 4 )

// automatic rounding when counter reached up to 16777215 (24 bits)
#define inc_mesg_cnt( control ) do { int cnt = ( ( control & ~0xff ) >> 8 ) + 1; control |= cnt << 8; } while(0)
#define get_mesg_cnt( control ) ( ( control & ~0xff ) >> 8 )

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

  queue_id = xQueueCreate( queue_info->length, sizeof(common_msg_t) );
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
// mesg - pointer to common_msg_t structure (common message), pointer will be dereferenced and in queue will
//        be copied data @mesg points to. structure must be zero-filled after creation !!!
// ticks_to_wait - amount of ticks, service will be sleep during which, 0 - means function return control immediately
// service_to_reply - id of service to send reply, specify 0 if you aren't interesting in sending of a reply
// return 0 if failed

// Note: message will be queued by copy, not by reference, so you may destroy data 'mesg' points to.
//============================================================================================================
int send_mesg( uint32_t service_id, common_msg_t* mesg, TickType_t ticks_to_wait, uint32_t service_to_reply )
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
      inc_mesg_cnt( mesg->control );
      set_id_to_reply( mesg->control, service_to_reply );

      res = xQueueSendToBack( service->queue_id, mesg, ticks_to_wait );
      if( res != pdTRUE )
        return 0;

      return 1;
    }
  }

  return 0;
}

// send reply message
// service_from_id - id of service which sends reply
// mesg   - pointer to structure defined reply, must be same structure that was dequeued from service queue
// ticks_to_wait - amount of ticks, service will be sleep during which, 0 - means function return control immediately
// return 0 if failed
//============================================================================================================
int send_reply( uint32_t service_from_id, const common_msg_t* mesg, TickType_t ticks_to_wait )
{
  int ret;
  common_msg_t temp;

  if( !service_from_id || !mesg || !get_id_to_reply( mesg->control ) )
    return 0;

  memcpy( &temp, mesg, sizeof(temp) );

  temp.type = 0;
  set_id_from_reply( temp.control, service_from_id );

  ret = send_mesg( get_id_to_reply( temp.control ), &temp, ticks_to_wait, 0 );
  if( !ret )
    return 0;

  return 1;
}

// wait reply up to @ticks_to_wait ticks
// this function may be used to allow wait for low priority service request processing
// from high priority service

// service_id - id of current service from which you are called this function
// mesg - pointer to structure which will be filled by reply message
// ticks_to_wait - amount of ticks, service will be sleep during which, 0 - means function return control immediately
// return 0 if failed
//============================================================================================================
int wait_reply( uint32_t service_id, common_msg_t* mesg, TickType_t ticks_to_wait )
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
      res = xQueueReceive( service->queue_id, ( void* )mesg, ticks_to_wait );
      if( res != pdTRUE )
        return 0;
    }
  }

  return 1;
}

// this function may be used for determine whether to send reply or not
// return 1 if reply must be send, 0 - otherwise
//============================================================================================================
int must_send_reply( const common_msg_t* mesg )
{
  if( !mesg )
    return 0;

  return get_id_to_reply( mesg->control ) != 0;
}
