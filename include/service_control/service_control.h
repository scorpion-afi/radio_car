#ifndef SERVICE_CONTROL_H
#define SERVICE_CONTROL_H

#include "common.h"

// this structure defines thread configuration parameters, thread is main part of service
typedef struct thread_t
{
  TaskFunction_t thread_name; // pointer to thread-handler

  // name of thread, which will be used for debugging and as service,
  // based on this thread, name
  const char* name;
  uint16_t stack_depth;		// depth of thread's stack, in stack elements, not in bytes
  void* params;				// pointer to parameters, which will be passed to thread-handler
  UBaseType_t priority;		// FreeRTOS thread priority
  TaskHandle_t* hndl;	// pointer to handle of created thread (used for destroy thread only), output parameter
} thread_t;

// this structure defines queue configuration parameters, each service has itself queue to handle requests
typedef struct queue_t
{
  size_t elm_size;          // size of one element in queue
  UBaseType_t length;			// length of queue
  QueueHandle_t* queue_id;	// pointer to handle of created queue, output parameter
} queue_t;


// if you want your service can be used by another services, add service name to this services_names array (look to service_control.c)
// and pass this name, via thread_info.name, to service_create function, see, for example, led_service creation
extern const char* services_names[];

// create service: thread with associated queue
// thread_info - pointers to struct described thread creation info (look to service_control.h)
// queue_info - pointers to struct described queue creation info (look to service_control.h)
// return service's id, 0 if failed
//============================================================================================================
extern uint32_t service_create( thread_t* thread_info, queue_t* queue_info );

// give service's id from service's name
// service_name - name of service (look to services_names array to know existed services names)
// return service's id, 0 if failed
//============================================================================================================
extern uint32_t get_service_id( const char* service_name );

// send a message to queue associated with service identified by @serv_id_to
// serv_id_to - id of service message must be sent which
// data - pointer to data to be copied (enqueue) into queue
// ticks_to_wait - amount of ticks, service will be sleep during which, 0 - means function returns control immediately
// return 0 if failed
// NOTE: function will copy amount of bytes specified during service creation, from memory @data points to
//============================================================================================================
extern int send_mesg( uint32_t serv_id_to, const void* data, TickType_t ticks_to_wait );

#endif // SERVICE_CONTROL_H
