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
  // size of queue's item isn't specified due to all service's queues store common_msg_t structures,
  // so item size is sizeof(common_msg_t) bytes
  UBaseType_t length;			// length of queue
  QueueHandle_t* queue_id;	// pointer to handle of created queue, output parameter
} queue_t;

// this is common message description, don't use last field 'control' - it's service relative stuff.
// you must use only this structure to messages exchanging between services.
// this structure will be copied into queue, so size of element of every queue inside service is equal to
// size of this structure
typedef struct common_msg_t
{
  // type of message:
  //  0 - means reply type
  //  1, 2,... - services relative types
  uint8_t type;

  // 3 * 4 bytes
  union
  {
    uint32_t  data[3];
    uint32_t* ptrs[3];

    uint8_t byte[12];
    uint16_t short_data[6];
  };

  // private section
  uint32_t control;
} common_msg_t;

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

// send a message to queue associated with service identified by @service_id
// service_id - id of service message must be sent which
// mesg - pointer to common_msg_t structure (common message), pointer will be dereferenced and in queue will
//        be copied data @mesg points to, structure must be zero-filled once, after CREATION !!!
// ticks_to_wait - amount of ticks, service will be sleep during which, 0 - means function returns control immediately
// service_to_reply - id of service to send reply, specify 0 if you aren't interesting in sending of a reply
// return 0 if failed

// Note: message will be queued by copy, not by reference, so you may destroy data @mesg points to.
//============================================================================================================
extern int send_mesg( uint32_t service_id, common_msg_t* mesg, TickType_t ticks_to_wait, uint32_t service_to_reply );

// send reply message
// service_from_id - id of service which sends reply
// mesg   - pointer to structure defined reply, must be SAME structure that was dequeued from service queue
// ticks_to_wait - amount of ticks, service will be sleep during which, 0 - means function return control immediately
// return 0 if failed
//============================================================================================================
extern int send_reply( uint32_t service_from_id, const common_msg_t* mesg, TickType_t ticks_to_wait );

// wait reply up to @ticks_to_wait ticks
// this function may be used to allow waiting for low priority service request processing
// from high priority service

// service_id - id of current service from which you call this function
// mesg - pointer to structure which will be filled by reply message, you may set NULL, if only you want is wait
// ticks_to_wait - amount of ticks, service will be sleep during which, 0 - means function return control immediately
// return 0 if failed
//============================================================================================================
extern int wait_reply( uint32_t service_id, common_msg_t* mesg, TickType_t ticks_to_wait );

// this function may be used for determine whether to send reply or not
// return 1 if reply must be send, 0 - otherwise
//============================================================================================================
extern inline int must_send_reply( const common_msg_t* mesg );

#endif // SERVICE_CONTROL_H
