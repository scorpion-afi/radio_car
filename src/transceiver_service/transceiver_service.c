// transceiver control service

#include "transceiver_service.h"
#include "transceiver_private.h"

#include "service_control.h"

// id of this service
static uint32_t cur_serv_id;


// transceiver service thread-handler
//==============================================================================
static void transceiver_thread( void* params )
{
  int ret;

  // initialize some peripheral, n_rf24l01 library and transceiver
  ret = init_n_rf24l01();
  if( ret )
    hardware_fail();

  // never get out from this function
  state_machine();
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

  // currently transceiver service doesn't serve any requests, hw works only as receiver !!!
  cur_serv_id = service_create( &thread, NULL );

  return cur_serv_id;
}
