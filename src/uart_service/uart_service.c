// transceiver control service

#include "uart_service.h"
#include "uart_private.h"

#include "service_control.h"

// id of this service
static uint32_t cur_serv_id;

// queue that stores requests to this service
static QueueHandle_t cur_serv_queue;

/**
 * @brief send string to PC
 *
 * param[in] str - string to send over uart interface, must be truncated by '/0' symbol
 *
 * Note: string will be copied in previously allocated memory in heap
 *       inside function.
 */
void uart_send_str( const char* const str )
{
  char* temp = NULL;
  size_t str_len;

  if( !str ) return;

  str_len = strlen( str );

  // '\n' + '\r' + '/0'
  temp = malloc( str_len + 2 + 1 );
  if( !temp ) return;

  memcpy( temp, str, str_len + 1 );
  temp[str_len + 1] = '\n';
  temp[str_len + 2] = '\r';

  xQueueSendToBack( cur_serv_queue, ( void* )&temp, portMAX_DELAY );
}

/*
// semaphor to implement delayed interrupt service (exti0: user button)
static xSemaphoreHandle exti_0_semaphor;

// exti line 0 irq handler
//==============================================================================
void EXTI0_IRQHandler(void)
{
  portBASE_TYPE force_context_switch = pdFALSE; // must be explicitly reset
  static int cnt;

  // antibounce mechanism - it's only for test purpose !!!
  if( ++cnt > 1 )
  {
    cnt = 0;

    xSemaphoreGiveFromISR( exti_0_semaphor, &force_context_switch );

    led_flash_irq( 200, 100 );

    // after irq processing will be finished switch to woken high priority task immediately
    if( force_context_switch )
      portEND_SWITCHING_ISR( force_context_switch );
  }

  // clear the EXTI line 0 pending bit
  EXTI_ClearITPendingBit( EXTI_Line0 );
}

// init exti line 0 (PA0 - user button)
//==============================================================================
static void init( void )
{
  GPIO_InitTypeDef gpio_init;
  EXTI_InitTypeDef exti_init;
  NVIC_InitTypeDef nvic_init;

  // create semaphore to implement delayed interrupt service
  vSemaphoreCreateBinary( exti_0_semaphor );
  if( !exti_0_semaphor )
    hardware_fail();

  // reset the EXTI peripheral registers to their default reset values
  EXTI_DeInit();

  // fills each gpio_init fields with its default value
  GPIO_StructInit( &gpio_init );

  // fills each exti_init fields with its reset value
  EXTI_StructInit( &exti_init );

  // enable GPIOA and AFIO clock
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE );

  // configure PA0 as input floating
  gpio_init.GPIO_Pin = GPIO_Pin_0;
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init( GPIOA, &gpio_init );

  // connect PA0 to EXTI line 0
  GPIO_EXTILineConfig( GPIO_PortSourceGPIOA, GPIO_PinSource0 );

  // configure EXTI line 0
  exti_init.EXTI_Line = EXTI_Line0;
  exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
  exti_init.EXTI_Trigger = EXTI_Trigger_Rising;
  exti_init.EXTI_LineCmd = ENABLE;
  EXTI_Init( &exti_init );

  // enable EXTI line 0 interrupt
  // preemption priority: 12 or 0xcf > configMAX_SYSCALL_INTERRUPT_PRIORITY,
  // so we can use FreeRTOS API inside interrupt handler
  // sub priority: we don't use sub priority, look to main.c
  nvic_init.NVIC_IRQChannel = EXTI0_IRQn;
  nvic_init.NVIC_IRQChannelPreemptionPriority = 12;
  nvic_init.NVIC_IRQChannelSubPriority = 0;
  nvic_init.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &nvic_init );
}*/

// transceiver service thread-handler
//==============================================================================
static void uart_thread( void* params )
{
  BaseType_t res;
  char* str = NULL;

  res = init_uart();
  if( res )
    hardware_fail();

  while( 1 )
  {
    res = xQueueReceive( cur_serv_queue, ( void* )&str, portMAX_DELAY );
    if( res != pdTRUE )
      hardware_fail();

    send_str( str );
    free( str );
  }

}

// this function is called before FreeRTOS scheduler starts, in main function
// return 0 if failed
//==============================================================================
int uart_service_create( void )
{
  thread_t thread =
  {
      .thread_name = uart_thread,
      .name = services_names[2],
      .stack_depth = 128,
      .params = NULL,
      .priority = 1,
      .hndl = NULL	// we aren't interesting in this handle
  };

  queue_t queue =
  {
      .elm_size = sizeof(char*),
      .length = 5,
      .queue_id = &cur_serv_queue // we will use this queue's id for reading events from queue
  };

  cur_serv_id = service_create( &thread, &queue );

  return cur_serv_id;
}
