//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#include "common.h"

#include "led_service.h"
#include "transceiver_service.h"

// sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// this function turn on blue led (PC8) and invoke HardFault exception to prevent wrong work flow.
// you can use this function to finish work if something went wrong
// Note: this function isn't FreeRTOS dependent, so you can use it everywhere
//===================================================================================
void hardware_fail( void )
{
  int* temp = NULL;

  //turn on blue led
  GPIO_SetBits( GPIOC, GPIO_Pin_8 );

  // invoke hard fault exception
  *temp = 0;
}

//
//===================================================================================
int create_os_objects( void )
{
  int ret;

  ret = led_service_create();
  if( !ret )
    return 0;

  ret = transceiver_service_create();
  if( !ret )
    return 0;

  return 1;
}

// prepare stuffs that aren't tied with FreeRTOS
//===================================================================================
void init_not_os_objects( void )
{
  GPIO_InitTypeDef gpio_init;

  // blue led linked with PC8 pin

  // enable GPIOC peripheral clock
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE );

  // configure pin in output push/pull mode
  gpio_init.GPIO_Pin = GPIO_Pin_8;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init( GPIOC, &gpio_init );

  // turn off blue led
  GPIO_ResetBits( GPIOC, GPIO_Pin_8 );
}

//
//===================================================================================
int main( int argc, char* argv[] )
{
  BaseType_t ret;

  // assign all priority bits to be preempt priority bits, as FreeRTOS requires
  // NOTE: stm32 microcontrollers have only 4 bits for priority, so only 16 interrupt levels
  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

  init_not_os_objects();

  ret = create_os_objects();
  if( !ret )
    hardware_fail();

  vTaskStartScheduler();

  return 0;
}

#pragma GCC diagnostic pop
