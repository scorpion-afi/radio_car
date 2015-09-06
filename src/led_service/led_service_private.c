#include "led_service_private.h"

// Port numbers: 0=A, 1=B, 2=C, 3=D, 4=E, 5=F, 6=G, ...
#define BLINK_PORT_NUMBER               (2)
#define BLINK_PIN_NUMBER                (9)
#define BLINK_ACTIVE_LOW                (1)

#define BLINK_GPIOx(_N)                 ((GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(_N)))
#define BLINK_PIN_MASK(_N)              (1 << (_N))
#define BLINK_RCC_MASKx(_N)             (RCC_APB2Periph_GPIOA << (_N))

// function, takes care about peripheral initialization used by this service
//===================================================================================
void led_init( void )
{
  // enable GPIO peripheral clock
  RCC_APB2PeriphClockCmd( BLINK_RCC_MASKx(BLINK_PORT_NUMBER), ENABLE );

  GPIO_InitTypeDef gpio_init;

  // configure pin in output push/pull mode
  gpio_init.GPIO_Pin = BLINK_PIN_MASK(BLINK_PIN_NUMBER);
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init( BLINK_GPIOx(BLINK_PORT_NUMBER), &gpio_init );

  // start with led turned off
  GPIO_ResetBits( BLINK_GPIOx(BLINK_PORT_NUMBER), BLINK_PIN_MASK(BLINK_PIN_NUMBER) );
}

// turn on led (about period/2 ms) and then turn off (about period/2 ms)
// Note: this function lulls thread !!!
//===================================================================================
void led_blink( uint32_t period )
{
	portTickType last_wake_time;

	last_wake_time = xTaskGetTickCount();

	GPIO_SetBits( BLINK_GPIOx(BLINK_PORT_NUMBER), BLINK_PIN_MASK(BLINK_PIN_NUMBER) );

	// thread will be slept about period/2 ms
	vTaskDelayUntil( &last_wake_time, ( (period/2) / portTICK_RATE_MS ) );

	GPIO_ResetBits( BLINK_GPIOx(BLINK_PORT_NUMBER), BLINK_PIN_MASK(BLINK_PIN_NUMBER) );

	// thread will be slept about period/2 ms
	vTaskDelayUntil( &last_wake_time, ( (period/2) / portTICK_RATE_MS ) );
}
