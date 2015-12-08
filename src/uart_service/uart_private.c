#include "uart_private.h"
#include "common.h"


/*// exti line 2 irq handler
//==============================================================================
void EXTI2_IRQHandler( void )
{
  read_payload();

  // clear the EXTI line 2 pending bit
  EXTI_ClearITPendingBit( EXTI_Line2 );
}*/


static void send_to_uart( uint8_t data )
{
  while( !(USART1->SR & USART_SR_TC) );
  USART1->DR = data;
}

void send_str( char* string )
{
  uint8_t i = 0;

  while( string[i] )
    send_to_uart( string[i++] );

  send_to_uart( '\r' );
  send_to_uart( '\n' );
}

/*static void enable_irq_handling( void )
{
  NVIC_InitTypeDef nvic_init;

  // enable EXTI line 2 interrupt
  // preemption priority: 12 or 0xcf > configMAX_SYSCALL_INTERRUPT_PRIORITY,
  // so we can use FreeRTOS API inside interrupt handler
  // sub priority: we don't use sub priority, look to main.c
  nvic_init.NVIC_IRQChannel = EXTI2_IRQn;
  nvic_init.NVIC_IRQChannelPreemptionPriority = 12;
  nvic_init.NVIC_IRQChannelSubPriority = 0;
  nvic_init.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &nvic_init );
}*/

/**
 * @brief initialize uart peripheral
 *
 * @return -1 if failed, 0 - otherwise
 */
//==============================================================================
int init_uart( void )
{
  GPIO_InitTypeDef gpio_init;
  USART_InitTypeDef usart_init;

  GPIO_StructInit( &gpio_init );
  USART_StructInit( &usart_init );

  // enable GPIOA peripheral clock
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );

  gpio_init.GPIO_Pin = GPIO_Pin_9;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOA, &gpio_init );

  gpio_init.GPIO_Pin = GPIO_Pin_10;
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOA, &gpio_init );

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1, ENABLE );

  usart_init.USART_BaudRate            = 115200;
  usart_init.USART_WordLength          = USART_WordLength_8b;
  usart_init.USART_StopBits            = USART_StopBits_1;
  usart_init.USART_Parity              = USART_Parity_No ;
  usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart_init.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init( USART1, &usart_init );
  USART_Cmd( USART1, ENABLE );

  return 0;
}
