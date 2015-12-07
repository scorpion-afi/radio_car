#include "transceiver_private.h"
#include "common.h"

#include "n_rf24l01/src/n_rf24l01_port.h"

#include "led_service.h"

// IRQ line - PA2
// CE line  - PA3
// SPI SCN  - PA4
// SPI SCK  - PA5
// SPI MISO - PA6
// SPI MOSI - PA7

#define IRQ_PIN  	GPIO_Pin_2
#define IRQ_PORT 	GPIOA
#define IRQ_PERIPH 	RCC_APB2Periph_GPIOA

#define CE_PIN   	GPIO_Pin_3
#define CE_PORT  	GPIOA
#define CE_PERIPH  	RCC_APB2Periph_GPIOA

#define SPI_SCN_PIN  	GPIO_Pin_4
#define SPI_SCN_PORT  	GPIOA
#define SPI_SCN_PERIPH	RCC_APB2Periph_GPIOA

#define SPI_SCK_PIN  	GPIO_Pin_5
#define SPI_MISO_PIN 	GPIO_Pin_6
#define SPI_MOSI_PIN 	GPIO_Pin_7

#define SPI_MODULE		SPI1
#define SPI_PINS_PORT	GPIOA
#define SPI_PERIPH		RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA


// exti line 2 irq handler
//==============================================================================
void EXTI2_IRQHandler( void )
{
  led_flash_irq( 450, 150 );

  // clear the EXTI line 2 pending bit
  EXTI_ClearITPendingBit( EXTI_Line2 );
}

/**
 * @brief
 *
 * @param[in] n -
 */
/*static void _delay_asm( u_char n )
{
 //asm()
}*/

// this function sets/clears CSN (NSS/SCN) pin
// value - value to set onto pin
//==============================================================================
static void _set_up_csn_pin( u_char value )
{
  if( value )
    GPIO_SetBits( GPIOA, SPI_SCN_PIN ); // set '1' on SPI_SCN_PIN
  else
    GPIO_ResetBits( GPIOA, SPI_SCN_PIN ); // set '0' on SPI_SCN_PIN
}

// write/read byte to/from spi 1 peripheral
//===================================================================================
static uint8_t _send_spi_data( u_char data )
{
    SPI_I2S_SendData( SPI1, data );

    while( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_RXNE ) == RESET  );

    return SPI_I2S_ReceiveData( SPI1 );
}

// this function sets/clears CE pin
// value - value to set onto pin
//==============================================================================
static void set_up_ce_pin( u_char value )
{
  if( value )
    GPIO_SetBits( GPIOA, GPIO_Pin_3 ); // set '1' on CE_PIN
  else
    GPIO_ResetBits( GPIOA, GPIO_Pin_3 ); // set '0' on CE_PIN
}

// send a command to n_rf24l01
// cmd - command to send
// status_reg - pointer n_rf24l01 status register will be written to
// data - pointer to data to be written to or to be read from n_rf24l01, depends on @type,
//       real amount of data to read from/write to is depends on @num parameter
// num - amount of bytes to read or write (except command byte) (max amount is COMMAND_DATA_SIZE)
// type - type of operation: 1 - write, 0 - read
// return -1 if failed, 0 otherwise
// if you want to read only n_rf24l01 status register you may pass @cmd as NOP and
// after function execution in memory @status_reg points to will be status register
// Note: if you pass @num as 0, you just ask to execute you command and return status register.
//       if you want just execute command you can do this: send_command( you_cmd, NULL, NULL, 0, 0 )
//====================================================================================
static void send_cmd( u_char cmd, u_char* status_reg, u_char* data, u_char num, u_char type )
{
  int i;

  // we can pass NULL as @data if we want to read only n_rf24l01 status register
  if( num && !data ) return;

  _set_up_csn_pin( 1 );
  _set_up_csn_pin( 0 );

  if( status_reg )
      *status_reg = _send_spi_data( cmd );
  else
      _send_spi_data( cmd );

  for( i = 0; i < num; i++ )
  {
      if( type ) _send_spi_data( *data++ );
      else *data++ = _send_spi_data( 0xff );
  }

  _set_up_csn_pin( 1 );

  return;
}

/**
 * @brief this function put to sleep library execution flow, max sleep interval ~1500 ms
 *
 * @param[in] mks - time in microseconds to sleep
 */
//==============================================================================
static void usleep( u_int mks )
{
  TickType_t delay;

  // TODO: dirty hack !!! (must be fixed)

  // minimal delay - portTICK_RATE_MS ms
  delay = (mks / 1000) / portTICK_RATE_MS;
  if( !delay )
    delay = 1;

  vTaskDelay( delay );
}

static const n_rf24l01_backend_t n_rf24l01_backend =
{
    .set_up_ce_pin = set_up_ce_pin,
    .send_cmd = send_cmd,
    .usleep = usleep };

//
//==============================================================================
static void init_gpio_periph( void )
{
  GPIO_InitTypeDef gpio_init;
  EXTI_InitTypeDef exti_init;
  NVIC_InitTypeDef nvic_init;

  // PA3 - CE (line for transmit/receive switch control)

  GPIO_StructInit( &gpio_init );

  // enable GPIOA peripheral clock
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );

  // configure pin in output push/pull mode
  gpio_init.GPIO_Pin = GPIO_Pin_3;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init( GPIOA, &gpio_init );

  // PA2 - IRQ (interrupt line)

  GPIO_StructInit( &gpio_init );
  EXTI_StructInit( &exti_init );

  // enable GPIOA clock and prepare pin to be in input floating mode
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );

  gpio_init.GPIO_Pin = GPIO_Pin_2;
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init( GPIOA, &gpio_init );

  // enable AFIO clock
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );

  // connect PA2 to EXTI line 2
  GPIO_EXTILineConfig( GPIO_PortSourceGPIOA, GPIO_PinSource2 );

  // configure EXTI line 2
  exti_init.EXTI_Line = EXTI_Line2;
  exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
  exti_init.EXTI_Trigger = EXTI_Trigger_Falling; // (IRQ line has low active level)
  exti_init.EXTI_LineCmd = ENABLE;
  EXTI_Init( &exti_init );

  // enable EXTI line 2 interrupt
  // preemption priority: 12 or 0xcf > configMAX_SYSCALL_INTERRUPT_PRIORITY,
  // so we can use FreeRTOS API inside interrupt handler
  // sub priority: we don't use sub priority, look to main.c
  nvic_init.NVIC_IRQChannel = EXTI2_IRQn;
  nvic_init.NVIC_IRQChannelPreemptionPriority = 12;
  nvic_init.NVIC_IRQChannelSubPriority = 0;
  nvic_init.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &nvic_init );
}

//
//==============================================================================
static void init_spi_periph( void )
{
  GPIO_InitTypeDef gpio_init;
  SPI_InitTypeDef  spi_init;

  // initialize spi to communicate with n_rf24l01 transceiver

  GPIO_StructInit( &gpio_init );
  SPI_StructInit( &spi_init );

  // enable SPI1 and GPIOA clock
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA, ENABLE );

  // we must controls SCN pin by software due to n_rf24l01 command must have
  // one level on SCN(NSS) pin during all command transaction, command can consist of
  // several spi transactions, between each we cann't change SCN(NSS).
  gpio_init.GPIO_Pin = GPIO_Pin_4;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOA, &gpio_init );

  // configure spi's pins (SPI_SCK and SPI_MOSI) in alternative work mode
  gpio_init.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOA, &gpio_init );

  // configure spi's pin (SPI_MISO) in input floating mode
  gpio_init.GPIO_Pin =  GPIO_Pin_6;
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( GPIOA, &gpio_init );

  spi_init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spi_init.SPI_Mode = SPI_Mode_Master;
  spi_init.SPI_DataSize = SPI_DataSize_8b;
  spi_init.SPI_CPOL = SPI_CPOL_Low;
  spi_init.SPI_CPHA = SPI_CPHA_1Edge;
  spi_init.SPI_NSS = SPI_NSS_Soft;
  spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  spi_init.SPI_FirstBit = SPI_FirstBit_MSB;

  // set NSS pin to 1
  SPI_NSSInternalSoftwareConfig( SPI1, SPI_NSSInternalSoft_Set );

  SPI_Init( SPI1, &spi_init );
  SPI_Cmd( SPI1, ENABLE );
}

// make first init steps to properly n_rf24l01 work
// returns -1 if failed, 0 - otherwise
//==============================================================================
int init_n_rf24l01( void )
{
  int ret;

  init_gpio_periph();
  init_spi_periph();

  // set '1' on CE_PIN
  set_up_ce_pin( 1 );

  // set NSS pin to 1
  _set_up_csn_pin( 1 );

  ret = n_rf24l01_init( &n_rf24l01_backend );
  if( ret )
    return -1;

  prepare_to_transmit();

  n_rf24l01_transmit_byte( 'L' );
  n_rf24l01_transmit_byte( 'e' );
  n_rf24l01_transmit_byte( 'n' );
  n_rf24l01_transmit_byte( 'a' );
  n_rf24l01_transmit_byte( ' ' );
  n_rf24l01_transmit_byte( 'I' );
  n_rf24l01_transmit_byte( ' ' );
  n_rf24l01_transmit_byte( 'l' );
  n_rf24l01_transmit_byte( 'i' );
  n_rf24l01_transmit_byte( 'k' );
  n_rf24l01_transmit_byte( 'e' );
  n_rf24l01_transmit_byte( ' ' );
  n_rf24l01_transmit_byte( 'y' );
  n_rf24l01_transmit_byte( 'o' );
  n_rf24l01_transmit_byte( 'u' );
  n_rf24l01_transmit_byte( '!' );

  return 0;
}
