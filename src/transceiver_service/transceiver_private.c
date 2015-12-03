#include "transceiver_private.h"
#include "common.h"

#include "n_rf24l01/src/n_rf24l01_port.h"

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

// this function sets/clears CE pin
// value - value to set onto pin
//==============================================================================
void set_up_ce_pin( u_char value )
{
  if( value )
    GPIO_SetBits( CE_PORT, CE_PIN ); // set '1' on CE_PIN
  else
    GPIO_ResetBits( CE_PORT, CE_PIN ); // set '0' on CE_PIN
}

// send a command to n_rf24l01
// cmd - command to send
// status_reg - pointer n_rf24l01 status register will be written to
// data - pointer to data to be written to or to be read from n_rf24l01, depends on @type,
//		 real amount of data to read from/write to is depends on @num parameter
// num - amount of bytes to read or write (except command byte) (max amount is COMMAND_DATA_SIZE)
// type - type of operation: 1 - write, 0 - read
// return -1 if failed, 0 otherwise
//==============================================================================
void send_cmd( u_char cmd, u_char* status_reg, u_char* data, u_char num, u_char type )
{

}

// this function put to sleep library execution flow, max sleep interval ~1500 ms
//==============================================================================
void usleep( u_int ms )
{
  /*int i;

  if( ms < ( 1000 / configTICK_RATE_HZ ) )
    for( i = 0; i < configCPU_CLOCK_HZ; i++ );*/
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
  GPIO_InitTypeDef gpio_init =
  {
      0, };

  // init pins: CE - line for transmit/receive control, IRQ - interrupt line

  // enable IRQ_PORT clock and prepare pin to be in input pull up mode (IRQ line has low active level)
  RCC_APB2PeriphClockCmd( IRQ_PERIPH, ENABLE );

  GPIO_StructInit( &gpio_init );
  gpio_init.GPIO_Pin = IRQ_PIN;
  gpio_init.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init( IRQ_PORT, &gpio_init );

  // enable CE_PORT clock and prepare pin to be in output push pull mode
  RCC_APB2PeriphClockCmd( CE_PERIPH, ENABLE );

  gpio_init.GPIO_Pin = CE_PIN;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( CE_PORT, &gpio_init );

  // set '1' on CE_PIN
  GPIO_SetBits( CE_PORT, CE_PIN );
}

//
//==============================================================================
static void init_spi_periph( void )
{
  GPIO_InitTypeDef gpio_init =
  {
      0, };
  SPI_InitTypeDef spi_init =
  {
      0, };

  // init spi to communicate with n_rf24l01 transceiver

  RCC_APB2PeriphClockCmd( SPI_PERIPH | SPI_SCN_PERIPH, ENABLE );

  // we must controls SCN pin by software due to n_rf24l01 command must have
  // one level on SCN(NSS) pin during all command transaction, command can consist of
  // several spi transactions, between each we cann't change SCN(NSS).
  gpio_init.GPIO_Pin = SPI_SCN_PIN;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( SPI_SCN_PORT, &gpio_init );

  // move spi's pins in alternative work mode
  gpio_init.GPIO_Pin = SPI_SCK_PIN | SPI_MOSI_PIN;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( SPI_PINS_PORT, &gpio_init );

  gpio_init.GPIO_Pin = SPI_MISO_PIN;
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init( SPI_PINS_PORT, &gpio_init );

  SPI_StructInit( &spi_init );
  spi_init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spi_init.SPI_Mode = SPI_Mode_Master;
  spi_init.SPI_DataSize = SPI_DataSize_8b;
  spi_init.SPI_CPOL = SPI_CPOL_Low;
  spi_init.SPI_CPHA = SPI_CPHA_1Edge;
  spi_init.SPI_NSS = SPI_NSS_Soft;
  spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  spi_init.SPI_FirstBit = SPI_FirstBit_MSB;

  SPI_Init( SPI_MODULE, &spi_init );
  SPI_Cmd( SPI_MODULE, ENABLE );

  // set NSS pin to 1
  SPI_NSSInternalSoftwareConfig( SPI_MODULE, SPI_NSSInternalSoft_Set );
  GPIO_SetBits( SPI_SCN_PORT, SPI_SCN_PIN );
}

// make first init steps to properly n_rf24l01 work
// returns -1 if failed, 0 - otherwise
//==============================================================================
int init_n_rf24l01( void )
{
  int ret;

  init_gpio_periph();
  init_spi_periph();

  ret = n_rf24l01_init( &n_rf24l01_backend );
  if( ret )
    return -1;

  return 0;
}
