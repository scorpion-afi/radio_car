/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>

#include "n_rf24l01_core.h"

/* CE line   - PA1
 * IRQ line  - PA2
 * SPI CSN   - PA4
 * SPI SCK   - PA5
 * SPI MISO  - PA6
 * SPI MOSI  - PA7
 *
 * BLUE_LED  - PC8
 * GREEN_LED - PC9
 * */

/* these defines ain't intended for portability,
 * just for convenient */
#define CE_PIN     GPIO1
#define IRQ_PIN    GPIO2

#define CSN_PIN    GPIO4
#define SCK_PIN    GPIO5
#define MISO_PIN   GPIO6
#define MOSI_PIN   GPIO7

#define BLUE_LED_PIN    GPIO8
#define GREEN_LED_PIN   GPIO9

static volatile int n_rf24l01_interrupt;

static void _blink_led( uint16_t gpio_pin, u_int ms )
{
  const int ticks_per_mks = 25000;

  /* at a 24MHz MCU frequency one instruction takes about 42 ns :), so 1 µs ~ 25 ticks */
  long unsigned int threshold = ms * ticks_per_mks;

  gpio_set( GPIOC, gpio_pin );

  for( long unsigned int i = 0; i < threshold; i++ )
    __asm( "nop" ); /* to prevent this loop being optimized out by a compiler */

  gpio_clear( GPIOC, gpio_pin );

  /* we need some delay between invocation of this function
   * (precisely between transitions of the led's state) */
  for( long unsigned int i = 0; i < 50 * ticks_per_mks; i++ )
    __asm( "nop" );
}

static void _set_up_csn_pin( u_char value )
{
  if( value )
    gpio_set( GPIOA, CSN_PIN );
  else
    gpio_clear( GPIOA, CSN_PIN );
}

static void _set_up_ce_pin( u_char value )
{
  if( value )
    gpio_set( GPIOA, CE_PIN );
  else
    gpio_clear( GPIOA, CE_PIN );
}

static void _usleep( u_int mks )
{
  /* at a 24MHz MCU frequency one instruction takes about 42 ns :), so 1 µs ~ 25 ticks */
  long unsigned int threshold = mks * 25;

  for( long unsigned int i = 0; i < threshold; i++ )
    __asm( "nop" ); /* to prevent this loop being optimized out by a compiler */
}

static void _send_cmd( u_char cmd, u_char* status_reg, u_char* data, u_char num, u_char direction )
{
  int i;

  /* @data can be passed as NULL if only an n_rf24l01 status register is going to be read */
  if( num && !data ) return;

  _set_up_csn_pin( 1 );
  _set_up_csn_pin( 0 );

  if( status_reg )
    *status_reg = spi_xfer( SPI1, cmd );
  else
    spi_xfer( SPI1, cmd );

  if( !num )
    goto finish;

  for( i = 0; i < num; i++ )
  {
    if( direction ) spi_xfer( SPI1, *data++ );
    else *data++ = spi_xfer( SPI1, 0xff );
  }

finish:
  _set_up_csn_pin( 1 );

  /* the n_rf24l01 chip needs 50ns to be ready for the new communication session
   * look at Tcwh in the documentation for the n_rf24l01 chip */
  _usleep( 1 );

  return;
}

/* gets called if there's some data on n_rf24l01 */
static void _handle_received_data( const void* data, u_int num )
{
  char* out = malloc( num );
  char* in = (char*)data;

  /* inverse back a received stream of bytes */
  for( u_int i = 0, j = num - 1; i < num; ++i, --j )
    out[j] = in[i];

  /* give the remote side some time to switch to receive mode */
  _usleep( 100000 );

  n_rf24l01_prepare_to_transmit();
  n_rf24l01_transmit_pkgs( out, num );
  n_rf24l01_prepare_to_receive();

  _blink_led( GREEN_LED_PIN, 50 );  /* blink to note: data has been sent */

  free( out );
}

static const n_rf24l01_backend_t n_rf24l01_backend =
{
    .set_up_ce_pin = _set_up_ce_pin,
    .send_cmd = _send_cmd,
    .usleep = _usleep,
    .handle_received_data = _handle_received_data
};

/* it's an isr for an interrupt on an EXTI2 source,
 * the vector table contains dummy isrs - weak symbols, so
 * a linker just replaces an appropriate dummy by this symbol */
void exti2_isr( void )
{
  n_rf24l01_upper_half_irq();

  /* TODO: if we get the next interrupt while processing a previous one?
   * TODO: what's about a protection for a var being under an access from isr and the main thread
   *       concurrently? */
  n_rf24l01_interrupt = 1;

  /* Note: we just reset an EXTI2 pending flag, a line is still active,
   * the n_rf24l01 library is responsible to reset it */
  exti_reset_request( EXTI2 );
}

static void _init_gpio_periph( void )
{
  rcc_periph_clock_enable( RCC_GPIOA );

  /* configure a CE line which controls an n_rf24l01's operational mode (receive/transmit) */
  gpio_set_mode( GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, CE_PIN );

  /* configure an IRQ line, which is connected to an n_rf24l01's irq line, as
   * an interrupt generation line using the EXTI subsystem */

  /* enable AFIO clock. */
  rcc_periph_clock_enable( RCC_AFIO );

  /* enable EXTI2 interrupt. */
  nvic_enable_irq( NVIC_EXTI2_IRQ );

  gpio_set_mode( GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, IRQ_PIN );

  /* configure the EXTI subsystem. */
  exti_select_source( EXTI2, GPIOA );
  exti_set_trigger( EXTI2, EXTI_TRIGGER_FALLING ); /* an IRQ line has a low active level */
  exti_enable_request( EXTI2 );

  /* configure LEDs */
  rcc_periph_clock_enable( RCC_GPIOC );
  gpio_set_mode( GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
                 GPIO_CNF_OUTPUT_PUSHPULL, BLUE_LED_PIN | GREEN_LED_PIN );
}

static void _init_spi_periph( void )
{
  rcc_periph_clock_enable( RCC_SPI1 );

  /* the n_rf24l01 chip requires a CSN/SCN/NSS pin to be active during whole communication
   * session (command + command's data/respond), so as the SPI hardware deactivates the
   * CSN/SCN/NSS pin after each spi transaction, we have to disable hardware control of
   * the CSN/SCN/NSS pin and control it manually */
  /* TODO: hardware control for the CSN/SCN/NSS pin has to work too, do it :) */
  gpio_set_mode( GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, CSN_PIN );

  gpio_set_mode( GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                 SCK_PIN | MOSI_PIN );
  gpio_set_mode( GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, MISO_PIN );

  spi_reset( SPI1 );

  /* set up SPI in Master mode with:
   *  clock baud rate: the lowest possible (24Mhz / 256 == 93750Hz)
   *  clock polarity: idle low
   *  clock phase: data valid on 1st clock pulse
   *  data frame format: 8-bit
   *  frame format: MSB First
   */
  spi_init_master( SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST );

  /* set NSS management to software
   *
   * Note:
   * setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management( SPI1 );
  spi_set_nss_high( SPI1 );

  /* enable SPI1 */
  spi_enable( SPI1 );
}

int main( void )
{
  int ret;

  rcc_clock_setup_in_hse_8mhz_out_24mhz();

  _init_gpio_periph();
  _init_spi_periph();

  ret = n_rf24l01_init( &n_rf24l01_backend );
  if( ret )
    return -1;

  _blink_led( GREEN_LED_PIN, 100 );  /* blink to note: we're ready to work... */

  n_rf24l01_prepare_to_transmit();
  n_rf24l01_transmit_pkgs( "Hi cruel world.", sizeof("Hi cruel world.") );

  /* the n_rf24l01 is not a full-duplex device so we have to switch
   * to the listen mode after we've transfered some data */
  n_rf24l01_prepare_to_receive();

  while( 1 )
  {
    if( n_rf24l01_interrupt )
    {
      n_rf24l01_interrupt = 0;

      _blink_led( BLUE_LED_PIN, 50 ); /* blink to note: got an interrupt, possible some data arrived */

      /* may cause a backend.handle_received_data() to be called, if some data arrived */
      n_rf24l01_bottom_half_irq();
    }
  }

  return 0;
}
