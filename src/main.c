//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#include <stdio.h>
#include <stdlib.h>

#include "stm32f10x.h"

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"


// commands set
#define R_REGISTER 		0x00
#define W_REGISTER 		0x20
#define W_TX_PAYLOAD	0xa0
#define NOP 			0xff

// registers set
#define CONFIG_RG 		0x00
#define EN_AA_RG		0x01
#define RF_SETUP_RG		0x06
#define STATUS_RG		0x07

//--------- 5-bytes registers ---------
#define RX_ADDR_P0_RG 	0x0A
#define RX_ADDR_P1_RG 	0x0B
#define TX_ADDR_RG 		0x10
//-------------------------------------

#define RX_PW_P0_RG		0x11

// bits definition:

//  CONFIG register
#define PRIM_RX	0x01
#define PWR_UP 	0x02

// each register has 5 bits address in registers map
// used for R_REGISTER and W_REGISTER commands
#define REG_ADDR_BITS 0x1f

#define COMMAND_DATA_SIZE (32)



// Port numbers: 0=A, 1=B, 2=C, 3=D, 4=E, 5=F, 6=G, ...
#define BLINK_PORT_NUMBER               (2)
#define BLINK_PIN_NUMBER                (9)
#define BLINK_ACTIVE_LOW                (1)

#define BLINK_GPIOx(_N)                 ((GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(_N)))
#define BLINK_PIN_MASK(_N)              (1 << (_N))
#define BLINK_RCC_MASKx(_N)             (RCC_APB2Periph_GPIOA << (_N))

// IRQ line - PA2
// CE line  - PA3
// SPI SCN  - PA4
// SPI SCK  - PA5
// SPI MISO - PA6
// SPI MOSI - PA7

#define IRQ_PIN  GPIO_Pin_2
#define CE_PIN   GPIO_Pin_3

#define IRQ_PORT GPIOA
#define CE_PORT  GPIOA

#define IRQ_PIN_PERIPH RCC_APB2Periph_GPIOA
#define CE_PIN_PERIPH  RCC_APB2Periph_GPIOA

#define SPI_SCN_PIN  GPIO_Pin_4
#define SPI_SCK_PIN  GPIO_Pin_5
#define SPI_MISO_PIN GPIO_Pin_6
#define SPI_MOSI_PIN GPIO_Pin_7

#define SPI_PORT  GPIOA

#define SPI_PERIPH  RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA

int set_up_pin( uint16_t pin, uint8_t value );
int send_cmd( uint8_t cmd, uint8_t* status_reg, uint8_t* data, uint8_t num, uint8_t type );
int transmit_byte( uint8_t byte );
int prepare_to_transmit( void );
int prepare_to_receive( void );
int write_register( uint8_t reg_addr, uint8_t byte );
int read_register( uint8_t reg_addr, uint8_t* byte );
int clear_bits( uint8_t reg_addr, uint8_t bits );
int set_bits( uint8_t reg_addr, uint8_t bits );
int clear_pending_interrupts( void );

// this function set/clear CE pin
// value - value to set onto pin
//======================================================================================================
int set_up_pin( uint16_t pin, uint8_t value )
{
	if( value )
		GPIO_SetBits( GPIOA, pin );
	else
		GPIO_ResetBits( GPIOA, pin );

	return 0;
}

//
//===================================================================================
uint8_t _send_spi_data( uint8_t data )
{
	SPI_I2S_SendData( SPI1, data );
	while( 1 )
	{
		if( SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_BSY ) == SET ) continue;
		break;
	}

	return SPI_I2S_ReceiveData( SPI1 );
}

// send a command to n_rf24l01
// cmd - command to send
// status_reg - pointer n_rf24l01 status register will be written to
// data - pointer to data to be written to or to be read from n_rf24l01, depends on @type,
//		 real amount of data to read from/write to is depends on @num parameter
// num - amount of bytes to read or write (except command byte) (max amount is COMMAND_DATA_SIZE)
// type - type of operation: 1 - write, 0 - read
// return -1 if failed, 0 otherwise
// if you want to read only n_rf24l01 status register you may pass @cmd as NOP and
// after function execution in memory @status_reg points to will be status register
// Note: if you pass @num as 0, you just ask to execute you command and return status register.
//       if you want just execute command you can do this: send_command( you_cmd, NULL, NULL, 0, 0 )
//===================================================================================
int send_cmd( uint8_t cmd, uint8_t* status_reg, uint8_t* data, uint8_t num, uint8_t type )
{
	int i;

	// we can't transmit more then COMMAND_DATA_SIZE bytes to n_rf24l01 at one transaction
	if( num > COMMAND_DATA_SIZE ) return -1;

	// we can pass NULL as @data if we want to read only n_rf24l01 status register
	if( num && !data ) return -1;

	set_up_pin( SPI_SCN_PIN, 1 );
	set_up_pin( SPI_SCN_PIN, 0);

	if( status_reg )
		*status_reg = _send_spi_data( cmd );
	else
		_send_spi_data( cmd );

	for( i = 0; i < num; i++ )
	{
		if( type ) _send_spi_data( *data++ );
		else *data++ = _send_spi_data( 0xff );
	}

	set_up_pin( SPI_SCN_PIN, 1 );

	return 0;
}

// transmit one byte, n_rf24l01 must be in transmit mode
//======================================================================================================
int transmit_byte( uint8_t byte )
{
	int ret;
	uint8_t reg = 0;

	ret = send_cmd( W_TX_PAYLOAD, NULL, &byte, 1, 1 );
	if( ret < 0 ) return -1;

	read_register( STATUS_RG, &reg );
	read_register( CONFIG_RG, &reg );

	// CE up... sleep 10 us... CE down - to actual data transmit (in space)
	set_up_pin( SPI_SCN_PIN, 1 );
	//usleep( 1 );
	set_up_pin( SPI_SCN_PIN, 0 );

	// TODO: we must read status register after n_rf24l01 interrupt is occurred to determine
	//       that data were successfully transmitted, but currently we just sleep 1 second.
	//sleep( 1 );

	ret = clear_pending_interrupts();
	if( ret < 0 ) return -1;

	return 0;
}

// turn n_rf24l01 into transmit mode
//======================================================================================================
int prepare_to_transmit( void )
{
	int ret;

	ret = set_up_pin( CE_PIN, 0 );
	if( ret < 0 ) return -1;

	ret = clear_bits( CONFIG_RG, PRIM_RX );
	if( ret < 0 ) return -1;
	//usleep( 140 );

	return 0;
}

// turn n_rf24l01 into receive mode
//======================================================================================================
int prepare_to_receive( void )
{
	int ret;

	ret = set_bits( CONFIG_RG, PRIM_RX );
	if( ret < 0 ) return -1;

	ret = set_up_pin( CE_PIN, 1 );
	if( ret < 0 ) return -1;
	//usleep( 140 );

	return 0;
}

// write register with @reg_addr from @ptr
// reg_addr - address of register to be written to
// byte - variable register's content will be read from
// only for 1-byte registers
//======================================================================================================
int write_register( uint8_t reg_addr, uint8_t byte )
{
	int ret;

	// clear first command-specified bits (for R_REGISTER and W_REGISTER)
	reg_addr &= REG_ADDR_BITS;

	ret = send_cmd( W_REGISTER | reg_addr, NULL, &byte, 1, 1 );
	if( ret < 0 ) return -1;

	return 0;
}

// read register with @reg_addr in @ptr
// reg_addr - address of register to be read from
// byte - pointer to memory register's content will be written into
// only for 1-byte registers
//======================================================================================================
int read_register( uint8_t reg_addr, uint8_t* byte )
{
	int ret;

	if( !byte ) return -1;

	// clear first command-specified bits (for R_REGISTER and W_REGISTER)
	reg_addr &= REG_ADDR_BITS;

	ret = send_cmd( R_REGISTER | reg_addr, NULL, byte, 1, 0 );
	if( ret < 0 ) return -1;

	return 0;
}

// clear specified bits in register
// reg_addr - address of register to be modified
// bits - bits to be cleared
// return -1 if failed, 0 otherwise
// only for 1-byte registers
//======================================================================================================
int clear_bits( uint8_t reg_addr, uint8_t bits )
{
	uint8_t data = 0;
	int ret;

	// clear first command-specified bits (for R_REGISTER and W_REGISTER)
	reg_addr &= REG_ADDR_BITS;

	// firstly we must read register...
	ret = send_cmd( R_REGISTER | reg_addr, NULL, &data, 1, 0 );
	if( ret < 0 ) return -1;

	// apply new bits...
	data &= ~bits;

	// then set new bits
	ret = send_cmd( W_REGISTER | reg_addr, NULL, &data, 1, 1 );
	if( ret < 0 ) return -1;

	return 0;
}

// set specified bits in register
// reg_addr - address of register to be modified
// bits - bits to be set
// return -1 if failed, 0 otherwise
// only for 1-byte registers
//======================================================================================================
int set_bits( uint8_t reg_addr, uint8_t bits )
{
	uint8_t data = 0;
	int ret;

	// clear first command-specified bits (for R_REGISTER and W_REGISTER)
	reg_addr &= REG_ADDR_BITS;

	// firstly we must read register...
	ret = send_cmd( R_REGISTER | reg_addr, NULL, &data, 1, 0 );
	if( ret < 0 ) return -1;

	// apply new bits...
	data |= bits;

	// then set new bits
	ret = send_cmd( W_REGISTER | reg_addr, NULL, &data, 1, 1 );
	if( ret < 0 ) return -1;

	return 0;
}

// for clear interrupts pending bits
//======================================================================================================
int clear_pending_interrupts( void )
{
	int ret;
	uint8_t status_reg = 0;

	// NOP command: for read status register
	ret = send_cmd( NOP, &status_reg, NULL, 0, 0 );
	if( ret < 0 ) return -1;

	ret = write_register( STATUS_RG, status_reg );
	if( ret < 0 ) return -1;

	ret = send_cmd( NOP, &status_reg, NULL, 0, 0);
	if( ret < 0 ) return -1;

	return 0;
}

//
//===================================================================================
void n_rf24l01_init( void )
{
	GPIO_InitTypeDef gpio_init = {0, };
	SPI_InitTypeDef spi_init = {0, };

	// init pins: CE - line for transmit/receive control, IRQ - interrupt line

	// enable IRQ_PORT clock and prepare pin to be in input pull up mode (IRQ line has low active level)
	RCC_APB2PeriphClockCmd( IRQ_PIN_PERIPH, ENABLE );

	GPIO_StructInit( &gpio_init );
	gpio_init.GPIO_Pin = IRQ_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init( IRQ_PORT, &gpio_init );

	// enable CE_PORT clock and prepare pin to be in output push pull mode
	RCC_APB2PeriphClockCmd( CE_PIN_PERIPH, ENABLE);

	gpio_init.GPIO_Pin = CE_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( CE_PORT, &gpio_init );

	// set n_rf24l01 to receive mode
	GPIO_SetBits( CE_PORT, CE_PIN );

	// init spi to communicate with n_rf24l01 transceiver

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA, ENABLE );

	// we will control SCN pin directly by software
	gpio_init.GPIO_Pin = SPI_SCN_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &gpio_init );

	// move spi's pins in alternative work mode
	gpio_init.GPIO_Pin = SPI_SCK_PIN | SPI_MOSI_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &gpio_init );

	gpio_init.GPIO_Pin = SPI_MISO_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &gpio_init );

	SPI_StructInit( &spi_init );
	spi_init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi_init.SPI_Mode = SPI_Mode_Master;
	spi_init.SPI_DataSize = SPI_DataSize_8b;
	spi_init.SPI_CPOL = SPI_CPOL_Low;
	spi_init.SPI_CPHA = SPI_CPHA_1Edge;
	spi_init.SPI_NSS = SPI_NSS_Soft;
	spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	spi_init.SPI_FirstBit = SPI_FirstBit_MSB;

	SPI_Init( SPI1, &spi_init );
	SPI_Cmd( SPI1, ENABLE );

	// set NSS pin to 1
	SPI_NSSInternalSoftwareConfig( SPI1, SPI_NSSInternalSoft_Set );
}

//
//===================================================================================
void led_init( void )
{
  // Enable GPIO Peripheral clock
  RCC_APB2PeriphClockCmd(BLINK_RCC_MASKx(BLINK_PORT_NUMBER), ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure pin in output push/pull mode
  GPIO_InitStructure.GPIO_Pin = BLINK_PIN_MASK(BLINK_PIN_NUMBER);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(BLINK_GPIOx(BLINK_PORT_NUMBER), &GPIO_InitStructure);

  // Start with led turned off
  GPIO_ResetBits(BLINK_GPIOx(BLINK_PORT_NUMBER), BLINK_PIN_MASK(BLINK_PIN_NUMBER));
}

//
//===================================================================================
int main(int argc, char* argv[])
{
  uint8_t reg = 0;
  uint8_t status_reg = 0;

  //led_init();
  //GPIO_SetBits( BLINK_GPIOx(BLINK_PORT_NUMBER), BLINK_PIN_MASK(BLINK_PIN_NUMBER) );

  n_rf24l01_init();

  write_register( EN_AA_RG, 0x00 );

  // turn on n_rf24l01 transceiver
  set_bits( CONFIG_RG, PWR_UP);

  // set data field size to 1 byte (we will transmit 1 byte for time)
  write_register( RX_PW_P0_RG, 0x01 );

  // set the lowermost transmit power
  clear_bits( RF_SETUP_RG, 0x06 );

  read_register( CONFIG_RG, &reg );
  read_register( RX_PW_P0_RG, &reg );
  read_register( RF_SETUP_RG, &reg );

  prepare_to_transmit();

  send_cmd( W_TX_PAYLOAD, NULL, "11", 1, 1 );

  send_cmd( NOP, &status_reg, NULL, 0, 0 );

  set_up_pin( CE_PIN, 1 );
  set_up_pin( CE_PIN, 0 );

  // NOP command: for read status register
  send_cmd( NOP, &status_reg, NULL, 0, 0 );
  write_register( STATUS_RG, status_reg );
  send_cmd( NOP, &status_reg, NULL, 0, 0 );

  // Infinite loop
  while (1)
  {

  }
  // Infinite loop, never return.
}

#pragma GCC diagnostic pop
