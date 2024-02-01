/* 
 * File:   spi_master.c
 * Author: DONY THOMAS
 *
 * Created on 21 January, 2024, 2:12 PM
 */

#define F_CPU 16000000UL // 16 MHz

#define PORT_SPI PORTB
#define DDR_SPI DDRB
#define SS PB2
#define MOSI PB3
#define MISO PB4
#define SCK PB5

#include <avr/io.h>
#include "spi_master.h"

void spi_master_init(void) {
    SPCR = 0x00;
    DDR_SPI = 0x00; // all input
    DDR_SPI |= (1 << MOSI) | (1 << SCK) | (1 << SS); // MOSI, SCK & SS as output, MISO as input 
    PORT_SPI |= (1 << MISO); // set pullup on MISO
    SPCR |= (1 << SPE) | (1 << MSTR); // SPI enabled as master
    SPCR &= ~(1 << CPOL) & ~(1 << CPHA); // SPI mode 0
    SPCR |= (1 << SPR1) | (1 << SPR0); // fosc/128
    PORT_SPI |= (1 << SS); // SS high 
}

unsigned char spi_master_tx_rx_byte(unsigned char tx_byte) {
    spi_master_init();
    unsigned char rx_byte;
    PORT_SPI &= ~(1 << SS); // SS low
    SPDR = tx_byte; // load tx byte to data register
    while (!(SPSR & (1 << SPIF))); // wait for transmission to finish
    rx_byte = SPDR; // read rx byte from data register - this will also clear SPIF flag
    PORT_SPI |= (1 << SS); // SS high  
    return rx_byte;
}

void spi_master_tx_rx_bytes(unsigned char* tx_bytes, unsigned char* rx_bytes, unsigned char length) {
    spi_master_init();
    PORT_SPI &= ~(1 << SS); // SS low
    for (unsigned char i = 0; i < length; i++) {
        SPDR = *(tx_bytes + i);
        while (!(SPSR & (1 << SPIF)));
        *(rx_bytes + i) = SPDR;
    }
    PORT_SPI |= (1 << SS); // SS high 
}
