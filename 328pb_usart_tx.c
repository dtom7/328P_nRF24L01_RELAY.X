/*
 * File:   usart_tx.c
 * Author: DONY THOMAS
 *
 * Created on 6 January, 2024, 4:56 PM
 */

#define F_CPU 16000000UL // 16 MHz 
#define BAUD 9600   
#define BAUD_TOL 2     

#include "328pb_usart_tx.h"
#include <avr/io.h>     
#include <avr/interrupt.h>   
#include <util/setbaud.h> 
#include <stdio.h>

void usart0_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // asynchronous, 8 bit-data, no parity, 1 stop
    UCSR0B = (1 << RXEN0) | (1 << TXEN0); // enable tx and rx
}

void usart0_tx_char(char data) {
    while (!(UCSR0A & (1 << UDRE0))); // wait for transmit buffer to receive new data
    //UCSR0A |= 1 << TXC0; 
    UDR0 = data;
    //while (!(UCSR0A & (1 << TXC0))); 
}

void usart0_tx_string(const char* string) {
    for (char i = 0; *(string + i) != '\0'; i++) {
        usart0_tx_char(*(string + i));
    }
    usart0_tx_char('\r');
    usart0_tx_char('\n');
}
