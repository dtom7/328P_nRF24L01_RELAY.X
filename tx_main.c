/*
 * File:   tx_main.c
 * Author: DONY THOMAS
 *
 * Created on 1 February, 2024, 10:52 PM
 */
#define F_CPU 16000000UL // 16 MHz

#include "nrf24l01_tx.h"
#include <util/delay.h>
#include "328pb_usart_tx.h"
#include <stdio.h>
#include <avr/io.h>

void tx_send(unsigned char tx_byte);
void loop(void);
void setup_ports(void);

int main(void) {
    _delay_ms(250); // give some time for nRF to enter power down mode from reset/start-up 
    setup_ports();
    usart0_init();
    usart0_tx_string("=================");
    initialize_tx();
    print_all_registers();
    usart0_tx_string("=================");
    if (validate_all_registers() == 1) {
        usart0_tx_string("validation passed");
        usart0_tx_string("=================");
        loop();
    } else {
        usart0_tx_string("validation failed");
    }
    usart0_tx_string("=================");
    while (1) {
    }
}

void loop(void) {
    while (1) {
        if (!(PIND & (1 << PD7))) { // ON
            _delay_ms(1000); // switch debounce delay
            tx_send(0xC8); // 200        
            usart0_tx_string("=================");
        }
        if (!(PIND & (1 << PD6))) { // OFF
            _delay_ms(1000); // switch debounce delay
            tx_send(0x64); // 100        
            usart0_tx_string("=================");
        }
    }
}

void setup_ports(void) {
    DDRD &= ~(1 << PD7); // PD7 as input for on switch
    PORTD |= (1 << PD7); // enable internal pullup resistor for PD7
    DDRD &= ~(1 << PD6); // PD6 as input for off switch
    PORTD |= (1 << PD6); // enable internal pullup resistor for PD6
    DDRD |= (1 << PD4); // PD4 as output for ack LED
    PORTD &= ~(1 << PD4); // turn off ack LED    
}

void tx_send(unsigned char tx_byte) {
    write_tx_payload(&tx_byte, 1);
    if (tx_is_successful()) {
        usart0_tx_string("tx_is_successful");
        if (ack_payload_is_available()) {
            usart0_tx_string("ack_payload_is_available");
            unsigned char bytes[2];
            read_ack_payload(bytes, 1);
            char str_buffer[25];
            sprintf(str_buffer, "ack payload: %d", bytes[1]);
            usart0_tx_string(str_buffer);
            if (bytes[1] == 0xFF) {
                PORTD |= (1 << PD4); // turn on ack LED 
                _delay_ms(500); // keep ack LED turned on for some time
            }
        } else {
            usart0_tx_string("ack_payload_not_available");
        }
    } else {
        usart0_tx_string("tx_failed");
    }
    print_tx_results();
    PORTD &= ~(1 << PD4); // turn off ack LED
}
