/*
 * File:   rx_main.c
 * Author: DONY THOMAS
 *
 * Created on 1 February, 2024, 10:52 PM
 */
#define F_CPU 16000000UL // 16 MHz

#include "nrf24l01_rx.h"
#include <util/delay.h>
#include "328pb_usart_tx.h"
#include <stdio.h>

void loop(void);

int main(void) {
    _delay_ms(250); // give some time for nRF to enter power down mode from reset/start-up
    usart0_init();
    usart0_tx_string("=================");
    initialize_rx();
    print_all_registers();
    usart0_tx_string("=================");
    if (validate_all_registers() == 1) {
        usart0_tx_string("validation passed");
        usart0_tx_string("=================");
        start_listening();
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
        search_and_process_packets();
        usart0_tx_string("=================");
    }
}
