/* 
 * File:   nrf24l01_rx.c
 * Author: DONY THOMAS
 *
 * Created on 31 January, 2024, 10:42 PM
 */
#define F_CPU 16000000UL // 16 MHz

#include "nrf24l01.h"
#include "nrf24l01_rx.h"
#include <avr/io.h>
#include <util/delay.h>

unsigned char rx_in_progress(void);
void process_received_packet(void);
void flush_and_clear(void);

void initialize_rx(void) {
    initialize(PRX);
}

unsigned char validate_all_registers(void) {
    return _validate_all_registers(0x7F);
}

void print_all_registers(void) {
    _print_all_registers();
}

void start_listening(void) {
    flush_and_clear();
    ce_up(); // transition to RX mode
    _delay_ms(10); // give some time to transition to RX mode
}

void search_and_process_packets(void) {
    unsigned char ack_p_bytes = 0xFF; // send current status as ack payload
    write_command(0xA8, &ack_p_bytes, 1); // 10101000 - data pipe 0
    usart0_tx_string("waiting for packet");
    while (rx_in_progress()); // wait for a packet
    usart0_tx_string("packet received");
    process_received_packet();
}

unsigned char rx_in_progress(void) {
    unsigned char STATUS_BYTE = read_status_register();
    // RX_DR means payload is available in RX FIFO 
    if (STATUS_BYTE & (1 << RX_DR)) {
        return 0;
    }
    return 1;
}

void process_received_packet(void) {
    unsigned char STATUS_BYTE = read_status_register();
    unsigned char RX_P_NO_BYTE = ((STATUS_BYTE & 0x0E) >> 1);
    usart_print_hex("RX_P_NO: ", RX_P_NO_BYTE); // expecting 0x00 here for data pipe 0
    unsigned char rx_pl_w_bytes[2];
    // read_register(RX_PW_P0, bytes, 1);
    read_command(R_RX_PL_WID, rx_pl_w_bytes, 1);
    // rx_pl_w_bytes[0] is always STATUS register
    usart_print_hex("R_RX_PL_WID: ", rx_pl_w_bytes[1]); // expecting 0x01 here for 1 byte
    unsigned char rx_p_bytes[2];
    read_command(R_RX_PAYLOAD, rx_p_bytes, 1);
    usart_print_dec("R_RX_PAYLOAD: ", rx_p_bytes[1]);
    // do some processing here
    flush_and_clear();
}

void flush_and_clear(void) {
    flush_rx_fifo();
    flush_tx_fifo();
    clear_irq_flags();
}