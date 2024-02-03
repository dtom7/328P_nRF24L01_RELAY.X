/* 
 * File:   nrf24l01_tx.c
 * Author: DONY THOMAS
 *
 * Created on 26 January, 2024, 8:17 PM
 */
#define F_CPU 16000000UL // 16 MHz

#include "nrf24l01.h"
#include "nrf24l01_tx.h"
#include <avr/io.h>

unsigned char tx_in_progress(void);

void initialize_tx(void) {
    initialize(PTX);
}

unsigned char validate_all_registers(void) {
    return _validate_all_registers(0x7E);  
}

void print_all_registers(void) {
    _print_all_registers();
}

void write_tx_payload(unsigned char* bytes, unsigned char bytes_count) {
    ce_down(); // transition to standby-1 mode
    flush_tx_fifo();
    flush_rx_fifo();
    clear_irq_flags();
    write_command(W_TX_PAYLOAD, bytes, bytes_count);
    ce_up(); // transition to tx mode
    while (tx_in_progress()); // wait for tx and re-tx to finish
    ce_down(); // transition to standby-1 mode
}

unsigned char tx_in_progress(void) {
    unsigned char STATUS_BYTE = read_status_register();
    // TX_DS means ack received after transmission
    // MAX_RT means number of retransmit exceeds max configured  
    if ((STATUS_BYTE & ((1 << TX_DS) | (1 << MAX_RT)))) {
        return 0;
    }
    return 1;
}

unsigned char tx_is_successful(void) {
    unsigned char STATUS_BYTE = read_status_register();
    // TX_DS means ack received after transmission
    // MAX_RT means number of retransmit exceeds max configured  
    if (STATUS_BYTE & (1 << TX_DS)) {
        return 1;
    }
    return 0;
}

unsigned char ack_payload_is_available(void) {
    unsigned char STATUS_BYTE = read_status_register();
    // RX_DR means RX_FIFO has ack payload 
    if (STATUS_BYTE & (1 << RX_DR)) {
        return 1;
    }
    return 0;
}

void read_ack_payload(unsigned char* bytes, unsigned char bytes_count) {
    read_command(R_RX_PAYLOAD, bytes, bytes_count);
}

void print_tx_results(void) {
    unsigned char bytes[2];
    read_register(OBSERVE_TX, bytes, 1);
    // bytes[0] is always STATUS register
    unsigned char ARC_CNT_BYTE = (bytes[1] & 0x0F);
    unsigned char PLOS_CNT_BYTE = ((bytes[1] & 0xF0) >> PLOS_CNT);
    usart_print_hex("ARC_CNT: ", ARC_CNT_BYTE);
    // PLOS_CNT is the number of packets that did not get through after maximum number of re-tx
    usart_print_hex("PLOS_CNT: ", PLOS_CNT_BYTE); // PLOS_CNT is 0, if the tx or re-tx succeeds
    reset_plos_cnt(); // resetting PLOS_CNT meaning it'll be 1 if all re-tx of a packet fails
}
