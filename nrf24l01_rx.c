/* 
 * File:   nrf24l01_rx.c
 * Author: DONY THOMAS
 *
 * Created on 31 January, 2024, 10:42 PM
 */
#define F_CPU 16000000UL // 16 MHz
#define CE PB1 // CE pin of nRF

#include "nrf24l01.h"
#include "nrf24l01_rx.h"
#include <avr/io.h>
#include <util/delay.h>

unsigned char rx_in_progress(void);
void process_received_packet(void);
void flush_and_clear(void);

void initialize_rx(void) {
    DDRB |= (1 << CE); // set CE as output
    // configure RF_SETUP register
    unsigned char RF_SETUP_BYTE = 0x00;
    RF_SETUP_BYTE &= ~(1 << RF_DR_LOW) & ~(1 << RF_DR_HIGH); // air data rate 1Mbps
    RF_SETUP_BYTE |= (1 << RF_PWR_HIGH) | (1 << RF_PWR_LOW); // RF output power 0dBm
    write_register(RF_SETUP, &RF_SETUP_BYTE, 1);
    // configure RF_CH register
    unsigned char RF_CH_BYTE = 0x64;
    write_register(RF_CH, &RF_CH_BYTE, 1); // RF channel frequency 100 meaning 2.5 GHz or 2500 MHz
    // configure SETUP_RETR register
    unsigned char SETUP_RETR_BYTE = 0x35; //0x31; // upto 1 re-transmit after a delay of 1000us if ack not received
    write_register(SETUP_RETR, &SETUP_RETR_BYTE, 1);
    // configure EN_AA register
    unsigned char EN_AA_BYTE = 0x01;
    write_register(EN_AA, &EN_AA_BYTE, 1); // enable auto ack in data pipe 0 ??
    // configure EN_RXADDR register
    unsigned char EN_RXADDR_BYTE = 0x00;
    EN_RXADDR_BYTE |= (1 << ERX_P0); // enable data pipe 0
    write_register(EN_RXADDR, &EN_RXADDR_BYTE, 1);
    // configure SETUP_AW register
    unsigned char SETUP_AW_BYTE = 0x02;
    write_register(SETUP_AW, &SETUP_AW_BYTE, 1); // address width 4 bytes
    // configure RX_ADDR_P0 register 
    unsigned char RX_ADDR_P0_BYTES[] = {0xE1, 0xE2, 0xE3, 0xE4};
    write_register(RX_ADDR_P0, RX_ADDR_P0_BYTES, 4); // rx address of data pipe 0 (4 bytes)
    // configure TX_ADDR register 
    write_register(TX_ADDR, RX_ADDR_P0_BYTES, 4); // transmit address (4 bytes)
    // configure FEATURE register
    unsigned char FEATURE_BYTE = 0x00;
    FEATURE_BYTE |= (1 << EN_DPL) | (1 << EN_ACK_PAY); // enable dynamic payload length and payload with ack
    write_register(FEATURE, &FEATURE_BYTE, 1);
    // configure DYNPD register 
    unsigned char DYNPD_BYTE = 0x01;
    write_register(DYNPD, &DYNPD_BYTE, 1); // enable dynamic payload length in data pipe 0
    // configure CONFIG register
    unsigned char CONFIG_BYTE = 0x00;
    CONFIG_BYTE |= (1 << MASK_RX_DR) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT); // mask all interrupts
    CONFIG_BYTE |= (1 << EN_CRC) | (1 << CRCO); // enable CRC with 2 bytes
    CONFIG_BYTE |= (1 << PWR_UP); // power up
    CONFIG_BYTE |= (1 << PRIM_RX); // RX
    write_register(CONFIG, &CONFIG_BYTE, 1);
    ce_down(); // transition to standby-1 mode
    _delay_ms(10); // give some time to transition to standby-1 mode
}

unsigned char validate_all_registers(void) {
    return (validate_register(STATUS, 0x0E) &
            validate_register(RF_SETUP, 0x06) &
            validate_register(RF_CH, 0x64) &
            validate_register(SETUP_RETR, 0x35) &
            validate_register(EN_AA, 0x01) &
            validate_register(EN_RXADDR, 0x01) &
            validate_register(SETUP_AW, 0x02) &
            validate_register(RX_ADDR_P0, 0xE1) &
            validate_register(TX_ADDR, 0xE1) &
            validate_register(FEATURE, 0x06) &
            validate_register(DYNPD, 0x01) &
            validate_register(CONFIG, 0x7F));
}

void print_all_registers(void) {
    print_register("STATUS", STATUS);
    print_register("RF_SETUP", RF_SETUP);
    print_register("RF_CH", RF_CH);
    print_register("SETUP_RETR", SETUP_RETR);
    print_register("EN_AA", EN_AA);
    print_register("EN_RXADDR", EN_RXADDR);
    print_register("SETUP_AW", SETUP_AW);
    print_register("RX_ADDR_P0", RX_ADDR_P0);
    print_register("TX_ADDR", TX_ADDR);
    print_register("FEATURE", FEATURE);
    print_register("DYNPD", DYNPD);
    print_register("CONFIG", CONFIG);
}

void start_listening(void) {
    flush_and_clear();
    ce_up(); // transition to RX mode
    _delay_ms(10); // give some time to transition to RX mode
}

void search_and_process_packets(void) {
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
    flush_tx_fifo();
    unsigned char ack_p_bytes = 0xFA; // 250
    write_command(0xA8, &ack_p_bytes, 1); // 10101000 - data pipe 0
    flush_rx_fifo();
    clear_irq_flags();
}

void flush_and_clear(void) {
    flush_rx_fifo();
    flush_tx_fifo();
    clear_irq_flags();
}