/* 
 * File:   nrf24l01_tx.c
 * Author: DONY THOMAS
 *
 * Created on 26 January, 2024, 8:17 PM
 */
#define F_CPU 16000000UL // 16 MHz
#define CE PB1 // CE pin of nRF

#include "nrf24l01.h"
#include "nrf24l01_tx.h"
#include "nrf24l01_tx_i.h"
#include "328pb_usart_tx.h"
#include "spi_master.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

void initialize_tx(void) {
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
    CONFIG_BYTE &= ~(1 << PRIM_RX); // TX
    write_register(CONFIG, &CONFIG_BYTE, 1);
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
            validate_register(CONFIG, 0x7E));
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

unsigned char validate_register(unsigned char reg, unsigned char expected_byte) {
    if (reg == STATUS) {
        unsigned char byte = read_status_register();
        if (byte == 0x0E) {
            return 1;
        }
    } else {
        unsigned char bytes[2];
        read_register(reg, bytes, 1);
        // bytes[0] is always STATUS register
        if (bytes[0] == 0x0E && bytes[1] == expected_byte) {
            return 1;
        }
    }
    return 0;
}

void print_register(const char* reg_name, unsigned char reg) {
    if (reg == STATUS) {
        unsigned char byte = read_status_register();
        usart_print(reg_name, byte);
    } else {
        unsigned char bytes[2];
        read_register(reg, bytes, 1);
        usart_print(reg_name, bytes[1]);
    }
}

void usart_print(const char* key, unsigned char value) {
    char str_buffer[30];
    sprintf(str_buffer, "%s: %#04X", key, value);
    usart0_tx_string(str_buffer);
}

void write_register(unsigned char reg, unsigned char* bytes, unsigned char bytes_count) {
    write_command_or_register((W_REGISTER | reg), bytes, bytes_count);
}

void write_command(unsigned char command, unsigned char* bytes, unsigned char bytes_count) {
    write_command_or_register(command, bytes, bytes_count);
}

void write_command_or_register(unsigned char command_or_reg, unsigned char* bytes, unsigned char bytes_count) {
    unsigned char tx_bytes[bytes_count + 1]; // to include the register
    tx_bytes[0] = command_or_reg; // register address or command as least significant byte
    for (unsigned char i = 0; i < bytes_count; i++) {
        tx_bytes[i + 1] = *(bytes + i); // push the bytes
    }
    unsigned char rx_bytes[bytes_count + 1]; // received bytes - not used
    spi_master_tx_rx_bytes(tx_bytes, rx_bytes, (bytes_count + 1));
}

// bytes array should have the capacity to include the status register

void read_register(unsigned char reg, unsigned char* bytes, unsigned char bytes_count) {
    if (reg == STATUS) {
        *bytes = read_status_register();
    } else {
        read_command_or_register((R_REGISTER | reg), bytes, bytes_count);
    }
}

// bytes array should have the capacity to include the status register

void read_command(unsigned char command, unsigned char* bytes, unsigned char bytes_count) {
    read_command_or_register(command, bytes, bytes_count);
}

void read_command_or_register(unsigned char command_or_reg, unsigned char* bytes, unsigned char bytes_count) {
    unsigned char tx_bytes[bytes_count + 1]; // to include the register
    tx_bytes[0] = command_or_reg; // register address or command as least significant byte
    for (unsigned char i = 0; i < bytes_count; i++) {
        tx_bytes[i + 1] = NOP; // push the bytes
    }
    spi_master_tx_rx_bytes(tx_bytes, bytes, (bytes_count + 1));
}

void write_tx_payload(unsigned char* bytes, unsigned char bytes_count) {
    ce_down(); // transition to standby-1 mode
    //usart0_tx_string("ce_down");
    flush_tx_fifo();
    //usart0_tx_string("flush_tx_fifo");
    flush_rx_fifo();
    //usart0_tx_string("flush_rx_fifo");
    clear_irq_flags();
    //usart0_tx_string("clear_irq_flags");
    //reset_plos_cnt();
    //usart0_tx_string("reset_plos_cnt");
    write_command(W_TX_PAYLOAD, bytes, bytes_count);
    //usart0_tx_string("write_command");
    ce_up(); // transition to tx mode
    //usart0_tx_string("ce_up");
    while (tx_in_progress()); // wait for tx and re-tx to finish
    //usart0_tx_string("tx_wait_over");
    ce_down(); // transition to standby-1 mode
    //usart0_tx_string("ce_down");
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
    usart_print("ARC_CNT: ", ARC_CNT_BYTE);
    // PLOS_CNT is the number of packets that did not get through after maximum number of retries
    usart_print("PLOS_CNT: ", PLOS_CNT_BYTE);
}

void flush_tx_fifo(void) {
    spi_master_tx_rx_byte(FLUSH_TX);
}

void flush_rx_fifo(void) {
    spi_master_tx_rx_byte(FLUSH_RX);
}

void reset_plos_cnt(void) {
    unsigned char RF_CH_BYTE = 0x64;
    write_register(RF_CH, &RF_CH_BYTE, 1);
}

unsigned char read_status_register(void) {
    return spi_master_tx_rx_byte(STATUS);
}

void ce_down(void) {
    PORTB &= ~(1 << CE);
}

void ce_up(void) {
    PORTB |= (1 << CE);
}

void clear_irq_flags(void) {
    unsigned char STATUS_BYTE = read_status_register();
    STATUS_BYTE |= (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT);
    write_register(STATUS, &STATUS_BYTE, 1);
}