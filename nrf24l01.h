/* 
 * File:   nrf24l01.h
 * Author: DONY THOMAS
 *
 * Created on 26 January, 2024, 8:17 PM
 */

#ifndef NRF24L01_H
#define	NRF24L01_H

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define CONT_WAVE   7
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

/* Non-P omissions */
#define LNA_HCURR 0

/* P model memory Map */
#define RPD                 0x09
#define W_TX_PAYLOAD_NO_ACK 0xB0

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

#define PTX 0
#define PRX 1

#define F_CPU 16000000UL // 16 MHz
#define CE PB1 // CE pin of nRF

#include "328pb_usart_tx.h"
#include "spi_master.h"
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

void initialize(unsigned char role);
void usart_print_hex(const char* key, unsigned char value);
void usart_print_dec(const char* key, unsigned char value);
void print_register(const char* reg_name, unsigned char reg);
void _print_all_registers(void);
unsigned char validate_register(unsigned char reg, unsigned char expected_value);
unsigned char _validate_all_registers(unsigned char config);
unsigned char read_status_register(void);
void write_register(unsigned char reg, unsigned char* bytes, unsigned char bytes_count);
void write_command(unsigned char command, unsigned char* bytes, unsigned char bytes_count);
void write_command_or_register(unsigned char command_or_reg, unsigned char* bytes, unsigned char bytes_count);
void read_register(unsigned char reg, unsigned char* bytes, unsigned char bytes_count);
void read_command(unsigned char command, unsigned char* bytes, unsigned char bytes_count);
void read_command_or_register(unsigned char command_or_reg, unsigned char* bytes, unsigned char bytes_count);
void ce_down(void);
void ce_up(void);
void clear_irq_flags(void);
void flush_tx_fifo(void);
void flush_rx_fifo(void);
void reset_plos_cnt(void);

void initialize(unsigned char role) {
    DDRB |= (1 << CE); // set CE as output
    // configure RF_SETUP register
    unsigned char RF_SETUP_BYTE = 0x00;
    RF_SETUP_BYTE &= ~(1 << RF_DR_LOW) & ~(1 << RF_DR_HIGH); // air data rate 1 Mbps
    RF_SETUP_BYTE |= (1 << RF_PWR_HIGH) | (1 << RF_PWR_LOW); // RF output power 0dBm
    write_register(RF_SETUP, &RF_SETUP_BYTE, 1);
    // configure RF_CH register
    unsigned char RF_CH_BYTE = 0x64;
    write_register(RF_CH, &RF_CH_BYTE, 1); // RF channel frequency 100 meaning 2.5 GHz or 2500 MHz
    // configure SETUP_RETR register
    unsigned char SETUP_RETR_BYTE = 0xFF; //0x31; // upto 1 re-transmit after a delay of 1000us if ack not received
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
    if (role == PRX) {
        CONFIG_BYTE |= (1 << PRIM_RX); // RX
    } else {
        CONFIG_BYTE &= ~(1 << PRIM_RX); // TX
    }
    write_register(CONFIG, &CONFIG_BYTE, 1);
    ce_down(); // transition to standby-1 mode
    _delay_ms(10); // give some time to transition to standby-1 mode
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

unsigned char _validate_all_registers(unsigned char config) {
    return (validate_register(STATUS, 0x0E) &
            validate_register(RF_SETUP, 0x06) &
            validate_register(RF_CH, 0x64) &
            validate_register(SETUP_RETR, 0xFF) &
            validate_register(EN_AA, 0x01) &
            validate_register(EN_RXADDR, 0x01) &
            validate_register(SETUP_AW, 0x02) &
            validate_register(RX_ADDR_P0, 0xE1) &
            validate_register(TX_ADDR, 0xE1) &
            validate_register(FEATURE, 0x06) &
            validate_register(DYNPD, 0x01) &
            validate_register(CONFIG, config));
}

void print_register(const char* reg_name, unsigned char reg) {
    if (reg == STATUS) {
        unsigned char byte = read_status_register();
        usart_print_hex(reg_name, byte);
    } else {
        unsigned char bytes[2];
        read_register(reg, bytes, 1);
        usart_print_hex(reg_name, bytes[1]);
    }
}

void _print_all_registers(void) {
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

void usart_print_hex(const char* key, unsigned char value) {
    char str_buffer[30];
    sprintf(str_buffer, "%s: %#04X", key, value);
    usart0_tx_string(str_buffer);
}

void usart_print_dec(const char* key, unsigned char value) {
    char str_buffer[30];
    sprintf(str_buffer, "%s: %d", key, value);
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

#endif	/* NRF24L01_H */

