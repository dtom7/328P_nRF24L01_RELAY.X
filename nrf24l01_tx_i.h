/* 
 * File:   nrf24l01_tx_.h
 * Author: DONY THOMAS
 *
 * Created on 30 January, 2024, 9:34 PM
 */

#ifndef NRF24L01_TX__H
#define	NRF24L01_TX__H

void usart_print(const char* key, unsigned char value);
void print_register(const char* reg_name, unsigned char reg);
unsigned char validate_register(unsigned char reg, unsigned char expected_value);
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

#endif	/* NRF24L01_TX__H */

