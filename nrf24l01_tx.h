/* 
 * File:   nrf24l01_tx.h
 * Author: DONY THOMAS
 *
 * Created on 25 January, 2024, 7:32 PM
 */

#ifndef NRF24L01_TX_H
#define	NRF24L01_TX_H

void initialize_tx(void);
unsigned char validate_all_registers(void);
void print_all_registers(void);
void write_tx_payload(unsigned char* bytes, unsigned char bytes_count);
unsigned char tx_in_progress(void);
unsigned char tx_is_successful(void);
unsigned char ack_payload_is_available(void);
void read_ack_payload(unsigned char* bytes, unsigned char bytes_count);
void print_tx_results(void);

#endif	/* NRF24L01_TX_H */

