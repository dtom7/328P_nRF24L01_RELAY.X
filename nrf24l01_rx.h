/* 
 * File:   nrf24l01_rx.h
 * Author: DONY THOMAS
 *
 * Created on 31 January, 2024, 10:42 PM
 */

#ifndef NRF24L01_RX_H
#define	NRF24L01_RX_H

void initialize_rx(void);
unsigned char validate_all_registers(void);
void print_all_registers(void);
void start_listening(void);
void search_and_process_packets(void);

#endif	/* NRF24L01_RX_H */

