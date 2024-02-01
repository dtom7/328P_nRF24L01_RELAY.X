/* 
 * File:   328PB_USART_TX.h
 * Author: DONY THOMAS
 *
 * Created on 6 January, 2024, 4:53 PM
 */

#ifndef ATMEGA328PB_USART_TX_H
#define	ATMEGA328PB_USART_TX_H

void usart0_init(void);
void usart0_tx_char(char data);
void usart0_tx_string(const char*string);

#endif	/* ATMEGA328PB_USART_TX_H */

