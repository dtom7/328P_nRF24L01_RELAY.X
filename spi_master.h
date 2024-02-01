/* 
 * File:   spi_master.h
 * Author: DONY THOMAS
 *
 * Created on 21 January, 2024, 2:12 PM
 */

#ifndef SPI_MASTER_H
#define	SPI_MASTER_H

unsigned char spi_master_tx_rx_byte(unsigned char tx_byte);
void spi_master_tx_rx_bytes(unsigned char* tx_bytes, unsigned char* rx_bytes, unsigned char length);

#endif	/* SPI_MASTER_H */

