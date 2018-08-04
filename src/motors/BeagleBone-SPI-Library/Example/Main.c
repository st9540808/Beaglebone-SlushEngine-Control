/**
 * Main.c
 *
 * Created on: Sep 6, 2017
 * Author: Vinay Divakar
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "SPI.h"

/* Enable SPI Debug */
#define SPI_DBG

/* Externs - SPI.c */
unsigned char TX_spi[SPIDEV_BYTES_NUM];
unsigned char RX_spi[SPIDEV_BYTES_NUM];

int main(void)
{
    int i = 0;
    memset(TX_spi, 0, sizeof TX_spi);
    memset(RX_spi, 0, sizeof RX_spi);
    TX_spi[0] = 0x56;
    TX_spi[1] = 0x43;

    if (SPI_DEV1_init(SPIDEV_BYTES_NUM, SPIDEV1_BUS_SPEED_HZ, SPI_SS_LOW,
                      SPIDEV_DELAY_US, SPIDEV_DATA_BITS_NUM,
                      SPI_MODE3) == -1)
        printf("(Main)spidev1.0 initialization failed\r\n");
    else
        printf("(Main)spidev1.0 initialized - READY\r\n");

#ifdef SPI_DBG
    while (1) {
        if (SPIDEV1_transfer(TX_spi, RX_spi, NO_OF_BYTES) == 0) {
            printf("(Main)spidev1.0: Transaction Complete\r\n");
            printf("%x%x\n", TX_spi[0], TX_spi[1]);
        } else
            printf("(Main)spidev1.0: Transaction Failed\r\n");

        usleep(100000);
        printf("%d\r\n", ++i);
    }
#endif
    return 0;
}
