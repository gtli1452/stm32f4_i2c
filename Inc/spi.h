#ifndef __SPI_H
#define __SPI_H

#ifndef __stdint_h
    #include "stdint.h"
#endif

/* ADATE320 SPI word structure */
typedef struct ATE_PACKET {
    uint8_t data_lo;
    uint8_t data_hi;
    uint8_t rw : 1;
    uint8_t address : 7;
    uint8_t channel : 2;
    uint8_t rsvd : 6;
} ate_packet_t;

typedef union SPI_PACKET {
    uint32_t spi_word;
    ate_packet_t spi;
} spi_packet_t;

extern volatile spi_packet_t sdo, sdi;

void  ate_hw_reset(void);

uint8_t write_spi(spi_packet_t sdo);

uint8_t read_spi(volatile spi_packet_t *sdi);

#endif
